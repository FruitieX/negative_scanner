use log::{debug, info, trace};
use opencv::{
    core::{self, min_max_loc, Point, Scalar},
    highgui,
    imgproc::{self, INTER_LINEAR},
    prelude::*,
    types::VectorOfMat,
    videoio, Result,
};
use rodio::{source::Source, Decoder, OutputStream, OutputStreamHandle};
use serialport::SerialPort;
use std::fs::File;
use std::io::BufReader;
use std::time::{Duration, Instant};

#[derive(PartialEq)]
enum Direction {
    Right,
    Left,
}

const FILM_MOVE_DIR: Direction = Direction::Right;

const MAX_SPEED: usize = 10000;

/// Crop factors used for detection.
/// For example a vertical value of 0.9 will discard the top and bottom 5%
/// of the image before performing frame detection.
///
/// This is useful for ignoring the edge of the frame either in the film
/// or in the scanner itself. I'm also using a horizontal crop because my
/// camera has a 16:9 HDMI output but the sensor is 3:2.
const DETECTION_HORIZONTAL_CROP: f64 = (3.0 / 2.0) / (16.0 / 9.0);
const DETECTION_VERTICAL_CROP: f64 = 0.90;

/// How many pixels one step corresponds to
const PX_PER_STEP: f64 = 1.7;

/// How many steps are lost when transitioning from
/// forward movement to backward movement due to slop
/// between the gear and film sprockets.
const SLOP_STEPS: usize = 20;

/// Always step this many extra steps when aligning frame, tries to ensure
/// that the leading edge of the frame never ends up visible.
const EXTRA_ALIGNMENT_STEPS: usize = 15;

/// Extra time to wait before running detection after a move command. Helps
/// avoid motion blur, and gives camera some time to adjust auto exposure.
const PRE_DETECTION_WAIT_MSEC: u64 = 200;

/// Gives my sausage fingers some time to detach themselves from the film
const PRE_FEED_NEW_FILM_WAIT: u64 = 500;

/// Account for camera video feed -> opencv input lag
const INPUT_LAG_MSEC: u64 = 200;

/// How many milliseconds it takes to move the film 1000 steps
const MSEC_PER_1000_STEPS: u64 = 400;

/// Time it takes for stepper motor to (de)accelerate between zero and max
/// speed (maxSpeed / acceleration)
const STEPPER_ACCEL_TIME: u64 = 0;

/// How many steps to move the film forward after taking a photo.
const NEXT_FRAME_SKIP_STEPS: usize = 984;

/// Slow film down for easier feeding when frame moves past this point (in
/// percentage of width). Film is slowed down until DETECTION_POS
const FEED_SLOW_POS: f64 = 0.00;

/// Stop film for more accurate position measurements when frame moves past this
/// point (in percentage of width)
const DETECTION_POS: f64 = 0.55;

/// End of roll detection when
const END_OF_ROLL_DETECTION_POS: f64 = 0.4;

/// Don't take photos if this is true.
const DRY_RUN: bool = false;

#[derive(Debug)]
struct State {
    current: ScannerState,
}

#[derive(Debug, PartialEq)]
enum ErrorReason {
    EndOfRoll,
    SuspiciousStartGap,
}

#[derive(Debug, PartialEq)]
enum ScannerState {
    TogglingFocus {
        init_timestamp: Instant,
        step: usize,
    },
    Error {
        init_timestamp: Instant,
        reason: ErrorReason,
    },
    AligningFrame {
        wait_until: Instant,
        last_seen_dist: Option<f64>,
    },
    TakingPhoto {
        wait_until: Instant,
    },
    WaitingForShutterClose,
    WaitingForShutterOpen,
    SkipToNextFrame {
        wait_until: Instant,
    },
}

impl Default for ScannerState {
    fn default() -> Self {
        ScannerState::AligningFrame {
            wait_until: Instant::now(),
            last_seen_dist: None,
        }
    }
}

impl State {
    pub fn new() -> State {
        info!("Searching for frames...");
        State {
            current: ScannerState::default(),
        }
    }

    pub fn set(&mut self, new: ScannerState) {
        if self.current != new {
            trace!("Entering state: {:?}", new);
        }
        self.current = new;
    }
}

#[derive(Clone, Debug)]
struct Contour {
    contour: Mat,
    area: f64,
    end_x: f64,
    start_x: f64,
}

struct Scanner {
    tx: std::sync::mpsc::Sender<String>,
    prev_dir: Option<Direction>,
    prev_cmd_was_stop: bool,
}

impl Scanner {
    pub fn new(mut port: Box<dyn SerialPort>) -> Scanner {
        let (tx, rx) = std::sync::mpsc::channel::<String>();

        std::thread::spawn(move || loop {
            let msg = rx.recv();

            if let Ok(msg) = msg {
                trace!("Writing to serial: {}", msg);
                let bytes: &[u8] = msg.as_bytes();
                trace!("Bytes: {:?}", bytes);
                port.write_all(bytes)
                    .expect("Writing to serial port failed");
            }
        });

        Scanner {
            tx,
            prev_dir: None,
            prev_cmd_was_stop: false,
        }
    }

    pub fn set_speed(&mut self, speed: Option<usize>) {
        if let Some(speed) = speed {
            self.write(&format!("{}s", speed))
        } else {
            self.write(&format!("{}s", MAX_SPEED))
        }
    }

    pub fn move_right(&mut self, mut steps: usize, speed: Option<usize>) {
        if self.prev_dir == Some(Direction::Left) {
            steps += SLOP_STEPS;
        }
        self.set_speed(speed);
        self.write(&format!("{}M", steps));
        self.prev_dir = Some(Direction::Right);
        self.prev_cmd_was_stop = false;
    }

    pub fn move_left(&mut self, mut steps: usize, speed: Option<usize>) {
        if self.prev_dir == Some(Direction::Right) {
            steps += SLOP_STEPS;
        }
        self.set_speed(speed);
        self.write(&format!("-{}M", steps));
        self.prev_dir = Some(Direction::Left);
        self.prev_cmd_was_stop = false;
    }

    pub fn stop(&mut self, silent: bool) {
        if !silent {
            info!("Immediately stopping motor");
        }

        self.write("0M");

        // Skip slop removal if previous cmd was stop
        if self.prev_cmd_was_stop {
            return;
        }

        // Remove slop from gears
        match self.prev_dir {
            Some(Direction::Right) => {
                self.move_left(0, None);
            }
            Some(Direction::Left) => {
                self.move_right(0, None);
            }
            _ => {}
        }

        self.prev_cmd_was_stop = true;
    }

    pub fn take_photo(&mut self) {
        info!("Taking photo");

        self.write("S");
    }

    pub fn focus(&mut self) {
        self.write("F");
    }

    pub fn stop_focus(&mut self) {
        self.write("f");
    }

    fn write(&mut self, buf: &str) {
        self.tx.send(buf.to_string()).ok();
    }
}

struct ColStats {
    col_mean: Mat,
    col_std_dev: Mat,
    col_diffs: Mat,
    col_mins: Mat,
    col_maxs: Mat,
}

/// Computes mean values, standard deviation, min and max pixel values within
/// each column.
fn get_col_stats(mat: &Mat) -> Result<ColStats> {
    let mut mean_value = Mat::default();
    let mut std_value = Mat::default();

    let mut col_mean = unsafe { Mat::new_nd(1, &mat.cols(), opencv::core::CV_64F)? };
    let mut col_std_dev = unsafe { Mat::new_nd(1, &mat.cols(), opencv::core::CV_64F)? };
    let mut col_diffs = unsafe { Mat::new_nd(1, &mat.cols(), opencv::core::CV_64F)? };
    let mut col_mins = unsafe { Mat::new_nd(1, &mat.cols(), opencv::core::CV_64F)? };
    let mut col_maxs = unsafe { Mat::new_nd(1, &mat.cols(), opencv::core::CV_64F)? };

    let mut mat_bw = Mat::default();
    imgproc::cvt_color(&mat, &mut mat_bw, imgproc::COLOR_RGB2GRAY, 0)?;

    for i in 0..mat.cols() {
        opencv::core::mean_std_dev(
            &mat.col(i)?,
            &mut mean_value,
            &mut std_value,
            &Mat::default(),
        )?;

        {
            let col_mean: &mut f64 = col_mean.at_mut(i)?;
            let vec: Vec<Vec<f64>> = mean_value.to_vec_2d()?;
            let sum: f64 = vec.iter().flatten().sum();
            let sum = sum / 255.0 / 3.0;
            *col_mean = sum;
        }

        {
            let col_std_dev: &mut f64 = col_std_dev.at_mut(i)?;
            let vec: Vec<Vec<f64>> = std_value.to_vec_2d()?;
            let sum: f64 = vec.iter().flatten().sum();
            let sum = sum / 255.0;
            *col_std_dev = sum;
        }

        // Min / max values
        {
            let mut min_val = Some(0.0);
            let mut max_val = Some(0.0);

            min_max_loc(
                &mat_bw.col(i)?,
                min_val.as_mut(),
                max_val.as_mut(),
                None,
                None,
                &Mat::default(),
            )?;

            let col_diff: &mut f64 = col_diffs.at_mut(i)?;
            *col_diff = (max_val.unwrap() - min_val.unwrap()) / 255.0;

            let col_mins: &mut f64 = col_mins.at_mut(i)?;
            *col_mins = min_val.unwrap() / 255.0;

            let col_maxs: &mut f64 = col_maxs.at_mut(i)?;
            *col_maxs = max_val.unwrap() / 255.0;
        }
    }

    Ok(ColStats {
        col_mean,
        col_std_dev,
        col_diffs,
        col_mins,
        col_maxs,
    })
}

fn dbg_col(mat: &Mat, winname: &str) -> Result<()> {
    let mut mat_show = mat.clone();
    core::transpose(&mat_show.clone(), &mut mat_show)?;
    core::flip(&mat_show.clone(), &mut mat_show, 0)?;
    imgproc::resize(
        &mat_show.clone(),
        &mut mat_show,
        core::Size_::new(540, 480),
        0.0,
        0.0,
        INTER_LINEAR,
    )?;
    highgui::imshow(winname, &mat_show)?;

    Ok(())
}

struct Audio {
    // If we drop the stream variable, we won't be able to play any sounds.
    #[allow(dead_code)]
    stream: OutputStream,
    stream_handle: OutputStreamHandle,
}

impl Audio {
    fn new() -> Audio {
        // Get a output stream handle to the default physical sound device
        let (stream, stream_handle) = OutputStream::try_default().unwrap();

        Audio {
            stream,
            stream_handle,
        }
    }

    fn play_sound(&self, filename: &str) {
        let filename = String::from(filename);

        // Load a sound from a file, using a path relative to Cargo.toml
        let file = BufReader::new(File::open(filename).unwrap());

        // Decode that sound file into a source
        let source = Decoder::new(file).unwrap();

        // Play the sound directly on the device
        self.stream_handle
            .play_raw(source.convert_samples())
            .unwrap();
    }
}

fn main() -> Result<()> {
    env_logger::init();
    let audio = Audio::new();

    let port_infos = serialport::available_ports().expect("No serial ports found");
    let port_info = port_infos.first().expect("No serial ports found");
    let port = serialport::new(port_info.port_name.clone(), 115_200)
        .timeout(Duration::from_secs(1))
        .open()
        .expect("Failed to open serial port");

    let mut pause = true;
    info!("Starting in paused/manual mode. Hit spacebar to start automation.");
    info!("Manual mode controls:");
    info!("r = Reset to initial state");
    info!("q = Quit");
    info!("e = Move film forward");
    info!("a = Move film back");
    info!("s = Shutter");
    info!("f = Focus");
    info!("n = Next frame");
    info!("u = Stop focus");

    let mut scanner = Scanner::new(port);

    let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)?; // 0 is the default camera

    if !videoio::VideoCapture::is_opened(&cam)? {
        panic!("Unable to open default camera");
    }

    let mut frame = Mat::default();

    // Capture next frame
    cam.read(&mut frame)?;

    let frame_width = frame.cols() as f64;
    let frame_height = frame.rows() as f64;

    let mut state = State::new();

    let mut diff_avg = 0.00;

    loop {
        // Capture next frame
        cam.read(&mut frame)?;

        let cropped_width = (frame_width * DETECTION_HORIZONTAL_CROP).floor() - 4.0;
        let cropped_x = ((frame_width - cropped_width) / 2.0).floor() + 2.0;
        let cropped_height = (frame_height * DETECTION_VERTICAL_CROP).floor();
        let cropped_y = ((frame_height - cropped_height) / 2.0).floor();

        // Make sure we don't crop outside image (crashes opencv)
        let cropped_x = cropped_x.min(frame_width - cropped_width);
        let cropped_y = cropped_y.min(frame_height - cropped_height);

        // Crop image
        let rect = core::Rect::new(
            cropped_x as i32,
            cropped_y as i32,
            cropped_width as i32,
            cropped_height as i32,
        );
        let frame_cropped = Mat::roi(&frame, rect)?;

        // Apply median blur to get rid of small scratches / noise
        let mut frame_blurred = Mat::default();
        imgproc::median_blur(&frame_cropped, &mut frame_blurred, 2 * 2 + 1)?;
        // highgui::imshow("frame_blurred", &frame_blurred)?;

        let ColStats {
            col_mean,
            col_std_dev,
            col_diffs,
            col_mins,
            col_maxs,
        } = get_col_stats(&frame_blurred)?;

        // dbg!(format!(
        //     "{:?}",
        //     &col_mean
        //         .to_vec_2d::<f64>()
        //         .into_iter()
        //         .flatten()
        //         .flatten()
        //         .map(|x| (x * 255.0).round() as i32)
        //         .collect::<Vec<i32>>()
        // ));

        // Only used for debugging
        {
            dbg_col(&col_std_dev, "col_std_dev")?;
            dbg_col(&col_mean, "col_mean")?;
            dbg_col(&col_diffs, "col_diffs")?;
            dbg_col(&col_maxs, "col_maxs")?;
            dbg_col(&col_mins, "col_mins")?;
        }

        // Find darkest and brightest column mean values
        // TODO: should maybe use col_mins and col_maxs?
        // let (_col_mean_darkest, col_mean_brightest) = {
        //     let mut min_val = Some(0.0);
        //     let mut max_val = Some(0.0);
        //     min_max_loc(
        //         &col_mean,
        //         min_val.as_mut(),
        //         max_val.as_mut(),
        //         None,
        //         None,
        //         &Mat::default(),
        //     )?;

        //     (min_val.unwrap(), max_val.unwrap())
        // };
        // dbg!(col_mean_brightest, col_mean_darkest);

        let (col_min, col_max) = {
            let mut min_val = Some(0.0);
            let mut max_val = Some(0.0);
            min_max_loc(
                &col_mins,
                min_val.as_mut(),
                None,
                None,
                None,
                &Mat::default(),
            )?;

            min_max_loc(
                &col_maxs,
                None,
                max_val.as_mut(),
                None,
                None,
                &Mat::default(),
            )?;

            (min_val.unwrap(), max_val.unwrap())
        };

        let new_diff_avg = col_max - col_min;
        let factor = 0.95;
        diff_avg = factor * diff_avg + (1.0 - factor) * new_diff_avg;
        trace!("diff_avg: {:.2}", diff_avg);

        let col_min_thr_val = col_max * 0.7;

        let mut col_min_thr = Mat::default();
        col_mins.convert_to(&mut col_min_thr, core::CV_8U, 255.0, 0.0)?;
        imgproc::threshold(
            &col_min_thr.clone(),
            &mut col_min_thr,
            col_min_thr_val * 255.0,
            255.0,
            imgproc::THRESH_BINARY,
        )?;
        dbg_col(&col_min_thr, "col_min_thr")?;

        let col_std_dev_thr_val = 0.075;
        let mut col_std_dev_thr = Mat::default();
        col_std_dev.convert_to(&mut col_std_dev_thr, core::CV_8U, 255.0, 0.0)?;
        imgproc::threshold(
            &col_std_dev_thr.clone(),
            &mut col_std_dev_thr,
            col_std_dev_thr_val * 255.0,
            255.0,
            imgproc::THRESH_BINARY_INV,
        )?;
        dbg_col(&col_std_dev_thr, "col_std_dev_thr")?;

        let col_diffs_thr_val = 0.25;
        let mut col_diffs_thr = Mat::default();
        col_diffs.convert_to(&mut col_diffs_thr, core::CV_8U, 255.0, 0.0)?;
        imgproc::threshold(
            &col_diffs_thr.clone(),
            &mut col_diffs_thr,
            col_diffs_thr_val * 255.0,
            255.0,
            imgproc::THRESH_BINARY_INV,
        )?;
        dbg_col(&col_diffs_thr, "col_diffs_thr")?;

        let mut mask = Mat::default();
        core::bitwise_and(&col_min_thr, &col_std_dev_thr, &mut mask, &Mat::default())?;
        core::bitwise_and(&mask.clone(), &col_diffs_thr, &mut mask, &Mat::default())?;
        dbg_col(&mask, "mask")?;

        // Convert frame to grayscale
        let mut frame_bw = Mat::default();
        imgproc::cvt_color(&frame_blurred, &mut frame_bw, imgproc::COLOR_RGB2GRAY, 0)?;
        highgui::imshow("frame_bw", &frame_bw)?;

        // Find contours of mask
        core::transpose(&mask.clone(), &mut mask)?;
        core::flip(&mask.clone(), &mut mask, 0)?;
        imgproc::resize(
            &mask.clone(),
            &mut mask,
            core::Size_::new(cropped_width as i32, cropped_height as i32),
            0.0,
            0.0,
            INTER_LINEAR,
        )?;

        let mut contours = VectorOfMat::new();
        imgproc::find_contours(
            &mask,
            &mut contours,
            imgproc::RETR_TREE,
            imgproc::CHAIN_APPROX_SIMPLE,
            Point::new(0, 0),
        )?;

        // Compute area and contour right edge x position
        let mut ctrs: Vec<Contour> = contours
            .iter()
            .map(|contour| {
                let area = imgproc::contour_area(&contour, false).unwrap();

                let bbox = imgproc::bounding_rect(&contour).unwrap();
                let seen_width = bbox.width as f64;
                let seen_end = bbox.x as f64 + seen_width;

                Contour {
                    contour,
                    area,
                    end_x: seen_end,
                    start_x: bbox.x as f64,
                }
            })
            // .filter(|moments| moments.area > 100_000.0)
            .filter(|ctr| ctr.area > 0.0)
            // Filter out small areas unless they are at the left or right of the frame
            // (These edges are critical to get the alignment of the film just right)
            // .filter(|ctr| {
            //     // TODO: magic numbers
            //     (ctr.start_x <= 10.0 || ctr.end_x >= cropped_width as f64 - 10.0)
            //         || ctr.area > 5000.0
            // })
            .collect();

        let filtered_contours: VectorOfMat = ctrs.iter().map(|ctr| ctr.contour.clone()).collect();
        imgproc::draw_contours(
            &mut frame,
            &filtered_contours,
            -1,
            Scalar::new(0.0, 255.0, 0.0, 255.0),
            2,
            imgproc::LINE_8,
            &core::no_array(),
            i32::MAX,
            Point::new(cropped_x as i32, cropped_y as i32),
        )?;

        ctrs.sort_by(|a, b| a.area.partial_cmp(&b.area).unwrap());
        let _largest_ctr = ctrs.iter().last().cloned();

        ctrs.sort_by(|a, b| a.end_x.partial_cmp(&b.end_x).unwrap());
        let last_ctr = match FILM_MOVE_DIR {
            Direction::Right => ctrs.first(),
            Direction::Left => ctrs.last(),
        };
        let first_ctr = match FILM_MOVE_DIR {
            Direction::Right => ctrs.last(),
            Direction::Left => ctrs.first(),
        };

        let edge_ctr = match FILM_MOVE_DIR {
            Direction::Right => first_ctr.map(|ctr| {
                if ctr.end_x >= cropped_width {
                    Some(ctr)
                } else {
                    None
                }
            }),
            Direction::Left => {
                first_ctr.map(|ctr| if ctr.start_x <= 0.0 { Some(ctr) } else { None })
            }
        }
        .flatten();

        // dbg!(&edge_ctr);

        let mut frame_inv = Mat::default();
        core::bitwise_not(&frame, &mut frame_inv, &core::no_array())?;
        highgui::imshow("frame", &frame_inv)?;

        // How many pixels film still needs to move according to detection
        let dist_to_next_gap_end = match (first_ctr, FILM_MOVE_DIR) {
            (Some(next_ctr), Direction::Right) => Some(cropped_width - next_ctr.start_x),
            (Some(next_ctr), Direction::Left) => Some(next_ctr.end_x),
            _ => None,
        };

        let dist_to_edge_gap_end = match (edge_ctr, FILM_MOVE_DIR) {
            (Some(edge_ctr), Direction::Right) => Some(cropped_width - edge_ctr.start_x),
            (Some(edge_ctr), Direction::Left) => Some(edge_ctr.end_x),
            _ => None,
        };

        // Read input from user
        let key: char = highgui::poll_key()? as u8 as char;

        if key == 'q' {
            break;
        }

        // Move film right
        if key == 'e' {
            pause = true;
            state.set(ScannerState::default());

            scanner.move_right(5, Some(200));
        }
        // FF right
        if key == '.' {
            pause = true;
            state.set(ScannerState::default());

            scanner.move_right(100, None);
        }

        // Move film left
        if key == 'a' {
            pause = true;
            state.set(ScannerState::default());

            scanner.move_left(5, Some(200));
        }
        // FF left
        if key == '\'' {
            pause = true;
            state.set(ScannerState::default());

            scanner.move_left(100, None);
        }

        // Shutter
        if key == 's' {
            pause = true;
            state.set(ScannerState::default());

            scanner.take_photo();
        }

        // Focus
        if key == 'f' {
            pause = true;
            state.set(ScannerState::default());

            scanner.focus();
        }

        // Stop focus
        if key == 'u' {
            pause = true;
            state.set(ScannerState::default());

            scanner.stop_focus();
        }

        // Move to next frame
        if key == 'n' {
            pause = true;
            state.set(ScannerState::default());

            scanner.move_right(NEXT_FRAME_SKIP_STEPS, None);
        }

        // Move film 1000 steps for calibration
        if key == 'c' {
            pause = true;
            state.set(ScannerState::default());

            scanner.move_right(1000, None);
        }

        // Move to next gap
        if key == 'g' {
            pause = true;
            state.set(ScannerState::default());

            if let Some(dist_to_next_gap_end) = dist_to_next_gap_end {
                let steps = dist_to_next_gap_end * PX_PER_STEP;
                let steps = steps.round(); //.max(5.0);
                let steps = steps + EXTRA_ALIGNMENT_STEPS as f64;
                match FILM_MOVE_DIR {
                    Direction::Right => {
                        scanner.move_right(steps as usize, None);
                    }
                    Direction::Left => {
                        scanner.move_left(steps as usize, None);
                    }
                }
            }
        }

        // Reset
        if key == 'r' {
            pause = true;
            state.set(ScannerState::default());
        }

        if key == 't' {
            state.set(ScannerState::TakingPhoto {
                wait_until: Instant::now(),
            });
            pause = false;
        }

        if key == ' ' {
            pause = !pause;
            if pause {
                state.set(ScannerState::default());
                info!("Paused. Hit spacebar to unpause.");
                scanner.stop(true);
            } else {
                state.set(ScannerState::TogglingFocus {
                    init_timestamp: Instant::now(),
                    step: 0,
                });
                info!("Unpaused. Hit spacebar to pause.");
            }
        }

        if pause {
            continue;
        }

        // Whether last ctr is at start of frame
        let gap_at_start = if let Some(last_ctr) = last_ctr {
            match FILM_MOVE_DIR {
                Direction::Right => last_ctr.start_x <= 0.0,
                Direction::Left => last_ctr.end_x >= cropped_width,
            }
        } else {
            false
        };

        let end_of_roll = if let Some(last_ctr) = last_ctr {
            if gap_at_start {
                match FILM_MOVE_DIR {
                    Direction::Right => last_ctr.end_x >= cropped_width * END_OF_ROLL_DETECTION_POS,
                    Direction::Left => {
                        last_ctr.start_x <= cropped_width * END_OF_ROLL_DETECTION_POS
                    }
                }
            } else {
                false
            }
        } else {
            false
        };

        match state.current {
            ScannerState::Error {
                ref reason,
                ..
            } => match reason {
                ErrorReason::EndOfRoll => {
                    if !end_of_roll {
                        state.set(ScannerState::default());
                    }
                }
                ErrorReason::SuspiciousStartGap => {
                    pause = true;
                }
            },
            ScannerState::TogglingFocus {
                init_timestamp,
                step,
            } => {
                if step == 0 {
                    info!("Toggling camera focus...");
                    scanner.focus();
                    state.set(ScannerState::TogglingFocus {
                        init_timestamp: Instant::now(),
                        step: 1,
                    });
                } else if step == 1 {
                    // Wait until enough time has passed
                    if Instant::now() < init_timestamp + Duration::from_millis(200) {
                        continue;
                    }

                    scanner.stop_focus();
                    state.set(ScannerState::TogglingFocus {
                        init_timestamp: Instant::now(),
                        step: 2,
                    });
                } else if step == 2 {
                    // Wait until UI is hidden after focusing
                    if Instant::now() < init_timestamp + Duration::from_millis(600) {
                        continue;
                    }
                    state.set(ScannerState::default());
                }
            }
            ScannerState::AligningFrame {
                wait_until,
                last_seen_dist,
            } => {
                match dist_to_edge_gap_end {
                    Some(dist_to_edge_gap_end) => {
                        let dist_from_start = cropped_width - dist_to_edge_gap_end;

                        let detection_pos_dist = (1.0 - DETECTION_POS) * cropped_width;
                        let moved_past_detection_point = dist_to_edge_gap_end < detection_pos_dist;

                        // Immediately stop film for more accurate measurements if frame just moved past
                        // detection point
                        if let Some(last_seen_dist) = last_seen_dist {
                            let just_moved_past_detection_point =
                                last_seen_dist >= detection_pos_dist && moved_past_detection_point;

                            if just_moved_past_detection_point {
                                debug!("Stopping film for more accurate position measurement");
                                audio.play_sound("film_feed.wav");
                                scanner.stop(true);
                                state.set(ScannerState::AligningFrame {
                                    wait_until: Instant::now()
                                        + Duration::from_millis(INPUT_LAG_MSEC)
                                        + Duration::from_millis(PRE_FEED_NEW_FILM_WAIT)
                                        + Duration::from_millis(STEPPER_ACCEL_TIME),
                                    last_seen_dist: Some(dist_to_edge_gap_end),
                                });

                                continue;
                            }
                        }

                        // Wait some time after previous move command
                        if Instant::now() < wait_until {
                            continue;
                        }

                        // Frame gap found in last_ctr, move film forward
                        // towards end of gap
                        let steps = dist_to_edge_gap_end * PX_PER_STEP;
                        let steps = steps.round(); //.max(5.0);
                        let steps = steps + EXTRA_ALIGNMENT_STEPS as f64;
                        info!(
                            "{} px to end of frame gap, moving motor {} steps",
                            dist_to_edge_gap_end, steps
                        );

                        let slow_mode = dist_from_start / cropped_width >= FEED_SLOW_POS
                            && dist_from_start / cropped_width < DETECTION_POS;
                        let speed = if slow_mode { Some(500) } else { None };

                        match FILM_MOVE_DIR {
                            Direction::Right => {
                                scanner.move_right(steps as usize, speed);
                            }
                            Direction::Left => {
                                scanner.move_left(steps as usize, speed);
                            }
                        }

                        if dist_to_edge_gap_end < cropped_width / 2.0 {
                            audio.play_sound("film_advance.wav");
                        }

                        let step_time = MSEC_PER_1000_STEPS as f64 / 1000.0 * steps;
                        let wait_time = Duration::from_millis(INPUT_LAG_MSEC)
                            + Duration::from_millis(STEPPER_ACCEL_TIME)
                            + Duration::from_millis(STEPPER_ACCEL_TIME)
                            + Duration::from_millis(PRE_DETECTION_WAIT_MSEC)
                            + Duration::from_millis(step_time as u64);

                        // Limit wait time to 1 sec
                        // let wait_time = wait_time.min(Duration::from_secs(1));

                        state.set(ScannerState::AligningFrame {
                            wait_until: Instant::now() + wait_time,
                            last_seen_dist: Some(dist_to_edge_gap_end),
                        })
                    }
                    None => {
                        // Wait some time after previous move command
                        if Instant::now() < wait_until {
                            continue;
                        }

                        if end_of_roll {
                            scanner.stop(true);
                            debug!("End of roll detected");
                            audio.play_sound("end_of_roll.wav");

                            state.set(ScannerState::Error {
                                init_timestamp: Instant::now(),
                                reason: ErrorReason::EndOfRoll,
                            });
                            continue;
                        }
                        if gap_at_start {
                            scanner.stop(true);
                            debug!(
                                "Suspicious gap at start of frame, stopping for manual controls"
                            );
                            audio.play_sound("end_of_roll.wav");

                            state.set(ScannerState::Error {
                                init_timestamp: Instant::now(),
                                reason: ErrorReason::SuspiciousStartGap,
                            });
                            continue;
                        }

                        // Could not find frame gaps, assume frame is
                        // aligned and take photo

                        info!("No frame gaps in feed, taking photo");
                        // scanner.stop(true);
                        // let wait_time = Duration::from_millis(INPUT_LAG_MSEC)
                        //     + Duration::from_millis(PRE_SHUTTER_WAIT_MSEC)
                        //     + Duration::from_millis(STEPPER_ACCEL_TIME);

                        state.set(ScannerState::TakingPhoto {
                            wait_until: Instant::now(), // + wait_time,
                        })
                    }
                }
            }
            ScannerState::TakingPhoto { wait_until } => {
                // Wait until enough time has passed so that the film has come to a stop
                if Instant::now() < wait_until {
                    continue;
                }

                audio.play_sound("shutter.wav");
                if DRY_RUN {
                    info!("DRY_RUN enabled: would take photo");
                } else {
                    scanner.take_photo();
                    scanner.stop_focus();
                }

                state.set(ScannerState::WaitingForShutterClose);
            }
            ScannerState::WaitingForShutterClose => {
                // Crude detection of when shutter black-out starts - wait
                // until we see less than 2000 non zero pixels (Fujifilm
                // X-T200 UI draws around 1000 non zero pixels during
                // shutter black-out)
                let non_zero_px = core::count_non_zero(&frame_bw)?;
                // TODO: magic numbers
                if non_zero_px <= 2000 || DRY_RUN {
                    state.set(ScannerState::WaitingForShutterOpen);
                } else {
                    info!("Waiting for start of shutter black-out")
                }
            }
            ScannerState::WaitingForShutterOpen => {
                // Crude detection of when shutter black-out ends - wait
                // until we see more than 2000 non zero pixels (Fujifilm
                // X-T200 UI draws around 1000 non zero pixels during
                // shutter black-out)
                let non_zero_px = core::count_non_zero(&frame_bw)?;
                // TODO: magic numbers
                if non_zero_px > 2000 {
                    info!("Moving film to next frame");

                    scanner.focus();
                    scanner.stop_focus();

                    // Move enough forward so that we start detecting the next frame
                    match FILM_MOVE_DIR {
                        Direction::Right => {
                            scanner.move_right(NEXT_FRAME_SKIP_STEPS, None);
                        }
                        Direction::Left => {
                            scanner.move_left(NEXT_FRAME_SKIP_STEPS, None);
                        }
                    }

                    let step_time =
                        MSEC_PER_1000_STEPS as f64 / 1000.0 * NEXT_FRAME_SKIP_STEPS as f64;

                    let wait_time = Duration::from_millis(INPUT_LAG_MSEC)
                        + Duration::from_millis(STEPPER_ACCEL_TIME)
                        + Duration::from_millis(PRE_DETECTION_WAIT_MSEC)
                        + Duration::from_millis(step_time as u64);

                    state.set(ScannerState::SkipToNextFrame {
                        wait_until: Instant::now() + wait_time,
                    });
                } else {
                    info!("Waiting for end of shutter black-out")
                }
            }

            ScannerState::SkipToNextFrame { wait_until } => {
                // Wait until enough time has passed so that the film has moved far enough
                if Instant::now() < wait_until {
                    continue;
                }

                state.set(ScannerState::default());
            }
        }
    }

    Ok(())
}
