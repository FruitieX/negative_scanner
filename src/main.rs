use log::{debug, error, info, log_enabled, trace, warn, Level};
use opencv::{
    core::{self, Point, Scalar},
    highgui, imgproc,
    prelude::*,
    types::VectorOfMat,
    videoio, Result,
};
use serialport::SerialPort;
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
const DETECTION_VERTICAL_CROP: f64 = 0.95;

/// How many pixels one step corresponds to
const PX_PER_STEP: f64 = 1.85;

/// How many steps are lost when transitioning from
/// forward movement to backward movement due to slop
/// between the gear and film sprockets.
const SLOP_STEPS: usize = 20;

/// Always step this many extra steps when aligning frame
const EXTRA_ALIGNMENT_STEPS: usize = 10;

/// Extra time to wait before taking photo. Helps avoids vibrations during
/// exposure.
const PRE_SHUTTER_WAIT_MSEC: u64 = 100;

const PRE_FEED_NEW_FILM_WAIT: u64 = 500;

/// Account for camera video feed -> opencv input lag
const INPUT_LAG_MSEC: u64 = 300;

/// How many milliseconds it takes to move the film 1000 steps
const MSEC_PER_1000_STEPS: u64 = 700;

/// Time it takes for stepper motor to (de)accelerate between zero and max
/// speed (maxSpeed / acceleration)
const STEPPER_ACCEL_TIME: u64 = 120;

/// How many steps to move the film forward after taking a photo
const NEXT_FRAME_SKIP_STEPS: usize = 975;

/// Stop film for more accurate position measurements when frame moves past this
/// point (in percentage of width)
const NEW_DETECTION_POS: f64 = 0.55;

#[derive(Debug)]
struct State {
    current: ScannerState,
}

#[derive(Debug)]
enum ScannerState {
    TogglingFocus {
        init_timestamp: Instant,
        step: usize,
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
        debug!("Entering state: {:?}", new);
        self.current = new;
    }
}

struct Contour {
    contour: Mat,
    area: f64,
    end_x: f64,
    start_x: f64,
}

struct Scanner {
    tx: std::sync::mpsc::Sender<String>,
    prev_dir: Option<Direction>,
}

impl Scanner {
    pub fn new(mut port: Box<dyn SerialPort>) -> Scanner {
        let (tx, rx) = std::sync::mpsc::channel::<String>();

        std::thread::spawn(move || loop {
            let msg = rx.recv();

            if let Ok(msg) = msg {
                debug!("Writing to serial: {}", msg);
                let bytes: &[u8] = msg.as_bytes();
                trace!("Bytes: {:?}", bytes);
                port.write_all(bytes)
                    .expect("Writing to serial port failed");
            }
        });

        Scanner { tx, prev_dir: None }
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
    }

    pub fn move_left(&mut self, mut steps: usize, speed: Option<usize>) {
        if self.prev_dir == Some(Direction::Right) {
            steps += SLOP_STEPS;
        }
        self.set_speed(speed);
        self.write(&format!("-{}M", steps));
        self.prev_dir = Some(Direction::Left);
    }

    pub fn stop(&mut self, silent: bool) {
        if !silent {
            info!("Immediately stopping motor");
        }

        self.write("0M");

        match self.prev_dir {
            Some(Direction::Right) => {
                self.move_left(0, None);
            }
            Some(Direction::Left) => {
                self.move_right(0, None);
            }
            _ => {}
        }
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

/// Checks whether frame contains any columns with all white pixels
fn has_full_cols(mat: &Mat) -> Result<bool> {
    // Try finding vertical line that spans from top to bottom:
    let vec2d: Vec<Vec<u8>> = mat.to_vec_2d()?;
    let width = mat.cols() as usize;

    // Iterate over columns
    for x in 0..width {
        // Look for white pixels on each row
        let full_col = vec2d.iter().all(|row| row[x] > 0);

        if full_col {
            return Ok(true);
        }
    }

    Ok(false)
}

/// Masks out any columns that don't contain all white pixels
fn mask_non_full_cols(mat: &Mat) -> Result<Mat> {
    // Try finding vertical line that spans from top to bottom:
    let mut vec2d: Vec<Vec<u8>> = mat.to_vec_2d()?;
    let width = mat.cols() as usize;

    // Iterate over columns
    for x in 0..width {
        // Look for white pixels on each row
        let full_col = vec2d.iter().all(|row| row[x] > 0);

        if !full_col {
            vec2d.iter_mut().for_each(|row| row[x] = 0);
        }
    }

    Mat::from_slice_2d(&vec2d)
}

fn main() -> Result<()> {
    env_logger::init();

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

    loop {
        // Capture next frame
        cam.read(&mut frame)?;

        let cropped_width = frame_width * DETECTION_HORIZONTAL_CROP;
        let cropped_x = (frame_width - cropped_width) / 2.0;
        let cropped_height = frame_height * DETECTION_VERTICAL_CROP;
        let cropped_y = (frame_height - cropped_height) / 2.0;

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
        imgproc::median_blur(&frame_cropped, &mut frame_blurred, 7 * 2 + 1)?;
        // highgui::imshow("frame_blurred", &frame_blurred)?;

        // Convert frame to grayscale
        let mut frame_bw = Mat::default();
        imgproc::cvt_color(&frame_blurred, &mut frame_bw, imgproc::COLOR_RGB2GRAY, 0)?;
        highgui::imshow("frame_bw", &frame_bw)?;

        // Try finding a threshold value that results in at least one column
        // full of white pixels
        let mut test_thr = 255.0;
        let mut frame_bw_thr = Mat::default();
        let mut found_full_col = false;
        // TODO: magic numbers
        while test_thr > 120.0 {
            imgproc::threshold(
                &frame_bw,
                &mut frame_bw_thr,
                test_thr,
                255.0,
                imgproc::THRESH_BINARY,
            )?;

            if has_full_cols(&frame_bw_thr)? {
                found_full_col = true;
                break;
            }

            test_thr -= 20.0;
        }

        if found_full_col {
            // Perform thresholding once more with slightly smaller thresh value (the goal is to detect
            // the entire frame gap rather than just the possibly one pixel wide gap)
            imgproc::threshold(
                &frame_bw,
                &mut frame_bw_thr,
                // TODO: magic numbers
                test_thr - 10.0,
                255.0,
                imgproc::THRESH_BINARY,
            )?;
        }

        // Mask out columns with pixels below the threshold
        highgui::imshow("pre_mask", &frame_bw_thr)?;
        let frame_bw_thr = mask_non_full_cols(&frame_bw_thr)?;
        highgui::imshow("frame_bw_thr", &frame_bw_thr)?;

        // Perform adaptive thresholding on the grayscale image to mask out any
        // parts with sharp edges.
        let mut adaptive_thr = Mat::default();
        imgproc::adaptive_threshold(
            &frame_bw,
            &mut adaptive_thr,
            255.0,
            imgproc::ADAPTIVE_THRESH_GAUSSIAN_C,
            imgproc::THRESH_BINARY,
            11 * 2 + 1,
            2.into(),
        )?;

        // Perform noise reduction on the adaptive thresholding result
        let sizes = vec![3, 3].into_iter().collect();
        let kernel = Mat::new_nd_vec_with_default(&sizes, 0, Scalar::from(255.0))?;
        let anchor = Point::new(-1, -1);
        imgproc::morphology_ex(
            &adaptive_thr.clone(),
            &mut adaptive_thr,
            imgproc::MORPH_CLOSE,
            &kernel,
            anchor,
            1,
            opencv::core::BORDER_CONSTANT,
            imgproc::morphology_default_border_value()?,
        )?;
        highgui::imshow("pre_mask_adaptive", &adaptive_thr)?;

        // Mask out columns with pixels below the threshold
        let adaptive_thr = mask_non_full_cols(&adaptive_thr)?;
        // highgui::imshow("adaptive_thr", &adaptive_thr)?;

        // Assume that gaps between negative frames exist in areas where both
        // methods of thresholding passed.
        let mut gaps = Mat::default();
        core::bitwise_and(&adaptive_thr, &frame_bw_thr, &mut gaps, &Mat::default())?;
        highgui::imshow("gaps", &gaps)?;

        // Find contours of gaps mask
        let mut contours = VectorOfMat::new();
        imgproc::find_contours(
            &gaps,
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
            .filter(|ctr| {
                // TODO: magic numbers
                (ctr.start_x <= 10.0 || ctr.end_x >= cropped_width as f64 - 10.0)
                    || ctr.area > 5000.0
            })
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
            Point::new(
                cropped_x as i32,
                cropped_y as i32
                    + ((cropped_height / 2.0 - frame_bw_thr.rows() as f64 / 2.0) as i32),
            ),
        )?;

        ctrs.sort_by(|a, b| a.area.partial_cmp(&b.area).unwrap());
        let largest_ctr = ctrs.iter().last();

        // ctrs.sort_by(|a, b| a.end_x.partial_cmp(&b.end_x).unwrap());
        // let last_ctr = ctrs.first();

        let mut frame_inv = Mat::default();
        core::bitwise_not(&frame, &mut frame_inv, &core::no_array())?;
        highgui::imshow("frame", &frame_inv)?;

        // Read input from user
        let key: char = highgui::poll_key()? as u8 as char;

        if key == 'q' {
            break;
        }

        // Move film right
        if key == 'e' {
            scanner.move_right(5, Some(200));
        }
        // FF right
        if key == '.' {
            scanner.move_right(100, None);
        }

        // Move film left
        if key == 'a' {
            scanner.move_left(5, Some(200));
        }
        // FF left
        if key == '\'' {
            scanner.move_left(100, None);
        }

        // Shutter
        if key == 's' {
            scanner.take_photo();
        }

        // Focus
        if key == 'f' {
            scanner.focus();
        }

        // Stop focus
        if key == 'u' {
            scanner.stop_focus();
        }

        if key == 'n' {
            scanner.move_right(NEXT_FRAME_SKIP_STEPS, None);
        }

        // Reset
        if key == 'r' {
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

        if !pause {
            match state.current {
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
                    match largest_ctr {
                        Some(largest_ctr) => {
                            // How many pixels film still needs to move according to detection
                            let dist_to_gap_end = match FILM_MOVE_DIR {
                                Direction::Right => cropped_width - largest_ctr.start_x,
                                Direction::Left => largest_ctr.end_x,
                            };
                            let area = largest_ctr.area;

                            // Ignore large delta changes for small areas (probably noise)
                            let small_area = area < 5000.0;

                            // Don't allow last_seen_dist to change by over some large delta
                            let large_delta = if let Some(last_seen_dist) = last_seen_dist {
                                let delta = (last_seen_dist - dist_to_gap_end).abs();
                                if delta < 100.0 {
                                    state.set(ScannerState::AligningFrame {
                                        wait_until,
                                        last_seen_dist: Some(dist_to_gap_end),
                                    });

                                    false
                                } else {
                                    if !small_area {
                                        warn!(
                                            "dist_to_gap_end changed by {}px in one frame, stopping (area: {})",
                                            delta, area
                                        );
                                    } else {
                                        info!(
                                            "dist_to_gap_end changed by {}px in one frame, not stopping due to small area (area: {})",
                                            delta, area
                                        );
                                    }

                                    true
                                }
                            } else {
                                // Always update last seen x position to state
                                state.set(ScannerState::AligningFrame {
                                    wait_until,
                                    last_seen_dist: Some(dist_to_gap_end),
                                });

                                false
                            };

                            if large_delta && !small_area {
                                scanner.stop(true);
                                pause = true;
                            }

                            if large_delta {
                                warn!("waiting if large delta goes away...");
                                continue;
                            }

                            let detection_pos_dist = (1.0 - NEW_DETECTION_POS) * cropped_width;
                            let moved_past_detection_point = dist_to_gap_end < detection_pos_dist;

                            // Immediately stop film for more accurate measurements if frame just moved past
                            // detection point
                            if let Some(last_seen_dist) = last_seen_dist {
                                let just_moved_past_detection_point = last_seen_dist
                                    >= detection_pos_dist
                                    && moved_past_detection_point;

                                if just_moved_past_detection_point {
                                    scanner.stop(true);
                                    state.set(ScannerState::AligningFrame {
                                        wait_until: Instant::now()
                                            + Duration::from_millis(INPUT_LAG_MSEC)
                                            + Duration::from_millis(PRE_FEED_NEW_FILM_WAIT)
                                            + Duration::from_millis(STEPPER_ACCEL_TIME),
                                        last_seen_dist: Some(dist_to_gap_end),
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
                            let steps = dist_to_gap_end * PX_PER_STEP;
                            let steps = steps.round(); //.max(5.0);
                            let steps = steps + EXTRA_ALIGNMENT_STEPS as f64;
                            info!(
                                "{} px to end of frame gap, moving motor {} steps",
                                dist_to_gap_end, steps
                            );

                            let speed = if moved_past_detection_point {
                                None
                            } else {
                                Some(100)
                            };

                            match FILM_MOVE_DIR {
                                Direction::Right => {
                                    scanner.move_right(steps as usize, speed);
                                }
                                Direction::Left => {
                                    scanner.move_left(steps as usize, speed);
                                }
                            }

                            let step_time = MSEC_PER_1000_STEPS as f64 / 1000.0 * steps;
                            let wait_time = Duration::from_millis(INPUT_LAG_MSEC)
                                + Duration::from_millis(STEPPER_ACCEL_TIME)
                                + Duration::from_millis(STEPPER_ACCEL_TIME)
                                + Duration::from_millis(step_time as u64);

                            // Limit wait time to 1 sec
                            let wait_time = wait_time.min(Duration::from_secs(1));

                            state.set(ScannerState::AligningFrame {
                                wait_until: Instant::now() + wait_time,
                                last_seen_dist: Some(dist_to_gap_end),
                            })
                        }
                        None => {
                            // Wait some time after previous move command
                            if Instant::now() < wait_until {
                                continue;
                            }

                            // Could not find frame gaps, assume frame is
                            // aligned and take photo

                            info!("No frame gaps in feed, stopping motor and taking photo");
                            scanner.stop(true);
                            let wait_time = Duration::from_millis(INPUT_LAG_MSEC)
                                + Duration::from_millis(PRE_SHUTTER_WAIT_MSEC)
                                + Duration::from_millis(STEPPER_ACCEL_TIME);

                            state.set(ScannerState::TakingPhoto {
                                wait_until: Instant::now() + wait_time,
                            })
                        }
                    }
                }
                ScannerState::TakingPhoto { wait_until } => {
                    // Wait until enough time has passed so that the film has come to a stop
                    if Instant::now() < wait_until {
                        continue;
                    }

                    scanner.take_photo();
                    scanner.stop_focus();

                    state.set(ScannerState::WaitingForShutterClose);
                }
                ScannerState::WaitingForShutterClose => {
                    // Crude detection of when shutter black-out starts - wait
                    // until we see less than 2000 non zero pixels (Fujifilm
                    // X-T200 UI draws around 1000 non zero pixels during
                    // shutter black-out)
                    let non_zero_px = core::count_non_zero(&frame_bw)?;
                    // TODO: magic numbers
                    if non_zero_px <= 2000 {
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
    }

    Ok(())
}
