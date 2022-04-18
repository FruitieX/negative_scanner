use std::time::{Duration, Instant};

use opencv::{
    core::{self, Point, Scalar},
    highgui, imgproc,
    prelude::*,
    types::VectorOfMat,
    videoio, Result,
};
use serialport::SerialPort;

const CONTROLS_WINDOW: &str = "controls";
const MANUAL_MODE: bool = false;

/// How many pixels one step corresponds to
const PX_PER_STEP: f64 = 4.5;

/// Extra time to wait before taking photo. Helps avoids vibrations during exposure.
const PRE_SHUTTER_WAIT_MSEC: u64 = 300;

/// Account for camera video feed -> opencv input lag
const INPUT_LAG_MSEC: u64 = 300;

/// How many milliseconds it takes to move the film 1000 steps
const MSEC_PER_1000_STEPS: u64 = 700;

/// Time it takes for stepper motor to (de)accelerate between zero and max speed
/// (maxSpeed / acceleration)
const STEPPER_ACCEL_TIME: u64 = 120;

/// Account for camera not responding to shutter button press instantly
const SHUTTER_EXTRA_WAIT_MSEC: u64 = 200;

/// How many steps to move the film forward after taking a photo
const NEXT_FRAME_SKIP_STEPS: usize = 2500;

///
const NEW_DETECTION_POS: f64 = 150.0;

#[derive(Debug)]
struct State {
    current: ScannerState,
}

#[derive(Debug)]
enum ScannerState {
    AligningFrame {
        wait_until: Instant,
        last_seen_x: Option<f64>,
    },
    TakingPhoto {
        wait_until: Instant,
    },
    WaitingForShutterOpen {
        wait_until: Instant,
    },
    SkipToNextFrame {
        wait_until: Instant,
    },
}

impl Default for ScannerState {
    fn default() -> Self {
        ScannerState::AligningFrame {
            wait_until: Instant::now(),
            last_seen_x: None,
        }
    }
}

impl State {
    pub fn new() -> State {
        println!("Searching for frames...");
        State {
            current: ScannerState::default(),
        }
    }

    pub fn set(&mut self, new: ScannerState) {
        // println!("Entering state: {:?}", new);
        self.current = new;
    }
}

struct Contour {
    area: f64,
    end_x: f64,
}

struct Scanner {
    tx: std::sync::mpsc::Sender<String>,
    prev_dir: bool,
}

impl Scanner {
    pub fn new(mut port: Box<dyn SerialPort>) -> Scanner {
        let (tx, rx) = std::sync::mpsc::channel::<String>();

        std::thread::spawn(move || {
            loop {
                let msg = rx.recv();

                if let Ok(msg) = msg {
                    // println!("Writing to serial: {}", msg);
                    let bytes: &[u8] = msg.as_bytes();
                    // println!("Bytes: {:?}", bytes);
                    port.write_all(bytes)
                        .expect("Writing to serial port failed");
                }
            }
        });

        Scanner {
            tx,
            prev_dir: false,
        }
    }

    pub fn move_forward(&mut self, steps: usize, _silent: bool) {
        // 1300 ~= one frame
        self.write(&format!("-{}M", steps));
        self.prev_dir = true;
    }

    pub fn move_back(&mut self, steps: usize) {
        self.write(&format!("{}M", steps));
        self.prev_dir = false;
    }

    pub fn stop(&mut self, silent: bool) {
        if !silent {
            println!("Immediately stopping motor");
        }

        self.write("0M");

        if self.prev_dir {
            self.write("50M");
        } else {
            self.write("-50M");
        }
    }

    pub fn take_photo(&mut self) {
        println!("Taking photo");

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
    let port_infos = serialport::available_ports().expect("No serial ports found");
    let port_info = port_infos.get(1).expect("No serial ports found");
    let port = serialport::new(port_info.port_name.clone(), 115_200)
        .timeout(Duration::from_secs(1))
        .open()
        .expect("Failed to open serial port");

    let mut pause = true;
    println!("Starting in paused/manual mode. Hit spacebar to start automation.");
    println!("Manual mode controls:");
    println!("r = Reset to initial state");
    println!("q = Quit");
    println!("e = Move film forward");
    println!("a = Move film back");
    println!("s = Shutter");
    println!("f = Focus");
    println!("n = Next frame");
    println!("u = Stop focus");

    let mut scanner = Scanner::new(port);

    println!("Toggling camera focus...");
    scanner.focus();
    std::thread::sleep(Duration::from_millis(500));
    scanner.stop_focus();

    let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)?; // 0 is the default camera

    if !videoio::VideoCapture::is_opened(&cam)? {
        panic!("Unable to open default camera");
    }

    // let mut vignette = opencv::imgcodecs::imread("vignette.png", IMREAD_COLOR)?;

    let mut frame = Mat::default();

    // Capture next frame
    cam.read(&mut frame)?;

    let frame_width = frame.cols();
    let frame_height = frame.rows();

    highgui::named_window(CONTROLS_WINDOW, 1)?;

    highgui::create_trackbar("x", CONTROLS_WINDOW, &mut 52, frame_width - 1, None)?;
    highgui::create_trackbar("y", CONTROLS_WINDOW, &mut 0, frame_height - 1, None)?;
    highgui::create_trackbar(
        "width",
        CONTROLS_WINDOW,
        &mut (frame_width - 52 - 52),
        frame_width,
        None,
    )?;
    highgui::create_trackbar(
        "height",
        CONTROLS_WINDOW,
        &mut frame_height.clone(),
        frame_height,
        None,
    )?;
    highgui::create_trackbar("param", CONTROLS_WINDOW, &mut 0, 255, None)?;

    let mut state = State::new();

    loop {
        // Capture next frame
        cam.read(&mut frame)?;

        // Read control window parameters
        let x = highgui::get_trackbar_pos("x", CONTROLS_WINDOW)?;
        let y = highgui::get_trackbar_pos("y", CONTROLS_WINDOW)?;
        let width = highgui::get_trackbar_pos("width", CONTROLS_WINDOW)?;
        let height = highgui::get_trackbar_pos("height", CONTROLS_WINDOW)?;
        // let param = highgui::get_trackbar_pos("param", CONTROLS_WINDOW)?;

        // Don't allow cropping outside image (crashes opencv)
        let x = x.min(frame_width - width);
        let y = y.min(frame_height - height);

        // Crop image
        let rect = core::Rect::new(x, y, width, height);
        let frame_cropped = Mat::roi(&frame, rect)?;

        // Apply median blur to get rid of small scratches / noise
        let mut frame_blurred = Mat::default();
        imgproc::median_blur(&frame_cropped, &mut frame_blurred, 5 * 2 + 1)?;
        // highgui::imshow("frame_blurred", &frame_blurred)?;

        // Convert frame to grayscale
        let mut frame_bw = Mat::default();
        imgproc::cvt_color(&frame_blurred, &mut frame_bw, imgproc::COLOR_RGB2GRAY, 0)?;
        // highgui::imshow("frame_bw", &frame_bw)?;

        // Try finding a threshold value that results in at least one column
        // full of white pixels
        let mut test_thr = 255.0;
        let mut frame_bw_thr = Mat::default();
        let mut found_full_col = false;
        while test_thr > 150.0 {
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

            test_thr -= 5.0;
        }

        if found_full_col {
            // Perform thresholding once more with slightly smaller thresh value
            imgproc::threshold(
                &frame_bw,
                &mut frame_bw_thr,
                test_thr - 10.0,
                255.0,
                imgproc::THRESH_BINARY,
            )?;
        }

        // Mask out columns with pixels below the threshold
        // highgui::imshow("pre_mask", &frame_bw_thr)?;
        let frame_bw_thr = mask_non_full_cols(&frame_bw_thr)?;
        // highgui::imshow("frame_bw_thr", &frame_bw_thr)?;

        let anchor = Point::new(-1, -1);

        // Perform adaptive thresholding on the grayscale image to mask out any
        // parts with sharp edges.
        let mut adaptive_thr = Mat::default();
        imgproc::adaptive_threshold(
            &frame_bw,
            &mut adaptive_thr,
            255.0,
            imgproc::ADAPTIVE_THRESH_GAUSSIAN_C,
            imgproc::THRESH_BINARY,
            7 * 2 + 1,
            2.into(),
        )?;

        // Perform noise reduction on the adaptive thresholding result
        let sizes = vec![2, 2].into_iter().collect();
        let kernel = Mat::new_nd_vec_with_default(&sizes, 0, Scalar::from(255.0))?;
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
        // highgui::imshow("pre_mask_adaptive", &adaptive_thr)?;

        // Mask out columns with pixels below the threshold
        let adaptive_thr = mask_non_full_cols(&adaptive_thr)?;
        // highgui::imshow("adaptive_thr", &adaptive_thr)?;

        // Assume that gaps between negative frames exist in areas where both
        // methods of thresholding passed.
        let mut gaps = Mat::default();
        core::bitwise_and(&adaptive_thr, &frame_bw_thr, &mut gaps, &Mat::default())?;
        highgui::imshow("gaps", &gaps)?;

        let mut contours = VectorOfMat::new();
        imgproc::find_contours(
            &gaps,
            &mut contours,
            imgproc::RETR_TREE,
            imgproc::CHAIN_APPROX_SIMPLE,
            Point::new(0, 0),
        )?;

        imgproc::draw_contours(
            &mut frame,
            &contours,
            -1,
            Scalar::new(0.0, 255.0, 0.0, 255.0),
            3,
            imgproc::LINE_8,
            &core::no_array()?,
            i32::MAX,
            Point::new(
                x,
                y + ((height as f64 / 2.0 - frame_bw_thr.rows() as f64 / 2.0) as i32),
            ),
        )?;

        let key: char = highgui::poll_key()? as u8 as char;

        if key == 'q' {
            break;
        }

        let mut cwms: Vec<Contour> = contours
            .iter()
            .map(|contour| {
                let area = imgproc::contour_area(&contour, false).unwrap();

                let bbox = imgproc::bounding_rect(&contour).unwrap();
                let seen_width = bbox.width as f64;
                let seen_end = bbox.x as f64 + seen_width;

                Contour {
                    area,
                    end_x: seen_end,
                }
            })
            // .filter(|moments| moments.area > 100_000.0)
            .collect();

        cwms.sort_by(|a, b| a.area.partial_cmp(&b.area).unwrap());
        let last_cwm = cwms.iter().last();

        // cwms.sort_by(|a, b| a.end_x.partial_cmp(&b.end_x).unwrap());
        // let last_cwm = cwms.first();

        // dbg!(last_cwm.map(|cwm| cwm.end_x));

        let mut frame_inv = Mat::default();
        core::bitwise_not(&frame, &mut frame_inv, &core::no_array()?)?;
        highgui::imshow("frame", &frame_inv)?;

        // Move film forward
        if key == 'e' {
            scanner.move_forward(60, false);
        }
        // FF forward
        if key == '.' {
            scanner.move_forward(100, false);
        }

        // Move film back
        if key == 'a' {
            scanner.move_back(60);
        }
        // FF back
        if key == '\'' {
            scanner.move_back(100);
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
            scanner.move_forward(NEXT_FRAME_SKIP_STEPS, false);
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
                println!("Paused. Hit spacebar to unpause.");
                scanner.stop(true);
            } else {
                println!("Unpaused. Hit spacebar to pause.");
            }
        }

        if !pause && !MANUAL_MODE {
            match state.current {
                ScannerState::AligningFrame {
                    wait_until,
                    last_seen_x,
                } => {
                    match last_cwm {
                        Some(last_cwm) => {
                            // Always update last seen x position to state
                            state.set(ScannerState::AligningFrame {
                                wait_until,
                                last_seen_x: Some(last_cwm.end_x),
                            });

                            // Immediately stop film if frame moves past
                            // detection point
                            if let Some(last_seen_x) = last_seen_x {
                                let moved_past_detection_point = last_seen_x >= NEW_DETECTION_POS
                                    && last_cwm.end_x < NEW_DETECTION_POS;

                                if moved_past_detection_point {
                                    scanner.stop(true);
                                    state.set(ScannerState::AligningFrame {
                                        wait_until: Instant::now()
                                            + Duration::from_millis(INPUT_LAG_MSEC)
                                            + Duration::from_millis(STEPPER_ACCEL_TIME),
                                        last_seen_x: Some(last_cwm.end_x),
                                    });

                                    continue;
                                }
                            }

                            // Wait some time after previous move command
                            if Instant::now() < wait_until {
                                continue;
                            }

                            // Frame gap found in last_cwm, move film forward
                            // towards end of gap
                            let steps = last_cwm.end_x * PX_PER_STEP;
                            let steps = steps.round(); //.max(5.0);
                            let steps = steps + 50.0;
                            println!(
                                "{} px to end of frame gap, moving motor {} steps",
                                last_cwm.end_x, steps
                            );
                            scanner.move_forward(steps as usize, true);

                            let step_time = MSEC_PER_1000_STEPS as f64 / 1000.0 * steps;
                            let wait_time = Duration::from_millis(INPUT_LAG_MSEC)
                                + Duration::from_millis(STEPPER_ACCEL_TIME)
                                + Duration::from_millis(STEPPER_ACCEL_TIME)
                                + Duration::from_millis(step_time as u64);

                            // Limit wait time to 1 sec
                            let wait_time = wait_time.min(Duration::from_secs(1));

                            state.set(ScannerState::AligningFrame {
                                wait_until: Instant::now() + wait_time,
                                last_seen_x: Some(last_cwm.end_x),
                            })
                        }
                        None => {
                            // Wait some time after previous move command
                            if Instant::now() < wait_until {
                                continue;
                            }

                            // Could not find frame gaps, assume frame is
                            // aligned and take photo

                            println!("No frame gaps in feed, stopping motor and taking photo");
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

                    let wait_time = Duration::from_millis(INPUT_LAG_MSEC)
                        + Duration::from_millis(SHUTTER_EXTRA_WAIT_MSEC);

                    state.set(ScannerState::WaitingForShutterOpen {
                        wait_until: Instant::now() + wait_time,
                    });
                }
                ScannerState::WaitingForShutterOpen { wait_until } => {
                    // Wait until enough time has passed so that we can see the camera shutter black-out
                    if Instant::now() < wait_until {
                        continue;
                    }

                    // Crude detection of when shutter black-out ends - wait
                    // until we see more than 2000 non zero pixels (Fujifilm
                    // X-T200 UI draws around 1000 non zero pixels during
                    // shutter black-out)
                    let non_zero_px = core::count_non_zero(&frame_bw)?;
                    if non_zero_px > 2000 {
                        println!("Moving film to next frame");

                        // Move enough forward so that we start detecting the next frame
                        scanner.move_forward(NEXT_FRAME_SKIP_STEPS, false);

                        let step_time =
                            MSEC_PER_1000_STEPS as f64 / 1000.0 * NEXT_FRAME_SKIP_STEPS as f64;

                        let wait_time = Duration::from_millis(INPUT_LAG_MSEC)
                            + Duration::from_millis(STEPPER_ACCEL_TIME)
                            + Duration::from_millis(step_time as u64);

                        state.set(ScannerState::SkipToNextFrame {
                            wait_until: Instant::now() + wait_time,
                        });
                    } else {
                        println!("Waiting for end of shutter black-out")
                    }
                }

                ScannerState::SkipToNextFrame { wait_until } => {
                    // scanner.focus();
                    // Wait until enough time has passed so that the film has moved far enough
                    if Instant::now() < wait_until {
                        continue;
                    }

                    // scanner.focus();
                    state.set(ScannerState::default());
                }
            }
        }
    }

    Ok(())
}
