use std::time::{Duration, SystemTime};

use opencv::{
    core::{self, Moments, Point, Scalar},
    highgui, imgproc,
    prelude::*,
    types::VectorOfMat,
    videoio, Result,
};
use serialport::SerialPort;

const CONTROLS_WINDOW: &str = "controls";
const MANUAL_MODE: bool = false;
const CENTER_TOLERANCE: f64 = 10.0;
const CENTER_TOLERANCE_MISSED: f64 = 8.0;
const PX_PER_STEP: f64 = 4.5;
const MOP_RECT_HEIGHT_PCT: f64 = 0.97;
// const NEXT_FRAME_SKIP_STEPS: usize = 2825;
const NEXT_FRAME_SKIP_STEPS: usize = 2670;

struct State {
    current: ScannerState,
}

#[derive(Debug)]
enum ScannerState {
    SearchingFrame,
    FoundFrame {
        init_timestamp: SystemTime,
        wait_time: f64,
        consecutive_missed_frames: usize,
    },
    TakingPhoto {
        init_timestamp: SystemTime,
    },
    WaitingForShutterOpen {
        init_timestamp: SystemTime,
    },
    SkipToNextFrame {
        init_timestamp: SystemTime,
    },
}

impl State {
    pub fn new() -> State {
        println!("Searching for frames...");
        State {
            current: ScannerState::SearchingFrame,
        }
    }

    pub fn set(&mut self, new: ScannerState) {
        // println!("Entering state: {:?}", new);
        self.current = new;
    }
}

struct ContourWithMoments {
    contour: Mat,
    frame_center_x: f64,
    moments: Moments,
    centroid: f64,
    area: f64,
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

    pub fn move_forward(&mut self, steps: usize, silent: bool) {
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

fn main() -> Result<()> {
    let port_infos = serialport::available_ports().expect("No serial ports found");
    let port_info = port_infos.iter().nth(1).expect("No serial ports found");
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

    let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)?; // 0 is the default camera

    if !videoio::VideoCapture::is_opened(&cam)? {
        panic!("Unable to open default camera");
    }

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
        &mut (frame_height - 0),
        frame_height,
        None,
    )?;

    let mut state = State::new();

    loop {
        // Capture next frame
        cam.read(&mut frame)?;

        // Read control window parameters
        let x = highgui::get_trackbar_pos("x", CONTROLS_WINDOW)?;
        let y = highgui::get_trackbar_pos("y", CONTROLS_WINDOW)?;
        let width = highgui::get_trackbar_pos("width", CONTROLS_WINDOW)?;
        let height = highgui::get_trackbar_pos("height", CONTROLS_WINDOW)?;

        // Don't allow cropping outside image
        let x = x.min(frame_width - width);
        let y = y.min(frame_height - height);

        // Crop image
        let rect = core::Rect::new(x, y, width, height);
        let frame_cropped = Mat::roi(&frame, rect)?;

        let mut frame_bw = Mat::default();
        imgproc::cvt_color(&frame_cropped, &mut frame_bw, imgproc::COLOR_RGB2GRAY, 0)?;

        let mut frame_blurred = Mat::default();
        imgproc::median_blur(&frame_bw, &mut frame_blurred, 7 * 2 + 1)?;

        let mut frame_thr = Mat::default();
        imgproc::adaptive_threshold(
            &frame_blurred,
            &mut frame_thr,
            255.0,
            imgproc::ADAPTIVE_THRESH_GAUSSIAN_C,
            imgproc::THRESH_BINARY,
            13,
            2.into(),
        )?;

        let sizes = vec![frame.cols(), 1].into_iter().collect();
        let s = Scalar::from(255.0);
        let kernel = Mat::new_nd_vec_with_default(&sizes, 0, s)?;

        let anchor = Point::new(-1, -1);
        let mut frame_mop = Mat::default();
        imgproc::morphology_ex(
            &frame_thr,
            &mut frame_mop,
            imgproc::MORPH_OPEN,
            &kernel,
            anchor,
            1,
            opencv::core::BORDER_CONSTANT,
            imgproc::morphology_default_border_value()?,
        )?;

        let mut frame_mop_blurred = Mat::default();
        imgproc::median_blur(&frame_mop, &mut frame_mop_blurred, 7 * 2 + 1)?;

        let mut frame_mop_blurred_inv = Mat::default();
        core::bitwise_not(
            &frame_mop_blurred,
            &mut frame_mop_blurred_inv,
            &core::no_array()?,
        )?;

        let frame_mop_blurred_inv = {
            let height = frame_mop_blurred_inv.rows() as f64 * MOP_RECT_HEIGHT_PCT;
            let mop_rect = core::Rect::new(
                0,
                ((frame_mop_blurred_inv.rows() as f64) / 2.0 - height / 2.0) as i32,
                frame_mop_blurred.cols(),
                height as i32,
            );

            Mat::roi(&frame_mop_blurred_inv, mop_rect)?
        };

        let mut contours = VectorOfMat::new();
        imgproc::find_contours(
            &frame_mop_blurred_inv,
            &mut contours,
            imgproc::RETR_TREE,
            imgproc::CHAIN_APPROX_SIMPLE,
            Point::new(0, 0),
        )?;

        let crop_rect = core::Rect::new(x, y, width, height);
        imgproc::rectangle(
            &mut frame,
            crop_rect,
            Scalar::new(255.0, 0.0, 0.0, 255.0),
            3,
            imgproc::LINE_8,
            0,
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
                y + ((height as f64 / 2.0 - frame_mop_blurred_inv.rows() as f64 / 2.0) as i32),
            ),
        )?;

        let key: char = highgui::poll_key()? as u8 as char;

        if key == 'q' {
            break;
        }

        highgui::imshow("frame_thr", &frame_thr)?;
        highgui::imshow("frame_mop", &frame_mop)?;
        highgui::imshow("frame_mop_blurred_inv", &frame_mop_blurred_inv)?;

        let mut cwms: Vec<ContourWithMoments> = contours
            .iter()
            .map(|contour| {
                let moments = imgproc::moments(&contour, false).expect("Calling moments failed");
                let centroid = moments.m10 / moments.m00;
                let area = imgproc::contour_area(&contour, false).unwrap();

                let bbox = imgproc::bounding_rect(&contour).unwrap();
                let seen_width = bbox.width as f64;
                let seen_end = bbox.x as f64 + seen_width;
                let aspect_ratio = 4.0 / 3.0 / 1.195;
                let h = bbox.height as f64;
                let w = h * aspect_ratio;
                let pos_x = if centroid > width as f64 / 2.0 {
                    bbox.x as f64
                } else {
                    seen_end - w
                };
                let frame_center_x = pos_x + w / 2.0;

                ContourWithMoments {
                    contour,
                    frame_center_x,
                    moments,
                    centroid,
                    area,
                }
            })
            .filter(|moments| moments.area > 100_000.0 * MOP_RECT_HEIGHT_PCT)
            .collect();

        cwms.sort_by(|a, b| a.area.partial_cmp(&b.area).unwrap());
        let last_cwm = cwms.iter().last();

        let center = (frame_cropped.cols() as f64) / 2.0;
        let missed = center - CENTER_TOLERANCE_MISSED;
        let in_position = center + CENTER_TOLERANCE;

        imgproc::draw_marker(
            &mut frame,
            Point::new(x + missed as i32, ((frame_height as f64) / 2.0) as i32),
            Scalar::new(255.0, 0.0, 0.0, 255.0),
            imgproc::MARKER_TRIANGLE_DOWN,
            20,
            1,
            8,
        )?;
        imgproc::draw_marker(
            &mut frame,
            Point::new(x + in_position as i32, ((frame_height as f64) / 2.0) as i32),
            Scalar::new(0.0, 255.0, 0.0, 255.0),
            imgproc::MARKER_TRIANGLE_UP,
            20,
            1,
            8,
        )?;

        if let Some(last) = last_cwm {
            imgproc::draw_marker(
                &mut frame,
                Point::new(
                    x + last.frame_center_x as i32,
                    ((frame_height as f64) / 2.0) as i32,
                ),
                Scalar::new(255.0, 255.0, 255.0, 255.0),
                imgproc::MARKER_TILTED_CROSS,
                20,
                1,
                8,
            )?;
            // if last.centroid < missed {
            //     println!("Frame {}px too far left", missed - last.centroid);
            // } else if last.centroid <= center + CENTER_TOLERANCE {
            //     println!("Frame in position (Δ{}px)", last.centroid - center);
            // } else {
            //     println!("Frame {}px too far right", last.centroid - in_position)
            // }
        }

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
            state.set(ScannerState::SearchingFrame);
        }

        if key == ' ' {
            pause = !pause;
            if pause {
                state.set(ScannerState::SearchingFrame);
                println!("Paused. Hit spacebar to unpause.");
                scanner.stop(true);
            } else {
                println!("Unpaused. Hit spacebar to pause.");
            }
        }

        if !pause && !MANUAL_MODE {
            match state.current {
                ScannerState::SearchingFrame => {
                    if last_cwm.is_some() {
                        // Found a frame, pause for a second to analyze exact distance
                        scanner.stop(false);
                        state.set(ScannerState::FoundFrame {
                            init_timestamp: SystemTime::now(),
                            consecutive_missed_frames: 0,
                            // Wait until enough time has passed so that the film has stopped moving (camera feed input lag)
                            wait_time: 0.7,
                        })
                    } else {
                        // Move motor at full speed until we find something
                        scanner.move_forward(500, true);
                    }
                }
                ScannerState::FoundFrame {
                    init_timestamp,
                    consecutive_missed_frames,
                    wait_time,
                } => {
                    if init_timestamp.elapsed().unwrap().as_secs_f64() < wait_time {
                        continue;
                    }

                    scanner.stop(false);

                    if let Some(last) = last_cwm {
                        state.set(ScannerState::FoundFrame {
                            init_timestamp,
                            consecutive_missed_frames: 0,
                            wait_time,
                        });

                        if last.frame_center_x < missed {
                            println!("Frame moved too far, pausing");
                            dbg!(last.frame_center_x, missed);
                            pause = !pause;
                        } else if last.frame_center_x <= center + CENTER_TOLERANCE {
                            scanner.focus();
                            println!(
                                "Taking picture with frame position Δ{}px",
                                last.frame_center_x - center
                            );
                            scanner.move_back(50);
                            state.set(ScannerState::TakingPhoto { init_timestamp });
                        } else {
                            println!("Stepping frame toward center");
                            let dist_px = last.frame_center_x - center;
                            println!("Distance to center: {}", dist_px);
                            let steps = dist_px * PX_PER_STEP;
                            let steps = steps.round(); //.max(5.0);
                            scanner.move_forward(steps as usize, false);
                            state.set(ScannerState::FoundFrame {
                                init_timestamp: SystemTime::now(),
                                consecutive_missed_frames: 0,
                                wait_time: 0.3 + steps / 1000.0,
                            });
                        }
                    } else if consecutive_missed_frames > 5 {
                        // If we can't see a frame, go back to SearchingFrame
                        state.set(ScannerState::SearchingFrame);
                    } else {
                        state.set(ScannerState::FoundFrame {
                            init_timestamp,
                            consecutive_missed_frames: consecutive_missed_frames + 1,
                            wait_time,
                        });
                    }
                }
                ScannerState::TakingPhoto { init_timestamp } => {
                    // Wait until enough time has passed so that the film has come to a stop
                    if init_timestamp.elapsed().unwrap().as_secs_f64() < 1.0 {
                        continue;
                    }

                    scanner.take_photo();
                    scanner.stop_focus();

                    state.set(ScannerState::WaitingForShutterOpen {
                        init_timestamp: SystemTime::now(),
                    });
                }
                ScannerState::WaitingForShutterOpen { init_timestamp } => {
                    // Wait until enough time has passed so that we can see the camera shutter black-out
                    if init_timestamp.elapsed().unwrap().as_secs_f64() < 1.0 {
                        continue;
                    }

                    // Crude detection of when shutter black-out ends - if last_cwm is some we are seeing a frame again
                    if last_cwm.is_some() {
                        // Move enough forward so that we start detecting the next frame
                        scanner.move_forward(NEXT_FRAME_SKIP_STEPS, false);

                        state.set(ScannerState::SkipToNextFrame {
                            init_timestamp: SystemTime::now(),
                        });
                    }
                }

                ScannerState::SkipToNextFrame { init_timestamp } => {
                    // scanner.focus();
                    // Wait until enough time has passed so that the film has moved far enough
                    if init_timestamp.elapsed().unwrap().as_secs_f64() < 2.0 {
                        continue;
                    }

                    // scanner.focus();
                    state.set(ScannerState::SearchingFrame);
                }
            }
        }
    }

    Ok(())
}
