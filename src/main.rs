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
const CENTROID_CENTERED: f64 = 300.0;
const CENTROID_MISSED: f64 = 230.0;

struct State {
    current: ScannerState,
}

#[derive(Debug)]
enum ScannerState {
    SearchingFrame,
    WaitingForShutterOpen { init_timestamp: SystemTime },
    SkipToNextFrame { init_timestamp: SystemTime },
}

impl State {
    pub fn new() -> State {
        println!("Searching for frames...");
        State {
            current: ScannerState::SearchingFrame,
        }
    }

    pub fn set(&mut self, new: ScannerState) {
        println!("Entering state: {:?}", new);
        self.current = new;
    }
}

struct ContourWithMoments {
    contour: Mat,
    moments: Moments,
    centroid: f64,
    area: f64,
}

struct Scanner {
    port: Box<dyn SerialPort>,
}

impl Scanner {
    pub fn new(port: Box<dyn SerialPort>) -> Scanner {
        Scanner { port }
    }

    pub fn move_forward(&mut self, steps: usize) {
        self.port
            .write_all("R".repeat(steps).as_bytes())
            .expect("Writing to serial port failed");
    }

    pub fn move_back(&mut self, steps: usize) {
        self.port
            .write_all("L".repeat(steps).as_bytes())
            .expect("Writing to serial port failed");
    }

    pub fn take_photo(&mut self) {
        self.port
            .write_all("S".as_bytes())
            .expect("Writing to serial port failed");
    }
}

fn main() -> Result<()> {
    let port_infos = serialport::available_ports().expect("No serial ports found");
    let port_info = port_infos.first().expect("No serial ports found");
    let port = serialport::new(port_info.port_name.clone(), 19_200)
        .timeout(Duration::from_secs(1))
        .open()
        .expect("Failed to open serial port");

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
    highgui::create_trackbar("y", CONTROLS_WINDOW, &mut 24, frame_height - 1, None)?;
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
        &mut (frame_height - 22 - 36),
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
        imgproc::median_blur(&frame_bw, &mut frame_blurred, 4 * 2 + 1)?;

        let mut frame_thr = Mat::default();
        imgproc::adaptive_threshold(
            &frame_blurred,
            &mut frame_thr,
            255.0,
            imgproc::ADAPTIVE_THRESH_GAUSSIAN_C,
            imgproc::THRESH_BINARY,
            5,
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

        let mut contours = VectorOfMat::new();
        imgproc::find_contours(
            &frame_mop_blurred_inv,
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
            Point::new(x, y),
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

        highgui::imshow("frame", &frame)?;
        highgui::imshow("frame_thr", &frame_thr)?;
        highgui::imshow("frame_mop", &frame_mop)?;

        let key: char = highgui::wait_key(1)? as u8 as char;

        if key == 'q' {
            break;
        }

        let mut cwms: Vec<ContourWithMoments> = contours
            .iter()
            .map(|contour| {
                let moments = imgproc::moments(&contour, false).expect("Calling moments failed");
                let centroid = moments.m10 / moments.m00;
                let area = imgproc::contour_area(&contour, false).unwrap();

                ContourWithMoments {
                    contour,
                    moments,
                    centroid,
                    area,
                }
            })
            .filter(|moments| moments.area > 100_000.0)
            .collect();

        cwms.sort_by(|a, b| a.area.partial_cmp(&b.area).unwrap());
        let last_cwm = cwms.iter().last();

        if MANUAL_MODE {
            // Move film forward
            if key == 'e' {
                scanner.move_forward(24);
            }

            // Move film back
            if key == 'a' {
                scanner.move_back(24);
            }

            // Shutter
            if key == 's' {
                scanner.take_photo();
            }
        } else {
            match state.current {
                ScannerState::SearchingFrame => {
                    if let Some(last) = last_cwm {
                        println!("Frame center detected at: {}", last.centroid);

                        if last.centroid < CENTROID_MISSED {
                            println!("Frame moved too far");
                            scanner.move_back(24);
                        } else if last.centroid < CENTROID_CENTERED {
                            println!("Frame in position");
                            scanner.take_photo();

                            state.set(ScannerState::WaitingForShutterOpen {
                                init_timestamp: SystemTime::now(),
                            });
                        } else {
                            println!("Stepping frame toward center");
                            scanner.move_forward(4);
                        }
                    } else {
                        scanner.move_forward(20);
                    }
                }
                ScannerState::WaitingForShutterOpen { init_timestamp } => {
                    // Wait until enough time has passed so that we can see the camera shutter black-out
                    if init_timestamp.elapsed().unwrap().as_secs_f64() < 1.0 {
                        continue;
                    }

                    // Crude detection of when shutter black-out ends - if last_cwm is some we are seeing a frame again
                    if last_cwm.is_some() {
                        // Move forward enough so that we start detecting the next frame
                        scanner.move_forward(850);

                        state.set(ScannerState::SkipToNextFrame {
                            init_timestamp: SystemTime::now(),
                        });
                    }
                }

                ScannerState::SkipToNextFrame { init_timestamp } => {
                    // Wait until enough time has passed so that the film has moved far enough
                    if init_timestamp.elapsed().unwrap().as_secs_f64() < 1.0 {
                        continue;
                    }

                    state.set(ScannerState::SearchingFrame);
                }
            }
        }
    }

    Ok(())
}
