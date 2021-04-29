use std::time::Duration;

use opencv::{
    core::{self, Moments, Point, Scalar},
    highgui, imgproc,
    prelude::*,
    types::VectorOfMat,
    videoio, Result,
};

const CONTROLS_WINDOW: &str = "controls";
const MANUAL_MODE: bool = true;
const CENTROID_CENTERED: f64 = 272.0;
const CENTROID_MISSED: f64 = 230.0;

struct State {
    searching_frame: bool,
    waiting_for_shutter: bool,
}

impl State {
    pub fn new() -> State {
        State {
            searching_frame: true,
            waiting_for_shutter: false,
        }
    }
}

struct ContourWithMoments {
    contour: Mat,
    moments: Moments,
    centroid: f64,
    area: f64,
}

fn main() -> Result<()> {
    let mut port = serialport::new("COM3", 19_200)
        .timeout(Duration::from_secs(1))
        .open()
        .expect("Failed to open serial port");

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
    highgui::create_trackbar("y", CONTROLS_WINDOW, &mut 22, frame_height - 1, None)?;
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
        &mut (frame_height - 22 - 28),
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

        if MANUAL_MODE {
            // Move film forward
            if key == 'e' {
                port.write_all("LLLLLLLLLLLLLLLLLLLLLLLL".as_bytes())
                    .expect("Writing to serial port failed");
            }

            // Move film back
            if key == 'a' {
                port.write_all("RRRRRRRRRRRRRRRRRRRRRRRR".as_bytes())
                    .expect("Writing to serial port failed");
            }

            // Shutter
            if key == 's' {
                port.write_all("S".as_bytes())
                    .expect("Writing to serial port failed");
            }
        } /* else */
        {
            let mut moments: Vec<ContourWithMoments> = contours
                .iter()
                .map(|contour| {
                    let moments =
                        imgproc::moments(&contour, false).expect("Calling moments failed");
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

            moments.sort_by(|a, b| a.area.partial_cmp(&b.area).unwrap());

            let last = moments.iter().last();

            if let Some(last) = last {
                println!("Last contour centroid: {}", last.centroid);

                if last.centroid < CENTROID_MISSED {
                    println!("Last centroid too far!");
                } else if last.centroid < CENTROID_CENTERED {
                    println!("Last centroid past center point!");
                }
            }
        }
    }

    Ok(())
}
