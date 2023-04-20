# negative_scanner

![image](https://user-images.githubusercontent.com/3673134/233270537-ee8bb01a-56ac-459e-8fd4-ad0e522cfc32.png)

- YouTube video: https://www.youtube.com/watch?v=Sj7jFEvvFl8
- "Blog" post, short writeup with pictures of prototypes: https://github.com/FruitieX/negative_scanner/blob/main/blog.md
- 3D printed parts and another writeup: https://www.printables.com/model/244900-modular-print-in-place-35mm-film-scanner

This project consists of a Rust program and Arduino sketch that helps scan 35mm
film negatives using a DSLR camera and a motorized negative feeder.

## Parts list

You will need some basic electronics workshop tools and items. This includes
breadboards, jumper wires, resistors, LEDs (optional)

### Motorized

- Stepper motor (e.g. https://www.amazon.de/gp/product/B06XVM38YW)
- AC adapter for the stepper motor (e.g. https://www.amazon.de/gp/product/B079P8G5FX)
- Stepper motor driver (e.g. https://www.amazon.de/gp/product/B07Z8QP8M9)
- Just about any Arduino board (e.g. https://www.amazon.de/-/en/Arduino-Nano-Every-Headers-ABX00033/dp/B07WWK29XF)

### Automatic

- Everything listed in the Motorized section
- A camera that supports both HDMI output and some form of remote shutter release. The following parts are compatible with a Fujifilm X-T200.
- HDMI capture card (e.g. https://www.amazon.de/gp/product/B08F37J3PP) and cable (HDMI <-> micro HDMI)
- 2x optocouplers for the shutter release (e.g. https://www.amazon.de/-/en/Reland-10pcs-4N38-DIP-6-Optocoupler/dp/B09BFMD5HP, though others probably work also)
- 3.5mm audio cable for shutter release

## Setup (Arduino)

Start by wiring up the following schematic:

![negative_scanner_bb](https://user-images.githubusercontent.com/3673134/180787356-b0ec154f-f14b-488b-8146-9200ffeec2b2.png)

Next flash your Arduino with the provided sketch. You can test that everything works by
opening up the serial monitor, setting baud rate to 115200 and writing commands like:

- `100M`: should move motor 100 steps clockwise
- `-100M`: should move motor 100 steps counter-clockwise
- `F`: should act as a "half press" for the camera shutter
- `S`: should act as a "full press" for the camera shutter
- `f`: cancels half press
- `400s`: sets max speed to 400 (default 4000)

## Setup (Rust)

Make sure you're running Rust v1.60 or above. Get started by using [rustup](https://rustup.rs/)

You also need to setup opencv-rust, instructions here: https://github.com/twistedfall/opencv-rust#getting-opencv
OpenCV setup on Windows is a bit complicated, more detailed instructions here: https://github.com/twistedfall/opencv-rust/issues/118#issuecomment-619608278

After that, you can use `cargo run --release` to run the program.

The program will attempt to use the first serial port and video capture device that is available.

After launching, the program is in manual mode. You should now be able to move the film with `a` and `e` keys, and take photos with the `s` key. (sorry, I'm a Dvorak user :-)
Pressing `n` will advance the film to the next frame. (based on a constant number of steps between frames that you may need to calibrate to match your setup & film)

By pressing spacebar, you enter automatic mode. Pressing spacebar again re-enters manual mode. In automatic mode, the program:
- Slowly steps the film forward, allowing you to feed in the next strip. Meanwhile, attempts to find areas of the video feed with bright vertical lines. These are hopefully areas between different frames in the film, or areas where there is no film yet/anymore.
- If these areas are found past a certain "detection point", the distance from the edge of the screen to this area is converted to
number of steps the motor should move, and the move is performed rapidly.
- If no more areas are found, take a photo
- Wait until the camera has finished taking the photo (currently based on counting the number of #000000 pixels, which may not work for all cameras)
- Repeat from step one

There's a number of constants you will most likely need to change at the top of the `src/main.rs` file, with explanations above each one of them. If you've never worked with Rust before and you need to dive deeper into the code, I recommend installing [rust-analyzer](https://marketplace.visualstudio.com/items?itemName=rust-lang.rust-analyzer) for autocompletions in your IDE (available for other editors, too).

Re-run with `cargo run --release` after making changes.
