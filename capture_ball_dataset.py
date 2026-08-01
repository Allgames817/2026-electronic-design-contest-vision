"""
CanMV K230 collector for a steel-ball data set and half-pipe position calibration.

Usage:
1. Edit CAPTURE_SET to "ball" or "pipe_calibration".
2. Run this file in CanMV IDE.
3. Press the board key once for every image.
4. Press Ctrl+C in the IDE when finished.

Images are written to:
    /sdcard/steel_ball_dataset/ball/
    /sdcard/steel_ball_dataset/pipe_calibration/

This script targets the media.sensor API used by CanMV K230 firmware v1.8.
"""

import gc
import os
import time

from machine import FPIOA, Pin
from media.display import *
from media.sensor import *


# ---------------------------------------------------------------------------
# User configuration
# ---------------------------------------------------------------------------

# "ball": collect steel-ball images for training/testing.
# "pipe_calibration": record ball images at known locations along the pipe.
CAPTURE_SET = "ball"

# Keep these values consistent with mjpeg_steel_ball_server.py. The OV5647
# captures 640x480, while the detector uses the centered 640x360 region.
SENSOR_WIDTH = 640
SENSOR_HEIGHT = 480
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 360
IMAGE_CROP_Y = (SENSOR_HEIGHT - IMAGE_HEIGHT) // 2
CAMERA_FPS = 30
JPEG_QUALITY = 95

# Show the live capture area on the physical ST7701 LCD while collecting.
ENABLE_PREVIEW = True
LCD_WIDTH = 800
LCD_HEIGHT = 480
# K230's ROI copy keeps a 640x480 backing buffer on this firmware. Display it
# from the upper-left corner so the buffer never extends beyond the LCD.
LCD_VIEW_X = 0
LCD_VIEW_Y = 0

# "button" is recommended because it avoids many near-duplicate images.
# Set to "interval" if the board key is unavailable.
TRIGGER_MODE = "interval"
INTERVAL_MS = 1000
MAX_IMAGES = 500

# Half-pipe and ball geometry, in millimetres. These values are written to
# capture_info.txt and will be used by the later pixel-to-position calibration.
PIPE_LENGTH_MM = 250.0
PIPE_OUTER_DIAMETER_MM = 20.0
PIPE_INNER_DIAMETER_MM = 13.0
BALL_DIAMETER_MM = 10.0

# Put a small removable mark at each position along the pipe centre line.
# The ball centre cannot reach the physical ends, hence 5 mm and 245 mm.
# At each position, press the key PIPE_IMAGES_PER_POSITION times, then move
# the ball to the next mark shown in the preview.
PIPE_CALIBRATION_POSITIONS_MM = (
    5.0, 30.0, 55.0, 80.0, 105.0, 130.0,
    155.0, 180.0, 205.0, 230.0, 245.0,
)
PIPE_IMAGES_PER_POSITION = 3

DATASET_ROOT = "/data/Picture"

# Optional image orientation correction.
H_MIRROR = False
V_FLIP = False


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def mkdir_p(path):
    current = ""
    for part in path.strip("/").split("/"):
        current += "/" + part
        try:
            os.mkdir(current)
        except OSError:
            # The directory normally already exists on subsequent runs.
            pass


def next_index(folder, prefix):
    """Continue numbering without overwriting images from an earlier run."""
    largest = -1
    try:
        names = os.listdir(folder)
    except OSError:
        return 0

    for name in names:
        if not name.startswith(prefix) or not name.endswith(".jpg"):
            continue
        number_text = name[len(prefix):-4]
        try:
            number = int(number_text)
            if number > largest:
                largest = number
        except ValueError:
            pass
    return largest + 1


def board_key_config():
    """
    Return (gpio_number, active_level).

    Values follow the CanMV example for the 01Studio AI Cube and LCKFB board.
    Unknown K230 boards fall back to GPIO21; edit here if your key is different.
    """
    board = os.uname()[-1]
    if board == "k230_canmv_lckfb":
        return 53, 1
    if board == "k230_canmv_01studio":
        return 21, 0

    print("Unknown board '%s'; using GPIO21, active-low" % board)
    return 21, 0


def init_key():
    gpio_number, active_level = board_key_config()
    fpioa = FPIOA()
    fpioa.set_function(gpio_number, FPIOA.GPIO0 + gpio_number)
    pull = Pin.PULL_UP if active_level == 0 else Pin.PULL_DOWN
    key = Pin(gpio_number, Pin.IN, pull)
    return key, active_level


def key_pressed_once(key, active_level):
    """Debounce the key and return True once per press."""
    if key.value() != active_level:
        return False

    time.sleep_ms(25)
    if key.value() != active_level:
        return False

    while key.value() == active_level:
        time.sleep_ms(10)
        os.exitpoint()
    return True


def save_jpeg(img, path):
    jpeg = img.compressed(JPEG_QUALITY)
    with open(path, "wb") as output:
        output.write(jpeg)
    size = jpeg.size()
    del jpeg
    return size


def write_metadata(folder):
    path = folder + "/capture_info.txt"
    with open(path, "w") as output:
        output.write("capture_set=%s\n" % CAPTURE_SET)
        output.write("sensor_width=%d\n" % SENSOR_WIDTH)
        output.write("sensor_height=%d\n" % SENSOR_HEIGHT)
        output.write("image_width=%d\n" % IMAGE_WIDTH)
        output.write("image_height=%d\n" % IMAGE_HEIGHT)
        output.write("image_crop_y=%d\n" % IMAGE_CROP_Y)
        output.write("jpeg_quality=%d\n" % JPEG_QUALITY)
        output.write("h_mirror=%s\n" % H_MIRROR)
        output.write("v_flip=%s\n" % V_FLIP)
        if CAPTURE_SET == "pipe_calibration":
            output.write("pipe_shape=half_round_pipe\n")
            output.write("pipe_length_mm=%.3f\n" % PIPE_LENGTH_MM)
            output.write(
                "pipe_outer_diameter_mm=%.3f\n" % PIPE_OUTER_DIAMETER_MM
            )
            output.write(
                "pipe_inner_diameter_mm=%.3f\n" % PIPE_INNER_DIAMETER_MM
            )
            output.write("ball_diameter_mm=%.3f\n" % BALL_DIAMETER_MM)
            output.write(
                "calibration_positions_mm=%s\n"
                % str(PIPE_CALIBRATION_POSITIONS_MM)
            )
            output.write(
                "images_per_position=%d\n" % PIPE_IMAGES_PER_POSITION
            )


def append_manifest(folder, filename, byte_count, position_mm=None):
    path = folder + "/manifest.csv"
    need_header = False
    try:
        os.stat(path)
    except OSError:
        need_header = True

    with open(path, "a") as output:
        if need_header:
            output.write(
                "filename,capture_set,pipe_position_mm,width,height,jpeg_bytes,ticks_ms\n"
            )
        output.write(
            "%s,%s,%s,%d,%d,%d,%d\n"
            % (
                filename,
                CAPTURE_SET,
                "" if position_mm is None else "%.3f" % position_mm,
                IMAGE_WIDTH,
                IMAGE_HEIGHT,
                byte_count,
                time.ticks_ms(),
            )
        )


def draw_preview(img, saved_count, message):
    # Draw only on the preview after the original frame has already been saved.
    img.draw_rectangle(8, 8, 360, 64, color=(0, 0, 0), fill=True)
    img.draw_string_advanced(
        16,
        12,
        20,
        "%s  saved:%d" % (CAPTURE_SET, saved_count),
        color=(0, 255, 0),
    )
    img.draw_string_advanced(
        16,
        40,
        16,
        message,
        color=(255, 255, 255),
    )
    Display.show_image(img, x=LCD_VIEW_X, y=LCD_VIEW_Y)


def main():
    if CAPTURE_SET not in ("ball", "pipe_calibration"):
        raise ValueError('CAPTURE_SET must be "ball" or "pipe_calibration"')
    if TRIGGER_MODE not in ("button", "interval"):
        raise ValueError('TRIGGER_MODE must be "button" or "interval"')
    if CAPTURE_SET == "pipe_calibration" and TRIGGER_MODE != "button":
        raise ValueError("pipe_calibration requires TRIGGER_MODE = 'button'")

    folder = DATASET_ROOT + "/" + CAPTURE_SET
    prefix = CAPTURE_SET + "_"
    mkdir_p(folder)
    write_metadata(folder)
    image_index = next_index(folder, prefix)
    if CAPTURE_SET == "pipe_calibration":
        target_images = (
            len(PIPE_CALIBRATION_POSITIONS_MM) * PIPE_IMAGES_PER_POSITION
        )
    else:
        target_images = MAX_IMAGES

    sensor = None
    display_started = False
    lcd_backlight = None
    key = None
    active_level = 0

    try:
        os.exitpoint(os.EXITPOINT_ENABLE)

        sensor = Sensor(
            width=SENSOR_WIDTH,
            height=SENSOR_HEIGHT,
            fps=CAMERA_FPS,
        )
        sensor.reset()
        sensor.set_hmirror(H_MIRROR)
        sensor.set_vflip(V_FLIP)
        sensor.set_framesize(
            width=SENSOR_WIDTH,
            height=SENSOR_HEIGHT,
            chn=CAM_CHN_ID_0,
        )
        sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_0)

        if ENABLE_PREVIEW:
            # The AI Cube uses an 800x480 ST7701 LCD.
            lcd_backlight = Pin(
                25,
                Pin.OUT,
                pull=Pin.PULL_NONE,
                drive=7,
            )
            lcd_backlight.value(1)
            Display.init(
                Display.ST7701,
                width=LCD_WIDTH,
                height=LCD_HEIGHT,
                fps=CAMERA_FPS,
                to_ide=False,
            )
            display_started = True

        if TRIGGER_MODE == "button":
            key, active_level = init_key()

        sensor.run()

        # Let auto exposure and auto white balance settle.
        for _ in range(30):
            sensor.snapshot(chn=CAM_CHN_ID_0)
        print("Capture set:", CAPTURE_SET)
        print("Output:", folder)
        print("Starting index:", image_index)
        print("Trigger:", TRIGGER_MODE)
        if CAPTURE_SET == "pipe_calibration":
            print("Place the ball at every marked pipe position in turn.")

        saved_count = 0
        last_capture_ms = time.ticks_ms() - INTERVAL_MS
        while saved_count < target_images:
            os.exitpoint()
            full_img = sensor.snapshot(chn=CAM_CHN_ID_0)
            img = full_img.copy(
                (0, IMAGE_CROP_Y, IMAGE_WIDTH, IMAGE_HEIGHT)
            )
            del full_img
            should_save = False

            if TRIGGER_MODE == "button":
                should_save = key_pressed_once(key, active_level)
            elif time.ticks_diff(time.ticks_ms(), last_capture_ms) >= INTERVAL_MS:
                should_save = True

            position_mm = None
            message = "press key to save"
            if CAPTURE_SET == "pipe_calibration":
                position_index = saved_count // PIPE_IMAGES_PER_POSITION
                position_mm = PIPE_CALIBRATION_POSITIONS_MM[position_index]
                repeat_index = (saved_count % PIPE_IMAGES_PER_POSITION) + 1
                message = "ball at %.0fmm  shot %d/%d" % (
                    position_mm,
                    repeat_index,
                    PIPE_IMAGES_PER_POSITION,
                )
            if TRIGGER_MODE == "interval":
                message = "interval capture"

            if should_save:
                filename = "%s%05d.jpg" % (prefix, image_index)
                full_path = folder + "/" + filename
                byte_count = save_jpeg(img, full_path)
                append_manifest(folder, filename, byte_count, position_mm)
                saved_count += 1
                image_index += 1
                last_capture_ms = time.ticks_ms()
                message = "saved " + filename
                print(message, byte_count, "bytes")

            if ENABLE_PREVIEW:
                draw_preview(img, saved_count, message)
            del img
            gc.collect()
            time.sleep_ms(10)

        print("Finished:", saved_count, "new images")

    except KeyboardInterrupt:
        print("Capture stopped by user")
    except BaseException as error:
        # Some CanMV firmware builds do not expose sys.print_exception().
        # Keep error reporting independent from that optional helper.
        if str(error) == "IDE interrupt":
            print("Capture stopped by CanMV IDE")
        else:
            print("Capture error:", error)
    finally:
        if isinstance(sensor, Sensor):
            sensor.stop()
        if display_started:
            Display.deinit()
        if lcd_backlight is not None:
            lcd_backlight.value(0)
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        gc.collect()


if __name__ == "__main__":
    main()
