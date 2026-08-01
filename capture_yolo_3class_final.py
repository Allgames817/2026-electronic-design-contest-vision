"""Collect the final 3-class YOLO data set on CanMV K230.

Classes to annotate later on the PC:
    0 ball
    1 zero
    2 limit  (both the left and right red markers use this class)

The saved image is the exact 640x360 camera view used by inference.  Press the
board key once per useful scene.  Existing files are never overwritten.

Output:
    /data/Picture/yolo_3class_raw/images/img_00000.jpg
    /data/Picture/yolo_3class_raw/manifest.csv

This targets CanMV K230 firmware v1.8 and the 800x480 ST7701 display used by
the current project.
"""

import gc
import os
import time

from machine import FPIOA, Pin
from media.display import *
from media.sensor import *


# ---------------------------- User settings ----------------------------

DATASET_ROOT = "/data/Picture/yolo_3class_raw"
IMAGE_FOLDER = DATASET_ROOT + "/images"
FILE_PREFIX = "img_"

# Change this for each collection session, for example:
# normal, bright, shadow, pipe_up, pipe_down, zero_occluded.
SESSION_TAG = "normal"

SENSOR_WIDTH = 640
SENSOR_HEIGHT = 480
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 360
IMAGE_CROP_Y = (SENSOR_HEIGHT - IMAGE_HEIGHT) // 2
CAMERA_FPS = 30
JPEG_QUALITY = 95

H_MIRROR = False
V_FLIP = False

ENABLE_PREVIEW = True
LCD_WIDTH = 800
LCD_HEIGHT = 480
LCD_BACKLIGHT_PIN = 25

# "button" is the recommended final setting.  "interval" is useful only
# while the ball/pipe/light is being continuously changed by hand.
TRIGGER_MODE = "interval"
INTERVAL_MS = 800
MAX_NEW_IMAGES = 1100


# ------------------------------- Helpers -------------------------------

def mkdir_p(path):
    current = ""
    for part in path.strip("/").split("/"):
        current += "/" + part
        try:
            os.mkdir(current)
        except OSError:
            pass


def next_index(folder, prefix):
    largest = -1
    try:
        names = os.listdir(folder)
    except OSError:
        return 0

    for name in names:
        if not name.startswith(prefix) or not name.endswith(".jpg"):
            continue
        try:
            value = int(name[len(prefix):-4])
            if value > largest:
                largest = value
        except ValueError:
            pass
    return largest + 1


def board_key_config():
    board = os.uname()[-1]
    if board == "k230_canmv_lckfb":
        return 53, 1
    if board == "k230_canmv_01studio":
        return 21, 0
    print("Unknown board '%s'; fallback GPIO21 active-low" % board)
    return 21, 0


def init_key():
    gpio_number, active_level = board_key_config()
    fpioa = FPIOA()
    fpioa.set_function(gpio_number, FPIOA.GPIO0 + gpio_number)
    pull = Pin.PULL_UP if active_level == 0 else Pin.PULL_DOWN
    key = Pin(gpio_number, Pin.IN, pull)
    return fpioa, key, active_level


def key_pressed_once(key, active_level):
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
    byte_count = jpeg.size()
    del jpeg
    return byte_count


def write_capture_info():
    path = DATASET_ROOT + "/capture_info.txt"
    with open(path, "w") as output:
        output.write("classes=0:ball,1:zero,2:limit\n")
        output.write("sensor=%dx%d\n" % (SENSOR_WIDTH, SENSOR_HEIGHT))
        output.write("saved_image=%dx%d\n" % (IMAGE_WIDTH, IMAGE_HEIGHT))
        output.write("crop_y=%d\n" % IMAGE_CROP_Y)
        output.write("h_mirror=%s\n" % H_MIRROR)
        output.write("v_flip=%s\n" % V_FLIP)
        output.write("jpeg_quality=%d\n" % JPEG_QUALITY)
        output.write("limit_center_distance_cm=10.0\n")


def append_manifest(filename, byte_count):
    path = DATASET_ROOT + "/manifest.csv"
    need_header = False
    try:
        os.stat(path)
    except OSError:
        need_header = True

    with open(path, "a") as output:
        if need_header:
            output.write(
                "filename,session,width,height,jpeg_bytes,ticks_ms\n"
            )
        output.write(
            "%s,%s,%d,%d,%d,%d\n" % (
                filename,
                SESSION_TAG,
                IMAGE_WIDTH,
                IMAGE_HEIGHT,
                byte_count,
                time.ticks_ms(),
            )
        )


def draw_preview(img, saved_count, message):
    # Drawing occurs after saving, so these overlays never enter the data set.
    img.draw_rectangle(8, 8, 430, 64, color=(0, 0, 0), fill=True)
    img.draw_string_advanced(
        16, 12, 20,
        "3CLS %s saved:%d" % (SESSION_TAG, saved_count),
        color=(0, 255, 0),
    )
    img.draw_string_advanced(
        16, 40, 16, message, color=(255, 255, 255)
    )
    Display.show_image(img, x=0, y=0)


def main():
    if TRIGGER_MODE not in ("button", "interval"):
        raise ValueError('TRIGGER_MODE must be "button" or "interval"')

    mkdir_p(IMAGE_FOLDER)
    write_capture_info()
    image_index = next_index(IMAGE_FOLDER, FILE_PREFIX)

    sensor = None
    display_started = False
    backlight = None
    key_fpioa = None
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
            backlight = Pin(
                LCD_BACKLIGHT_PIN,
                Pin.OUT,
                pull=Pin.PULL_NONE,
                drive=7,
            )
            backlight.value(1)
            Display.init(
                Display.ST7701,
                width=LCD_WIDTH,
                height=LCD_HEIGHT,
                fps=CAMERA_FPS,
                to_ide=False,
            )
            display_started = True

        if TRIGGER_MODE == "button":
            key_fpioa, key, active_level = init_key()

        sensor.run()

        # Discard early frames while exposure and white balance settle.
        for _ in range(30):
            sensor.snapshot(chn=CAM_CHN_ID_0)

        print("Final 3-class collector")
        print("Output:", IMAGE_FOLDER)
        print("Session:", SESSION_TAG)
        print("Starting index:", image_index)
        print("Trigger:", TRIGGER_MODE)

        saved_count = 0
        last_capture_ms = time.ticks_ms() - INTERVAL_MS

        while saved_count < MAX_NEW_IMAGES:
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

            message = "press key: save one image"
            if TRIGGER_MODE == "interval":
                message = "interval: keep changing the scene"

            if should_save:
                filename = "%s%05d.jpg" % (FILE_PREFIX, image_index)
                full_path = IMAGE_FOLDER + "/" + filename
                byte_count = save_jpeg(img, full_path)
                append_manifest(filename, byte_count)
                print("saved", filename, byte_count, "bytes")
                saved_count += 1
                image_index += 1
                last_capture_ms = time.ticks_ms()
                message = "saved " + filename

            if ENABLE_PREVIEW:
                draw_preview(img, saved_count, message)

            del img
            gc.collect()
            time.sleep_ms(10)

        print("Finished:", saved_count, "new images")

    except KeyboardInterrupt:
        print("Capture stopped by user")
    except BaseException as error:
        if str(error) == "IDE interrupt":
            print("Capture stopped by CanMV IDE")
        else:
            print("Capture error:", error)
    finally:
        if isinstance(sensor, Sensor):
            sensor.stop()
        if display_started:
            Display.deinit()
        if backlight is not None:
            backlight.value(0)
        key = None
        key_fpioa = None
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        gc.collect()


if __name__ == "__main__":
    main()
