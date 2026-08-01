"""Run the newly trained best.kmodel on K230/CanMV.

Required SD-card files:
    /sdcard/best.kmodel
    /sdcard/steel_ball_best.py

This runner is for a single-class YOLOv8 detection kmodel with a
320x320 input. It selects the highest-confidence steel-ball box,
calculates its center in the 640x360 camera coordinate system, calibrates
the mechanical center O from 100 valid samples, and sends both:

    BALL,cx,cy,confidence\r\n
    POS,signed_position_cm\r\n

over UART1. A missing ball is sent as BALL,-1,-1,0 and POS,NA.
"""

from libs.PipeLine import PipeLine, ScopedTiming
from libs.YOLO import YOLOv8
from machine import FPIOA, Pin, UART
import gc
import sys
import time


# ---------------------------- Model settings ----------------------------
KMODEL_PATH = "/sdcard/best.kmodel"
LABELS = ["steel_ball"]

MODEL_INPUT_SIZE = [320, 320]

CONFIDENCE_THRESHOLD = 0.35
NMS_THRESHOLD = 0.45

# Camera frame sent to AI. Width should be aligned to 16.
RGB888P_SIZE = [640, 360]

# Explicitly initialize the 800x480 ST7701 LCD through PipeLine.
DISPLAY_MODE = "st7701"
DISPLAY_SIZE = None
LCD_BACKLIGHT_PIN = 25

# UART1 to TI MSPM0: K230 pin 3 TX -> MSPM0 RX,
#                      K230 pin 4 RX <- MSPM0 TX.
# K230 GND and MSPM0 GND must be connected.
UART_TX_PIN = 3
UART_RX_PIN = 4
UART_BAUDRATE = 115200

# ---------------------------- Calibration -----------------------------
# center.cfg stores only the mechanical center O.
CENTER_CONFIG_PATH = "/sdcard/center.cfg"
# The +5 cm and -5 cm positions are stored independently.
PLUS_CONFIG_PATH = "/sdcard/PLUS.cfg"
MINUS_CONFIG_PATH = "/sdcard/Minus.cfg"
CALIBRATION_SAMPLE_COUNT = 100
FORCE_CENTER_CALIBRATION = False

# Set to "PLUS" while the ball is fixed at +5 cm, set to "MINUS" while it
# is fixed at -5 cm, then restore None after both calibration runs.
POSITION_CALIBRATION_MODE = None

# Reference-line half height in the 640x360 camera coordinate system.
PIPE_HALF_HEIGHT = 60

# Keep a global reference so MicroPython GC does not release the GPIO object.
lcd_backlight = None


class CenterCalibrator:
    """Collect valid ball centers and persist the averaged mechanical center."""

    def __init__(self, path, sample_count):
        self.path = path
        self.sample_count = sample_count
        self.sum_x = 0.0
        self.sum_y = 0.0
        self.count = 0
        self.center_x = None
        self.center_y = None
        self.active = False

    def load(self):
        values = {}
        try:
            with open(self.path, "r") as config_file:
                for line in config_file:
                    if "=" not in line:
                        continue
                    key, value = line.strip().split("=", 1)
                    values[key] = float(value)
            self.center_x = values["CENTER_X"]
            self.center_y = values["CENTER_Y"]
            print(
                "Loaded center: CENTER_X=%.2f, CENTER_Y=%.2f" %
                (self.center_x, self.center_y)
            )
            return True
        except Exception:
            self.center_x = None
            self.center_y = None
            return False

    def start(self):
        self.sum_x = 0.0
        self.sum_y = 0.0
        self.count = 0
        self.center_x = None
        self.center_y = None
        self.active = True
        print("Center calibration started")
        print("Keep the ball still at the mechanical center O")

    def add(self, ball):
        if not self.active or ball is None:
            return False

        camera_cx, camera_cy, _, _, _ = ball
        self.sum_x += camera_cx
        self.sum_y += camera_cy
        self.count += 1
        print(
            "CAL CENTER %03d/%03d ball: %d %d" %
            (self.count, self.sample_count, camera_cx, camera_cy)
        )
        if self.count < self.sample_count:
            return False

        self.center_x = self.sum_x / self.count
        self.center_y = self.sum_y / self.count
        self.active = False
        self.save()
        print("Center calibration complete")
        print("Center=%.2f, %.2f" % (self.center_x, self.center_y))
        print("Saved to:", self.path)
        return True

    def save(self):
        with open(self.path, "w") as config_file:
            config_file.write("CENTER_X=%.3f\n" % self.center_x)
            config_file.write("CENTER_Y=%.3f\n" % self.center_y)

    def has_center(self):
        return self.center_x is not None and self.center_y is not None

    def is_valid(self):
        return (
            self.has_center() and
            0.0 <= self.center_x < RGB888P_SIZE[0] and
            0.0 <= self.center_y < RGB888P_SIZE[1]
        )

    def error(self, ball):
        if ball is None or not self.has_center():
            return None
        camera_cx, camera_cy, _, _, _ = ball
        return camera_cx - self.center_x, camera_cy - self.center_y


class PositionCalibrator:
    """Collect the average x-coordinate at the +5 cm or -5 cm point."""

    def __init__(self, plus_path, minus_path, sample_count):
        self.plus_path = plus_path
        self.minus_path = minus_path
        self.sample_count = sample_count
        self.mode = None
        self.sum_x = 0.0
        self.count = 0
        self.plus_x = None
        self.minus_x = None
        self.active = False

    def load(self):
        try:
            with open(self.plus_path, "r") as config_file:
                for line in config_file:
                    if line.startswith("PLUS_X="):
                        self.plus_x = float(line.strip().split("=", 1)[1])
        except Exception:
            self.plus_x = None

        try:
            with open(self.minus_path, "r") as config_file:
                for line in config_file:
                    if line.startswith("MINUS_X="):
                        self.minus_x = float(line.strip().split("=", 1)[1])
        except Exception:
            self.minus_x = None

        if self.plus_x is not None:
            print("Loaded PLUS_X=%.2f" % self.plus_x)
        if self.minus_x is not None:
            print("Loaded MINUS_X=%.2f" % self.minus_x)
        return self.plus_x is not None or self.minus_x is not None

    def start(self, mode):
        mode = str(mode).upper()
        if mode != "PLUS" and mode != "MINUS":
            raise ValueError(
                'POSITION_CALIBRATION_MODE must be "PLUS", "MINUS", or None'
            )
        self.mode = mode
        self.sum_x = 0.0
        self.count = 0
        self.active = True
        print("%s 5 cm calibration started" % mode)
        print("Keep the ball still at the %s 5 cm position" % mode)

    def add(self, ball):
        if not self.active or ball is None:
            return False

        camera_cx = ball[0]
        self.sum_x += camera_cx
        self.count += 1
        print(
            "CAL %s %03d/%03d ball x: %d" % (
                self.mode,
                self.count,
                self.sample_count,
                camera_cx,
            )
        )
        if self.count < self.sample_count:
            return False

        position_x = self.sum_x / self.count
        if self.mode == "PLUS":
            self.plus_x = position_x
        else:
            self.minus_x = position_x
        self.active = False
        self.save()
        print("%s 5 cm calibration complete" % self.mode)
        print("%s_X=%.2f" % (self.mode, position_x))
        return True

    def save(self):
        if self.mode == "PLUS":
            with open(self.plus_path, "w") as config_file:
                config_file.write("PLUS_X=%.3f\n" % self.plus_x)
            print("Saved to:", self.plus_path)
        else:
            with open(self.minus_path, "w") as config_file:
                config_file.write("MINUS_X=%.3f\n" % self.minus_x)
            print("Saved to:", self.minus_path)

    def is_valid(self, center_x):
        """Return True for the user's +5(left), 0, -5(right) coordinates."""
        return (
            center_x is not None and
            self.plus_x is not None and
            self.minus_x is not None and
            0.0 <= self.plus_x < center_x and
            center_x < self.minus_x < RGB888P_SIZE[0]
        )

    def position_cm(self, camera_cx, center_x):
        """Convert camera x to signed cm with separate left/right scales."""
        if not self.is_valid(center_x):
            return None
        if camera_cx <= center_x:
            return (
                5.0 * (center_x - camera_cx) /
                (center_x - self.plus_x)
            )
        return (
            -5.0 * (camera_cx - center_x) /
            (self.minus_x - center_x)
        )


def init_uart():
    """Configure UART1 as 115200, 8 data bits, no parity, 1 stop bit."""
    fpioa = FPIOA()
    fpioa.set_function(
        UART_TX_PIN,
        FPIOA.UART1_TXD,
        ie=1,
        oe=1
    )
    fpioa.set_function(
        UART_RX_PIN,
        FPIOA.UART1_RXD,
        ie=1,
        oe=1
    )

    uart = UART(
        UART.UART1,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE
    )
    return fpioa, uart


def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def camera_x_to_osd(camera_x, display_size):
    return int(
        float(camera_x) * display_size[0] / RGB888P_SIZE[0] + 0.5
    )


def camera_y_to_osd(camera_y, display_size):
    return int(
        float(camera_y) * display_size[1] / RGB888P_SIZE[1] + 0.5
    )


def draw_position_references(
    osd_img,
    display_size,
    center_calibrator,
    position_calibrator
):
    """Draw +5 cm, 0 cm and -5 cm markers in the calibrated pipe band."""
    if (
        not center_calibrator.is_valid() or
        not position_calibrator.is_valid(center_calibrator.center_x)
    ):
        return False

    camera_y_top = clamp(
        center_calibrator.center_y - PIPE_HALF_HEIGHT,
        0,
        RGB888P_SIZE[1] - 1
    )
    camera_y_bottom = clamp(
        center_calibrator.center_y + PIPE_HALF_HEIGHT,
        0,
        RGB888P_SIZE[1] - 1
    )
    osd_y_top = camera_y_to_osd(camera_y_top, display_size)
    osd_y_bottom = camera_y_to_osd(camera_y_bottom, display_size)
    label_y = clamp(osd_y_top - 22, 0, display_size[1] - 22)

    markers = (
        (position_calibrator.plus_x, "+5cm"),
        (center_calibrator.center_x, "0cm"),
        (position_calibrator.minus_x, "-5cm"),
    )
    for camera_x, label in markers:
        osd_x = camera_x_to_osd(camera_x, display_size)
        osd_img.draw_line(
            osd_x,
            osd_y_top,
            osd_x,
            osd_y_bottom,
            color=(255, 0, 0),
            thickness=2
        )
        label_x = clamp(osd_x - 24, 0, display_size[0] - 64)
        osd_img.draw_string_advanced(
            label_x,
            label_y,
            20,
            label,
            color=(255, 0, 0)
        )
    return True


def get_best_ball_center(detections, display_size):
    """Return the best ball as (camera_cx, camera_cy, osd_cx, osd_cy, score).

    YOLOv8 detection results have the form:
        detections[0]: boxes [x, y, width, height] in display coordinates
        detections[1]: class indexes
        detections[2]: confidence scores

    The model has one steel-ball class. If several boxes remain after NMS,
    the box with the highest confidence is used.
    """
    if not detections or len(detections[0]) == 0:
        return None

    boxes = detections[0]
    scores = detections[2]
    best_index = 0
    best_score = float(scores[0])

    for index in range(1, len(boxes)):
        score = float(scores[index])
        if score > best_score:
            best_index = index
            best_score = score

    x, y, width, height = boxes[best_index]
    osd_cx = int(round(float(x) + float(width) * 0.5))
    osd_cy = int(round(float(y) + float(height) * 0.5))

    # Convert the display-space box center back to the 640x360 camera space.
    camera_cx = int(round(
        float(osd_cx) * RGB888P_SIZE[0] / display_size[0]
    ))
    camera_cy = int(round(
        float(osd_cy) * RGB888P_SIZE[1] / display_size[1]
    ))
    camera_cx = clamp(camera_cx, 0, RGB888P_SIZE[0] - 1)
    camera_cy = clamp(camera_cy, 0, RGB888P_SIZE[1] - 1)

    return camera_cx, camera_cy, osd_cx, osd_cy, best_score


def send_ball_center(uart, ball):
    """Send one newline-terminated ASCII frame to the MSPM0."""
    if ball is None:
        frame = "BALL,-1,-1,0\r\n"
    else:
        camera_cx, camera_cy, _, _, score = ball
        confidence = clamp(int(round(score * 1000)), 0, 1000)
        frame = "BALL,%d,%d,%d\r\n" % (
            camera_cx,
            camera_cy,
            confidence
        )

    uart.write(frame.encode("utf-8"))


def send_ball_position(uart, position_cm):
    """Send a separate position frame without changing the BALL protocol."""
    if position_cm is None:
        frame = "POS,NA\r\n"
    else:
        frame = "POS,%+.2f\r\n" % position_cm
    uart.write(frame.encode("utf-8"))


def main():
    global lcd_backlight

    pipeline = None
    detector = None
    uart = None
    uart_fpioa = None

    try:
        print("Loading:", KMODEL_PATH)

        lcd_backlight = Pin(
            LCD_BACKLIGHT_PIN,
            Pin.OUT,
            pull=Pin.PULL_NONE,
            drive=7
        )
        lcd_backlight.value(1)
        print("ST7701 backlight ON")

        uart_fpioa, uart = init_uart()
        uart.write(b"BALL,BOOT\r\n")
        print(
            "UART1 ready: TX pin %d, RX pin %d, %d baud" %
            (UART_TX_PIN, UART_RX_PIN, UART_BAUDRATE)
        )

        pipeline = PipeLine(
            rgb888p_size=RGB888P_SIZE,
            display_size=DISPLAY_SIZE,
            display_mode=DISPLAY_MODE
        )
        pipeline.create(to_ide=True)
        actual_display_size = pipeline.get_display_size()

        detector = YOLOv8(
            task_type="detect",
            mode="video",
            kmodel_path=KMODEL_PATH,
            labels=LABELS,
            rgb888p_size=RGB888P_SIZE,
            model_input_size=MODEL_INPUT_SIZE,
            display_size=actual_display_size,
            conf_thresh=CONFIDENCE_THRESHOLD,
            nms_thresh=NMS_THRESHOLD,
            debug_mode=0
        )
        detector.config_preprocess()

        print("best.kmodel initialized")
        print("model input:", MODEL_INPUT_SIZE)
        print("Press Ctrl+C in CanMV IDE to stop")

        calibrator = CenterCalibrator(
            CENTER_CONFIG_PATH,
            CALIBRATION_SAMPLE_COUNT
        )
        if FORCE_CENTER_CALIBRATION or not calibrator.load():
            calibrator.start()

        position_calibrator = PositionCalibrator(
            PLUS_CONFIG_PATH,
            MINUS_CONFIG_PATH,
            CALIBRATION_SAMPLE_COUNT
        )
        position_calibrator.load()
        if POSITION_CALIBRATION_MODE is not None:
            if calibrator.active or not calibrator.is_valid():
                raise RuntimeError(
                    "Calibrate center.cfg before PLUS/MINUS positions"
                )
            position_calibrator.start(POSITION_CALIBRATION_MODE)
        elif (
            not calibrator.active and
            (
                not calibrator.is_valid() or
                not position_calibrator.is_valid(calibrator.center_x)
            )
        ):
            print("Calibration data missing or invalid")
            print("Expected PLUS_X < CENTER_X < MINUS_X")

        clock = time.clock()

        while True:
            clock.tick()

            with ScopedTiming("total", 1):
                frame = pipeline.get_frame()
                detections = detector.run(frame)

                detector.draw_result(detections, pipeline.osd_img)

                ball = get_best_ball_center(
                    detections,
                    actual_display_size
                )
                calibrator.add(ball)
                position_calibrator.add(ball)
                send_ball_center(uart, ball)

                calibration_ready = (
                    not calibrator.active and
                    not position_calibrator.active and
                    calibrator.is_valid() and
                    position_calibrator.is_valid(calibrator.center_x)
                )
                position_cm = None
                if calibration_ready and ball is not None:
                    position_cm = position_calibrator.position_cm(
                        ball[0],
                        calibrator.center_x
                    )
                send_ball_position(uart, position_cm)

                if ball is not None:
                    camera_cx, camera_cy, osd_cx, osd_cy, score = ball
                    pipeline.osd_img.draw_cross(
                        osd_cx,
                        osd_cy,
                        color=(0, 255, 0)
                    )
                    pipeline.osd_img.draw_string_advanced(
                        0,
                        24,
                        20,
                        "BALL: %d,%d  %.2f" %
                        (camera_cx, camera_cy, score),
                        color=(0, 255, 0)
                    )

                if calibrator.active:
                    pipeline.osd_img.draw_string_advanced(
                        0,
                        48,
                        20,
                        "CAL O: %d/%d" % (
                            calibrator.count,
                            calibrator.sample_count
                        ),
                        color=(255, 255, 0)
                    )
                elif position_calibrator.active:
                    pipeline.osd_img.draw_string_advanced(
                        0,
                        48,
                        20,
                        "CAL %s: %d/%d" % (
                            position_calibrator.mode,
                            position_calibrator.count,
                            position_calibrator.sample_count
                        ),
                        color=(255, 255, 0)
                    )
                else:
                    references_drawn = draw_position_references(
                        pipeline.osd_img,
                        actual_display_size,
                        calibrator,
                        position_calibrator
                    )
                    if not references_drawn:
                        pipeline.osd_img.draw_string_advanced(
                            0,
                            48,
                            20,
                            "CAL DATA MISSING",
                            color=(255, 255, 0)
                        )
                    elif ball is None:
                        pipeline.osd_img.draw_string_advanced(
                            0,
                            48,
                            20,
                            "ball:lost",
                            color=(0, 255, 0)
                        )
                    elif position_cm is not None:
                        position_text_x = clamp(
                            osd_cx + 12,
                            0,
                            actual_display_size[0] - 150
                        )
                        position_text_y = clamp(
                            osd_cy - 28,
                            0,
                            actual_display_size[1] - 24
                        )
                        pipeline.osd_img.draw_string_advanced(
                            position_text_x,
                            position_text_y,
                            20,
                            "ball:%+.2fcm" % position_cm,
                            color=(0, 255, 0)
                        )

                pipeline.osd_img.draw_string_advanced(
                    0,
                    0,
                    20,
                    "FPS: %.2f" % clock.fps(),
                    color=(255, 0, 0)
                )
                pipeline.show_image()

            gc.collect()

    except KeyboardInterrupt:
        print("Stopped by user")
    except Exception as error:
        print("Inference failed")
        print("Check that best.kmodel is a 320x320 YOLOv8 detect model.")
        sys.print_exception(error)
    finally:
        if detector is not None:
            detector.deinit()
        if pipeline is not None:
            pipeline.destroy()
        if uart is not None:
            try:
                uart.deinit()
            except Exception:
                pass
            uart = None
        uart_fpioa = None
        if lcd_backlight is not None:
            lcd_backlight.value(0)
            lcd_backlight = None
        gc.collect()
        print("Resources released")


main()
