"""K230 steel-ball detection, MJPEG streaming and per-test H.264 recording.

boot.py calls main() automatically after power-on.  Keep this file and
steel_ball.kmodel in the SD-card root.
"""

import gc
import os
import socket
import sys
import time
import uctypes
import network

from libs.Utils import hwc2chw
from libs.YOLO import YOLOv8
from media.display import *
from media.media import *
from media.sensor import *
from media.vencoder import *


# Set these two values locally before uploading the script.
WIFI_SSID = "YOUR_WIFI_SSID"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"
WIFI_CONNECT_TIMEOUT_MS = 30000
APP_VERSION = "2026-07-30-h264-recorder-v1"

# The file name must exactly match the name stored on the K230 board.
KMODEL_PATH = "/sdcard/steel_ball.kmodel"
LABELS = ["steel_ball"]
MODEL_INPUT_SIZE = [320, 320]
CONFIDENCE_THRESHOLD = 0.5
NMS_THRESHOLD = 0.5

HTTP_PORT = 8080
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480
# OV5647 direct Sensor output is negotiated at its native 640x480 mode.
# Channel 0 drives the LCD, channel 1 supplies AI/MJPEG, and channel 2 records.
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
AI_WIDTH = 640
AI_HEIGHT = 360
CROP_Y = (FRAME_HEIGHT - AI_HEIGHT) // 2
CAMERA_FPS = 30
# Browser preview is intentionally lighter than local recording. This prevents
# 2.4 GHz Wi-Fi from building a backlog of stale MJPEG frames.
STREAM_FPS = 12
STREAM_INTERVAL_MS = 1000 // STREAM_FPS
JPEG_QUALITY = 32

# Every power-on starts a new test recording in the CanMV data partition.
AUTO_RECORD_ON_BOOT = True
VIDEO_DIR = "/data/Video"
RECORD_WIDTH = 800
RECORD_HEIGHT = 480
RECORD_CHANNEL = CAM_CHN_ID_2
RECORD_VENC_CHANNEL = VENC_CHN_ID_0

STREAM_HEADER = b"HTTP/1.1 200 OK\r\nCache-Control: no-cache, no-store, must-revalidate\r\nPragma: no-cache\r\nExpires: 0\r\nConnection: close\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n"
FRAME_HEADER = b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"


def ensure_video_dir():
    try:
        os.mkdir(VIDEO_DIR)
    except OSError:
        pass


def list_recordings():
    ensure_video_dir()
    names = []
    try:
        for name in os.listdir(VIDEO_DIR):
            if name.startswith("test_") and name.endswith(".h264"):
                names.append(name)
    except OSError:
        return []
    names.sort(reverse=True)
    return names


def next_recording_path():
    used = {}
    for name in list_recordings():
        used[name] = True
    index = 1
    while True:
        name = "test_%03d.h264" % index
        if name not in used:
            return VIDEO_DIR + "/" + name
        index += 1


class VideoRecorder:
    """Main-thread raw H.264 recorder; avoids unstable MP4 muxer APIs."""

    def __init__(self, sensor):
        self.encoder = Encoder()
        self.encoder.SetOutBufs(
            RECORD_VENC_CHANNEL,
            8,
            RECORD_WIDTH,
            RECORD_HEIGHT,
        )
        source = sensor.bind_info(chn=RECORD_CHANNEL)["src"]
        target = (
            VIDEO_ENCODE_MOD_ID,
            VENC_DEV_ID,
            RECORD_VENC_CHANNEL,
        )
        self.link = MediaManager.link(source, target)
        self.recording = False
        self.stop_requested = False
        self.filename = ""
        self.temp_filename = ""
        self.last_saved_filename = ""
        self.last_error = ""
        self.engine_started = False
        self.file = None
        self.h264_header = b""
        self.first_idr_found = False

    def start(self):
        if self.recording:
            return False
        self.filename = next_recording_path()
        self.temp_filename = self.filename + ".part"
        self.last_error = ""
        self.stop_requested = False
        try:
            self._ensure_encoder()
            self.file = open(self.temp_filename, "wb")
            self.first_idr_found = False
            self.recording = True
            print("Recording started:", self.filename)
        except Exception as error:
            self.recording = False
            self.last_error = str(error)
            self._discard_partial()
            raise
        return True

    def stop(self, wait=True):
        print("Recording stop requested:", self.filename)
        self.stop_requested = True
        if wait:
            started_at = time.ticks_ms()
            while self.recording:
                if time.ticks_diff(time.ticks_ms(), started_at) > 6000:
                    break
                time.sleep_ms(50)

    def release(self):
        self.stop(wait=True)
        if self.recording:
            self._finish_recording()
        if self.engine_started:
            self.encoder.Stop(RECORD_VENC_CHANNEL)
            self.encoder.Destroy(RECORD_VENC_CHANNEL)
            self.engine_started = False
        self.link = None

    def _ensure_encoder(self):
        if not self.engine_started:
            channel_attr = ChnAttrStr(
                self.encoder.PAYLOAD_TYPE_H264,
                self.encoder.H264_PROFILE_MAIN,
                RECORD_WIDTH,
                RECORD_HEIGHT,
            )
            self.encoder.Create(RECORD_VENC_CHANNEL, channel_attr)
            self.encoder.Start(RECORD_VENC_CHANNEL)
            self.engine_started = True
            print("Recorder encoder kept running between tests")

    def _discard_partial(self):
        try:
            os.remove(self.temp_filename)
        except:
            pass

    def _finish_recording(self):
        print("Finalizing recording:", self.filename)
        try:
            if self.file is not None:
                self.file.close()
                self.file = None
            if self.first_idr_found and not self.last_error:
                try:
                    os.rename(self.temp_filename, self.filename)
                    self.last_saved_filename = self.filename
                    print("Recording saved:", self.filename)
                except Exception as error:
                    self.last_error = str(error)
                    try:
                        os.remove(self.temp_filename)
                    except:
                        pass
                    print("Recording discarded:", self.filename)
            else:
                # A partial MP4 has no reliable playback index. Do not leave
                # it in the list as if it were a completed test recording.
                self._discard_partial()
                print("Recording discarded:", self.filename)
        except BaseException as error:
            self.last_error = str(error)
            print("Recording finalization error:")
            sys.print_exception(error)
        self.recording = False
        self.stop_requested = False
        gc.collect()

    def poll(self):
        """Drain H.264 in the main loop and write it directly to a file."""
        if not self.engine_started:
            return
        stream_data = StreamData()
        for _ in range(4):
            result = self.encoder.GetStream(
                RECORD_VENC_CHANNEL,
                stream_data,
                timeout=0,
            )
            if result != 0:
                break
            try:
                for pack_index in range(stream_data.pack_cnt):
                    if stream_data.stream_type[pack_index] == self.encoder.STREAM_TYPE_HEADER:
                        self.h264_header = bytes(uctypes.bytearray_at(
                            stream_data.data[pack_index],
                            stream_data.data_size[pack_index],
                        ))

                if self.recording:
                    if not self.first_idr_found:
                        for pack_index in range(stream_data.pack_cnt):
                            stream_type = stream_data.stream_type[pack_index]
                            data_size = stream_data.data_size[pack_index]
                            if stream_type == self.encoder.STREAM_TYPE_HEADER:
                                self.file.write(uctypes.bytearray_at(
                                    stream_data.data[pack_index], data_size
                                ))
                            elif stream_type == self.encoder.STREAM_TYPE_I:
                                if self.h264_header:
                                    self.file.write(self.h264_header)
                                self.file.write(uctypes.bytearray_at(
                                    stream_data.data[pack_index], data_size
                                ))
                                self.first_idr_found = True
                                break
                    else:
                        for pack_index in range(stream_data.pack_cnt):
                            self.file.write(uctypes.bytearray_at(
                                stream_data.data[pack_index],
                                stream_data.data_size[pack_index],
                            ))
            except BaseException as error:
                self.last_error = str(error)
                print("Recording write error:")
                sys.print_exception(error)
                self.stop_requested = True
            finally:
                self.encoder.ReleaseStream(RECORD_VENC_CHANNEL, stream_data)

            if self.recording and self.stop_requested:
                self._finish_recording()
                break


def send_http(client, status, content_type, body, extra_headers=b""):
    header = (
        b"HTTP/1.1 " + status + b"\r\n"
        b"Content-Type: " + content_type + b"\r\n"
        b"Content-Length: %d\r\n" % len(body) +
        extra_headers +
        b"Connection: close\r\n\r\n"
    )
    client.write(header)
    if body:
        client.write(body)


def redirect_control(client):
    send_http(
        client,
        b"303 See Other",
        b"text/plain",
        b"",
        b"Location: /control\r\n",
    )


def request_path_and_query(request):
    try:
        target = request.split(b" ", 2)[1].decode()
    except:
        return "/", ""
    if "?" in target:
        return target.split("?", 1)
    return target, ""


def safe_recording_name(name):
    if not name:
        return None
    if "/" in name or "\\" in name or ".." in name:
        return None
    if not name.startswith("test_") or not name.endswith(".h264"):
        return None
    return name


def query_name(query):
    for item in query.split("&"):
        if item.startswith("name="):
            return safe_recording_name(item[5:])
    return None


def control_page(recorder, play_name=None):
    if recorder.recording:
        state = "RECORDING: " + recorder.filename.split("/")[-1]
    elif recorder.last_error:
        state = "ERROR: " + recorder.last_error
    else:
        state = "STOPPED"

    active_name = ""
    if recorder.recording:
        active_name = recorder.filename.split("/")[-1]

    rows = []
    for name in list_recordings():
        if name == active_name:
            continue
        rows.append(
            '<li>%s <a href="/video?name=%s" download>'
            'download (play in VLC)</a></li>' % (name, name)
        )
    if not rows:
        rows.append("<li>No completed recordings</li>")

    player = ""
    if play_name and play_name == active_name:
        player = (
            "<p><b>This recording is still being written. "
            "Click Stop and save before playback.</b></p>"
        )
    elif play_name:
        player = (
            '<p><b>%s is raw H.264.</b> Download it and play it with VLC, '
            'or convert it to MP4 on the PC.</p>' % play_name
        )

    html = """<!doctype html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>K230 test recording</title>
<style>body{font-family:sans-serif;max-width:900px;margin:auto;padding:16px}
a.button{display:inline-block;padding:12px;margin:4px;background:#1769aa;color:white;text-decoration:none}
.state{font-weight:bold;padding:10px;background:#eee}li{margin:10px}</style>
</head><body><h2>K230 steel-ball test recording</h2>
<div class="state">%s</div>
<p><a class="button" href="/record/start">Start new test</a>
<a class="button" href="/record/stop">Stop and save</a></p>
<p>Raw H.264 videos are stored in /data/Video. Download and play with VLC.</p>
%s<h3>Saved tests</h3><ul>%s</ul>
<p><a href="http://%s:%d/">Back to live detection</a></p>
</body></html>""" % (
        state,
        player,
        "".join(rows),
        CONTROL_SERVER_IP,
        HTTP_PORT,
    )
    return html.encode()


def parse_range(request, file_size):
    start = 0
    end = file_size - 1
    marker = b"Range: bytes="
    position = request.find(marker)
    if position < 0:
        marker = b"range: bytes="
        position = request.find(marker)
    if position < 0:
        return start, end, False
    line = request[position + len(marker):].split(b"\r\n", 1)[0]
    try:
        parts = line.split(b"-", 1)
        if parts[0]:
            start = int(parts[0])
        if len(parts) > 1 and parts[1]:
            end = int(parts[1])
        if start < 0 or start >= file_size:
            return None, None, True
        if end >= file_size:
            end = file_size - 1
        if end < start:
            return None, None, True
    except:
        return None, None, True
    return start, end, True


def send_video(client, request, name, recorder):
    if not name:
        send_http(client, b"400 Bad Request", b"text/plain", b"Bad name")
        return
    path = VIDEO_DIR + "/" + name
    if recorder.recording and path == recorder.filename:
        send_http(
            client, b"409 Conflict", b"text/plain",
            b"Stop the recording before playback.",
        )
        return
    try:
        file_size = os.stat(path)[6]
        start, end, ranged = parse_range(request, file_size)
        if start is None:
            send_http(
                client, b"416 Range Not Satisfiable", b"text/plain", b"",
                b"Content-Range: bytes */%d\r\n" % file_size,
            )
            return
        length = end - start + 1
        if ranged:
            status = b"206 Partial Content"
            range_header = (
                b"Content-Range: bytes %d-%d/%d\r\n"
                % (start, end, file_size)
            )
        else:
            status = b"200 OK"
            range_header = b""
        header = (
            b"HTTP/1.1 " + status + b"\r\n"
            b"Content-Type: video/h264\r\n"
            b"Content-Disposition: attachment; filename=\"" +
            name.encode() + b"\"\r\n"
            b"Accept-Ranges: bytes\r\n"
            b"Content-Length: %d\r\n" % length +
            range_header +
            b"Connection: close\r\n\r\n"
        )
        client.write(header)
        with open(path, "rb") as video:
            video.seek(start)
            remaining = length
            while remaining:
                chunk = video.read(min(8192, remaining))
                if not chunk:
                    break
                client.write(chunk)
                remaining -= len(chunk)
    except OSError:
        send_http(client, b"404 Not Found", b"text/plain", b"Not found")


CONTROL_SERVER_IP = "0.0.0.0"


def live_index_body(ip):
    html = """<!doctype html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>K230 steel ball detection</title>
<style>body{margin:0;background:#111;color:#ddd;text-align:center;font-family:sans-serif}
img{max-width:100%%;height:auto}a{color:#6cf;font-size:18px}</style>
</head><body><h3>K230 steel-ball detection</h3><img src="/stream">
<p><a target="_blank" href="/control">Recording control and playback</a></p>
</body></html>"""
    return html.encode()


def connect_wifi():
    if WIFI_SSID == "YOUR_WIFI_SSID":
        raise RuntimeError("Set WIFI_SSID and WIFI_PASSWORD before running")

    sta = network.WLAN(network.STA_IF)
    sta.active(True)

    while True:
        if sta.isconnected() and sta.ifconfig()[0] != "0.0.0.0":
            print("Wi-Fi connected:", sta.ifconfig())
            return sta.ifconfig()[0]

        print("Connecting to Wi-Fi...")
        sta.connect(WIFI_SSID, WIFI_PASSWORD)
        started_at = time.ticks_ms()
        while not sta.isconnected() or sta.ifconfig()[0] == "0.0.0.0":
            os.exitpoint()
            if (
                time.ticks_diff(time.ticks_ms(), started_at)
                >= WIFI_CONNECT_TIMEOUT_MS
            ):
                print("Wi-Fi timeout; recording continues, retrying...")
                break
            time.sleep_ms(300)
        time.sleep_ms(1000)


def start_camera():
    sensor = None
    recorder = None
    display_started = False
    media_started = False
    try:
        sensor = Sensor(
            width=DISPLAY_WIDTH,
            height=DISPLAY_HEIGHT,
            fps=CAMERA_FPS,
        )
        sensor.reset()

        # Channel 0 is bound directly to the physical LCD.  This preview keeps
        # running even when no browser is connected to the MJPEG server.
        sensor.set_framesize(
            width=DISPLAY_WIDTH,
            height=DISPLAY_HEIGHT,
            chn=CAM_CHN_ID_0,
        )
        sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)
        bind_info = sensor.bind_info(chn=CAM_CHN_ID_0)
        Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)

        # Channel 1 remains RGB888 for YOLO drawing and JPEG compression.
        sensor.set_framesize(
            width=FRAME_WIDTH,
            height=FRAME_HEIGHT,
            chn=CAM_CHN_ID_1,
        )
        sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_1)

        # Channel 2 feeds the hardware H.264 encoder.  Recording therefore
        # does not depend on the browser stream and continues if Wi-Fi drops.
        sensor.set_framesize(
            width=RECORD_WIDTH,
            height=RECORD_HEIGHT,
            alignment=12,
            chn=RECORD_CHANNEL,
        )
        sensor.set_pixformat(Sensor.YUV420SP, chn=RECORD_CHANNEL)
        recorder = VideoRecorder(sensor)

        Display.init(
            Display.ST7701,
            width=DISPLAY_WIDTH,
            height=DISPLAY_HEIGHT,
            to_ide=False,
        )
        display_started = True
        MediaManager.init()
        media_started = True
        sensor.run()
        print("ST7701 camera preview: %dx%d" % (
            DISPLAY_WIDTH,
            DISPLAY_HEIGHT,
        ))
        return sensor, recorder
    except:
        if recorder is not None:
            recorder.release()
        if sensor is not None:
            try:
                sensor.stop()
            except:
                pass
        if display_started:
            Display.deinit()
        if media_started:
            MediaManager.deinit()
        raise


def start_yolo():
    yolo = YOLOv8(
        task_type="detect",
        mode="image",
        kmodel_path=KMODEL_PATH,
        labels=LABELS,
        rgb888p_size=[AI_WIDTH, AI_HEIGHT],
        model_input_size=MODEL_INPUT_SIZE,
        conf_thresh=CONFIDENCE_THRESHOLD,
        nms_thresh=NMS_THRESHOLD,
        max_boxes_num=50,
        debug_mode=0,
    )
    yolo.config_preprocess()
    print("YOLOv8 initialized:", KMODEL_PATH)
    return yolo


def send_mjpeg_frame(client, sensor, yolo):
    """Send one frame, then return so HTTP controls stay responsive."""
    img_full = sensor.snapshot(chn=CAM_CHN_ID_1)
    # Reproduce the 16:9 framing used by the original PipeLine-based detector.
    img = img_full.copy((0, CROP_Y, AI_WIDTH, AI_HEIGHT))
    img_np_chw = hwc2chw(img.to_numpy_ref())
    detections = yolo.run(img_np_chw)
    yolo.draw_result(detections, img)
    jpg = img.compressed(JPEG_QUALITY)

    client.write(FRAME_HEADER % jpg.size())
    client.write(jpg)
    client.write(b"\r\n")

    del jpg
    del detections
    del img_np_chw
    del img
    del img_full
    gc.collect()


def read_http_request(client):
    request = b""
    empty_retries = 0
    while b"\r\n\r\n" not in request:
        os.exitpoint()
        try:
            chunk = client.recv(512)
            if not chunk:
                return b""
            request += chunk
            empty_retries = 0
        except OSError as error:
            if error.args[0] == 11:
                empty_retries += 1
                if empty_retries >= 300:
                    return b""
                time.sleep_ms(10)
                continue
            raise
    return request


def serve(sensor, yolo, recorder, ip):
    global CONTROL_SERVER_IP
    CONTROL_SERVER_IP = ip
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.setblocking(False)
    server.bind(socket.getaddrinfo("0.0.0.0", HTTP_PORT)[0][-1])
    server.listen(5)
    print("Open http://%s:%d/" % (ip, HTTP_PORT))
    print("Recording control: http://%s:%d/control" % (ip, HTTP_PORT))

    stream_client = None
    last_stream_send_ms = 0

    try:
        while True:
            os.exitpoint()
            recorder.poll()
            client = None
            try:
                client, remote = server.accept()
                client.setblocking(False)
                print("HTTP client connected:", remote)
                request = read_http_request(client)
                client.setblocking(True)
                print("HTTP request:", request[:64])
                if request.startswith(b"GET /stream "):
                    if stream_client is not None:
                        try:
                            stream_client.close()
                        except:
                            pass
                    client.write(STREAM_HEADER)
                    stream_client = client
                    client = None
                    last_stream_send_ms = 0
                    print("MJPEG detection client connected")
                elif request.startswith(b"GET /record/start "):
                    recorder.start()
                    redirect_control(client)
                elif request.startswith(b"GET /record/stop "):
                    # Finalization runs in the recorder thread.  Do not block
                    # the HTTP/MJPEG loop while the MP4 container is closed.
                    recorder.stop(wait=False)
                    redirect_control(client)
                elif request.startswith(b"GET /video"):
                    path, query = request_path_and_query(request)
                    send_video(client, request, query_name(query), recorder)
                elif request.startswith(b"GET /play"):
                    path, query = request_path_and_query(request)
                    body = control_page(recorder, query_name(query))
                    send_http(
                        client,
                        b"200 OK",
                        b"text/html; charset=utf-8",
                        body,
                    )
                elif request.startswith(b"GET /control"):
                    body = control_page(recorder)
                    send_http(
                        client,
                        b"200 OK",
                        b"text/html; charset=utf-8",
                        body,
                    )
                elif request:
                    body = live_index_body(ip)
                    send_http(
                        client,
                        b"200 OK",
                        b"text/html; charset=utf-8",
                        body,
                    )
            except OSError as error:
                if error.args[0] != 11:
                    print("HTTP client error:", error)
            except Exception as error:
                # A malformed control request or a recorder error must not
                # end the autonomous camera and recording program.
                print("HTTP request handler error:")
                sys.print_exception(error)
            finally:
                if client is not None:
                    client.close()

            if stream_client is not None:
                try:
                    now_ms = time.ticks_ms()
                    if (
                        not last_stream_send_ms
                        or time.ticks_diff(now_ms, last_stream_send_ms)
                        >= STREAM_INTERVAL_MS
                    ):
                        send_mjpeg_frame(stream_client, sensor, yolo)
                        last_stream_send_ms = now_ms
                    else:
                        time.sleep_ms(1)
                except OSError as error:
                    print("MJPEG client disconnected:", error)
                    try:
                        stream_client.close()
                    except:
                        pass
                    stream_client = None
            else:
                time.sleep_ms(10)
            gc.collect()
    finally:
        if stream_client is not None:
            stream_client.close()
        server.close()


def main():
    sensor = None
    yolo = None
    recorder = None
    media_started = False
    try:
        print("K230 stream app:", APP_VERSION)
        sensor, recorder = start_camera()
        media_started = True
        if AUTO_RECORD_ON_BOOT:
            recorder.start()
        yolo = start_yolo()
        # Recording starts before Wi-Fi connection, so a missing hotspot cannot
        # prevent a complete local record of the test.
        ip = connect_wifi()
        serve(sensor, yolo, recorder, ip)
    finally:
        if recorder is not None:
            recorder.release()
        if yolo is not None:
            yolo.deinit()
        if sensor is not None:
            sensor.stop()
        if media_started:
            Display.deinit()
            time.sleep_ms(100)
            MediaManager.deinit()
        gc.collect()


if __name__ == "__main__":
    main()
