"""CanMV K230 MJPEG HTTP server.

Configure WIFI_SSID and WIFI_PASSWORD, upload this file to the board, then run it.
Open http://<printed-ip>:8080/ from a device on the same Wi-Fi network.
"""

import gc
import os
import socket
import time
import network

from media.media import *
from media.sensor import *


# --- User configuration ----------------------------------------------------
WIFI_SSID = "YOUR_WIFI_SSID"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"

HTTP_PORT = 8080
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_FPS = 30
JPEG_QUALITY = 45


INDEX_BODY = b"""<!doctype html><html><head><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">
<title>K230 MJPEG</title><style>body{margin:0;background:#111;color:#ddd;text-align:center;font-family:sans-serif}img{max-width:100%;height:auto}</style>
</head><body><h3>K230 camera</h3><img src=\"/stream\"></body></html>"""

INDEX_HEADER = b"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: %d\r\nConnection: close\r\n\r\n" % len(INDEX_BODY)

STREAM_HEADER = b"""HTTP/1.1 200 OK\r
Cache-Control: no-cache, no-store, must-revalidate\r
Pragma: no-cache\r
Expires: 0\r
Connection: close\r
Content-Type: multipart/x-mixed-replace; boundary=frame\r
\r
"""

FRAME_HEADER = b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"


def connect_wifi():
    if WIFI_SSID == "YOUR_WIFI_SSID":
        raise RuntimeError("Set WIFI_SSID and WIFI_PASSWORD before running")

    sta = network.WLAN(0)
    sta.connect(WIFI_SSID, WIFI_PASSWORD)

    print("Connecting to Wi-Fi...")
    while sta.ifconfig()[0] == "0.0.0.0":
        os.exitpoint()
        time.sleep_ms(300)

    ip = sta.ifconfig()[0]
    print("Wi-Fi connected:", sta.ifconfig())
    return ip


def start_camera():
    MediaManager.init()
    try:
        sensor = Sensor(width=FRAME_WIDTH, height=FRAME_HEIGHT, fps=CAMERA_FPS)
        sensor.reset()
        sensor.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT)
        sensor.set_pixformat(Sensor.RGB565)
        sensor.run()
        return sensor
    except:
        MediaManager.deinit()
        raise


def send_mjpeg(client, sensor):
    client.write(STREAM_HEADER)
    print("MJPEG client connected")

    while True:
        os.exitpoint()
        img = sensor.snapshot()
        jpg = img.compressed(JPEG_QUALITY)
        client.write(FRAME_HEADER % jpg.size())
        client.write(jpg)
        client.write(b"\r\n")
        del jpg
        del img
        gc.collect()


def read_http_request(client):
    """Wait for the browser's HTTP request instead of treating EAGAIN as disconnect."""
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
            if error.args[0] == 11:  # EAGAIN: no data has arrived yet.
                empty_retries += 1
                if empty_retries >= 300:  # Ignore an idle browser pre-connect after 3 seconds.
                    return b""
                time.sleep_ms(10)
                continue
            raise
    return request


def serve(sensor, ip):
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.setblocking(True)
    address = socket.getaddrinfo("0.0.0.0", HTTP_PORT)[0][-1]
    server.bind(address)
    server.listen(1)
    print("Open http://%s:%d/" % (ip, HTTP_PORT))

    try:
        while True:
            os.exitpoint()
            client = None
            try:
                client, remote = server.accept()
                client.setblocking(False)
                print("HTTP client connected:", remote)
                request = read_http_request(client)
                client.setblocking(True)
                print("HTTP request:", request[:64])
                if request.startswith(b"GET /stream "):
                    send_mjpeg(client, sensor)
                elif request:
                    client.write(INDEX_HEADER)
                    client.write(INDEX_BODY)
            except OSError as error:
                # A browser normally closes the MJPEG connection when the page is left.
                if error.args[0] != 11:
                    print("Client disconnected:", error)
            finally:
                if client is not None:
                    client.close()
                gc.collect()
    finally:
        server.close()


def main():
    sensor = None
    media_started = False
    try:
        ip = connect_wifi()
        sensor = start_camera()
        media_started = True
        serve(sensor, ip)

    finally:
        if sensor is not None:
            sensor.stop()
        if media_started:
            MediaManager.deinit()
        gc.collect()


if __name__ == "__main__":
    main()
