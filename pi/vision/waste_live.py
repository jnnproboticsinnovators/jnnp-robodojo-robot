#!/usr/bin/env python3
"""
JNNP RoboDojo - LIVE waste detection from the Pi Camera (Ubuntu 24.04 + libcamera).

Run on the Pi (inside ~/waste_ai, with the venv active):
    python3 waste_live.py

  * Prints what it sees, e.g.   plastic 82% CENTRE
  * Live video with boxes in the laptop browser:  http://<PI-IP>:8000
  * Ctrl+C to stop.

Needs in the same folder: best.onnx and waste_onnx.py
"""
import os
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import json

import cv2
import numpy as np

from waste_onnx import WasteDetector, draw

WIDTH, HEIGHT = 640, 480
PORT = 8000

# Camera through GStreamer + libcamera, raw BGR frames written to stdout
CAMERA_CMD = os.environ.get("CAMERA_CMD") or (
    "gst-launch-1.0 -q libcamerasrc ! "
    f"video/x-raw,width={WIDTH},height={HEIGHT} ! videoconvert ! "
    "video/x-raw,format=BGR ! fdsink fd=1 sync=false"
)

latest_frame = None          # newest camera frame (camera thread writes it)
latest_jpeg = None           # newest frame WITH boxes, for the browser
latest_det = {'time': 0, 'items': []}   # newest detections, for the teleop's autonomous mode
lock = threading.Lock()
running = True
cam_proc = None             # the camera program (so we can stop it on exit)


def camera_thread():
    """Keeps reading the camera so we always use the NEWEST frame (no delay build-up)."""
    global latest_frame, running, cam_proc
    frame_bytes = WIDTH * HEIGHT * 3
    proc = cam_proc = subprocess.Popen("exec " + CAMERA_CMD, shell=True, stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, bufsize=frame_bytes)
    while running:
        data = proc.stdout.read(frame_bytes)
        if not data or len(data) < frame_bytes:
            if running:
                print("Camera stopped sending pictures. Check the ribbon cable / that no other program uses the camera.")
            running = False
            break
        frame = np.frombuffer(data, np.uint8).reshape(HEIGHT, WIDTH, 3)
        with lock:
            latest_frame = frame
    proc.kill()


class StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == "/detections.json":          # read by the teleop (autonomous mode)
            with lock:
                body = json.dumps(latest_det).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/":
            page = (b"<html><body style='background:#111;color:#eee;font-family:sans-serif;text-align:center'>"
                    b"<h2>JNNP RoboDojo - live waste detection</h2>"
                    b"<img src='/stream' style='width:90%;max-width:960px'></body></html>")
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(page)
            return
        if self.path != "/stream":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while running:
                with lock:
                    jpg = latest_jpeg
                if jpg is not None:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError):
            pass


def side_of(x1, x2):
    cx = (x1 + x2) / 2
    return "LEFT" if cx < WIDTH / 3 else "RIGHT" if cx > 2 * WIDTH / 3 else "CENTRE"


def main():
    global latest_jpeg, latest_det, running
    signal.signal(signal.SIGTERM, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt()))
    print("Loading AI (best.onnx)...")
    detector = WasteDetector("best.onnx")

    threading.Thread(target=camera_thread, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", PORT), StreamHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"Live video: open  http://<PI-IP>:{PORT}  in the laptop browser.  Ctrl+C to stop.")

    last_report = ""
    try:
        while running:
            with lock:
                frame = None if latest_frame is None else latest_frame.copy()
            if frame is None:
                time.sleep(0.05)
                continue

            t = time.time()
            found = detector.detect(frame)
            with lock:
                latest_det = {'time': time.time(), 'items': [
                    {'name': n, 'conf': round(c, 2), 'side': side_of(x1, x2),
                     'cx': round((x1 + x2) / 2 / WIDTH, 3),       # 0 = left edge, 1 = right edge
                     'height': round((y2 - y1) / HEIGHT, 3),      # box height / image height (bigger = closer)
                     'bottom': round(y2 / HEIGHT, 3)}
                    for n, c, x1, y1, x2, y2 in found]}
            fps = 1 / max(time.time() - t, 1e-3)

            shown = draw(frame, found)
            cv2.line(shown, (WIDTH // 3, 0), (WIDTH // 3, HEIGHT), (128, 128, 128), 1)
            cv2.line(shown, (2 * WIDTH // 3, 0), (2 * WIDTH // 3, HEIGHT), (128, 128, 128), 1)
            cv2.putText(shown, f"{fps:.1f} FPS", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            ok, jpg = cv2.imencode(".jpg", shown, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                with lock:
                    latest_jpeg = jpg.tobytes()

            report = ", ".join(f"{n} {c:.0%} {side_of(x1, x2)}" for n, c, x1, y1, x2, y2 in found) or "nothing"
            if report != last_report:
                print(time.strftime("%H:%M:%S"), "|", report)
                last_report = report
    except KeyboardInterrupt:
        pass
    running = False
    if cam_proc:
        cam_proc.kill()      # free the camera
    server.shutdown()
    print("Stopped.")


if __name__ == "__main__":
    main()
