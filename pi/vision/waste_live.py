#!/usr/bin/env python3
"""
JNNP RoboDojo - LIVE waste detection from the Pi Camera (Ubuntu 24.04 + libcamera).

Run on the Pi (inside ~/waste_ai, with the venv active):
    python3 waste_live.py

  * Prints what it sees, e.g.   plastic 82% CENTRE
  * Live video with boxes in the laptop browser:  http://<PI-IP>:8000
  * Ctrl+C to stop.

Needs in the same folder: best.onnx, waste_onnx.py and screen_focus.py

Speed + focus settings (below):
  * SCREEN     - the AI finds the TABLET SCREEN by itself (green outline on the video) and looks
                 ONLY at the picture on it - not the bezel, the plastic holder or the room. Tilted
                 tablets are straightened first. When no screen is seen it uses FOCUS_BOX
                 (yellow box in the middle) instead.
  * LAPTOP AI  - the Pi is slow at AI. When the laptop runs  robotai  it takes the pictures from
                 here, does the AI ~20x faster and sends the results back; the Pi then stops its
                 own AI (it says so) and simply shows the laptop's boxes. Stop robotai -> after
                 3 s the Pi does the AI again by itself.
  * ROTATE_180 - the camera is mounted upside down: turn the picture the right way up.
  * the camera runs at 10 pictures/s (not 30) and the AI always takes the NEWEST picture,
    so nothing queues up and there is no lag when the picture in front changes.
  * the AI uses all 4 Pi cores with full optimisation.
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
from screen_focus import ScreenFocus

WIDTH, HEIGHT = 1280, 960     # more detail for the screen picture (was 640 x 480)
PORT = 8000
CAMERA_FPS = 10
ROTATE_180 = True            # picture was upside down ("MADE IN CHINA" upside down)
FOCUS_BOX = (0.55, 0.65)     # width, height of the area the AI looks at (fraction of the picture);
                             # (1.0, 1.0) = whole picture

# Camera through GStreamer + libcamera, raw BGR frames written to stdout
def _autofocus_option():
    """Camera Module 3 (imx708) has AUTOFOCUS: keep it focusing all the time (sharp at the close
    tablet screen). Uses whatever name this libcamera version gives the setting; nothing if none."""
    try:
        out = subprocess.run(["gst-inspect-1.0", "libcamerasrc"], capture_output=True, text=True,
                             timeout=10).stdout
    except Exception:
        return ""
    if "auto-focus-mode" in out:
        return " auto-focus-mode=AfModeContinuous"
    if "af-mode" in out:
        return " af-mode=continuous"
    return ""


AUTOFOCUS = _autofocus_option()
CAMERA_CMD = os.environ.get("CAMERA_CMD") or (
    "gst-launch-1.0 -q libcamerasrc" + AUTOFOCUS + " ! "
    f"video/x-raw,width={WIDTH},height={HEIGHT},framerate={CAMERA_FPS}/1 ! videoconvert ! "
    "video/x-raw,format=BGR ! fdsink fd=1 sync=false"
)

latest_frame = None          # newest camera frame (camera thread writes it)
latest_jpeg = None           # newest frame WITH boxes, for the browser
latest_det = {'time': 0, 'items': []}   # newest detections, for the teleop's autonomous mode
laptop = {'time': 0.0, 'boxes': [], 'quad': None, 'fps': 0.0}   # results sent by robotai (laptop)
lock = threading.Lock()
running = True
cam_proc = None             # the camera program (so we can stop it on exit)


def camera_thread():
    """Keeps reading the camera so we always use the NEWEST frame (no delay build-up)."""
    global latest_frame, running, cam_proc
    frame_bytes = WIDTH * HEIGHT * 3
    cmd = CAMERA_CMD
    proc = cam_proc = subprocess.Popen("exec " + cmd, shell=True, stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, bufsize=frame_bytes)
    got_frame = False
    while running:
        data = proc.stdout.read(frame_bytes)
        if (not data or len(data) < frame_bytes) and not got_frame and AUTOFOCUS and AUTOFOCUS in cmd:
            print("Camera did not accept the autofocus setting - starting it without autofocus.")
            proc.kill()
            cmd = cmd.replace(AUTOFOCUS, "")
            proc = cam_proc = subprocess.Popen("exec " + cmd, shell=True, stdout=subprocess.PIPE,
                                    stderr=subprocess.DEVNULL, bufsize=frame_bytes)
            continue
        got_frame = True
        if not data or len(data) < frame_bytes:
            if running:
                print("Camera stopped sending pictures. Check the ribbon cable / that no other program uses the camera.")
            running = False
            break
        frame = np.frombuffer(data, np.uint8).reshape(HEIGHT, WIDTH, 3)
        if ROTATE_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        with lock:
            latest_frame = frame
    proc.kill()


class StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_POST(self):
        """robotai on the laptop sends its detections here."""
        if self.path != "/laptop_detections":
            self.send_error(404)
            return
        try:
            d = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
            with lock:
                laptop.update(time=time.time(), boxes=d.get("boxes", []), quad=d.get("quad"),
                              fps=d.get("fps", 0.0))
                latest_det.update(time=time.time(), items=d.get("items", []))
            self.send_response(200)
            self.end_headers()
        except Exception:
            self.send_error(400)

    def do_GET(self):
        if self.path == "/raw.jpg":                  # plain camera picture, for robotai (laptop)
            with lock:
                frame = latest_frame
            if frame is None:
                self.send_error(503)
                return
            ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(jpg.tobytes())
            return
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
    print("Camera autofocus:", "ON (continuous)" if AUTOFOCUS else "not available in this libcamera")
    print("Loading AI (best.onnx)...")
    detector = WasteDetector("best.onnx")
    try:                                             # faster: all 4 cores, full graph optimisation
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        so.intra_op_num_threads = os.cpu_count() or 4
        so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        detector.session = ort.InferenceSession("best.onnx", so, providers=["CPUExecutionProvider"])
        detector.input_name = detector.session.get_inputs()[0].name
    except Exception as e:
        print("(using default AI settings:", e, ")")
    print(f"AI input size: {detector.size} x {detector.size}"
          + ("   (tip: a 320 model is about 4x faster)" if detector.size > 320 else ""))
    screen = ScreenFocus(FOCUS_BOX)

    threading.Thread(target=camera_thread, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", PORT), StreamHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"Live video: open  http://<PI-IP>:{PORT}  in the laptop browser.  Ctrl+C to stop.")

    last_report = ""
    laptop_mode = False
    try:
        while running:
            with lock:
                frame = None if latest_frame is None else latest_frame.copy()
                lap = dict(laptop)
            if frame is None:
                time.sleep(0.05)
                continue

            if time.time() - lap['time'] < 3.0:          # the laptop (robotai) is doing the AI
                if not laptop_mode:
                    print("Laptop AI (robotai) connected - the Pi's own AI is paused.")
                    laptop_mode = True
                shown = draw(frame, [tuple(b) for b in lap['boxes']])
                if lap['quad']:
                    cv2.polylines(shown, [np.array(lap['quad'], np.int32)], True, (0, 255, 0), 2)
                cv2.putText(shown, f"LAPTOP AI  {lap['fps']:.1f} FPS", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                ok, jpg = cv2.imencode(".jpg", shown, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    with lock:
                        latest_jpeg = jpg.tobytes()
                time.sleep(0.1)
                continue
            if laptop_mode:
                print("Laptop AI stopped - the Pi does the AI again.")
                laptop_mode = False

            t = time.time()
            # find the tablet screen; the AI looks ONLY at the (straightened) picture on it,
            # and its boxes are moved back to full-picture positions
            screen.update(frame)
            crop, to_frame = screen.crop(frame)
            found = [(n, c) + to_frame(x1, y1, x2, y2) for n, c, x1, y1, x2, y2 in detector.detect(crop)]
            with lock:
                latest_det = {'time': time.time(), 'items': [
                    {'name': n, 'conf': round(c, 2), 'side': side_of(x1, x2),
                     'cx': round((x1 + x2) / 2 / WIDTH, 3),       # 0 = left edge, 1 = right edge
                     'height': round((y2 - y1) / HEIGHT, 3),      # box height / image height (bigger = closer)
                     'bottom': round(y2 / HEIGHT, 3)}
                    for n, c, x1, y1, x2, y2 in found]}
            fps = 1 / max(time.time() - t, 1e-3)
            ms = (time.time() - t) * 1000

            shown = draw(frame, found)
            cv2.line(shown, (WIDTH // 3, 0), (WIDTH // 3, HEIGHT), (128, 128, 128), 1)
            cv2.line(shown, (2 * WIDTH // 3, 0), (2 * WIDTH // 3, HEIGHT), (128, 128, 128), 1)
            screen.draw(shown)                       # green = screen found, yellow = middle box
            cv2.putText(shown, f"{fps:.1f} FPS  ({ms:.0f} ms per detection)", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
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
