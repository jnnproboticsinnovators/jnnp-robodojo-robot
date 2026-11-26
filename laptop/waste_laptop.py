#!/usr/bin/env python3
"""
JNNP RoboDojo - FAST waste detection: the LAPTOP does the AI on the Pi camera's pictures.

    robotai          (laptop, after pressing C in robotdrive so the camera is ON)

Why: the Pi needs many seconds per picture for the AI (0.1 FPS). The laptop does the same AI
about 20x faster. This program:
  1. takes the newest picture from the Pi camera (http://<pi>:8000/raw.jpg)
  2. finds the TABLET SCREEN (green outline) and cuts out ONLY the picture on it -
     no bezel, no plastic holder, no room
  3. runs best.onnx on it
  4. shows a window with the boxes, and sends the results back to the Pi (the Pi's browser page
     and the robot's autonomous mode use them; the Pi pauses its own slow AI meanwhile)
  q in the window or Ctrl+C = stop (the Pi takes over again after 3 s).

Needs in your home folder (one-time copy, see the instructions): best.onnx, waste_onnx.py,
screen_focus.py, and  pip3 install --break-system-packages onnxruntime
"""
import json
import os
import sys
import time
import urllib.request

import cv2
import numpy as np

sys.path.insert(0, os.path.expanduser("~"))
from screen_focus import ScreenFocus          # noqa: E402
import waste_onnx                             # noqa: E402
from waste_onnx import WasteDetector, draw    # noqa: E402

MODEL = os.path.expanduser("~/best.onnx")
SHOW_CONF = 0.35     # boxes from 35% sure (blurry screen pictures score lower than real photos;
                     # the 2-of-3 vote below keeps wrong flickers away). Raise to 0.5 if it guesses.
VOTES = 3            # an item is reported once seen in 2 of the last 3 pictures (no flicker)


def side_of(x1, x2, w):
    cx = (x1 + x2) / 2
    return "LEFT" if cx < w / 3 else "RIGHT" if cx > 2 * w / 3 else "CENTRE"


def main():
    host = os.environ.get("PI", "").split("@")[-1]
    if len(sys.argv) > 1:
        host = sys.argv[1].split("@")[-1]
    if not host:
        sys.exit("Pi address unknown - run it with the word  robotai")
    base = f"http://{host}:8000"

    waste_onnx.CONFIDENCE = 0.10      # get even unsure answers; SHOW_CONF decides what is drawn
    detector = WasteDetector(MODEL)
    try:
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        detector.session = ort.InferenceSession(MODEL, so, providers=["CPUExecutionProvider"])
        detector.input_name = detector.session.get_inputs()[0].name
    except Exception:
        pass
    screen = ScreenFocus()
    history = []
    last_report, waiting_msg = "", False
    fps = 0.0
    print(f"Laptop AI on the Pi camera ({base}).  q in the window or Ctrl+C to stop.")
    try:
        while True:
            t0 = time.time()
            try:
                with urllib.request.urlopen(base + "/raw.jpg", timeout=3) as r:
                    frame = cv2.imdecode(np.frombuffer(r.read(), np.uint8), cv2.IMREAD_COLOR)
                waiting_msg = False
            except Exception as e:
                wait_img = np.zeros((480, 640, 3), np.uint8)
                cv2.putText(wait_img, "Waiting for the robot camera...", (90, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.imshow("RoboDojo waste AI (laptop)  -  q = quit", wait_img)
                if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q")):
                    break
                if not waiting_msg:
                    print(f"No camera picture from the Pi ({e}).\n"
                          "  -> In robotdrive press C (camera ON) and wait ~10 s.\n"
                          "  -> If it says 404: the new waste_live.py is not on the Pi yet.")
                    waiting_msg = True
                time.sleep(0.5)
                continue
            if frame is None:
                continue
            h, w = frame.shape[:2]

            screen.update(frame)
            crop, to_frame = screen.crop(frame)
            all_found = [(n, c) + to_frame(x1, y1, x2, y2) for n, c, x1, y1, x2, y2 in detector.detect(crop)]
            found = [f for f in all_found if f[1] >= SHOW_CONF]
            guess = max(all_found, key=lambda f: f[1]) if all_found else None

            history = (history + [{n for n, *_ in found}])[-VOTES:]
            steady = [f for f in found if sum(f[0] in s for s in history) >= min(2, len(history))]

            dt = time.time() - t0
            fps = 0.7 * fps + 0.3 / max(dt, 1e-3) if fps else 1 / max(dt, 1e-3)
            items = [{'name': n, 'conf': round(c, 2), 'side': side_of(x1, x2, w),
                      'cx': round((x1 + x2) / 2 / w, 3), 'height': round((y2 - y1) / h, 3),
                      'bottom': round(y2 / h, 3)} for n, c, x1, y1, x2, y2 in steady]
            body = json.dumps({'items': items, 'boxes': [list(f) for f in steady], 'fps': round(fps, 1),
                               'quad': screen.quad.tolist() if screen.found() else None}).encode()
            try:
                urllib.request.urlopen(urllib.request.Request(
                    base + "/laptop_detections", data=body,
                    headers={"Content-Type": "application/json"}), timeout=2).close()
            except Exception:
                pass

            shown = draw(screen.draw(frame.copy()), steady)
            # what the AI THINKS even when it is not sure enough to draw a box
            gtxt = (f"AI best guess: {guess[0]} {guess[1]:.0%}" if guess else "AI best guess: nothing (below 10%)")
            cv2.putText(shown, gtxt, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(shown, gtxt, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(shown, f"LAPTOP AI {fps:.1f} FPS", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2)
            if screen.found():                          # small picture: exactly what the AI sees
                ph = shown.shape[0] // 4
                pw = max(1, int(crop.shape[1] * ph / max(crop.shape[0], 1)))
                if pw < shown.shape[1] // 2:
                    shown[-ph - 10:-10, -pw - 10:-10] = cv2.resize(crop, (pw, ph))
                    cv2.rectangle(shown, (shown.shape[1] - pw - 11, shown.shape[0] - ph - 11),
                                  (shown.shape[1] - 9, shown.shape[0] - 9), (0, 255, 0), 1)
            cv2.imshow("RoboDojo waste AI (laptop)  -  q = quit", shown)
            if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q")):
                break

            report = ", ".join(f"{n} {c:.0%} {side_of(x1, x2, w)}" for n, c, x1, y1, x2, y2 in steady) \
                or "nothing"
            if report != last_report:
                print(time.strftime("%H:%M:%S"), "|", report, flush=True)
                last_report = report
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    print("Stopped (the Pi does the AI again in 3 s).")


if __name__ == "__main__":
    main()
