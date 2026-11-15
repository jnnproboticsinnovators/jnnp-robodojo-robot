"""
RDJ2026 waste detection on the Raspberry Pi camera.
Uses YOUR trained model (best.pt from Colab): classes metal, organic, plastic.
Run in the Pi's desktop terminal (VNC), inside vision-env. Press Q to quit.
"""
import time
import cv2
from picamera2 import Picamera2
from ultralytics import YOLO

MODEL = "best.pt"          # after exporting to NCNN use "best_ncnn_model" (much faster)
IMG_SIZE = 416             # trained at 640; 416 is a speed/accuracy balance on the Pi
CONFIDENCE = 0.4           # only show detections the AI is at least 40% sure about

# one colour per waste type (OpenCV uses Blue, Green, Red order)
COLOURS = {"metal": (255, 200, 0), "organic": (0, 200, 0), "plastic": (0, 0, 255)}

model = YOLO(MODEL, task="detect")
print("Model knows these classes:", model.names)

cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"}))
cam.start()

last = time.time()
while True:
    frame = cam.capture_array()
    result = model(frame, imgsz=IMG_SIZE, conf=CONFIDENCE, verbose=False)[0]

    counts = {}
    for box in result.boxes:
        name = model.names[int(box.cls)]
        conf = float(box.conf)
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cx = (x1 + x2) // 2
        side = "LEFT" if cx < 213 else "RIGHT" if cx > 426 else "CENTRE"
        colour = COLOURS.get(name, (255, 255, 255))
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 3)
        cv2.putText(frame, f"{name} {conf:.0%} {side}", (x1, max(y1 - 8, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
        counts[name] = counts.get(name, 0) + 1

    fps = 1 / (time.time() - last)
    last = time.time()
    summary = "  ".join(f"{k}: {v}" for k, v in counts.items()) or "no waste seen"
    cv2.putText(frame, f"{fps:.1f} FPS | {summary}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.line(frame, (213, 0), (213, 480), (128, 128, 128), 1)
    cv2.line(frame, (426, 0), (426, 480), (128, 128, 128), 1)
    cv2.imshow("RDJ2026 waste detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.stop()
cv2.destroyAllWindows()
