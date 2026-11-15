"""
STEP 3 - AI object detection with YOLO.
Recognises 80 everyday object types (bottle, cup, bowl, banana, apple,
cell phone, book, scissors, person...). Waste-like objects get a thick box.
Run in the Pi's desktop terminal (VNC), inside vision-env. Press Q to quit.
The first run downloads the model file (about 6 MB).
"""
import time
import cv2
from picamera2 import Picamera2
from ultralytics import YOLO

MODEL = "yolo11n.pt"        # after STEP 3b use "yolo11n_ncnn_model" (faster)
CONFIDENCE = 0.4            # only show detections the AI is at least 40% sure about
WASTE = {"bottle", "cup", "wine glass", "bowl", "banana", "apple", "orange",
         "fork", "knife", "spoon", "book", "cell phone", "scissors"}

model = YOLO(MODEL)

cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"}))
cam.start()

last = time.time()
while True:
    frame = cam.capture_array()
    result = model(frame, imgsz=320, conf=CONFIDENCE, verbose=False)[0]

    for box in result.boxes:
        name = model.names[int(box.cls)]
        conf = float(box.conf)
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cx = (x1 + x2) // 2
        side = "LEFT" if cx < 213 else "RIGHT" if cx > 426 else "CENTRE"
        is_waste = name in WASTE
        colour = (0, 0, 255) if is_waste else (200, 200, 200)
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 3 if is_waste else 1)
        cv2.putText(frame, f"{name} {conf:.0%} {side if is_waste else ''}", (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)

    fps = 1 / (time.time() - last)
    last = time.time()
    cv2.putText(frame, f"{fps:.1f} FPS", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.imshow("AI detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.stop()
cv2.destroyAllWindows()
