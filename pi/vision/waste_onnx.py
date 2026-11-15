#!/usr/bin/env python3
"""
JNNP RoboDojo - waste detection with best.onnx (no PyTorch / Ultralytics needed).
Works on the Raspberry Pi (Ubuntu 24.04) and on the laptop.

Test on a photo:
    python3 waste_onnx.py photo.jpg
It prints what it found and saves  photo_result.jpg  with boxes drawn.

Classes (same order as training): 0 metal, 1 organic, 2 plastic
"""
import sys
import time
import cv2
import numpy as np
import onnxruntime as ort

CLASSES = ["metal", "organic", "plastic"]
COLOURS = {"metal": (255, 200, 0), "organic": (0, 200, 0), "plastic": (0, 0, 255)}  # BGR
CONFIDENCE = 0.40   # only keep detections at least 40% sure
NMS_IOU = 0.45      # remove duplicate boxes on the same object


class WasteDetector:
    def __init__(self, model_path="best.onnx"):
        self.session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        shape = self.session.get_inputs()[0].shape          # [1, 3, 640, 640]
        self.size = shape[2] if isinstance(shape[2], int) else 640

    def _letterbox(self, img):
        """Resize keeping proportions, pad to a square (what YOLO was trained on)."""
        h, w = img.shape[:2]
        scale = min(self.size / h, self.size / w)
        nh, nw = int(round(h * scale)), int(round(w * scale))
        resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        top = (self.size - nh) // 2
        left = (self.size - nw) // 2
        canvas = np.full((self.size, self.size, 3), 114, dtype=np.uint8)
        canvas[top:top + nh, left:left + nw] = resized
        return canvas, scale, left, top

    def detect(self, frame_bgr):
        """Returns a list of (name, confidence, x1, y1, x2, y2) in original image pixels."""
        h, w = frame_bgr.shape[:2]
        img, scale, padx, pady = self._letterbox(frame_bgr)
        blob = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        blob = np.ascontiguousarray(blob.transpose(2, 0, 1)[None])   # 1x3xSxS

        out = self.session.run(None, {self.input_name: blob})[0]     # 1 x (4+nc) x N
        out = out[0].T                                                # N x (4+nc)
        boxes_xywh = out[:, :4]
        scores_all = out[:, 4:]
        class_ids = scores_all.argmax(axis=1)
        scores = scores_all.max(axis=1)

        keep = scores >= CONFIDENCE
        boxes_xywh, scores, class_ids = boxes_xywh[keep], scores[keep], class_ids[keep]
        if len(scores) == 0:
            return []

        # centre-x, centre-y, width, height  ->  x, y, width, height (top-left) in original image
        cx, cy, bw, bh = boxes_xywh.T
        x1 = (cx - bw / 2 - padx) / scale
        y1 = (cy - bh / 2 - pady) / scale
        bw, bh = bw / scale, bh / scale
        rects = np.stack([x1, y1, bw, bh], axis=1)

        idx = cv2.dnn.NMSBoxes(rects.tolist(), scores.tolist(), CONFIDENCE, NMS_IOU)
        results = []
        for i in np.array(idx).flatten():
            x, y, bw_, bh_ = rects[i]
            results.append((CLASSES[int(class_ids[i])] if int(class_ids[i]) < len(CLASSES) else str(class_ids[i]),
                            float(scores[i]),
                            int(max(0, x)), int(max(0, y)),
                            int(min(w - 1, x + bw_)), int(min(h - 1, y + bh_))))
        return results


def draw(frame, detections):
    for name, conf, x1, y1, x2, y2 in detections:
        colour = COLOURS.get(name, (255, 255, 255))
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 3)
        cv2.putText(frame, f"{name} {conf:.0%}", (x1, max(y1 - 8, 18)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)
    return frame


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 waste_onnx.py photo.jpg [best.onnx]")
        sys.exit(1)
    photo = sys.argv[1]
    model = sys.argv[2] if len(sys.argv) > 2 else "best.onnx"

    detector = WasteDetector(model)
    frame = cv2.imread(photo)
    if frame is None:
        print(f"Could not open {photo}")
        sys.exit(1)

    t = time.time()
    found = detector.detect(frame)
    ms = (time.time() - t) * 1000

    if found:
        for name, conf, x1, y1, x2, y2 in found:
            side = "LEFT" if (x1 + x2) / 2 < frame.shape[1] / 3 else \
                   "RIGHT" if (x1 + x2) / 2 > 2 * frame.shape[1] / 3 else "CENTRE"
            print(f"  {name:8s} {conf:.0%}  {side}  box=({x1},{y1})-({x2},{y2})")
    else:
        print("  no waste found")
    print(f"Detection took {ms:.0f} ms")

    out_path = photo.rsplit(".", 1)[0] + "_result.jpg"
    cv2.imwrite(out_path, draw(frame, found))
    print(f"Saved {out_path}")
