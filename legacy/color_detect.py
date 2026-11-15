"""
STEP 2 - Find an object by its colour (classic OpenCV, no AI).
Hold a RED object in front of the camera. A box is drawn around it and the
program says whether it is LEFT, CENTRE or RIGHT - this is how the robot
will later decide which way to turn.
Run in the Pi's desktop terminal (VNC). Press Q to quit.
"""
import cv2
import numpy as np
from picamera2 import Picamera2

# Red in HSV colour space wraps around 0, so it needs two ranges
LOWER1, UPPER1 = np.array([0, 120, 70]),   np.array([10, 255, 255])
LOWER2, UPPER2 = np.array([170, 120, 70]), np.array([180, 255, 255])
MIN_AREA = 800        # ignore tiny specks

cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"}))
cam.start()

while True:
    frame = cam.capture_array()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)            # easier to pick colours in HSV
    mask = cv2.inRange(hsv, LOWER1, UPPER1) | cv2.inRange(hsv, LOWER2, UPPER2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))  # remove noise

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        biggest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(biggest) > MIN_AREA:
            x, y, w, h = cv2.boundingRect(biggest)
            cx = x + w // 2
            side = "LEFT" if cx < 213 else "RIGHT" if cx > 426 else "CENTRE"
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"RED object: {side}", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.line(frame, (213, 0), (213, 480), (255, 255, 255), 1)  # left/centre/right guides
    cv2.line(frame, (426, 0), (426, 480), (255, 255, 255), 1)
    cv2.imshow("Colour detection", frame)
    cv2.imshow("Mask (white = red pixels)", mask)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.stop()
cv2.destroyAllWindows()
