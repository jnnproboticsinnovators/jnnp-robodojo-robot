"""
STEP 1 - Live camera view with OpenCV.
Run in the Pi's desktop terminal (VNC). Press Q in the window to quit.
"""
import cv2
from picamera2 import Picamera2

cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"}))
cam.start()

while True:
    frame = cam.capture_array()             # one picture from the camera (OpenCV colour order)
    cv2.putText(frame, "RoboDojo camera - press Q to quit", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.imshow("Camera", frame)             # show it in a window
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.stop()
cv2.destroyAllWindows()
