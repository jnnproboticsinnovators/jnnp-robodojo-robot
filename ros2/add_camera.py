#!/usr/bin/env python3
"""Adds a working Gazebo camera sensor to camera_link in the JNNP RoboDojo URDF (run once)."""
import glob, os, shutil, sys

files = [f for f in glob.glob(os.path.expanduser("~/ros2_ws/src/jnnp_robodojo_robot/**/*"), recursive=True)
         if f.endswith((".urdf", ".xacro", ".urdf.xacro"))]
if not files:
    sys.exit("Could not find the robot URDF in ~/ros2_ws/src/jnnp_robodojo_robot")
CAMERA = '''
  <!-- Pi Camera Module 3 (66 deg wide), 640x480 at 15 fps, looking forward from camera_link -->
  <gazebo reference="camera_link">
    <sensor name="pi_camera" type="camera">
      <gz_frame_id>camera_link</gz_frame_id>
      <topic>camera/image_raw</topic>
      <always_on>true</always_on>
      <visualize>true</visualize>
      <update_rate>15</update_rate>
      <camera>
        <camera_info_topic>camera/camera_info</camera_info_topic>
        <horizontal_fov>1.15</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.05</near>
          <far>15</far>
        </clip>
      </camera>
    </sensor>
  </gazebo>

</robot>'''

done = 0
for urdf in sorted(files):
    text = open(urdf).read()
    if 'name="camera_link"' not in text:
        continue
    if 'type="camera"' in text:
        print(f"Camera sensor already in {urdf} - skipped.")
        continue
    pos = text.rfind("</robot>")
    shutil.copy(urdf, urdf + ".before_camera")
    open(urdf, "w").write(text[:pos] + CAMERA.lstrip("\n") + text[pos + len("</robot>"):])
    print(f"Camera sensor added to {urdf}")
    done += 1
if done == 0:
    print("Nothing changed (no robot file with camera_link that still needs a camera).")
