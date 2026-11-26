# CAD design files

Mechanical design of the JNNP RoboDojo robot (26 x 28 cm footprint, 26 cm tall, acrylic chassis).

```
cad/
  step/   STEP (.step / .stp) files - open in SolidWorks, Fusion 360, FreeCAD, Onshape ...
```

The STL meshes used by ROS 2 (RViz / Gazebo) are in
`ros2_ws/src/jnnp_robodojo_robot/meshes/`; the STEP files here are the editable source models.

Files:

| File | Contents |
| --- | --- |
| `step/RDJ_robot.step` | full robot assembly from Fusion 360 (63 solid bodies), exported 14 Oct 2026 |

Suggested names for further parts: `robot_full_assembly.step`, `chassis_base.step`, `front_cabin.step`,
`back_cabin_dump_bin.step`, `wheel_holder.step`, `lidar_mount.step`, `camera_mount.step`.
