cmake_minimum_required(VERSION 3.8)
project(jnnp_robodojo_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Every folder listed here MUST exist in src/jnnp_robodojo_robot/
install(
  DIRECTORY launch urdf meshes rviz worlds config
  DESTINATION share/${PROJECT_NAME}
)

# Python nodes (dynamic barrier, RViz field labels)
install(
  PROGRAMS scripts/dynamic_barrier.py scripts/field_markers.py scripts/fake_robot.py scripts/arrow_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
