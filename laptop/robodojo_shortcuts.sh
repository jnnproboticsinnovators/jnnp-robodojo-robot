# >>> JNNP RoboDojo shortcuts >>>
# Type ONE word in a NEW Ubuntu window (one window per program).
#
# ---- GAZEBO (simulation on the laptop) ----
#   gazebo     -> Gazebo with the field and the robot
#   slam       -> SLAM (draws the map)
#   rviz       -> RViz (shows the map)
#   gzteleop   -> arrow teleop for the GAZEBO robot
#   savemap    -> saves the map to ~/maps/robodojo_field
#   nav        -> Nav2 using the saved map
#
# ---- PHYSICAL ROBOT (Raspberry Pi) ----
#   robotteleop -> logs in to the Pi and starts the REAL robot teleop
#   pi          -> just logs in to the Pi
#   robotros    -> makes THIS laptop window talk ROS with the real robot (Pi)
#                  (uses the Zenoh router that runs on the Pi; the Pi address comes from $PI)
#
# ---- REAL ROBOT WITH ROS (SLAM / Nav2) - 2 windows ----
#   robotengine -> window 1: logs in to the Pi and starts robodojo_ros.py (LiDAR + wheels in ROS)
#   robotdrive  -> window 2: arrow teleop for the real robot through ROS
#                  (press C in it = robot camera + waste AI on / off, watch at http://<PI-IP>:8000)
#   (do NOT use teleop / robotteleop at the same time as robotengine)
#
# ---- REAL ROBOT MAPPING (SLAM) - add these windows ----
#   robotslam    -> window 3: starts SLAM on the Pi (draws the map from the LiDAR)
#   robotrviz    -> window 4: RViz on the laptop - SEE the map being drawn
#   robotsavemap -> when the map is finished: saves it on the Pi in ~/maps/robodojo_field
#   robotviewmap -> copies the saved map to the laptop and opens it as a picture
#
# ---- REAL ROBOT AUTONOMOUS NAVIGATION (Nav2) - instead of robotslam ----
#   robotnav     -> starts Nav2 on the Pi with the saved map (then use robotrviz:
#                   "2D Pose Estimate" = where the robot is, "Nav2 Goal" = where to go)
#   robotnav trial_6a  -> the same with another saved map
#   robotmission trial_6a -> automatic mission: home -> load -> offload (dump) -> home, for every region
#   robotlocalize trial_6a -> finds the robot on the map by itself (instead of 2D Pose Estimate)
#
# Your own older "teleop" shortcut is left exactly as it was.

unalias gazebo slam rviz gzteleop savemap nav robotteleop robotros robotengine robotdrive robotslam robotrviz robotsavemap robotviewmap robotnav robotmission robotlocalize Gazebo SLAM RVIZ 2>/dev/null

function _robodojo_env {
  source /opt/ros/jazzy/setup.bash
  [ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
  # Gazebo runs only on this laptop: keep its ROS + Gazebo messages on the laptop itself
  # (needed since WSL uses "mirrored" networking for the real robot)
  unset RMW_IMPLEMENTATION ZENOH_SESSION_CONFIG_URI ZENOH_CONFIG_OVERRIDE ROS_STATIC_PEERS
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  export GZ_IP=127.0.0.1
}

# ---- GAZEBO ----
function gazebo { _robodojo_env; ros2 launch jnnp_robodojo_robot gazebo.launch.py "$@"; }
function slam { _robodojo_env; ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true "$@"; }
function rviz { _robodojo_env; ros2 run rviz2 rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz --ros-args -p use_sim_time:=true; }
function gzteleop { _robodojo_env; python3 ~/arrow_teleop.py; }
function savemap { _robodojo_env; mkdir -p ~/maps; ros2 run nav2_map_server map_saver_cli -f ~/maps/robodojo_field --ros-args -p use_sim_time:=true; }
function nav { _robodojo_env; ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=$HOME/maps/robodojo_field.yaml params_file:=$HOME/ros2_ws/nav2_params_robodojo.yaml "$@"; }
function Gazebo { gazebo "$@"; }
function SLAM { slam "$@"; }
function RVIZ { rviz; }

# ---- PHYSICAL ROBOT ----
function robotteleop { ssh -t "${PI:-user@PI-IP}" "python3 ~/robot_teleop_ubuntu.py"; }
function robotros {
  source /opt/ros/jazzy/setup.bash
  local pi_ip="${PI:-user@PI-IP}"; pi_ip="${pi_ip#*@}"
  local default_cfg="$(ros2 pkg prefix rmw_zenoh_cpp 2>/dev/null)/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5"
  if [ ! -f "$default_cfg" ]; then
    echo "Zenoh is not installed on this laptop:  sudo apt install -y ros-jazzy-rmw-zenoh-cpp"; return 1
  fi
  # session settings = default settings, but connect to the router on the Pi instead of this laptop
  mkdir -p ~/.robodojo
  # client mode: everything goes through the Pi's router (direct peer links can't reach the Pi's programs)
  sed -e "s#tcp/localhost:7447#tcp/${pi_ip}:7447#g" -e 's#^  mode: "peer",#  mode: "client",#' \
      "$default_cfg" > ~/.robodojo/zenoh_to_pi.json5
  if ! grep -q "tcp/${pi_ip}:7447" ~/.robodojo/zenoh_to_pi.json5 || ! grep -q '^  mode: "client",' ~/.robodojo/zenoh_to_pi.json5; then
    echo "Could not set the Pi address in the Zenoh settings - tell Claude."; return 1
  fi
  unset ZENOH_CONFIG_OVERRIDE
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  export ZENOH_SESSION_CONFIG_URI="$HOME/.robodojo/zenoh_to_pi.json5"
  pkill -9 -f ros2_daemon 2>/dev/null      # old frozen ROS helper (it makes commands hang)
  echo "This window now talks ROS with the real robot (Pi ${pi_ip})."
}
function robotengine { ssh -t "${PI:-user@PI-IP}" "bash -ic 'python3 ~/robodojo_ros.py'"; }
function robotdrive { robotros && ROBODOJO_REAL=1 python3 ~/arrow_teleop.py; }
function robotslam { ssh -t "${PI:-user@PI-IP}" "bash -ic 'ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false'"; }
function robotrviz { robotros && ros2 run rviz2 rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz; }
function robotsavemap { ssh -t "${PI:-user@PI-IP}" "bash -ic 'mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/robodojo_field && ls -l ~/maps'"; }
# robotnav            -> Nav2 with the map robodojo_field
# robotnav trial_6a   -> Nav2 with another map in ~/maps on the Pi
function robotnav { local m="${1:-robodojo_field}"; ssh -t "${PI:-user@PI-IP}" "bash -ic 'ros2 launch nav2_bringup bringup_launch.py use_sim_time:=false map:=\$HOME/maps/${m}.yaml params_file:=\$HOME/nav2_params_real.yaml'"; }
# robotmission trial_6a -> runs the mission in ~/maps/trial_6a_regions.yaml (home -> load -> offload -> home ...)
function robotmission { local m="${1:-trial_6a}"; ssh -t "${PI:-user@PI-IP}" "bash -ic 'python3 ~/robodojo_mission.py \$HOME/maps/${m}_regions.yaml'"; }
function robotviewmap {
  scp "${PI:-user@PI-IP}":~/maps/robodojo_field.pgm ~/robodojo_field.pgm || return 1
  local out; out=$(python3 ~/robodojo_map2png.py | tail -1 | sed 's/^Map picture saved: //; s/ (.*$//')
  echo "Map picture: $out"
  explorer.exe "$(wslpath -w "$out")" 2>/dev/null || true
}
function robotlocalize {
  local reg=""
  if [ -n "$1" ]; then
    reg=~/"${1}_regions.yaml"
    [ -f "$reg" ] || reg="$(ls -t /mnt/c/Users/*/Downloads/${1}_regions*.yaml 2>/dev/null | head -1)"
  fi
  robotros && python3 ~/robot_localize.py $reg
}
# <<< JNNP RoboDojo shortcuts <<<
