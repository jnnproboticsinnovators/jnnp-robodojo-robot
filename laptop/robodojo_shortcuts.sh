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
#   ONE-WORD STARTERS (Gazebo) - start several programs in ONE window, Ctrl+C there stops them all:
#   gzmap      -> Gazebo + SLAM + RViz   (then: gzteleop in a 2nd window to drive, savemap when done)
#   gzrun      -> Gazebo + Nav2 + RViz with the saved map  (no SLAM - for every run after mapping)
#   gzturntest -> test: robot turns left then right by itself and reports how far (Gazebo open)
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
#   robotnavlog  -> shows the important lines (errors / "active") of the last robotnav
#
#   ONE-WORD STARTERS (real robot) - several programs in ONE window, Ctrl+C there stops them all
#   (on the laptop AND on the Pi). Needs robotkey once (log in without typing the password).
#   robotkey   -> ONE TIME: lets the laptop log in to the Pi without the password
#   robottimesetup -> ONE TIME: lets the Pi copy the laptop's clock (robotrun / robotmap then do it)
#   robottime  -> set the Pi's clock to the laptop's now (Nav2 needs them equal)
#   robotmap   -> robotengine + robotslam + robotrviz  (then robotdrive in a 2nd window,
#                 robotsavemap when the map is good)
#   robotrun   -> robotengine + robotnav + robotrviz + robotlocalize with the saved map
#                 (robot at the start spot!)
#   robotrun trial_6a -> the same with another saved map
#   robotnavcheck -> robot not moving? checks everything and says what to fix (gznavcheck = Gazebo)
#   robotmapcheck -> map not clean? checks IMU / wheels / LiDAR and calibrates them
#   robotcover   -> robot drives around the WHOLE field by itself (gzcover = Gazebo)
#
# Your own older "teleop" shortcut is left exactly as it was.

unalias gazebo slam rviz gzteleop savemap nav gzmap gzrun gzturntest robotteleop robotros robotengine robotdrive robotslam robotrviz robotsavemap robotviewmap robotnav robotnavlog robotmission robotlocalize robotkey robotmap robotrun robotnavcheck robotmapcheck gznavcheck robotcover gzcover robotmaps robotaddwifi robotdelwifi robotwifis robottime robottimesetup robotai robotremark gzremark robotwhere gzwhere robotswap gzswap Gazebo SLAM RVIZ 2>/dev/null

function _robodojo_env {
  source /opt/ros/jazzy/setup.bash
  [ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
  # Gazebo runs only on this laptop: keep its ROS + Gazebo messages on the laptop itself
  # (needed since WSL uses "mirrored" networking for the real robot)
  unset ZENOH_SESSION_CONFIG_URI ZENOH_CONFIG_OVERRIDE ROS_STATIC_PEERS FASTDDS_BUILTIN_TRANSPORTS
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  export GZ_IP=127.0.0.1
  # ROS talk between the Gazebo programs goes through ZENOH - the same way the REAL robot already
  # works. Tested 21 Nov: with the default (DDS) the programs on this laptop could NOT find each
  # other (robot_description / odom never arrived), only sometimes after  wsl --shutdown.
  # Zenoh needs one small "router" program on the laptop; it is started once and keeps running.
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  _gz_router_ok || _gz_router
}
function _gz_router_ok { timeout 1 bash -c "</dev/tcp/127.0.0.1/7447" 2>/dev/null; }
# (re)start the laptop's Zenoh router and WAIT until it really answers on its port
function _gz_router {
  mkdir -p ~/.robodojo
  pkill -f "rmw_zenohd" 2>/dev/null; sleep 1
  setsid ros2 run rmw_zenoh_cpp rmw_zenohd > ~/.robodojo/zenohd.log 2>&1 < /dev/null &
  local k
  for k in $(seq 1 30); do _gz_router_ok && return 0; sleep 0.5; done
  echo "The laptop's ROS router (Zenoh) did not start. Its last lines:"; tail -5 ~/.robodojo/zenohd.log
  return 1
}
# start Gazebo (+ the robot) and wait until the robot is really placed. If the robot is not placed
# within 60 s, clean everything, restart the router and try ONE more time by itself.
function _gz_start { # $1 = fast or empty
  local try w
  for try in 1 2; do
    _gz_clean
    source /opt/ros/jazzy/setup.bash; [ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
    _gz_router || return 1               # fresh router every start: no stale state from earlier runs
    _robodojo_env
    if [ "$1" = "fast" ]; then
      echo "1/3 Gazebo starting WITHOUT its window (fast mode) ..."
      gazebo world:="-s $(_gz_world)" > ~/.robodojo/gazebo.log 2>&1 &
    else
      echo "1/3 Gazebo starting ..."; gazebo > ~/.robodojo/gazebo.log 2>&1 &
    fi
    for w in $(seq 1 30); do
      grep -q "Entity creation successful" ~/.robodojo/gazebo.log 2>/dev/null && { sleep 5; echo "    Gazebo is running (robot placed)."; return 0; }
      sleep 2
      [ $((w % 10)) -eq 0 ] && echo "    ... still waiting for Gazebo ($((w * 2)) s)"
    done
    [ $try -eq 1 ] && echo "    Robot not placed after 60 s - restarting everything once by itself ..."
  done
  echo "Gazebo did not place the robot (2 tries). Lines about it:"
  grep -E "robot_state_publisher|create-|ERROR|rror" ~/.robodojo/gazebo.log | tail -6
  echo "Router:"; tail -3 ~/.robodojo/zenohd.log
  echo "-> send these lines to Claude"
  return 1
}
# remove everything an earlier Gazebo/Nav2 run left behind (programs + shared-memory files)
function _gz_clean {
  pkill -INT -f "gz sim" 2>/dev/null; pkill -INT -f parameter_bridge 2>/dev/null
  pkill -INT -f "robot_state_publisher|slam_toolbox|nav2_|amcl|map_server|lifecycle_manager|component_container|rviz2|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|collision_monitor|waypoint_follower|docking_server|smoother_server" 2>/dev/null
  sleep 3
  pkill -9 -f "gz sim" 2>/dev/null
  pkill -9 -f "robot_state_publisher|slam_toolbox|nav2_|amcl|map_server|lifecycle_manager|component_container|rviz2|parameter_bridge|_ros2_daemon" 2>/dev/null
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/fast_datasharing* 2>/dev/null
}

# ---- GAZEBO ----
function gazebo { _robodojo_env; ros2 launch jnnp_robodojo_robot gazebo.launch.py "$@"; }
# Gazebo SLAM settings (~/.robodojo/slam_gazebo.yaml, made from the standard ones at every start):
#   In Gazebo the robot's position is EXACT (fix_gazebo_odom), so SLAM just draws the LiDAR at that
#   position. SLAM's own "scan matching" and "loop closing" only add mistakes there (they shifted and
#   twisted parts of the map). Only LiDAR hits up to 4 m are used (far hits leave dots and stripes).
#   The REAL robot (robotslam / robotmap) is not affected - it keeps full SLAM.
function _robodojo_slam_params {
  mkdir -p ~/.robodojo
  python3 - ~/.robodojo/slam_gazebo.yaml <<'PYEOF'
import sys, yaml
src = '/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml'
d = yaml.safe_load(open(src))
p = d['slam_toolbox']['ros__parameters']
p.update({'use_sim_time': True, 'max_laser_range': 4.0, 'resolution': 0.05,
          'minimum_travel_distance': 0.1, 'minimum_travel_heading': 0.1,
          'use_scan_matching': False, 'do_loop_closing': False})
yaml.safe_dump(d, open(sys.argv[1], 'w'), sort_keys=False)
print('Gazebo SLAM: exact-position mapping (no scan matching), LiDAR up to 4 m')
PYEOF
}
function slam { _robodojo_env; _robodojo_slam_params; ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true slam_params_file:=$HOME/.robodojo/slam_gazebo.yaml "$@"; }
function rviz {      # Gazebo RViz remembers your view in ~/.robodojo/gazebo.rviz (File -> Save Config As)
  local cfg=/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz
  [ -f ~/.robodojo/gazebo.rviz ] && cfg=~/.robodojo/gazebo.rviz
  _robodojo_env; ros2 run rviz2 rviz2 -d "$cfg" --ros-args -p use_sim_time:=true
}
function gzteleop { _robodojo_env; python3 ~/arrow_teleop.py; }
function savemap {
  _robodojo_env; mkdir -p ~/maps
  # Only refuse when the topic list really came back WITHOUT /map. If the list is just slow
  # (ROS helper starting up, Gazebo busy) we do not guess - we try to save and report the result.
  local tl rc
  tl=$(timeout 20 ros2 topic list --no-daemon 2>/dev/null); rc=$?
  if [ $rc -eq 0 ] && ! grep -qx /map <<<"$tl"; then
    echo "No map is being made right now - is gzmap still running and READY in its window?"
    echo "(start gzmap, drive with gzteleop, then savemap while gzmap is still open)"; return 1
  fi
  echo "Saving the map (up to 15 s) ..."
  # wait up to 15 s for the map (Gazebo runs slower than real time on this laptop; 2 s was too short)
  if ros2 run nav2_map_server map_saver_cli -f ~/maps/robodojo_field --ros-args -p use_sim_time:=true \
      -p save_map_timeout:=15.0 -p map_subscribe_transient_local:=true >~/.robodojo/savemap.log 2>&1 \
     && [ -s ~/maps/robodojo_field.pgm ]; then
    echo "Saved: ~/maps/robodojo_field.pgm + .yaml  (gzrun uses it)"; _keep_marks robodojo_field
  else
    echo "NOT saved. Is gzmap still running and READY in window 1? Last lines:"
    tail -3 ~/.robodojo/savemap.log; return 1
  fi
}
function nav { _robodojo_env; ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=$HOME/maps/robodojo_field.yaml params_file:=$HOME/ros2_ws/nav2_params_robodojo.yaml "$@"; }
# start several programs in the background of this window; Ctrl+C stops all of them
function _robodojo_group {
  ( trap 'trap - INT TERM EXIT; echo; echo "Stopping everything..."; kill 0' INT TERM EXIT
    "$@"
    wait )
}
function _gzmap_steps {
  mkdir -p ~/.robodojo
  rm -f ~/.robodojo/marks_current.yaml      # new map = new competition marks
  _gz_start "$1" || return 1
  echo "2/3 SLAM starting ...";  slam > ~/.robodojo/slam.log 2>&1 &
  sleep 5
  echo "3/3 RViz starting ...";  rviz > ~/.robodojo/rviz.log 2>&1 &
  echo
  echo "READY. Open a 2nd window and type:  gzteleop   (drive the whole field; M marks the competition places)"
  echo "When the map is good, in another window type:  savemap"
  echo "Ctrl+C here stops Gazebo, SLAM and RViz."
}
# Gazebo world file, for "fast" mode (simulation without the 3D Gazebo window)
function _gz_world {
  local w=~/internship2026/gamefield_software/src/gamefield/worlds/gamefield.world
  [ -f "$w" ] || w="$(ros2 pkg prefix jnnp_robodojo_robot 2>/dev/null)/share/jnnp_robodojo_robot/worlds/robodojo_arena.sdf"
  echo "$w"
}
function _gzrun_steps {
  mkdir -p ~/.robodojo
  if [ ! -f ~/maps/robodojo_field.yaml ]; then echo "No saved map yet - make one with gzmap + savemap first."; return 1; fi
  _gz_start "$1" || return 1
  echo "2/3 Nav2 starting with the saved map (wait ~30 s) ..."; nav use_composition:=False > ~/.robodojo/nav.log 2>&1 &
  sleep 10
  echo "3/3 RViz starting ...";  rviz > ~/.robodojo/rviz.log 2>&1 &
  local i
  for i in $(seq 1 40); do
    grep -q "Managed nodes are active" ~/.robodojo/nav.log 2>/dev/null && [ "$(grep -c 'Managed nodes are active' ~/.robodojo/nav.log)" -ge 2 ] && break
    sleep 2
  done
  echo
  if [ "$(grep -c 'Managed nodes are active' ~/.robodojo/nav.log 2>/dev/null)" -ge 2 ]; then
    echo "Navigation is ACTIVE. Finding the robot on the map and checking it lines up (gzlocalize) ..."
    gzlocalize
    echo "READY - if it said ALIGNED OK: in RViz click  Nav2 Goal  then a pink/white spot on the map."
  else
    echo "Nav2 is not active yet. Important lines:"
    grep -hE "ERROR|Failed|frame does not exist" ~/.robodojo/nav.log | tail -10
  fi
  echo "Optional: gzteleop in a 2nd window to drive by hand.   Ctrl+C here stops everything."
}
# gzview -> opens the 3D Gazebo window on a Gazebo that is ALREADY running (e.g. after  gzrun fast
#           says READY). Start it late so the window cannot hold up the start. Close the window
#           any time - the simulation keeps running.
function gzview { _robodojo_env; echo "Opening the 3D Gazebo window (takes ~10-30 s) ..."; gz sim -g > ~/.robodojo/gzview.log 2>&1 & }
# gzlocalize -> Gazebo: finds the robot on the map from the LiDAR and CHECKS Nav2 agrees (ALIGNED OK)
function gzlocalize { _robodojo_env; python3 ~/robot_localize.py --sim --guess -2.05 -1.12; }
# gzturntest -> the Gazebo robot turns left 3 s and right 3 s by itself and reports how far it went
function gzturntest { _robodojo_env; python3 ~/gz_turn_test.py; }
# gzmap fast -> same, but without the 3D Gazebo window: the simulation runs ~3x faster, so the
#               robot answers the arrow keys at once (watch the map grow in RViz; gzview = 3D)
function gzmap { _robodojo_group _gzmap_steps "$@"; }
# gzrun fast -> same, but without the 3D Gazebo window (much faster simulation; watch in RViz)
function gzrun { _robodojo_group _gzrun_steps "$@"; }
function Gazebo { gazebo "$@"; }
function SLAM { slam "$@"; }
function RVIZ { rviz; }

# ---- FIND THE PI ON EITHER HOTSPOT ----
# PI_LIST (set in ~/.bashrc) = every address the Pi can have, e.g. one per phone hotspot:
#   export PI_LIST="user@ADDRESS-ON-PHONE-1 user@ADDRESS-ON-PHONE-2"
# Every robot word first checks which one answers and uses that one - no editing when you switch phones.
function _pi_find {
  local c ip
  for c in $PI ${PI_LIST:-}; do
    ip="${c#*@}"
    if timeout 1 bash -c "</dev/tcp/$ip/22" 2>/dev/null; then
      [ "$PI" != "$c" ] && echo "(robot found on the other hotspot: $ip)"
      export PI="$c"
      ssh-keygen -F "$ip" >/dev/null 2>&1 || ssh-keyscan -H "$ip" >> ~/.ssh/known_hosts 2>/dev/null
      return 0
    fi
  done
  # Not at a known address: the phone/router gave the Pi a NEW number. Find it by the Pi's
  # fixed Wi-Fi ID (MAC) in Windows' list of nearby devices - works on any hotspot or router.
  local user="${PI%@*}" mac="${PI_MAC:-}" net
  [ -n "$mac" ] || { echo "Robot (Pi) not found (tried: $PI ${PI_LIST:-}). Tip: export PI_MAC=<the Pi Wi-Fi MAC, e.g. aa-bb-cc-dd-ee-ff> to let it search by itself."; return 1; }
  echo "(Pi not at a known address - searching this Wi-Fi for it ...)"
  for net in $(ipconfig.exe 2>/dev/null | tr -d '\r' | grep -a "IPv4" | grep -oE '[0-9]+\.[0-9]+\.[0-9]+\.' | grep -v '^172\.'); do
    ping.exe -n 1 -w 300 "${net}255" >/dev/null 2>&1          # wake up the device list
    for ip in "${net}159" "${net}249"; do ping.exe -n 1 -w 300 "$ip" >/dev/null 2>&1; done
  done
  ip=$(arp.exe -a 2>/dev/null | tr -d '\r' | grep -i "$mac" | awk '{print $1}' | head -1)
  if [ -n "$ip" ] && timeout 3 bash -c "</dev/tcp/$ip/22" 2>/dev/null; then
    export PI="$user@$ip"
    grep -q "@$ip\b" ~/.bashrc 2>/dev/null || echo "export PI_LIST=\"\$PI_LIST $user@$ip\"" >> ~/.bashrc
    export PI_LIST="${PI_LIST:-} $user@$ip"
    echo "(robot found at the new address $ip - remembered for next time)"
    ssh-keygen -F "$ip" >/dev/null 2>&1 || ssh-keyscan -H "$ip" >> ~/.ssh/known_hosts 2>/dev/null
    return 0
  elif [ -n "$ip" ]; then
    echo "The Pi IS on this Wi-Fi ($ip) but does not answer. Wait 30 s (still starting?) and try again."
    echo "Still no answer on jnnp_net? The router's 'Isolate Clients' is blocking it - untick it."
    return 1
  fi
  echo "Robot (Pi) not found on any hotspot (tried: $PI ${PI_LIST:-})."
  echo "-> Is the Pi switched on? Is THIS laptop connected to the same phone hotspot as the Pi?"
  return 1
}
unalias pi 2>/dev/null
function pi { _pi_find && ssh "$PI" "$@"; }

# ---- WI-FI NETWORKS ON THE PI ----
# robotaddwifi "NAME" "PASSWORD"  -> the Pi also joins this Wi-Fi from now on (keeps the others)
# robotwifis                      -> lists the Wi-Fi names the Pi knows
# robotdelwifi "NAME"             -> the Pi forgets this Wi-Fi
# (Wi-Fi settings live in /etc/netplan/60-second-hotspot.yaml on the Pi; a backup is kept)
function _pi_wifi_edit {     # $1 = add|del  $2 = name  $3 = password
  _pi_find || return 1
  local n64 p64
  n64=$(printf '%s' "$2" | base64 -w0); p64=$(printf '%s' "${3:-}" | base64 -w0)
  ssh -t "${PI:-user@PI-IP}" "sudo python3 -c \"
import base64, os, shutil, subprocess, yaml
f = '/etc/netplan/60-second-hotspot.yaml'
name = base64.b64decode('$n64').decode(); pw = base64.b64decode('$p64').decode()
d = yaml.safe_load(open(f)) if os.path.exists(f) else None
d = d or {'network': {'version': 2, 'wifis': {}}}
w = d['network'].setdefault('wifis', {}).setdefault('wlan0', {'dhcp4': True, 'optional': True})
ap = w.setdefault('access-points', {})
if os.path.exists(f): shutil.copy(f, f + '.backup')
if '$1' == 'add':
    ap[name] = {'password': pw} if pw else {}
    print('Added Wi-Fi:', name)
else:
    print(('Removed Wi-Fi: ' + name) if ap.pop(name, None) is not None else ('Not found: ' + name))
yaml.safe_dump(d, open(f, 'w'), default_flow_style=False, sort_keys=False)
os.chmod(f, 0o600)
r = subprocess.run(['netplan', 'generate'], capture_output=True, text=True)
print('Settings OK.' if r.returncode == 0 else 'PROBLEM: ' + r.stderr + ' (old settings in ' + f + '.backup)')
print('Wi-Fi networks the Pi knows now:', ', '.join(ap) or '(none)')
\""
}
function robotaddwifi {
  [ -n "$1" ] || { echo 'Use:  robotaddwifi "WIFI NAME" "PASSWORD"   (quotes needed if there are spaces or symbols)'; return 1; }
  [ -z "$2" ] || [ ${#2} -ge 8 ] || { echo "Wi-Fi passwords are at least 8 characters - check it."; return 1; }
  _pi_wifi_edit add "$1" "$2" || return 1
  echo "The Pi uses it the next time its current Wi-Fi goes away, or after a restart."
  echo "(To switch now: turn off the hotspot it is on - it joins another known one within ~1-2 min.)"
}
function robotdelwifi { [ -n "$1" ] || { echo 'Use:  robotdelwifi "WIFI NAME"'; return 1; }; _pi_wifi_edit del "$1"; }
function robotwifis {
  _pi_find || return 1
  ssh -t "${PI:-user@PI-IP}" "sudo netplan get wifis.wlan0.access-points 2>/dev/null | grep -E '^[^ ].*:' | sed 's/:.*//; s/^/  /'; echo; echo -n 'Connected now to: '; iwgetid -r 2>/dev/null || echo '(unknown)'"
}

# ---- PHYSICAL ROBOT ----
function robotteleop { _pi_find || return 1; ssh -t "${PI:-user@PI-IP}" "python3 ~/robot_teleop_ubuntu.py"; }
function robotros {
  _pi_find || return 1
  # the laptop's own Zenoh router is only for Gazebo - stop it so robot windows talk to the Pi only
  pkill -f "rmw_zenohd" 2>/dev/null
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
function robotengine { _pi_find || return 1; ssh -t "${PI:-user@PI-IP}" "bash -ic 'python3 ~/robodojo_ros.py'"; }
function robotdrive { robotros && PI="$PI" ROBODOJO_REAL=1 python3 ~/arrow_teleop.py; }
# robotai -> FAST waste AI: the laptop does the detection on the Pi camera pictures (Pi is too slow).
#            Press C in robotdrive first (camera ON). Window: green = tablet screen found. q = stop.
function robotai { _pi_find || return 1; PI="$PI" python3 ~/waste_laptop.py; }
# robotslam uses ~/slam_params_real.yaml on the Pi when it exists (made by make_slam_params.py:
#   SLAM trusts the IMU heading and refuses big jumps - no more turned map sections)
function robotslam { _pi_find || return 1; ssh -t "${PI:-user@PI-IP}" "bash -ic 'P=; [ -f \$HOME/slam_params_real.yaml ] && P=slam_params_file:=\$HOME/slam_params_real.yaml; ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false \$P'"; }
# RViz remembers YOUR view: arrange it (turn / zoom), then File -> Save Config As -> ~/.robodojo/robot.rviz
function robotrviz {
  local cfg=/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz
  [ -f ~/.robodojo/robot.rviz ] && cfg=~/.robodojo/robot.rviz
  robotros && ros2 run rviz2 rviz2 -d "$cfg"
}
# robotsavemap            -> saves the map as  robodojo_field  (as before)
# robotsavemap big_field  -> saves it with your own name (letters, numbers, _ and - only)
# robotmaps               -> lists all maps saved on the Pi
function _map_name_ok {
  [[ "$1" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "Map name '$1': use only letters, numbers, _ and - (no spaces), e.g.  big_field_v2"; return 1; }
}
function robotsavemap {
  local m="${1:-robodojo_field}"
  _map_name_ok "$m" || return 1
  _pi_find || return 1
  ssh -t "${PI:-user@PI-IP}" "bash -ic 'mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/${m} && echo && echo \"Map saved as: ${m}   (use it with:  robotrun ${m})\" && ls -l ~/maps/${m}.*'"
  _keep_marks "$m"
}
# the places marked with M in robotdrive / gzteleop during mapping are kept with the map
function _keep_marks {
  mkdir -p ~/maps
  if [ -f ~/.robodojo/marks_current.yaml ]; then
    cp ~/.robodojo/marks_current.yaml ~/maps/"$1"_points.yaml
    echo "Competition places saved too: ~/maps/$1_points.yaml   (robotcompete $1 / gzcompete uses them)"
  else
    echo "(No competition places marked with M yet - mark them while mapping, or later with robotpoints $1)"
  fi
}
# robotmapzip NAME -> puts NAME_map.zip on the Windows Desktop (picture .png + map .pgm/.yaml),
#                     ready to email / WhatsApp. The map must already be saved with robotsavemap NAME.
function robotmapzip {
  local m="${1:-robodojo_field}" d t
  _map_name_ok "$m" || return 1
  _pi_find || return 1
  d=/mnt/c/Users/Admin/Desktop; [ -d "$d" ] || d=/mnt/c/Users/Admin/OneDrive/Desktop
  [ -d "$d" ] || d=$(ls -d /mnt/c/Users/*/Desktop 2>/dev/null | grep -v -e Public -e Default | head -1)
  t=$(mktemp -d)/$m; mkdir -p "$t"
  scp -q "${PI:-user@PI-IP}":~/maps/$m.pgm "${PI:-user@PI-IP}":~/maps/$m.yaml "$t"/ || {
    echo "No saved map called '$m'. Saved maps:"; robotmaps; return 1; }
  python3 -c "import cv2; cv2.imwrite('$t/$m.png', cv2.imread('$t/$m.pgm', -1))"
  (cd "$(dirname "$t")" && python3 -m zipfile -c "$d/${m}_map.zip" "$m") || return 1
  echo "Done: ${m}_map.zip is on your Desktop ($d)"
  echo "  inside: $m.png (picture) + $m.pgm + $m.yaml (navigation map files)"
}
function robotmaps {
  _pi_find || return 1
  echo "Maps saved on the Pi (newest first):"
  ssh "${PI:-user@PI-IP}" 'cd ~/maps 2>/dev/null && ls -t *.yaml 2>/dev/null | grep -v _regions | while read f; do printf "  %-28s %s\n" "${f%.yaml}" "$(date -r "$f" "+%d %b %H:%M")"; done'
  echo "Use one with:  robotrun NAME    (picture: robotviewmap NAME)"
}
# robotnav            -> Nav2 with the map robodojo_field
# robotnav trial_6a   -> Nav2 with another map in ~/maps on the Pi
# (checks first that robotengine is running - Nav2 needs its /odom and /scan;
#  all Nav2 parts run in ONE program = faster start on the Pi; everything is also saved in ~/nav_log.txt)
function robotnav {
  _pi_find || return 1
  local m="${1:-robodojo_field}"
  ssh -t "${PI:-user@PI-IP}" "bash -ic '
    if ! pgrep -f \"[r]obodojo_ros.py\" >/dev/null; then
      echo; echo \"STOP: robotengine is not running. Start robotengine in another window first, then robotnav.\"; echo; exit 1
    fi
    [ -f \$HOME/maps/${m}.yaml ] || { echo \"No map \$HOME/maps/${m}.yaml on the Pi\"; exit 1; }
    ros2 launch nav2_bringup bringup_launch.py use_sim_time:=false use_composition:=True map:=\$HOME/maps/${m}.yaml params_file:=\$HOME/nav2_params_real.yaml 2>&1 | tee \$HOME/nav_log.txt'"
}
# robotnavlog -> shows only the important lines of the last robotnav (errors, "active")
function robotnavlog { _pi_find || return 1; ssh "${PI:-user@PI-IP}" "grep -E 'ERROR|Failed|failed|Managed nodes|Aborting|frame does not exist' ~/nav_log.txt | tail -20"; }
# robotmission trial_6a -> runs the mission in ~/maps/trial_6a_regions.yaml (home -> load -> offload -> home ...)
function robotmission { _pi_find || return 1; local m="${1:-trial_6a}"; ssh -t "${PI:-user@PI-IP}" "bash -ic 'python3 ~/robodojo_mission.py \$HOME/maps/${m}_regions.yaml'"; }
function robotviewmap {
  local m="${1:-robodojo_field}"
  _map_name_ok "$m" || return 1
  _pi_find || return 1
  scp "${PI:-user@PI-IP}":~/maps/${m}.pgm ~/${m}.pgm || { echo "No map called '${m}' on the Pi - type  robotmaps  to see the names."; return 1; }
  local out; out=$(python3 ~/robodojo_map2png.py ~/${m}.pgm | tail -1 | sed 's/^Map picture saved: //; s/ (.*$//')
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

# ---- CLOCKS: the Pi copies the laptop's time (works without internet) ----
# Nav2 (on the Pi) rejects RViz goals (laptop time) if the clocks differ by more than ~0.5 s.
# robottimesetup -> ONE TIME: lets the Pi set its clock without asking for the password
# robottime      -> sets the Pi's clock to the laptop's and shows the difference (robotrun / robotmap do it too)
function robottimesetup {
  _pi_find || return 1
  local u="${PI%@*}"
  echo "Enter the Pi password if asked (one time):"
  ssh -t "$PI" "echo '$u ALL=(root) NOPASSWD: /usr/bin/date, /usr/bin/timedatectl' | sudo tee /etc/sudoers.d/robodojo-time >/dev/null && sudo chmod 440 /etc/sudoers.d/robodojo-time && echo DONE"
}
function _clock_diff {      # seconds the Pi is ahead of the laptop (Wi-Fi delay removed)
  # one connection, 8 quick time questions; keep the answer with the shortest round trip
  python3 - "$PI" <<'PYEOF'
import subprocess, sys, time
p = subprocess.Popen(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', sys.argv[1],
                      'while read x; do date +%s.%N; done'],
                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                     text=True, bufsize=1)
best = None
try:
    for _ in range(8):
        t0 = time.time(); p.stdin.write('x\n'); p.stdin.flush()
        line = p.stdout.readline(); t1 = time.time()
        if not line.strip():
            sys.exit(1)
        off, rtt = float(line) - (t0 + t1) / 2, t1 - t0
        if best is None or rtt < best[0]:
            best = (rtt, off)
        time.sleep(0.05)
finally:
    try:
        p.stdin.close()
    except Exception:
        pass
    p.terminate()
print(f'{best[1]:+.2f}')
PYEOF
}
function robottime {
  _pi_find || return 1
  local d0 d i
  d0=$(_clock_diff) || { echo "Cannot reach the Pi without a password - run robotkey first."; return 1; }
  d=$d0
  for i in 1 2 3; do
    if python3 -c "import sys; sys.exit(0 if abs($d) < 0.2 else 1)"; then
      if [ "$i" = 1 ]; then echo "Clocks OK: Pi is $d s from the laptop."; else echo "Clocks: Pi was $d0 s off -> now $d s.  OK"; fi
      return 0
    fi
    # move the Pi's clock by the measured difference (done on the Pi itself)
    if ! ssh -o BatchMode=yes "$PI" "sudo -n timedatectl set-ntp false >/dev/null 2>&1; sudo -n date -s @\$(python3 -c 'import time; print(f\"{time.time() - ($d):.3f}\")') >/dev/null"; then
      echo "The Pi did not allow setting its clock - run robottimesetup once, then robottime again."; return 1
    fi
    sleep 0.5
    d=$(_clock_diff) || return 1
  done
  echo "Clocks: Pi was $d0 s off -> now $d s.  Still not close - check the Wi-Fi signal and run robottime again."
  return 1
}

# ---- REAL ROBOT one-word starters ----
function robotkey {
  _pi_find || return 1
  [ -f ~/.ssh/id_ed25519 ] || ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519 -q
  echo "Enter the Pi password ONE last time:"
  ssh-copy-id -i ~/.ssh/id_ed25519.pub "${PI:-user@PI-IP}" >/dev/null 2>&1 || ssh-copy-id -i ~/.ssh/id_ed25519.pub "${PI:-user@PI-IP}"
  if ssh -o BatchMode=yes -o ConnectTimeout=5 "${PI:-user@PI-IP}" true 2>/dev/null; then
    echo "DONE - the laptop now logs in to the Pi without a password."
  else
    echo "Did not work - check the Pi is on and the password was right, then run robotkey again."
  fi
}
function _robot_ready {
  _pi_find || return 1
  if ! ssh -o BatchMode=yes -o ConnectTimeout=6 "${PI:-user@PI-IP}" true 2>/dev/null; then
    echo "Cannot log in to the Pi without a password."
    echo "-> Is the Pi on and on the same hotspot?  If yes, type  robotkey  once, then try again."
    return 1
  fi
}
function _robot_cleanup {
  echo; echo "Stopping everything on the Pi (engine, SLAM, Nav2) ..."
  ssh -o BatchMode=yes -o ConnectTimeout=5 "${PI:-user@PI-IP}" \
    "pkill -INT -f '[o]nline_async_launch|[b]ringup_launch'; sleep 3; pkill -INT -f '[r]obodojo_ros.py'" 2>/dev/null
}
function _robot_start_engine {
  if ssh -o BatchMode=yes "${PI:-user@PI-IP}" "pgrep -f '[r]obodojo_ros.py' >/dev/null"; then
    echo "1/3 robotengine is already running - using it."; return 0
  fi
  echo "1/3 robotengine starting ..."
  robotengine < /dev/null > ~/.robodojo/engine.log 2>&1 &
  local i
  # SLAM / Nav2 do not need to wait for the LiDAR to spin up (they simply wait for its first
  # scan), so carry on as soon as the engine says READY. Stop ONLY if the engine really died.
  for i in $(seq 1 20); do
    grep -q "LiDAR scanning" ~/.robodojo/engine.log 2>/dev/null && { echo "    robot ready (LiDAR scanning, ${i} s)."; return 0; }
    grep -q "READY v" ~/.robodojo/engine.log 2>/dev/null && { echo "    robot ready (${i} s) - LiDAR spinning up."; return 0; }
    sleep 1
  done
  if ssh -o BatchMode=yes "${PI:-user@PI-IP}" "pgrep -f '[r]obodojo_ros.py' >/dev/null"; then
    echo "    robotengine is running (slow to report) - continuing."; return 0
  fi
  echo "    robotengine STOPPED - it could not start. Last lines:"; tail -5 ~/.robodojo/engine.log
  echo "    (Arduino USB cable? LiDAR cable? then type the same word again)"
  return 1
}
function _robot_stop_old {    # a CLEAN start: nothing left over from an earlier run
  echo "0/3 Clearing anything left from earlier runs ..."
  # on the Pi: old SLAM / Nav2 / localize first, then the robot engine (restarting the engine
  # also resets the wheel position to 0, so the new map / run starts from where the robot stands)
  ssh -o BatchMode=yes "${PI:-user@PI-IP}" "
    if pgrep -f '[o]nline_async_launch|[b]ringup_launch|[s]lam_toolbox|[c]omponent_container' >/dev/null; then
      pkill -INT -f '[o]nline_async_launch|[b]ringup_launch'; sleep 4
      pkill -9 -f '[a]sync_slam_toolbox|[c]omponent_container|[c]ontroller_server|[p]lanner_server|[a]mcl|[m]ap_server|[b]t_navigator' 2>/dev/null
    fi
    if pgrep -f '[r]obodojo_ros.py' >/dev/null; then pkill -INT -f '[r]obodojo_ros.py'; sleep 2; pkill -9 -f '[r]obodojo_ros.py' 2>/dev/null; fi
    true"
  # on the laptop: old RViz windows, old teleop/localize/check programs and the frozen ROS helper
  pkill -f '[r]viz2' 2>/dev/null
  pkill -f '[r]obot_localize.py|[n]av_check.py|[f]ield_cover.py' 2>/dev/null
  pkill -9 -f '[r]os2_daemon' 2>/dev/null
  sleep 1
}
function _robotmap_steps {
  trap '_robot_cleanup; trap - INT TERM EXIT; kill 0' INT TERM
  mkdir -p ~/.robodojo
  rm -f ~/.robodojo/marks_current.yaml      # new map = new competition marks (key M in robotdrive)
  _robot_ready || return 1
  robottime
  _robot_stop_old
  _robot_start_engine || return 1
  echo "2/3 SLAM starting on the Pi ..."
  robotslam < /dev/null > ~/.robodojo/slam.log 2>&1 &
  sleep 3      # RViz can open while SLAM finishes starting
  echo "3/3 RViz starting ..."
  robotrviz > ~/.robodojo/rviz.log 2>&1 &
  echo
  echo "READY. Open a 2nd window and type:  robotdrive   (drive the whole field slowly)"
  echo "When the map is good, in another window type:  robotsavemap"
  echo "Ctrl+C HERE stops the engine, SLAM and RViz."
}
function _robotrun_steps {
  trap '_robot_cleanup; trap - INT TERM EXIT; kill 0' INT TERM
  local m="${1:-robodojo_field}"
  mkdir -p ~/.robodojo
  _robot_ready || return 1
  robottime
  if ! ssh -o BatchMode=yes "${PI:-user@PI-IP}" "test -f ~/maps/${m}.yaml"; then
    echo "No saved map ~/maps/${m}.yaml on the Pi - make one with robotmap + robotsavemap first."; return 1
  fi
  _robot_stop_old
  local t0=$SECONDS
  _robot_start_engine || return 1
  echo "    ($((SECONDS - t0)) s)"
  # Nav2 and RViz start at the SAME time (RViz needs ~10-20 s to open on the laptop anyway)
  echo "2/3 Nav2 starting on the Pi with map '${m}' ..."
  robotnav "$m" < /dev/null > ~/.robodojo/nav.log 2>&1 &
  echo "3/3 RViz opening on the laptop ..."
  robotrviz > ~/.robodojo/rviz.log 2>&1 &
  local i n
  # 23 Nov: on a slow start Nav2 needed more than 90 s, robotrun gave up and SKIPPED the alignment
  # -> the robot did not know where it was. Now wait up to 3 min, and never skip the alignment.
  for i in $(seq 1 180); do
    n=$(grep -c 'Managed nodes are active' ~/.robodojo/nav.log 2>/dev/null)
    [ "${n:-0}" -ge 2 ] && break
    [ $((i % 30)) -eq 0 ] && echo "    still waiting for Nav2 ($i s) ..."
    sleep 1
  done
  echo
  if [ "$(grep -c 'Managed nodes are active' ~/.robodojo/nav.log 2>/dev/null)" -ge 2 ]; then
    echo "Navigation is ACTIVE after $((SECONDS - t0)) s. Finding the robot on the map (robotlocalize) ..."
    if [ "$m" = "robodojo_field" ]; then robotlocalize; else robotlocalize "$m"; fi
    echo
    echo "READY after $((SECONDS - t0)) s. In RViz: the red LiDAR dots should sit on the black walls (if not: 2D Pose Estimate),"
    echo "then click  Nav2 Goal  and a white spot on the map."
  else
    echo "Nav2 is NOT active after 3 min - the robot is NOT aligned. Do not run missions now."
    echo "Important lines:"
    grep -hE "ERROR|Failed|frame does not exist|STOP" ~/.robodojo/nav.log | tail -10
    echo "Last lines:"; tail -5 ~/.robodojo/nav.log
    echo "-> Ctrl+C here, put the robot on its start marks, and run  robotrun $m  again."
  fi
  echo "Keep robotdrive open in a 2nd window: SPACE = emergency stop.   Ctrl+C HERE stops everything."
  echo "TIP: keep this running for the whole session - send new goals any time, no need to restart."
}
# robotnavcheck / gznavcheck -> "why is the robot not moving?" - checks everything Nav2 needs (~10 s)
function robotnavcheck { robotros && python3 ~/nav_check.py; }
# robotmapcheck -> "why is my map not clean?" sensors, IMU (direction/drift/size), wheel slip,
#                  wheel distance; offers to save the corrected calibration on the Pi (~1 min)
function robotmapcheck { robotros && python3 ~/map_check.py; }
function gznavcheck { _robodojo_env; python3 ~/nav_check.py --gazebo; }
# robotcover / gzcover -> the robot drives around the WHOLE field by itself (after robotrun / gzrun READY)
#   robotcover --spacing 0.4  (more points)     robotcover --loop  (repeat until Ctrl+C)
# ---- COMPETITION RUN (one command) ----
# robotmark NAME -> mark the competition places on an EXISTING map (no remapping): run it while
#                   robotrun NAME says READY. Drive with the arrows, press M at each place (K = skip),
#                   q when done -> saved as ~/maps/NAME_points.yaml
function robotmark { [ -n "$1" ] || { echo "Usage: robotmark NAME   (while robotrun NAME is running)"; return 1; }
  rm -f ~/.robodojo/marks_current.yaml; mkdir -p ~/.robodojo
  robotros && PI="$PI" ROBODOJO_REAL=1 python3 ~/arrow_teleop.py; _keep_marks "$1"; }
# robotremark NAME PLACE -> mark ONE place again (e.g. robotremark jane1 gate_check), keeping all the
#                           others. robotrun NAME must be READY. Drive there, press M, then q.
# gzremark PLACE         -> the same in Gazebo (while gzrun fast is running)
function _remark_prepare {   # $1 = map name  $2 = place
  [ -f ~/maps/"$1"_points.yaml ] || { echo "No marks for $1 yet - use robotmark $1 / gzmark first."; return 1; }
  mkdir -p ~/.robodojo
  cp ~/maps/"$1"_points.yaml ~/maps/"$1"_points.backup.yaml   # undo: copy this back
  python3 - ~/maps/"$1"_points.yaml ~/.robodojo/marks_current.yaml "$2" <<'PYEOF' || return 1
import sys, yaml
src, dst, place = sys.argv[1:4]
d = yaml.safe_load(open(src)) or {}
raw = d.get('_raw') or {}
names = ['screen', 'load_plastic', 'load_organic', 'load_metal', 'route_grass', 'route_obstacles',
         'route_ramp_top', 'route_ramp_bottom', 'gate_check', 'after_gate', 'detour',
         'offload_plastic', 'offload_organic', 'offload_metal']
if place not in names:
    sys.exit('Unknown place "%s". Use one of: %s' % (place, ' '.join(names)))
raw.pop(place, None); raw.pop(place + '_skip', None)
d.pop(place, None)
d['_raw'] = raw
yaml.safe_dump(d, open(dst, 'w'), sort_keys=False)
print('Re-marking only: %s  (all other places are kept). Drive there, face the right way, press M, then q.' % place)
PYEOF
}
function robotremark { [ -n "$2" ] || { echo "Usage: robotremark NAME PLACE   e.g.  robotremark jane1 gate_check"; return 1; }
  _remark_prepare "$1" "$2" || return 1
  robotros && PI="$PI" ROBODOJO_REAL=1 python3 ~/arrow_teleop.py; _remark_finish "$1" "$2"; }
function _remark_finish {   # keep the new marks only if the place was really marked (M)
  if grep -q "^  $2:" ~/.robodojo/marks_current.yaml 2>/dev/null; then _keep_marks "$1"
  else
    echo "NOT changed: $2 was not marked (no M pressed). Your old marks are kept as they were."
  fi
}
function gzremark { [ -n "$1" ] || { echo "Usage: gzremark PLACE   e.g.  gzremark gate_check"; return 1; }
  _remark_prepare robodojo_field "$1" || return 1
  _robodojo_env; python3 ~/arrow_teleop.py; _remark_finish robodojo_field "$1"; }
# robotwhere NAME -> CHECK THE MARKS: drive the robot (robotdrive) onto a station, then run this.
#                    It says where the robot is on the map and WHICH MARK is nearest
#                    (e.g. standing on the BLUE station must say load_plastic). gzwhere = Gazebo.
function _where {   # $1 = map name  $2 = sim flag
  python3 - "$HOME/maps/$1_points.yaml" "$2" <<'PYEOF'
import math, sys, time, yaml, rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
P = yaml.safe_load(open(sys.argv[1]))
rclpy.init(); n = rclpy.create_node('robodojo_where'); got = {}
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', lambda m: got.setdefault('p', m), q)
t = time.time()
while 'p' not in got and time.time() - t < 5:
    rclpy.spin_once(n, timeout_sec=0.2)
if 'p' not in got:
    sys.exit('No robot position from navigation - is robotrun / gzrun running (ALIGNED OK)?')
x, y = got['p'].pose.pose.position.x, got['p'].pose.pose.position.y
pl = sorted((math.hypot(v['x'] - x, v['y'] - y), k) for k, v in P.items()
            if isinstance(v, dict) and 'x' in v)
print(f'Robot is at x {x:.2f}  y {y:.2f}')
for d, k in pl[:3]:
    print(f'   {k:16s} {d:4.2f} m away')
d, k = pl[0]
print(f'-> NEAREST MARK: {k}' + ('' if d < 0.4 else '   (but > 0.4 m away: no mark is really here)'))
PYEOF
}
function robotwhere { [ -n "$1" ] || { echo "Usage: robotwhere NAME   e.g. robotwhere jane1"; return 1; }
  robotros >/dev/null && _where "$1"; }
function gzwhere { _robodojo_env; _where robodojo_field sim; }
# robotswap NAME PLACE1 PLACE2 -> swap two marks without driving (e.g. the plastic and organic
#                                 stations were saved the wrong way round). gzswap = Gazebo.
function _swap {   # $1 file  $2 $3 places
  [ -f "$1" ] || { echo "No marks file $1"; return 1; }
  cp "$1" "${1%.yaml}.backup.yaml"
  python3 - "$1" "$2" "$3" <<'PYEOF'
import sys, yaml
f, a, b = sys.argv[1:4]
d = yaml.safe_load(open(f))
for box in (d, d.get('_raw') or {}):
    if a not in box or b not in box:
        if box is d:
            sys.exit(f'Both must be marked: {a}, {b}')
        continue
    box[a], box[b] = box[b], box[a]
yaml.safe_dump(d, open(f, 'w'), sort_keys=False)
print(f'Swapped {a} <-> {b}  (backup: {f[:-5]}.backup.yaml)')
PYEOF
}
function robotswap { [ -n "$3" ] || { echo "Usage: robotswap NAME PLACE1 PLACE2   e.g. robotswap jane1 load_plastic load_organic"; return 1; }
  _swap ~/maps/"$1"_points.yaml "$2" "$3"; }
function gzswap { [ -n "$2" ] || { echo "Usage: gzswap PLACE1 PLACE2"; return 1; }; _swap ~/maps/robodojo_field_points.yaml "$1" "$2"; }
# robotturntest -> measures whether the robot knows how much it turned (gyro vs LiDAR).
#                  robotrun NAME must be READY (ALIGNED OK); robot in an open spot.
function robotturntest { robotros >/dev/null && python3 ~/robot_turntest.py; }
# gzmark -> the same in Gazebo (while  gzrun fast  is running)
function gzmark { rm -f ~/.robodojo/marks_current.yaml; mkdir -p ~/.robodojo
  _robodojo_env; python3 ~/arrow_teleop.py; _keep_marks robodojo_field; }
# robotpoints NAME  -> once per map: click screen, stations, route, gate, detour in RViz
#                      (Publish Point button). Needs  robotrun NAME  running (READY).
function robotpoints { [ -n "$1" ] || { echo "Usage: robotpoints NAME   (the map name used with robotrun)"; return 1; }
  robotros || return 1; python3 ~/robot_points.py "$1"; }
# robotcompete NAME [--load-wait 5] [--gate-wait 5] -> THE WHOLE RUN by itself. Before: robotrun NAME
#                      (ALIGNED OK) and camera ON (C in robotdrive). Press Enter on the judge's GO.
function robotcompete { [ -n "$1" ] || { echo "Usage: robotcompete NAME"; return 1; }
  local m="$1"; shift; robotros || return 1
  python3 ~/robot_compete.py "$m" --pi "${PI#*@}" "$@"; }
# gzpoints / gzcompete [plastic|metal|organic] -> the same in Gazebo (after  gzrun fast  says READY)
function gzpoints { _robodojo_env; python3 ~/robot_points.py robodojo_field --sim; }
function gzcompete { _robodojo_env; local w="${1:-plastic}"; [ $# -gt 0 ] && shift
  python3 ~/robot_compete.py robodojo_field --sim --waste "$w" "$@"; }
function robotcover { robotros && python3 ~/field_cover.py "$@"; }
function gzcover { _robodojo_env; python3 ~/field_cover.py "$@"; }
function robotmap { ( _robotmap_steps; wait ); }
function robotrun { ( _robotrun_steps "$@"; wait ); }
# <<< JNNP RoboDojo shortcuts <<<
