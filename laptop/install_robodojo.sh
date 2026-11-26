#!/bin/bash
# JNNP RoboDojo - install ALL the newest files at once (laptop + Gazebo + robot).
#
#   bash ~/Downloads/install_robodojo.sh        <- not needed, see below
#   1) download all the files from Claude into Windows Downloads
#   2) in the laptop's Ubuntu window:
#        cp "$(ls -t /mnt/c/Users/*/Downloads/install_robodojo*.sh | head -1)" ~/install_robodojo.sh && bash ~/install_robodojo.sh
#   Robot not switched on now?  The laptop + Gazebo part is done; later, with the robot on, run:
#        bash ~/install_robodojo.sh robot

LAPTOP="arrow_teleop.py robodojo_shortcuts.sh robot_compete.py robot_turntest.py robot_points.py robot_localize.py fix_gazebo_nav_self.py waste_laptop.py screen_focus.py"
PI_HOME="make_nav2_params.py make_slam_params.py add_wall_guard.py"
PI_VISION="waste_live.py screen_focus.py"
STAGE=~/robodojo_pi_files            # robot files are kept here until the robot is reachable

D=$(ls -d /mnt/c/Users/*/Downloads 2>/dev/null | grep -v -e Public -e Default | head -1)
newest() {   # newest download of a file, also "name (3).py"
  local n="${1%.*}" e="${1##*.}"
  ls -t "$D/$n".$e "$D/$n "*."$e" "$D/$n("*."$e" ~/Downloads/"$n"*."$e" 2>/dev/null | head -1
}

if [ "$1" != "robot" ]; then
  echo "=== 1. Laptop files (from $D) ==="
  ok=1
  for f in $LAPTOP; do
    src=$(newest "$f")
    if [ -n "$src" ]; then cp "$src" ~/"$f" && echo "  OK       $f"; else echo "  MISSING  $f  (download it from Claude)"; ok=0; fi
  done
  mkdir -p "$STAGE"
  for f in $PI_HOME waste_live.py; do
    src=$(newest "$f")
    if [ -n "$src" ]; then cp "$src" "$STAGE/$f" && echo "  OK       $f   (for the robot)"; else echo "  MISSING  $f  (download it from Claude)"; ok=0; fi
  done
  cp ~/screen_focus.py "$STAGE/screen_focus.py" 2>/dev/null
  echo
  echo "=== 2. Gazebo settings ==="
  python3 ~/fix_gazebo_nav_self.py | grep -E "Robot shape|Path follower|Path planning|Blocked|Done|LiDAR|Saved"
  echo
fi

echo "=== 3. Robot (Pi) files ==="
source ~/robodojo_shortcuts.sh >/dev/null 2>&1
if _pi_find >/dev/null 2>&1; then
  scp -q "$STAGE"/make_nav2_params.py "$STAGE"/make_slam_params.py "$PI":~/ && \
  { [ -f "$STAGE"/add_wall_guard.py ] && scp -q "$STAGE"/add_wall_guard.py "$PI":~/ && pi "python3 ~/add_wall_guard.py" || true; } && \
  scp -q "$STAGE"/waste_live.py "$STAGE"/screen_focus.py "$PI":~/waste_ai/ && \
  pi "python3 ~/make_nav2_params.py | grep -E 'Robot shape|Progress|Blocked'; python3 ~/make_slam_params.py | tail -1" && \
  echo "  Robot files installed."
else
  echo "  Robot not reachable now (switched off or other Wi-Fi). Laptop + Gazebo part is DONE."
  echo "  Later, with the robot on and on the same Wi-Fi:   bash ~/install_robodojo.sh robot"
fi
echo
echo "Finally: close this Ubuntu window and open a NEW one (so the new words are loaded)."
