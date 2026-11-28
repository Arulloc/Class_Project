#!/bin/bash
# run_demo.sh: one capture, for a video of the class demo (about 1-2 minutes).
#
# Runs the whole system once, as in Stage C: the camera finds the tag, the
# Kalman filter predicts the handle, MoveIt moves the arm to the hover pose,
# and the capture program times the descent. When the capture succeeds, the
# target is attached to the gripper (scripts/demo_pickup.py; the simulated
# gripper cannot close) and MoveIt lifts it 23 cm. Up to 3 attempts.
#
# Before running: Gazebo and the arm (Terminal 1) and MoveIt (Terminal 2),
# exactly as for Stage C. Start your screen recording, then run:
#   bash scripts/run_demo.sh
# Options:
#   PERIOD=5 bash scripts/run_demo.sh    faster spin (default 10 s per turn)
#   CAMERA=0 bash scripts/run_demo.sh    use the exact position, no camera

cd "$(dirname "$0")/.."
source install/setup.bash
OUT="$(pwd)/results/demo"
mkdir -p "$OUT"
export PYTHONUNBUFFERED=1
PERIOD=${PERIOD:-10}
CAMERA=${CAMERA:-1}
F_PERIOD=$(python3 -c "print(float($PERIOD))")
CAM_X=0.75; CAM_Y=0.0; CAM_Z=0.75; CAM_PITCH=1.107; CAM_YAW=3.1416
KF_R=0.00024
STATUS="$OUT/status.txt"

stop_all() {
  pkill -f target_motion > /dev/null 2>&1
  pkill -f "rotating_target/predictor" > /dev/null 2>&1
  pkill -f "scripts/capture.py" > /dev/null 2>&1
  pkill -f "scripts/tag_detector.py" > /dev/null 2>&1
  pkill -f "scripts/demo_pickup.py" > /dev/null 2>&1
  sleep 2
}

stop_all
if ! ros2 service list | grep -q "^/compute_ik$"; then
  echo "STOPPED: MoveIt is not running (start it in Terminal 2 first)."
  exit 1
fi

echo "Setting up..."
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity \
  "{name: rotating_target}" > /dev/null 2>&1
ros2 launch rotating_target target.launch.py mode:=spin spin_period:="$PERIOD" \
  > "$OUT/target.log" 2>&1 &
if ! timeout 90 ros2 topic echo /target/handle_true \
     geometry_msgs/msg/PoseStamped --once > /dev/null 2>&1; then
  echo "STOPPED: target did not start; see $OUT/target.log"; stop_all; exit 1
fi

REMAP=""
if [ "$CAMERA" = "1" ]; then
  ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity \
    "{name: ext_camera}" > /dev/null 2>&1
  ros2 run gazebo_ros spawn_entity.py -entity ext_camera \
    -file "$(pwd)/models/standalone_camera.urdf" \
    -x $CAM_X -y $CAM_Y -z $CAM_Z -R 0 -P $CAM_PITCH -Y $CAM_YAW \
    > "$OUT/camera_spawn.log" 2>&1
  for i in $(seq 1 30); do
    IMG_TOPIC=$(ros2 topic list | grep 'image_raw$' | head -1)
    INFO_TOPIC=$(ros2 topic list | grep 'camera_info$' | head -1)
    [ -n "$IMG_TOPIC" ] && [ -n "$INFO_TOPIC" ] && break
    sleep 1
  done
  python3 scripts/tag_detector.py --ros-args -p use_sim_time:=true \
    -p cam_x:=$CAM_X -p cam_y:=$CAM_Y -p cam_z:=$CAM_Z \
    -p cam_pitch:=$CAM_PITCH -p cam_yaw:=$CAM_YAW \
    -p image_topic:=$IMG_TOPIC -p info_topic:=$INFO_TOPIC \
    > "$OUT/detector.log" 2>&1 &
  if ! timeout 60 ros2 topic echo /target/handle_measured \
       geometry_msgs/msg/PoseStamped --once > /dev/null 2>&1; then
    echo "STOPPED: the camera is not finding the tag; see $OUT/detector.log"
    stop_all; exit 1
  fi
  REMAP="-r /target/handle_true:=/target/handle_measured"
  echo "Camera is tracking the tag."
fi

ros2 run rotating_target predictor --ros-args -p method:=kalman \
  -p spin_period:=$F_PERIOD -p kf_r:=$KF_R $REMAP > "$OUT/predictor.log" 2>&1 &
python3 scripts/demo_pickup.py --ros-args -p use_sim_time:=true \
  -p status_file:="$STATUS" > "$OUT/pickup.log" 2>&1 &
sleep 3

RESULT=MISSED
for TRY in 1 2 3; do
  echo "Attempt $TRY: moving to the hover pose (MoveIt)..."
  echo "WAITING" > "$STATUS"
  timeout 60 python3 scripts/move_to_point_moveit.py 0.37 0 0.42 --rpy 3.1416 0 0 \
    >> "$OUT/moveit.log" 2>&1
  echo "Attempt $TRY: waiting for the handle..."
  timeout 300 python3 scripts/capture.py --ros-args -p use_sim_time:=true \
    -p method:=kalman -p spin_period:=$F_PERIOD -p trials:=1 \
    -p ik_source:=moveit > "$OUT/capture_try$TRY.log" 2>&1
  RESULT=$(cat "$STATUS" 2>/dev/null)
  grep -h "CAPTURED\|MISSED" "$OUT/pickup.log" | tail -1
  [ "$RESULT" = "CAPTURED" ] && break
done

if [ "$RESULT" = "CAPTURED" ]; then
  echo "Lifting the target (MoveIt)..."
  timeout 60 python3 scripts/move_to_point_moveit.py 0.37 0 0.65 --rpy 3.1416 0 0 \
    >> "$OUT/moveit.log" 2>&1
  echo "Done. The target is held up in the gripper."
else
  echo "No capture in 3 attempts; see $OUT/pickup.log"
fi

echo "Press Enter to finish (stops the demo programs; the target stays where it is)."
read
stop_all
