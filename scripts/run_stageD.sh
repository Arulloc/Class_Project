#!/bin/bash
# run_stageD.sh: Stage D experiments - capture using the camera, with extra
# measurement delay added, with and without delay correction.
#
# Same as Stage C (camera -> predictor -> capture), except that the predictor
# holds every camera measurement back for an extra DELAY seconds before using
# it, imitating a slower camera or computer. Each delay is run twice:
#   corrected   - the predictor knows when each measurement was taken and
#                 predicts forward from that time (allows for the delay)
#   uncorrected - the predictor treats each late measurement as current
# Capture is still graded against the exact handle position.
# Results go to results/stageD/ (or results/stageD_moveit/ with MOVEIT=1).
# Then run:  python3 scripts/analyze_stageD.py results/stageD_moveit
#
# Before running: Gazebo and the arm must already be running (Terminal 1),
# and with MOVEIT=1, MoveIt too (Terminal 2), exactly as for Stage C.
# Do NOT start the target, camera, detector or predictor yourself.
#
# Usage:  MOVEIT=1 bash scripts/run_stageD.sh        full set (about 40 minutes)
#         MOVEIT=1 bash scripts/run_stageD.sh test   0.4 s delay, kalman, 3 tries each
#         DELAYS="0.6" MOVEIT=1 bash scripts/run_stageD.sh   only some delays
#             (keeps the other results)

cd "$(dirname "$0")/.."
source install/setup.bash
MOVEIT=${MOVEIT:-0}
if [ "$MOVEIT" = "1" ]; then
  OUT="$(pwd)/results/stageD_moveit"
  IK_SOURCE=moveit
else
  OUT="$(pwd)/results/stageD"
  IK_SOURCE=own
fi

# Same camera pose and Kalman setting as Stage C
CAM_X=0.75; CAM_Y=0.0; CAM_Z=0.75; CAM_PITCH=1.107; CAM_YAW=3.1416
KF_R=0.00024
mkdir -p "$OUT"
export PYTHONUNBUFFERED=1

# One spin speed (10 s spin = 0.63 rad/s, the speed used for the delay test
# in Stage A), the two methods that captured reliably with the camera, and
# three added delays, each with and without correction.
PERIOD=${PERIOD:-10}
METHODS=${METHODS:-"average kalman"}
DELAYS=${DELAYS:-"0.2 0.4 0.6"}
CORRECTIONS="1 0"
TRIALS=10
if [ "$1" = "test" ]; then
  METHODS="kalman"; DELAYS="0.4"; TRIALS=3
fi

{
  echo "date: $(date -Iseconds)"
  echo "branch: $(git branch --show-current)"
  echo "commit: $(git rev-parse --short HEAD)"
  echo "uncommitted changes: $(git status --porcelain | wc -l) files"
  echo "methods: $METHODS   period: $PERIOD   delays: $DELAYS   trials: $TRIALS   input: camera"
  echo "arm poses from: $IK_SOURCE"
  echo "camera pose: x=$CAM_X y=$CAM_Y z=$CAM_Z pitch=$CAM_PITCH yaw=$CAM_YAW   kf_r=$KF_R"
} >> "$OUT/run_info.txt"

stop_all() {
  pkill -f target_motion > /dev/null 2>&1
  pkill -f "rotating_target/predictor" > /dev/null 2>&1
  pkill -f "scripts/capture.py" > /dev/null 2>&1
  pkill -f "scripts/tag_detector.py" > /dev/null 2>&1
  sleep 2
}

remove_target() {
  ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity \
    "{name: rotating_target}" > /dev/null 2>&1
}

stop_all

if [ "$MOVEIT" = "1" ] && ! ros2 service list | grep -q "^/compute_ik$"; then
  echo "STOPPED: MOVEIT=1 but MoveIt is not running (no /compute_ik service)."
  exit 1
fi

# camera, unchanged, in the same place as Stage C
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
if [ -z "$INFO_TOPIC" ] || ! timeout 30 ros2 topic echo "$INFO_TOPIC" \
     sensor_msgs/msg/CameraInfo --once > /dev/null 2>&1; then
  echo "STOPPED: camera not publishing; see $OUT/camera_spawn.log"; exit 1
fi
echo "camera ready"

F_PERIOD=$(python3 -c "print(float($PERIOD))")
echo "=== spin period ${PERIOD} s"
remove_target
ros2 launch rotating_target target.launch.py mode:=spin spin_period:="$PERIOD" \
  > "$OUT/target_P${PERIOD}.log" 2>&1 &
TARGET_PID=$!
if ! timeout 90 ros2 topic echo /target/handle_true \
     geometry_msgs/msg/PoseStamped --once > /dev/null 2>&1; then
  echo "STOPPED: target did not start; see $OUT/target_P${PERIOD}.log"
  kill -TERM $TARGET_PID 2>/dev/null; stop_all; exit 1
fi
for i in 1 2 3 4 5; do
  NPUB=$(ros2 topic info /target/handle_true 2>/dev/null | awk '/Publisher count/ {print $3}')
  [ -n "$NPUB" ] && break
  sleep 2
done
if [ "$NPUB" != "1" ]; then
  echo "STOPPED: $NPUB publishers on /target/handle_true (must be 1)."
  kill -TERM $TARGET_PID 2>/dev/null; stop_all; exit 1
fi

python3 scripts/tag_detector.py --ros-args -p use_sim_time:=true \
  -p cam_x:=$CAM_X -p cam_y:=$CAM_Y -p cam_z:=$CAM_Z \
  -p cam_pitch:=$CAM_PITCH -p cam_yaw:=$CAM_YAW \
  -p image_topic:=$IMG_TOPIC -p info_topic:=$INFO_TOPIC \
  -p log_file:="$OUT/vision_P${PERIOD}.csv" > "$OUT/detector_P${PERIOD}.log" 2>&1 &
if ! timeout 60 ros2 topic echo /target/handle_measured \
     geometry_msgs/msg/PoseStamped --once > /dev/null 2>&1; then
  echo "STOPPED: the camera is not finding the tag; see $OUT/detector_P${PERIOD}.log"
  kill -TERM $TARGET_PID 2>/dev/null; stop_all; exit 1
fi

for D in $DELAYS; do
  F_D=$(python3 -c "print(float($D))")
  for C in $CORRECTIONS; do
    if [ "$C" = "1" ]; then COMP=true; CNAME=corrected; else COMP=false; CNAME=uncorrected; fi
    for M in $METHODS; do
      TAG="P${PERIOD}_${M}_d${D}_c${C}"
      echo "  delay ${D} s, ${CNAME}, ${M}"
      ros2 run rotating_target predictor --ros-args -p method:=$M \
        -p spin_period:=$F_PERIOD -p kf_r:=$KF_R \
        -p delay:=$F_D -p compensate:=$COMP \
        -r /target/handle_true:=/target/handle_measured \
        > "$OUT/predictor_${TAG}.log" 2>&1 &
      sleep 3
      if [ "$MOVEIT" = "1" ]; then
        timeout 60 python3 scripts/move_to_point_moveit.py 0.37 0 0.42 --rpy 3.1416 0 0 \
          >> "$OUT/moveit_${TAG}.log" 2>&1 || echo "    (MoveIt move to hover failed; capture.py will move the arm itself)"
      fi
      timeout 900 python3 scripts/capture.py --ros-args -p use_sim_time:=true \
        -p method:=$M -p spin_period:=$F_PERIOD -p trials:=$TRIALS \
        -p ik_source:=$IK_SOURCE -p log_file:="$OUT/stageD_${TAG}.csv" > "$OUT/capture_${TAG}.log" 2>&1
      STATUS=$?
      pkill -f "rotating_target/predictor" > /dev/null 2>&1
      F="$OUT/stageD_${TAG}.csv"
      if [ -f "$F" ] && [ "$(wc -l < "$F")" -gt 1 ]; then
        N=$(($(wc -l < "$F") - 1))
        OK=$(awk -F, 'NR > 1 {s += $12} END {print s + 0}' "$F")
        echo "    captured $OK of $N"
      else
        echo "    no results; see $OUT/capture_${TAG}.log"
      fi
      if [ "$STATUS" = "2" ]; then
        echo "  STOPPED: the arm was knocked out of position (delay $D, $CNAME, $M)."
        echo "  See $OUT/capture_${TAG}.log. Restart Gazebo before running again."
        kill -TERM $TARGET_PID 2>/dev/null; stop_all; remove_target; exit 1
      fi
      sleep 2
    done
  done
done
kill -TERM $TARGET_PID 2>/dev/null; wait $TARGET_PID 2>/dev/null
stop_all
remove_target

echo ""
echo "Done. Results in $OUT"
echo "Next:  python3 scripts/analyze_stageD.py $OUT"
