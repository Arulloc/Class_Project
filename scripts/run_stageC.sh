#!/bin/bash
# run_stageC.sh: Stage C experiments - capture using the external camera.
#
# Same as Stage B, except the predictor no longer gets the exact handle
# position: it gets the position measured by the camera (scripts/tag_detector.py
# finds the AprilTag on the target). Capture is still graded against the
# exact position. Results go to results/stageC/.
# Then run:  python3 scripts/analyze_stageB.py results/stageC
#            python3 scripts/analyze_vision.py
#
# Before running: Gazebo and the arm must already be running (Terminal 1),
# launched with the updated launch file (so the AprilTag image shows up).
# Do NOT start the target, camera, detector or predictor yourself.
#
# Usage:  bash scripts/run_stageC.sh          full set (about 25-30 minutes)
#         bash scripts/run_stageC.sh test     10 s spin, reactive + kalman, 3 tries each
#         MOVEIT=1 bash scripts/run_stageC.sh [test]
#             use MoveIt (Chris's move_to_point_moveit.py) to move the arm to
#             the hover pose, and MoveIt's inverse kinematics for the two poses.
#             MoveIt must already be running (Terminal 2):
#             ros2 launch kinova_gen3_6dof_robotiq_2f_85_moveit_config sim.launch.py \
#                 sim_ignition:=false use_sim_time:=true

cd "$(dirname "$0")/.."
source install/setup.bash
MOVEIT=${MOVEIT:-0}
if [ "$MOVEIT" = "1" ]; then
  OUT="$(pwd)/results/stageC_moveit"
  IK_SOURCE=moveit
else
  OUT="$(pwd)/results/stageC"
  IK_SOURCE=own
fi

# Camera pose in the world (m, rad): 25 cm beyond the target, 75 cm up,
# looking back down at the tag at about 63 degrees, on the far side from the
# arm so the arm does not block the view. The detector gets the same numbers.
CAM_X=0.75; CAM_Y=0.0; CAM_Z=0.75; CAM_PITCH=1.107; CAM_YAW=3.1416
# Measurement noise the Kalman filter assumes for the camera, as an angle
# variance (rad^2): about 2 mm at the 0.13 m handle radius, (0.002/0.13)^2
KF_R=0.00024
mkdir -p "$OUT"
export PYTHONUNBUFFERED=1

METHODS="reactive last_two average kalman"
# Spin periods to run; to run only some (e.g. after a stop):
#   PERIODS="5 3" bash scripts/run_stageC.sh   (keeps the other results)
PERIODS=${PERIODS:-"20 10 5 3"}
TRIALS=10
if [ "$1" = "test" ]; then
  METHODS="reactive kalman"; PERIODS="10"; TRIALS=3
fi

{
  echo "date: $(date -Iseconds)"
  echo "branch: $(git branch --show-current)"
  echo "commit: $(git rev-parse --short HEAD)"
  echo "uncommitted changes: $(git status --porcelain | wc -l) files"
  echo "methods: $METHODS   periods: $PERIODS   trials: $TRIALS   input: camera"
  echo "arm poses from: $IK_SOURCE"
  echo "camera pose: x=$CAM_X y=$CAM_Y z=$CAM_Z pitch=$CAM_PITCH yaw=$CAM_YAW   kf_r=$KF_R"
} >> "$OUT/run_info.txt"

stop_all() {
  # Stop every target controller, predictor and capture program, however
  # they were started, so only one of each ever runs.
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

# Put Aleeya's camera (models/standalone_camera.urdf, unchanged) in place,
# removing any old copy first. Its link has gravity turned off, so it stays
# where it is spawned.
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity \
  "{name: ext_camera}" > /dev/null 2>&1
ros2 run gazebo_ros spawn_entity.py -entity ext_camera \
  -file "$(pwd)/models/standalone_camera.urdf" \
  -x $CAM_X -y $CAM_Y -z $CAM_Z -R 0 -P $CAM_PITCH -Y $CAM_YAW \
  > "$OUT/camera_spawn.log" 2>&1
# Find the camera's topics (their names come from the camera file)
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
echo "camera topics: $IMG_TOPIC  $INFO_TOPIC" | tee -a "$OUT/run_info.txt"
echo "camera ready"

for PERIOD in $PERIODS; do
  F_PERIOD=$(python3 -c "print(float($PERIOD))")
  echo "=== spin period ${PERIOD} s"
  remove_target
  ros2 launch rotating_target target.launch.py mode:=spin spin_period:="$PERIOD" \
    > "$OUT/target_P${PERIOD}.log" 2>&1 &
  TARGET_PID=$!
  if ! timeout 90 ros2 topic echo /target/handle_true \
       geometry_msgs/msg/PoseStamped --once > /dev/null 2>&1; then
    echo "  target did not start; see $OUT/target_P${PERIOD}.log"
    kill -TERM $TARGET_PID 2>/dev/null; stop_all
    continue
  fi
  NPUB=$(ros2 topic info /target/handle_true 2>/dev/null | awk '/Publisher count/ {print $3}')
  if [ "$NPUB" != "1" ]; then
    echo "  STOPPED: $NPUB publishers on /target/handle_true (must be 1)."
    kill -TERM $TARGET_PID 2>/dev/null; stop_all; exit 1
  fi

  # Camera tag detector for this spin period (also logs camera error)
  python3 scripts/tag_detector.py --ros-args -p use_sim_time:=true \
    -p cam_x:=$CAM_X -p cam_y:=$CAM_Y -p cam_z:=$CAM_Z \
    -p cam_pitch:=$CAM_PITCH -p cam_yaw:=$CAM_YAW \
    -p image_topic:=$IMG_TOPIC -p info_topic:=$INFO_TOPIC \
    -p log_file:="$OUT/vision_P${PERIOD}.csv" > "$OUT/detector_P${PERIOD}.log" 2>&1 &
  if ! timeout 60 ros2 topic echo /target/handle_measured \
       geometry_msgs/msg/PoseStamped --once > /dev/null 2>&1; then
    echo "  STOPPED: the camera is not finding the tag; see $OUT/detector_P${PERIOD}.log"
    kill -TERM $TARGET_PID 2>/dev/null; stop_all; exit 1
  fi

  for M in $METHODS; do
    TAG="P${PERIOD}_${M}"
    # The predictor reads the camera's measurement instead of the exact one
    ros2 run rotating_target predictor --ros-args -p method:=$M \
      -p spin_period:=$F_PERIOD -p kf_r:=$KF_R \
      -r /target/handle_true:=/target/handle_measured \
      > "$OUT/predictor_${TAG}.log" 2>&1 &
    sleep 3
    if [ "$MOVEIT" = "1" ]; then
      # Chris's MoveIt script: wrist 12 cm above the hover point, gripper down
      python3 scripts/move_to_point_moveit.py 0.37 0 0.42 --rpy 3.1416 0 0 \
        >> "$OUT/moveit_${TAG}.log" 2>&1 || echo "  (MoveIt move to hover failed; capture.py will move the arm itself)"
    fi
    # Generous time limit: 10 tries at a 20 s spin take about 4 minutes
    timeout 900 python3 scripts/capture.py --ros-args -p use_sim_time:=true \
      -p method:=$M -p spin_period:=$F_PERIOD -p trials:=$TRIALS \
      -p ik_source:=$IK_SOURCE -p log_file:="$OUT/stageC_${TAG}.csv" > "$OUT/capture_${TAG}.log" 2>&1
    STATUS=$?
    pkill -f "rotating_target/predictor" > /dev/null 2>&1
    F="$OUT/stageC_${TAG}.csv"
    if [ -f "$F" ] && [ "$(wc -l < "$F")" -gt 1 ]; then
      N=$(($(wc -l < "$F") - 1))
      OK=$(awk -F, 'NR > 1 {s += $12} END {print s + 0}' "$F")
      echo "  $M: captured $OK of $N"
    else
      echo "  $M: no results; see $OUT/capture_${TAG}.log"
    fi
    if [ "$STATUS" = "2" ]; then
      echo "  STOPPED: the arm was knocked out of position during $M at ${PERIOD} s."
      echo "  See $OUT/capture_${TAG}.log. Restart Gazebo before running again."
      kill -TERM $TARGET_PID 2>/dev/null; stop_all; remove_target; exit 1
    fi
    sleep 2
  done
  kill -TERM $TARGET_PID 2>/dev/null; wait $TARGET_PID 2>/dev/null
  stop_all
done
remove_target

echo ""
echo "Done. Results in $OUT"
echo "Next:  python3 scripts/analyze_stageB.py results/stageC"
echo "       python3 scripts/analyze_vision.py"
