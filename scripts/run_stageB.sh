#!/bin/bash
# run_stageB.sh: Stage B experiments - capture with ground truth (no camera).
#
# For each spin period it (re)starts the spinning target. Then, for each
# prediction method in turn, it starts the predictor with that method and runs
# scripts/capture.py, which makes the arm try to catch the handle TRIALS times
# and logs one CSV row per attempt into results/stageB/.
# Then run:  python3 scripts/analyze_stageB.py
#
# Before running: Gazebo and the arm must already be running (Terminal 1).
# Do NOT start the target or the predictor yourself; this script does it.
#
# Usage:  bash scripts/run_stageB.sh          full set (about 25-30 minutes)
#         bash scripts/run_stageB.sh test     10 s spin, reactive + kalman, 3 tries each
#         NOISE=0.005 bash scripts/run_stageB.sh
#         MOVEIT=1 ... use MoveIt (Chris's move_to_point_moveit.py) to move the
#             arm to hover, and MoveIt's inverse kinematics for the two poses.
#             MoveIt must already be running. Results go to a *_moveit folder.
#             same, but the predictor sees positions with 5 mm of noise
#             (imitating a camera); results go to results/stageB_noise5mm/

cd "$(dirname "$0")/.."
source install/setup.bash
NOISE=${NOISE:-0}
F_NOISE=$(python3 -c "print(float($NOISE))")
if [ "$F_NOISE" = "0.0" ]; then
  OUT="$(pwd)/results/stageB"
else
  OUT="$(pwd)/results/stageB_noise$(python3 -c "print(f'{$NOISE*1000:g}')")mm"
fi
MOVEIT=${MOVEIT:-0}
IK_SOURCE=own
if [ "$MOVEIT" = "1" ]; then
  OUT="${OUT}_moveit"
  IK_SOURCE=moveit
  if ! ros2 service list | grep -q "^/compute_ik$"; then
    echo "STOPPED: MOVEIT=1 but MoveIt is not running (no /compute_ik service)."
    exit 1
  fi
fi
mkdir -p "$OUT"
export PYTHONUNBUFFERED=1

METHODS="reactive last_two average kalman"
PERIODS="20 10 5 3"
TRIALS=10
if [ "$1" = "test" ]; then
  METHODS="reactive kalman"; PERIODS="10"; TRIALS=3
fi

{
  echo "date: $(date -Iseconds)"
  echo "branch: $(git branch --show-current)"
  echo "commit: $(git rev-parse --short HEAD)"
  echo "uncommitted changes: $(git status --porcelain | wc -l) files"
  echo "methods: $METHODS   periods: $PERIODS   trials: $TRIALS   noise: $F_NOISE m"
  echo "arm poses from: $IK_SOURCE"
} > "$OUT/run_info.txt"

stop_all() {
  # Stop every target controller, predictor and capture program, however
  # they were started, so only one of each ever runs.
  pkill -f target_motion > /dev/null 2>&1
  pkill -f "rotating_target/predictor" > /dev/null 2>&1
  pkill -f "scripts/capture.py" > /dev/null 2>&1
  sleep 2
}

remove_target() {
  ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity \
    "{name: rotating_target}" > /dev/null 2>&1
}

stop_all
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
  # Count the publishers (ask a few times: the first answer can come back empty)
  for i in 1 2 3 4 5; do
    NPUB=$(ros2 topic info /target/handle_true 2>/dev/null | awk '/Publisher count/ {print $3}')
    [ -n "$NPUB" ] && break
    sleep 2
  done
  if [ "$NPUB" != "1" ]; then
    echo "  STOPPED: $NPUB publishers on /target/handle_true (must be 1)."
    kill -TERM $TARGET_PID 2>/dev/null; stop_all; exit 1
  fi

  for M in $METHODS; do
    TAG="P${PERIOD}_${M}"
    ros2 run rotating_target predictor --ros-args -p method:=$M \
      -p spin_period:=$F_PERIOD -p noise_std:=$F_NOISE \
      > "$OUT/predictor_${TAG}.log" 2>&1 &
    sleep 3
    if [ "$MOVEIT" = "1" ]; then
      # Chris's MoveIt script: wrist 12 cm above the hover point, gripper down
      timeout 60 python3 scripts/move_to_point_moveit.py 0.37 0 0.42 --rpy 3.1416 0 0 \
        >> "$OUT/moveit_${TAG}.log" 2>&1 || echo "  (MoveIt move to hover failed; capture.py will move the arm itself)"
    fi
    # Generous time limit: 10 tries at a 20 s spin take about 4 minutes
    timeout 900 python3 scripts/capture.py --ros-args -p use_sim_time:=true \
      -p method:=$M -p spin_period:=$F_PERIOD -p trials:=$TRIALS \
      -p ik_source:=$IK_SOURCE -p log_file:="$OUT/stageB_${TAG}.csv" > "$OUT/capture_${TAG}.log" 2>&1
    STATUS=$?
    pkill -f "rotating_target/predictor" > /dev/null 2>&1
    F="$OUT/stageB_${TAG}.csv"
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
echo "Next:  python3 scripts/analyze_stageB.py"
