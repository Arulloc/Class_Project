#!/bin/bash
# run_stageA.sh: Stage A experiments - prediction only, no camera, no arm motion.
#
# For each condition below it (re)starts the spinning target, runs all four
# prediction methods at the same time on the same data, and saves one CSV per
# method into results/stageA/. Then run:  python3 scripts/analyze_stageA.py
#
# Background processes are stopped with TERM (a script's background jobs
# ignore Ctrl+C-style INT signals).
#
# Before running: Gazebo and the arm must already be running (Terminal 1, as in
# the README). Do NOT start the target yourself; this script does it.
#
# Usage:  bash scripts/run_stageA.sh          full set (about 20 minutes)
#         bash scripts/run_stageA.sh test     baseline only, 20 s (quick check)

cd "$(dirname "$0")/.."
source install/setup.bash
OUT="$(pwd)/results/stageA"
mkdir -p "$OUT"

METHODS="reactive last_two average kalman"
RUN_TIME=60          # seconds of logged predictions per condition
WARMUP=5             # seconds of predictions ignored at the start

# Conditions: spin_period(s) horizon(s) noise(m) delay(s) compensate
# Baseline is 10 1.0 0 0 true; each sweep changes one setting from it.
CONDITIONS=(
  "20 1.0 0 0 true"     "10 1.0 0 0 true"     "5 1.0 0 0 true"     "3 1.0 0 0 true"   # speed
  "10 0.5 0 0 true"     "10 2.0 0 0 true"                                             # horizon
  "10 1.0 0.002 0 true" "10 1.0 0.005 0 true"                                         # noise
  "10 1.0 0 0.1 true"   "10 1.0 0 0.2 true"   "10 1.0 0 0.4 true"                     # delay, corrected
  "10 1.0 0 0.1 false"  "10 1.0 0 0.2 false"  "10 1.0 0 0.4 false"                    # delay, not corrected
)
if [ "$1" = "test" ]; then
  CONDITIONS=("10 1.0 0 0 true"); RUN_TIME=20
fi

# Record what was run, for the evidence register
{
  echo "date: $(date -Iseconds)"
  echo "branch: $(git branch --show-current)"
  echo "commit: $(git rev-parse --short HEAD)"
  echo "uncommitted changes: $(git status --porcelain | wc -l) files"
  echo "ROS: $ROS_DISTRO   run_time: $RUN_TIME s   warmup: $WARMUP s"
  printf 'condition: %s\n' "${CONDITIONS[@]}"
} > "$OUT/run_info.txt"

remove_target() {
  ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity \
    "{name: rotating_target}" > /dev/null 2>&1
}

for cond in "${CONDITIONS[@]}"; do
  read -r PERIOD HORIZON NOISE DELAY COMP <<< "$cond"
  NOISE_MM=$(python3 -c "print(f'{$NOISE*1000:g}')")
  DELAY_MS=$(python3 -c "print(f'{$DELAY*1000:g}')")
  C=$([ "$COMP" = "true" ] && echo 1 || echo 0)
  TAG="P${PERIOD}_h${HORIZON%.0}_n${NOISE_MM}_d${DELAY_MS}_c${C}"
  echo "=== $TAG: spin ${PERIOD} s, ${HORIZON} s ahead, noise ${NOISE_MM} mm, delay ${DELAY_MS} ms, corrected $COMP"

  # Fresh target at this spin period
  remove_target
  ros2 launch rotating_target target.launch.py mode:=spin spin_period:="$PERIOD" \
    > "$OUT/target_$TAG.log" 2>&1 &
  TARGET_PID=$!
  if ! timeout 60 ros2 topic echo /target/handle_true --once > /dev/null 2>&1; then
    echo "  target did not start; see $OUT/target_$TAG.log"
    kill -TERM $TARGET_PID 2>/dev/null; wait $TARGET_PID 2>/dev/null
    continue
  fi

  # All four methods at once, each with its own node name and output topics
  PIDS=""
  for M in $METHODS; do
    timeout 600 ros2 run rotating_target predictor --ros-args \
      -r __node:=predictor_$M \
      -r /target/handle_predicted:=/target/handle_predicted_$M \
      -r /target/prediction_error:=/target/prediction_error_$M \
      -p method:=$M -p horizon:=$HORIZON -p noise_std:=$NOISE \
      -p delay:=$DELAY -p compensate:=$COMP -p spin_period:=$PERIOD \
      -p run_time:=$RUN_TIME -p warmup:=$WARMUP \
      -p log_file:="$OUT/stageA_${TAG}_${M}.csv" \
      > "$OUT/predictor_${TAG}_${M}.log" 2>&1 &
    PIDS="$PIDS $!"
  done
  wait $PIDS
  for M in $METHODS; do
    F="$OUT/stageA_${TAG}_${M}.csv"
    if [ -f "$F" ]; then
      echo "  $M: $(($(wc -l < "$F") - 1)) checks"
    else
      echo "  $M: no results; see $OUT/predictor_${TAG}_${M}.log"
    fi
  done

  kill -TERM $TARGET_PID 2>/dev/null; wait $TARGET_PID 2>/dev/null
done
remove_target

echo ""
echo "Done. Results in $OUT"
echo "Next:  python3 scripts/analyze_stageA.py"
