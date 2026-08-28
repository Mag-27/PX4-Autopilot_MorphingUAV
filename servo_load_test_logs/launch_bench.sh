#!/bin/bash
# Launch PX4 SITL against the foldrotor3_bench (or foldrotor3) gz model,
# feeding pxh> commands from a persistent FIFO so a test script can drive it
# non-interactively. See ../open_loop_commands.md, "Servo load test under
# motor thrust" section, for the full test procedure this supports.
#
# Usage: ./launch_bench.sh [work_dir] [gz_model]
#   work_dir  scratch dir for the fifo/log (default: /tmp/foldrotor3_bench_test)
#   gz_model  PX4_SIM_MODEL value (default: gz_foldrotor3_bench)
set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORK_DIR="${1:-/tmp/foldrotor3_bench_test}"
MODEL="${2:-gz_foldrotor3_bench}"

mkdir -p "$WORK_DIR"
[ -p "$WORK_DIR/pxh_in" ] || mkfifo "$WORK_DIR/pxh_in"

# hold the fifo open so px4's stdin doesn't see EOF between commands sent by
# a separate `echo "cmd" > "$WORK_DIR/pxh_in"` from another shell
sleep infinity > "$WORK_DIR/pxh_in" &
echo $! > "$WORK_DIR/holder.pid"

cd "$REPO_ROOT/build/px4_sitl_default/src/modules/simulation/gz_bridge"

# PX4_SYS_AUTOSTART=4026 bypasses the PX4_SIM_MODEL->airframe-filename
# lookup in rcS, so PX4_SIM_MODEL can point at foldrotor3_bench (no matching
# airframe file) while still loading airframe 4026's params.
PX4_SYS_AUTOSTART=4026 \
PX4_SIM_MODEL="$MODEL" \
PX4_GZ_WORLD=default \
GZ_IP=127.0.0.1 \
HEADLESS=1 \
"$REPO_ROOT/build/px4_sitl_default/bin/px4" < "$WORK_DIR/pxh_in" > "$WORK_DIR/px4.log" 2>&1
