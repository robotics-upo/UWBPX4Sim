#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

detect_default_ros_ws() {
  local candidate

  for candidate in "$SCRIPT_DIR/../.." "$SCRIPT_DIR/../../.."; do
    candidate="$(cd "$candidate" && pwd)"
    if [[ -d "$candidate/src" && -d "$candidate/install" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  for candidate in "$SCRIPT_DIR/../.." "$SCRIPT_DIR/../../.."; do
    candidate="$(cd "$candidate" && pwd)"
    if [[ -d "$candidate/src" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  cd "$SCRIPT_DIR/../.." && pwd
}

DEFAULT_ROS_WS="$(detect_default_ros_ws)"

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
ROS_WS="${ROS_WS:-$DEFAULT_ROS_WS}"
GZ_WORLD="${GZ_WORLD:-default}"
UWB_LAYOUT_FILE="${UWB_LAYOUT_FILE:-${UWB_LAYOUT_JSON:-$SCRIPT_DIR/config/uwb_layout.four_vehicle_example.yaml}}"
LAYOUT_TOOL="$SCRIPT_DIR/tools/configure_uwb_layout.py"

LAUNCH_LOCALIZATION=0
RECORD_BAG=0

print_usage() {
  cat <<EOF
Usage: $(basename "$0") [--localization] [--bag]

Options:
  --localization  Launch uwb_localization together with the simulator.
  --bag           Start ros2 bag recording in tmux pane 1.3.
  (env) UWB_LAYOUT_FILE  Layout YAML with per-robot spawn_pose and sensor placement (default: config/uwb_layout.four_vehicle_example.yaml)
  (env) UWB_LAYOUT_JSON  Legacy alias for UWB_LAYOUT_FILE
  (env) GZ_WORLD  Gazebo world name from PX4 Tools/simulation/gz/worlds (default: default)
  -h, --help           Show this help message.
EOF
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --localization)
        LAUNCH_LOCALIZATION=1
        ;;
      --bag)
        RECORD_BAG=1
        ;;
      -h|--help)
        print_usage
        exit 0
        ;;
      *)
        echo "[ERROR] Unknown argument: $1" >&2
        print_usage >&2
        exit 1
        ;;
    esac
    shift
  done
}

find_qgc() {
  local candidate

  if [[ -n "${QGC_PATH:-}" ]]; then
    printf '%s\n' "$QGC_PATH"
    return 0
  fi

  for candidate in "$HOME/Desktop"/QGroundControl*.AppImage "$HOME/Downloads"/QGroundControl*.AppImage; do
    if [[ -f "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  return 1
}

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[ERROR] Missing required command: $1" >&2
    exit 1
  fi
}

require_directory() {
  if [[ ! -d "$1" ]]; then
    echo "[ERROR] Directory not found: $1" >&2
    exit 1
  fi
}

build_ros_shell_command() {
  local ros_command="$1"
  local wrapped

  printf -v wrapped "%q -lc %q" "$ROS_SHELL" "source \"$ROS_SETUP_SCRIPT\" && $ros_command"
  printf '%s' "$wrapped"
}

QGC_PATH="$(find_qgc || true)"
parse_args "$@"

if [[ -z "$QGC_PATH" ]]; then
  echo "[ERROR] QGroundControl AppImage not found. Set QGC_PATH or place it in ~/Desktop or ~/Downloads." >&2
  exit 1
fi

require_directory "$ROS_WS"
require_directory "$PX4_DIR"
if [[ ! -f "$UWB_LAYOUT_FILE" ]]; then
  echo "[ERROR] Layout YAML not found: $UWB_LAYOUT_FILE" >&2
  exit 1
fi
require_command tmux
require_command MicroXRCEAgent
require_command python3

if [[ -f "$ROS_WS/install/setup.bash" ]]; then
  ROS_SHELL="bash"
  ROS_SETUP_SCRIPT="$ROS_WS/install/setup.bash"
elif [[ -f "$ROS_WS/install/setup.zsh" ]]; then
  ROS_SHELL="zsh"
  ROS_SETUP_SCRIPT="$ROS_WS/install/setup.zsh"
  require_command zsh
else
  echo "[ERROR] Could not find a ROS setup script under $ROS_WS/install." >&2
  exit 1
fi

PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
if [[ ! -x "$PX4_BIN" ]]; then
  echo "[ERROR] PX4 SITL binary not found at $PX4_BIN. Build PX4 with 'make px4_sitl' first." >&2
  exit 1
fi

if [[ ! -f "$LAYOUT_TOOL" ]]; then
  echo "[ERROR] Layout tool not found: $LAYOUT_TOOL" >&2
  exit 1
fi

mapfile -t SPAWN_SPECS < <(python3 "$LAYOUT_TOOL" --layout "$UWB_LAYOUT_FILE" --emit-spawn-layout)
if (( ${#SPAWN_SPECS[@]} == 0 )); then
  echo "[ERROR] No vehicles found in layout: $UWB_LAYOUT_FILE" >&2
  exit 1
fi

PX4_GZ_MODELS_DIR="$PX4_DIR/Tools/simulation/gz/models"
for spec in "${SPAWN_SPECS[@]}"; do
  IFS=$'\t' read -r _vehicle_type _vehicle_id model_name _autostart _x _y _z _roll _pitch _yaw <<< "$spec"
  if [[ ! -d "$PX4_GZ_MODELS_DIR/$model_name" ]]; then
    echo "[ERROR] Missing generated Gazebo model: $PX4_GZ_MODELS_DIR/$model_name" >&2
    echo "[ERROR] Copy the generated models from UWBPX4Sim_repo/models/custom_models into PX4 first." >&2
    exit 1
  fi
done

SESSION="${SESSION:-sim_session}"
UWB_BRIDGE_CONFIG="${UWB_BRIDGE_CONFIG:-$SCRIPT_DIR/ROS2/px4_sim_offboard/config/uwb_bridge.yaml}"

# Build UWB pair topic list directly from the bridge config.
UWB_PAIR_TOPICS=()
if [[ -f "$UWB_BRIDGE_CONFIG" ]]; then
  mapfile -t UWB_PAIR_TOPICS < <(
    awk -F'"' '/ros_topic_name:/ {print $2}' "$UWB_BRIDGE_CONFIG"
  )
else
  echo "[WARN] UWB bridge config not found at $UWB_BRIDGE_CONFIG. Falling back to /eliko/Distances only." >&2
fi

graceful_shutdown() {
  echo "[CLEANUP] Graceful shutdown…"

  if tmux has-session -t "$SESSION" 2>/dev/null; then
    while IFS= read -r pane_id; do
      tmux send-keys -t "$pane_id" C-c 2>/dev/null || true
    done < <(tmux list-panes -a -t "$SESSION" -F "#{session_name}:#{window_index}.#{pane_index}")
    sleep 5

    echo "[CLEANUP] Killing simulators…"
    killall -9 gzserver gzclient ruby 2>/dev/null || true

    tmux kill-session -t "$SESSION" 2>/dev/null || true
  fi
}

trap graceful_shutdown EXIT INT TERM

# Kill any previous session
tmux kill-session -t "$SESSION" 2>/dev/null || true

# Start new tmux session
tmux new-session -d -s $SESSION
TOTAL_MAIN_PANES=$((2 + ${#SPAWN_SPECS[@]}))
for ((pane_idx = 1; pane_idx < TOTAL_MAIN_PANES; pane_idx++)); do
  tmux split-window -t $SESSION:0
done

tmux select-layout -t $SESSION:0 tiled

# Set up QGroundControl
tmux send-keys -t $SESSION:0.0 "chmod +x \"$QGC_PATH\" && \"$QGC_PATH\"" C-m
# Set up Micro XRCE-DDS Agent
tmux send-keys -t $SESSION:0.1 "MicroXRCEAgent udp4 -p 8888" C-m
SPAWNED_MODELS=()
vehicle_idx=0
for spec in "${SPAWN_SPECS[@]}"; do
  IFS=$'\t' read -r vehicle_type vehicle_id model_name autostart x y z roll pitch yaw <<< "$spec"
  pane_index=$((2 + vehicle_idx))
  instance_index=$((1 + vehicle_idx))
  sleep_delay=$((2 + vehicle_idx * 2))
  pose_csv="${x},${y},${z},${roll},${pitch},${yaw}"
  SPAWNED_MODELS+=("$model_name")

  if (( vehicle_idx == 0 )); then
    spawn_cmd="sleep $sleep_delay && cd \"$PX4_DIR\" && PX4_GZ_MODEL_POSE='$pose_csv' PX4_SYS_AUTOSTART=$autostart PX4_SIM_MODEL=gz_$model_name PX4_GZ_WORLD=${GZ_WORLD} \"$PX4_BIN\" -i $instance_index"
  else
    spawn_cmd="sleep $sleep_delay && cd \"$PX4_DIR\" && PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_POSE='$pose_csv' PX4_SYS_AUTOSTART=$autostart PX4_SIM_MODEL=gz_$model_name PX4_GZ_WORLD=${GZ_WORLD} \"$PX4_BIN\" -i $instance_index"
  fi

  tmux send-keys -t $SESSION:0.$pane_index "$spawn_cmd" C-m
  vehicle_idx=$((vehicle_idx + 1))
done

# Offboard / ROS-side block kept here for later reactivation.
# The four-vehicle spawn test above intentionally does not run it yet.
#
# # Wait for everything to launch and spawn and then launch the ROS 2 nodes
# tmux send-keys -t $SESSION:0.4 "$(build_ros_shell_command "sleep 25 && cd \"$ROS_WS\" && ros2 launch px4_sim_offboard offboard_launch.py")" C-m
# tmux send-keys -t $SESSION:0.5 "$(build_ros_shell_command "sleep 27 && cd \"$ROS_WS\" && ros2 topic echo /eliko/Distances")" C-m
#
# #odometry window
# tmux new-window -t $SESSION:1 -n optimization
# tmux split-window -h -t $SESSION:1.0 
#
# # Add two more panes:
# tmux split-window -v -t $SESSION:1.0    # Create pane 1.2 from 1.0
# tmux split-window -v -t $SESSION:1.1    # Create pane 1.3 from 1.1
#
# tmux select-layout -t $SESSION:1 tiled
#
# # Echo each vehicles odometry topic
# tmux send-keys -t $SESSION:1.0 "$(build_ros_shell_command "sleep 27 && cd \"$ROS_WS\" && ros2 topic echo /agv/odom")" C-m
# tmux send-keys -t $SESSION:1.1 "$(build_ros_shell_command "sleep 27 && cd \"$ROS_WS\" && ros2 topic echo /uav/odom")" C-m
#
# if (( LAUNCH_LOCALIZATION )); then
#   tmux send-keys -t $SESSION:1.2 "$(build_ros_shell_command "sleep 30 && cd \"$ROS_WS\" && ros2 launch uwb_localization localization.launch.py")" C-m
# fi
#
# if (( RECORD_BAG )); then
#   BAG_TOPICS=(
#     /uav/gt
#     /agv/gt
#     /uav/odom
#     /agv/odom
#     /eliko/Distances
#   )
#
#   # Add all individual anchor-tag UWB topics.
#   BAG_TOPICS+=("${UWB_PAIR_TOPICS[@]}")
#
#   # Add localization outputs only when localization node is launched.
#   if (( LAUNCH_LOCALIZATION )); then
#     BAG_TOPICS+=(
#       /eliko_optimization_node/optimized_T
#       /eliko_optimization_node/optimized_T_nopr
#       /eliko_optimization_node/ransac_optimized_T
#       /pose_graph_node/uav_anchor
#       /pose_graph_node/agv_anchor
#     )
#   fi
#
#   # Deduplicate topics while preserving order.
#   UNIQUE_BAG_TOPICS=()
#   declare -A seen_topics=()
#   for topic in "${BAG_TOPICS[@]}"; do
#     [[ -z "${topic:-}" ]] && continue
#     if [[ -z "${seen_topics[$topic]+x}" ]]; then
#       UNIQUE_BAG_TOPICS+=("$topic")
#       seen_topics[$topic]=1
#     fi
#   done
#
#   BAG_CMD="sleep 30 && cd \"$ROS_WS\" && ros2 bag record"
#   for topic in "${UNIQUE_BAG_TOPICS[@]}"; do
#     BAG_CMD+=" $topic"
#   done
#   tmux send-keys -t $SESSION:1.3 "$(build_ros_shell_command "$BAG_CMD")" C-m
# else
#   tmux send-keys -t $SESSION:1.3 "echo '[INFO] Bag recording disabled. Use --bag to enable.'" C-m
# fi

# This launcher is intentionally limited to the Gazebo/PX4 spawn test.
# We still run the plain UWB Gazebo bridge so the direct aItJ ranging topics are available in ROS 2.
tmux new-window -t $SESSION:1 -n bridge
tmux split-window -h -t $SESSION:1.0
tmux select-layout -t $SESSION:1 tiled
tmux send-keys -t $SESSION:1.0 "$(build_ros_shell_command "sleep 12 && cd \"$ROS_WS\" && ros2 launch px4_sim_offboard uwb_bridge_launch.py")" C-m
spawned_models_str="${SPAWNED_MODELS[*]}"
tmux send-keys -t $SESSION:1.1 "echo '[INFO] UWB spawn test launcher.' && echo '[INFO] Layout file: $UWB_LAYOUT_FILE' && echo '[INFO] Spawned models: $spawned_models_str' && echo '[INFO] Direct ROS topics should appear under /uwb_gz_simulator/distances/aItJ and /uwb_gz_simulator/distances_ground_truth/aItJ.' && echo '[INFO] /eliko/Distances is still not expected here because agv_offboard_control remains disabled.'" C-m


# Attach
tmux attach-session -t $SESSION
