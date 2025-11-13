#!/usr/bin/env bash
set -euo pipefail

# 获取脚本所在目录，然后获取工作空间根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$(dirname "$SCRIPT_DIR")"
ROS_SETUP="/opt/ros/humble/setup.bash"
INSTALL_SETUP="$BASE_DIR/install/setup.bash"

cd "$BASE_DIR" || exit 1

tmux new-session -d -s robot_sys -n rosbash "bash -lc 'source $ROS_SETUP; [ -f $INSTALL_SETUP ] && source $INSTALL_SETUP; ros2 launch mecanum_robot apriltag_robot.launch.py'"
tmux new-window -t robot_sys -n bridge "bash -lc 'source $ROS_SETUP; [ -f $INSTALL_SETUP ] && source $INSTALL_SETUP; python3 scripts/ros2_websocket_bridge.py'"
tmux new-window -t robot_sys -n waypoint "bash -lc 'source $ROS_SETUP; [ -f $INSTALL_SETUP ] && source $INSTALL_SETUP; cd scripts; python3 waypoint_recorder.py'"
tmux new-window -t robot_sys -n web "bash -lc 'cd web; python3 -m http.server 8080'"
echo "tmux 会话已创建：tmux attach -t robot_sys"
tmux attach -t robot_sys
