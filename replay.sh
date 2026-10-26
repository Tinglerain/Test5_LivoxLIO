#!/usr/bin/env bash
# 播放 Livox 录包。
#
# 用法：
#   ./replay.sh
#   ./replay.sh /path/to/lunatic_0.mcap
#   ./replay.sh /path/to/rosbag2_directory
#
# 默认播放 bag/lunatic_0.mcap。单次播放，不循环，避免地图重复叠加。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG="${1:-${LIVOX_BAG:-$ROOT/bag/lunatic_0.mcap}}"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "请先 source ROS 2 环境，例如："
  echo "  source /opt/ros/jazzy/setup.bash"
  exit 1
fi

if [[ ! -e "$BAG" ]]; then
  echo "找不到录包：$BAG"
  echo "把 Livox 录包放到 bag/lunatic_0.mcap，或执行："
  echo "  ./replay.sh /path/to/lunatic_0.mcap"
  exit 1
fi

exec ros2 bag play "$BAG"
