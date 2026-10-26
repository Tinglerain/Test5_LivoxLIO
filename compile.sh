#!/usr/bin/env bash
# 编译 LIO-lite 工作空间。
#
# 用法：
#   ./compile.sh
#
# 要求当前终端已经 source 对应的 ROS 2 环境（一般已在 ~/.bashrc）。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "未检测到 ROS_DISTRO。请先 source ROS 2 环境，例如："
  echo "  source /opt/ros/jazzy/setup.bash"
  exit 1
fi

for tool in colcon cmake ninja; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo "缺少工具：$tool"
    case "$tool" in
      colcon) echo "安装：sudo apt install python3-colcon-common-extensions" ;;
      ninja)   echo "安装：sudo apt install ninja-build" ;;
      cmake)   echo "安装：sudo apt install cmake" ;;
    esac
    exit 1
  fi
done

echo "ROS_DISTRO=$ROS_DISTRO"
echo "Ninja=$(ninja --version)"

colcon build --symlink-install --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release -Wno-dev

echo "构建完成：$ROOT"
