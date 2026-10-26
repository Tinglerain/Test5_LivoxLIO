#!/usr/bin/env bash
# 检查 small_glim 版 Test5 所需环境。只报告问题，不修改系统。
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

pass=0
fail=0
warn_count=0

ok() { echo "  [ok]   $1"; pass=$((pass + 1)); }
bad() {
  echo "  [缺失] $1"
  echo "         修复：$2"
  fail=$((fail + 1))
}
warn() {
  echo "  [注意] $1"
  warn_count=$((warn_count + 1))
}

echo "============================================================"
echo "  Test5_LivoxLIO 环境自检（small_glim 版）"
echo "============================================================"

echo
echo "系统与 ROS 2"
if [[ -r /etc/os-release ]]; then
  source /etc/os-release
  if [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "24.04" ]]; then
    ok "$PRETTY_NAME"
  else
    bad "系统为 ${PRETTY_NAME:-未知}，要求 Ubuntu 24.04" "使用 Ubuntu 24.04 环境"
  fi
else
  warn "无法读取 /etc/os-release"
fi
if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
  if [[ -z "${ROS_DISTRO:-}" ]]; then
    bad "ROS_DISTRO 未设置" "source /opt/ros/jazzy/setup.bash"
  else
    bad "ROS_DISTRO=$ROS_DISTRO，要求 jazzy" "source /opt/ros/jazzy/setup.bash"
  fi
else
  ok "ROS_DISTRO=$ROS_DISTRO"
fi

echo
echo "构建工具"
for tool in bash python3 g++-13 gcc-13 cmake colcon ninja pkg-config ros2; do
  if command -v "$tool" >/dev/null 2>&1; then
    ok "$tool: $(command -v "$tool")"
  else
    case "$tool" in
      colcon) bad "colcon" "sudo apt install python3-colcon-common-extensions" ;;
      ninja)  bad "ninja" "sudo apt install ninja-build" ;;
      cmake)  bad "cmake" "sudo apt install cmake" ;;
      pkg-config) bad "pkg-config" "sudo apt install pkg-config" ;;
      g++-13|gcc-13) bad "$tool" "sudo apt install g++-13 gcc-13" ;;
      *)      bad "$tool" "检查 PATH 或重新安装 ROS 2" ;;
    esac
  fi
done

echo
echo "C++ 依赖"
if [[ -f /usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake || -f /usr/lib/x86_64-linux-gnu/cmake/GTSAM/GTSAMConfig.cmake ]]; then
  ok "GTSAM 4.3"
else
  bad "GTSAM" "按 GTSAM 官方文档构建安装 4.3a0"
fi
if [[ -f /usr/local/lib/cmake/gtsam_points/gtsam_points-config.cmake || -f /usr/local/lib/cmake/gtsam_points/gtsam_pointsConfig.cmake ]]; then
  ok "gtsam_points 1.2"
else
  bad "gtsam_points" "按 gtsam_points 官方文档构建安装 v1.2.0"
fi
if command -v pkg-config >/dev/null 2>&1 && pkg-config --exists pcl_common; then
  ok "PCL $(pkg-config --modversion pcl_common)"
else
  bad "PCL" "sudo apt install libpcl-dev ros-\$ROS_DISTRO-pcl-ros；确认 pkg-config --exists pcl_common 可通过"
fi

echo
echo "ROS 2 依赖"
for pkg in sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs rclcpp rclcpp_components foxglove_bridge rosbag2_storage_mcap; do
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    ok "$pkg"
  else
    bad "$pkg" "sudo apt install ros-\$ROS_DISTRO-${pkg//_/-}"
  fi
done

echo
echo "源文件"
for file in \
  src/small_glim/src/small_glim_node.cpp \
  src/small_glim/launch/small_glim.launch.py \
  src/small_glim/config/params_node.yaml; do
  if [[ -f "$file" ]]; then
    ok "$file"
  else
    bad "$file" "检查仓库是否完整"
  fi
done

echo
echo "录包"
BAG="${LIVOX_BAG:-$ROOT/bag/lunatic_0.mcap}"
if [[ -e "$BAG" ]]; then
  ok "录包：$BAG"
  if ros2 bag info "$BAG" >/tmp/test5_bag_info.txt 2>&1; then
    if grep -q '/livox/lidar/pointcloud' /tmp/test5_bag_info.txt; then
      ok "包含 /livox/lidar/pointcloud"
    else
      bad "录包缺少 /livox/lidar/pointcloud" "使用带 PointCloud2 的 Livox 录包"
    fi
    if grep -q '/livox/imu' /tmp/test5_bag_info.txt; then
      ok "包含 /livox/imu"
    else
      bad "录包缺少 /livox/imu" "录制 IMU 话题"
    fi
  else
    warn "无法读取录包信息，见 /tmp/test5_bag_info.txt"
  fi
else
  warn "未找到录包：$BAG"
  echo "         放好后直接在项目根目录执行：./replay.sh"
fi

echo
echo "============================================================"
if [[ "$fail" -eq 0 ]]; then
  echo "  全部通过（$pass 项，$warn_count 个提醒）"
  echo "  下一步：./compile.sh && source install/setup.bash"
else
  echo "  有 $fail 项需要处理，按上面的修复命令操作后重跑本脚本。"
fi
echo "============================================================"
exit $(( fail > 0 ? 1 : 0 ))
