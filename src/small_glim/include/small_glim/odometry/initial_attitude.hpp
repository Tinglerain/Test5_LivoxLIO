// 初始姿态对齐（新人实现）。
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace small_glim {

/// 重力对齐：求一个旋转，把给定的“重力方向”转到 +Z。
///
/// 约定：
///   - gravity_direction 不需要预先归一化；
///   - 长度接近 0 时返回单位矩阵；
///   - 返回 3x3 旋转矩阵。
Eigen::Matrix3d gravityAlignment(const Eigen::Vector3d& gravity_direction);

/// 航向对齐：求一个绕 Z 的旋转，把给定方向投影到水平面后转到 +X。
///
/// 约定：
///   - 只使用 x/y 分量，忽略 z；
///   - 水平投影长度接近 0 时返回单位矩阵；
///   - 返回 3x3 旋转矩阵。
Eigen::Matrix3d yawAlignment(const Eigen::Vector3d& x_axis_world);

} // namespace small_glim
