// 位姿插值（新人实现）。
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace small_glim {

/// 两位姿之间插值。
///
/// 约定：
///   - alpha = 0 返回 a，alpha = 1 返回 b，alpha 限制到 [0, 1]；
///   - 平移线性插值；
///   - 旋转用单位四元数 slerp；
///   - 不要插值欧拉角。
Eigen::Isometry3d interpolatePose(
    const Eigen::Isometry3d& a,
    const Eigen::Isometry3d& b,
    double alpha);

} // namespace small_glim
