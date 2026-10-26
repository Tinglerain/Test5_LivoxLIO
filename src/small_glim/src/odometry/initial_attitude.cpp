#include <small_glim/odometry/initial_attitude.hpp>

#include <cmath>

namespace small_glim {

// ----------------------------------------------------------------------------
// TODO 4：gravityAlignment
// ----------------------------------------------------------------------------
//
// 【原理】
//
// 静止时加速度计测到的是“比力”，方向指向天（+Z），大小约 9.8。
// 但 IMU 可能装歪了，测到的方向不是 +Z。
//
// 重力对齐就是求一个旋转 R，使得：
//
//   R * gravity_direction_normalized = +Z
//
// 有了 R，IMU 系就被“摆正”了：roll/pitch 由重力确定。
//
// 【契约】
//
//   - 输入不必归一化；
//   - 长度 < 1e-6 -> 返回单位矩阵；
//   - 返回 3x3 旋转矩阵。
//
// 【推荐写法】
//
//   if (g.norm() < 1e-6) return I;
//   return Quaterniond::FromTwoVectors(g.normalized(), UnitZ())
//       .toRotationMatrix();
//
// 【易错】
//
//   把方向写反（+Z -> g）     -> 旋转方向相反，初始姿态错误
//   不归一化                   -> FromTwoVectors 内部会归一化，但仍应显式归一化
//   零向量不判空               -> 归一化产生 NaN
//
// 【测试】
//
//   InitialAttitude.GravityIdentityWhenAlreadyUp
//   InitialAttitude.GravityMapsMeasuredDirectionToZ
//   InitialAttitude.GravityZeroReturnsIdentity
Eigen::Matrix3d gravityAlignment(const Eigen::Vector3d& gravity_direction) {
    (void)gravity_direction;
    return Eigen::Matrix3d::Identity();
}

// ----------------------------------------------------------------------------
// TODO 5：yawAlignment
// ----------------------------------------------------------------------------
//
// 【原理】
//
// 重力只能确定 roll/pitch，绕 Z 的 yaw 仍然自由。
// 工程约定：让 IMU 的 x 轴在水平面上的投影对齐世界 +X。
//
// 做法：
//
//   heading = (x, y, 0) 并归一化
//   yaw = -atan2(heading.y, heading.x)
//   R = AngleAxisd(yaw, UnitZ)
//
// 这个 R 左乘到已经水平对齐的旋转上，就消掉了 yaw 自由度。
//
// 【契约】
//
//   - 只使用 x/y，忽略 z；
//   - 水平投影长度 < 1e-6 -> 返回单位矩阵；
//   - 返回 3x3 旋转矩阵。
//
// 【易错】
//
//   yaw 符号写反           -> 航向朝反方向
//   不把 z 清零            -> 若用完整向量判断零/长度，会被 z 污染；atan2 本身只用 x/y
//   不归一化               -> atan2 仍可用，但语义不清
//
// 【测试】
//
//   InitialAttitude.YawMapsHeadingToX
//   InitialAttitude.YawIgnoresZComponent
//   InitialAttitude.YawZeroReturnsIdentity
Eigen::Matrix3d yawAlignment(const Eigen::Vector3d& x_axis_world) {
    (void)x_axis_world;
    return Eigen::Matrix3d::Identity();
}

} // namespace small_glim
