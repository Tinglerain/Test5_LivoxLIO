// ============================================================================
//  位姿插值（common 模块）。
//
//  这个文件里有一个函数要你实现：
//
//    TODO 1  interpolatePose
//
//  它被“点云去畸变”直接调用。写错的话，地图会糊、会重影，轨迹也会抖。
//
//  函数签名和契约在 include/small_glim/common/pose_utils.hpp。
//  现在的函数体只是让工程能编译通过的占位，返回值是错的。
// ============================================================================
#include <small_glim/common/pose_utils.hpp>

#include <algorithm>

namespace small_glim {

// ----------------------------------------------------------------------------
// TODO 1：interpolatePose
// ----------------------------------------------------------------------------
//
// 【它解决什么问题】
//
// 一帧激光点云不是“咔嚓”一下拍完的，而是雷达在约 0.1 秒内连续扫出来的。
//
//   时间:  t0 ------------------------------> t1
//   点:    点0   点1   点2   ...   点1000   最后一个点
//
// 如果这 0.1 秒里机器人转了 30 度，那么这一帧里的点其实是在不同姿态下测到的。
// 直接把它们当成“同一时刻”的点，地图就会糊。
//
// 去畸变的做法是：
//
//   对每个点，先算出它采样时刻的位姿，再把点变换到帧起始时刻。
//
// 这中间就需要“已知 t0 位姿和 t1 位姿，求中间某时刻位姿”的能力。
// 这就是这个函数。
//
// 【契约】
//
//   - alpha = 0 返回 a；
//   - alpha = 1 返回 b；
//   - alpha 超出 [0, 1] 时夹到边界；
//   - 平移线性插值；
//   - 旋转用四元数 slerp。
//
// 【为什么旋转不能直接线性插值】
//
// 旋转矩阵的元素不是独立变量，直接对 9 个数做线性插值，结果通常不再是旋转
// （行列式不等于 1，轴也不对）。欧拉角线性插值也有问题：绕圈、万向锁。
//
// 正确做法是把旋转看成单位四元数，在四元数球面上走最短弧，也就是 slerp。
//
// 【推荐写法】
//
//   t = clamp(alpha, 0, 1)
//   qa = Quaterniond(a.linear())
//   qb = Quaterniond(b.linear())
//   q  = qa.slerp(t, qb)
//   p  = (1-t) * a.translation() + t * b.translation()
//   result.linear() = q.toRotationMatrix()
//   result.translation() = p
//
// 【自己代一个例子】
//
//   a = 单位位姿（在原点，不旋转）
//   b = 平移 (2,0,0)，绕 z 转 90 度
//   alpha = 0.5
//
//   结果应该是：平移 (1,0,0)，绕 z 转 45 度。
//
// 【常见错误和后果】
//
//   旋转矩阵元素直接插值    -> 结果不是旋转，点云被拉歪
//   只用 b 的旋转           -> 帧内旋转完全没补偿，地图糊
//   忘记 clamp alpha        -> 点被推到帧外，出现飞点
//   用单位四元数代替 qa     -> 插值起点错，前半帧全歪
//
// 【对应测试】
//
//   PoseUtils.InterpolateMidpoint
//   PoseUtils.InterpolateClampsAlpha
Eigen::Isometry3d interpolatePose(
    const Eigen::Isometry3d& a,
    const Eigen::Isometry3d& b,
    double alpha) {
    (void)a;
    (void)b;
    (void)alpha;
    return Eigen::Isometry3d::Identity();
}

} // namespace small_glim
