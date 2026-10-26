// 建图工具（新人实现）。
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace small_glim {

/// 按相对位姿判断是否插入关键帧。
///
/// 约定：
///   - 相对位姿 = last^{-1} * current；
///   - 平移超过 trans_thresh 或旋转超过 rot_thresh_deg 返回 true；
///   - 旋转角从旋转矩阵提取，单位度。
bool isKeyframePose(
    const Eigen::Isometry3d& last,
    const Eigen::Isometry3d& current,
    double trans_thresh,
    double rot_thresh_deg);

/// 按到原点距离裁剪点云。
///
/// 约定：
///   - min_range <= 0 表示不限下界，max_range <= 0 表示不限上界；
///   - 非有限点丢弃；
///   - 保留 min_range <= ||p|| <= max_range；
///   - 保持输入顺序。
pcl::PointCloud<pcl::PointXYZ> filterByRange(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    double min_range,
    double max_range);

} // namespace small_glim
