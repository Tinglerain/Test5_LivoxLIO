// 建图工具（已给定，不是 TODO）。
#include <small_glim/mapping/mapping_utils.hpp>

#include <cmath>

namespace small_glim {

bool isKeyframePose(
    const Eigen::Isometry3d& last,
    const Eigen::Isometry3d& current,
    double trans_thresh,
    double rot_thresh_deg) {
    const Eigen::Isometry3d relative = last.inverse() * current;
    const double translation = relative.translation().norm();
    const double rotation_deg = std::abs(
        Eigen::AngleAxisd(Eigen::Quaterniond(relative.linear())).angle()) *
        180.0 / 3.14159265358979323846;
    return translation > trans_thresh || rotation_deg > rot_thresh_deg;
}

pcl::PointCloud<pcl::PointXYZ> filterByRange(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    double min_range,
    double max_range) {
    const bool use_min = min_range > 0.0;
    const bool use_max = max_range > 0.0;
    const double min_squared = use_min ? min_range * min_range : 0.0;
    const double max_squared = use_max ? max_range * max_range : 0.0;

    pcl::PointCloud<pcl::PointXYZ> output;
    output.reserve(cloud.size());
    for (const auto& point : cloud.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            continue;
        }
        const double squared = static_cast<double>(point.x) * point.x +
                               static_cast<double>(point.y) * point.y +
                               static_cast<double>(point.z) * point.z;
        if (use_min && squared < min_squared) continue;
        if (use_max && squared > max_squared) continue;
        output.push_back(point);
    }
    return output;
}

} // namespace small_glim
