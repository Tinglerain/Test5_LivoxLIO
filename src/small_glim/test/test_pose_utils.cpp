#include <cmath>

#include <gtest/gtest.h>

#include <small_glim/common/pose_utils.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::Isometry3d yawPose(double yaw, const Eigen::Vector3d& translation) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.translation() = translation;
    return pose;
}

} // namespace

TEST(PoseUtils, InterpolateMidpoint) {
    const auto a = Eigen::Isometry3d::Identity();
    const auto b = yawPose(kPi / 2.0, Eigen::Vector3d(2.0, 0.0, 0.0));
    const auto middle = small_glim::interpolatePose(a, b, 0.5);

    EXPECT_NEAR(middle.translation().x(), 1.0, 1e-9);
    EXPECT_NEAR(middle.translation().y(), 0.0, 1e-9);
    const Eigen::Quaterniond q(middle.linear());
    EXPECT_NEAR(Eigen::AngleAxisd(q).angle(), kPi / 4.0, 1e-6);
}

TEST(PoseUtils, InterpolateClampsAlpha) {
    const auto a = Eigen::Isometry3d::Identity();
    const auto b = yawPose(0.5, Eigen::Vector3d(2.0, 0.0, 0.0));
    const auto below = small_glim::interpolatePose(a, b, -1.0);
    const auto above = small_glim::interpolatePose(a, b, 2.0);

    EXPECT_LT((below.matrix() - a.matrix()).norm(), 1e-12);
    EXPECT_LT((above.matrix() - b.matrix()).norm(), 1e-12);
}
