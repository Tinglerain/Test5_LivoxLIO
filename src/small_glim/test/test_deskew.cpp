#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <small_glim/preprocess/cloud_deskewing.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::Isometry3d yawPose(double yaw, const Eigen::Vector3d& t) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.translation() = t;
    return pose;
}

} // namespace

TEST(Deskew, IdentityPosesKeepPoints) {
    small_glim::CloudDeskewing deskewing;
    const std::vector<double> times{0.0, 1.0};
    const std::vector<Eigen::Isometry3d> poses{
        Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity()};
    const std::vector<Eigen::Vector4d> points{
        Eigen::Vector4d(1.0, 0.0, 0.0, 1.0),
        Eigen::Vector4d(1.0, 0.0, 0.0, 1.0)};

    const auto output = deskewing.deskew(
        Eigen::Isometry3d::Identity(), times, poses, 0.0, times, points);
    ASSERT_EQ(output.size(), 2U);
    EXPECT_NEAR(output[0].x(), 1.0, 1e-9);
    EXPECT_NEAR(output[1].x(), 1.0, 1e-9);
}

TEST(Deskew, RotationIsCompensated) {
    small_glim::CloudDeskewing deskewing;
    const std::vector<double> imu_times{0.0, 1.0};
    const std::vector<Eigen::Isometry3d> poses{
        Eigen::Isometry3d::Identity(),
        yawPose(kPi / 2.0, Eigen::Vector3d::Zero())};
    const std::vector<double> times{0.0, 0.5, 1.0};
    const std::vector<Eigen::Vector4d> points{
        Eigen::Vector4d(1.0, 0.0, 0.0, 1.0),
        Eigen::Vector4d(1.0, 0.0, 0.0, 1.0),
        Eigen::Vector4d(1.0, 0.0, 0.0, 1.0)};

    const auto output = deskewing.deskew(
        Eigen::Isometry3d::Identity(), imu_times, poses, 0.0, times, points);
    ASSERT_EQ(output.size(), 3U);
    EXPECT_NEAR(output[0].x(), 1.0, 1e-9);
    EXPECT_NEAR(output[0].y(), 0.0, 1e-9);
    EXPECT_NEAR(output[1].x(), std::sqrt(0.5), 1e-6);
    EXPECT_NEAR(output[1].y(), std::sqrt(0.5), 1e-6);
    EXPECT_NEAR(output[2].x(), 0.0, 1e-6);
    EXPECT_NEAR(output[2].y(), 1.0, 1e-6);
}

TEST(Deskew, InvalidTimesReturnInput) {
    small_glim::CloudDeskewing deskewing;
    const std::vector<double> imu_times{0.0, 1.0};
    const std::vector<Eigen::Isometry3d> poses{
        Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity()};
    const std::vector<double> times{1.0, 0.0};
    const std::vector<Eigen::Vector4d> points{
        Eigen::Vector4d(1.0, 2.0, 3.0, 1.0),
        Eigen::Vector4d(4.0, 5.0, 6.0, 1.0)};

    const auto output = deskewing.deskew(
        Eigen::Isometry3d::Identity(), imu_times, poses, 0.0, times, points);
    ASSERT_EQ(output.size(), points.size());
    EXPECT_NEAR((output[0] - points[0]).norm(), 0.0, 1e-12);
    EXPECT_NEAR((output[1] - points[1]).norm(), 0.0, 1e-12);
}
