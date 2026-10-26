#include <cmath>

#include <gtest/gtest.h>

#include <small_glim/odometry/initial_attitude.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;

} // namespace

TEST(InitialAttitude, GravityIdentityWhenAlreadyUp) {
    const Eigen::Matrix3d R =
        small_glim::gravityAlignment(Eigen::Vector3d::UnitZ());
    EXPECT_LT((R - Eigen::Matrix3d::Identity()).norm(), 1e-12);
}

TEST(InitialAttitude, GravityMapsMeasuredDirectionToZ) {
    const Eigen::Vector3d g(1.0, 0.0, 0.0);
    const Eigen::Matrix3d R = small_glim::gravityAlignment(g);
    EXPECT_LT((R * g.normalized() - Eigen::Vector3d::UnitZ()).norm(), 1e-9);
    EXPECT_NEAR(R.determinant(), 1.0, 1e-9);
}

TEST(InitialAttitude, GravityZeroReturnsIdentity) {
    const Eigen::Matrix3d R =
        small_glim::gravityAlignment(Eigen::Vector3d::Zero());
    EXPECT_LT((R - Eigen::Matrix3d::Identity()).norm(), 1e-12);
}

TEST(InitialAttitude, YawMapsHeadingToX) {
    const Eigen::Vector3d heading(0.0, 1.0, 0.0);
    const Eigen::Matrix3d R = small_glim::yawAlignment(heading);
    EXPECT_LT((R * heading - Eigen::Vector3d::UnitX()).norm(), 1e-9);
}

TEST(InitialAttitude, YawIgnoresZComponent) {
    const Eigen::Matrix3d a =
        small_glim::yawAlignment(Eigen::Vector3d(0.0, 1.0, 0.0));
    const Eigen::Matrix3d b =
        small_glim::yawAlignment(Eigen::Vector3d(0.0, 1.0, 9.0));
    EXPECT_LT((a - b).norm(), 1e-12);
}

TEST(InitialAttitude, YawZeroReturnsIdentity) {
    const Eigen::Matrix3d R =
        small_glim::yawAlignment(Eigen::Vector3d(0.0, 0.0, 0.0));
    EXPECT_LT((R - Eigen::Matrix3d::Identity()).norm(), 1e-12);
}
