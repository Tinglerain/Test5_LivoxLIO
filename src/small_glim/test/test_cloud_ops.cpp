#include <cmath>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <small_glim/preprocess/cloud_ops.hpp>

TEST(CloudOps, ComputeCovarianceOfTwoPoints) {
    const std::vector<Eigen::Vector4d> points{
        Eigen::Vector4d(0.0, 0.0, 0.0, 1.0),
        Eigen::Vector4d(2.0, 0.0, 0.0, 1.0)};
    const std::size_t neighbors[2] = {0, 1};

    Eigen::Vector4d mean;
    const Eigen::Matrix4d cov =
        small_glim::computeCovariance(points, neighbors, 2, &mean);

    EXPECT_NEAR(mean.x(), 1.0, 1e-9);
    EXPECT_NEAR(mean.y(), 0.0, 1e-9);
    EXPECT_NEAR(mean.z(), 0.0, 1e-9);
    EXPECT_NEAR(cov(0, 0), 1.0, 1e-9);
    EXPECT_NEAR(cov(1, 1), 0.0, 1e-9);
    EXPECT_NEAR(cov(3, 3), 0.0, 1e-9);
}

TEST(CloudOps, ComputeCovarianceRejectsZeroK) {
    const std::vector<Eigen::Vector4d> points{
        Eigen::Vector4d(0.0, 0.0, 0.0, 1.0)};
    Eigen::Vector4d mean = Eigen::Vector4d::Ones();
    const Eigen::Matrix4d cov =
        small_glim::computeCovariance(points, nullptr, 0, &mean);
    EXPECT_LT(cov.norm(), 1e-12);
    EXPECT_LT(mean.norm(), 1e-12);
}

TEST(CloudOps, RegularizePlaneCovarianceOnIdentity) {
    const Eigen::Matrix4d cov = Eigen::Matrix4d::Identity();
    const Eigen::Matrix4d regularized =
        small_glim::regularizePlaneCovariance(cov);
    EXPECT_NEAR(regularized(0, 0), 1e-3, 1e-12);
    EXPECT_NEAR(regularized(1, 1), 1.0, 1e-12);
    EXPECT_NEAR(regularized(2, 2), 1.0, 1e-12);
    EXPECT_NEAR(regularized(3, 3), 0.0, 1e-12);
}

TEST(CloudOps, RegularizePlaneCovarianceReturnsEigenSystem) {
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix4d cov = Eigen::Matrix4d::Zero();
    cov.block<3, 3>(0, 0) =
        rotation * Eigen::Vector3d(0.01, 0.5, 2.0).asDiagonal() * rotation.transpose();

    Eigen::Vector3d eigenvalues;
    Eigen::Matrix3d eigenvectors;
    const Eigen::Matrix4d regularized = small_glim::regularizePlaneCovariance(
        cov, &eigenvalues, &eigenvectors);

    EXPECT_NEAR(eigenvalues[0], 0.01, 1e-12);
    EXPECT_NEAR(eigenvalues[1], 0.5, 1e-12);
    EXPECT_NEAR(eigenvalues[2], 2.0, 1e-12);
    EXPECT_NEAR(std::abs(eigenvectors.col(0).dot(rotation.col(0))), 1.0, 1e-12);

    Eigen::Matrix4d expected = Eigen::Matrix4d::Zero();
    expected.block<3, 3>(0, 0) =
        rotation * Eigen::Vector3d(1e-3, 1.0, 1.0).asDiagonal() * rotation.transpose();
    EXPECT_LT((regularized - expected).norm(), 1e-12);
}
