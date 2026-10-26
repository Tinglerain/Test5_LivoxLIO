#include <small_glim/preprocess/cloud_covariance_estimation.hpp>
#include <small_glim/common/logger.hpp>
#include <small_glim/preprocess/cloud_ops.hpp>
#include <gtsam_points/util/parallelism.hpp>

namespace small_glim {

CloudCovarianceEstimation::CloudCovarianceEstimation(const int num_threads):
    regularization_method(RegularizationMethod::PLANE),
    num_threads(num_threads) {}

void CloudCovarianceEstimation::estimate(
    const std::vector<Eigen::Vector4d>& points,
    const std::vector<size_t>& neighbors,
    std::vector<Eigen::Vector4d>& normals,
    std::vector<Eigen::Matrix4d>& covs
) const {
    if (points.empty()) {
        return;
    }

    const size_t k = neighbors.size() / points.size();
    if (k * points.size() != neighbors.size()) {
        logger::fatal("cloud_cov_estimation", "k * points.size() != neighbors.size()");
        std::exit(EXIT_FAILURE);
    }

    estimate(points, neighbors, k, normals, covs);
}

void CloudCovarianceEstimation::estimate(
    const std::vector<Eigen::Vector4d>& points,
    const std::vector<size_t>& neighbors,
    const size_t k_neighbors,
    std::vector<Eigen::Vector4d>& normals,
    std::vector<Eigen::Matrix4d>& covs
) const {
    if (points.empty()) {
        return;
    }

    const size_t k_correspondences = neighbors.size() / points.size();
    assert(k_correspondences * points.size() == neighbors.size());
    assert(k_neighbors <= k_correspondences);

    compute_covariances(points, neighbors, k_neighbors, k_correspondences, &normals, covs);
}

std::vector<Eigen::Matrix4d> CloudCovarianceEstimation::estimate(
    const std::vector<Eigen::Vector4d>& points,
    const std::vector<size_t>& neighbors,
    const size_t k_neighbors
) const {
    if (points.empty()) {
        return std::vector<Eigen::Matrix4d>();
    }

    const size_t k_correspondences = neighbors.size() / points.size();
    assert(k_correspondences * points.size() == neighbors.size());
    assert(k_neighbors <= k_correspondences);

    std::vector<Eigen::Matrix4d> covs;
    compute_covariances(points, neighbors, k_neighbors, k_correspondences, nullptr, covs);
    return covs;
}

std::vector<Eigen::Matrix4d> CloudCovarianceEstimation::estimate(
    const std::vector<Eigen::Vector4d>& points,
    const std::vector<size_t>& neighbors
) const {
    if (points.empty()) {
        return std::vector<Eigen::Matrix4d>();
    }

    const size_t k = neighbors.size() / points.size();
    if (k * points.size() != neighbors.size()) {
        logger::fatal("cloud_cov_estimation", "k * points.size() != neighbors.size()");
        std::exit(EXIT_FAILURE);
    }

    return estimate(points, neighbors, k);
}

void CloudCovarianceEstimation::compute_covariances(
    const std::vector<Eigen::Vector4d>& points,
    const std::vector<size_t>& neighbors,
    const size_t k_neighbors,
    const size_t k_correspondences,
    std::vector<Eigen::Vector4d>* normals,
    std::vector<Eigen::Matrix4d>& covs
) const {
    covs.resize(points.size());
    if (normals) {
        normals->resize(points.size());
    }

    const auto calc_cov = [&](size_t i) {
        const size_t begin = k_correspondences * i;
        Eigen::Vector4d mean = Eigen::Vector4d::Zero();
        const Eigen::Matrix4d cov = computeCovariance(
            points, neighbors.data() + begin, k_neighbors, &mean);

        if (normals) {
            Eigen::Matrix3d eigenvectors;
            covs[i] = regularize(cov, nullptr, &eigenvectors);
            (*normals)[i] << eigenvectors.col(0), 0.0;
            if (points[i].dot((*normals)[i]) > 0.0) {
                (*normals)[i] = -(*normals)[i];
            }
        } else {
            covs[i] = regularize(cov);
        }
        covs[i](3, 3) = 0.0;
    };

    #pragma omp parallel for num_threads(num_threads) schedule(guided, 8)
    for (size_t i = 0; i < points.size(); i++) {
        calc_cov(i);
    }
}

Eigen::Matrix4d CloudCovarianceEstimation::regularize(
    const Eigen::Matrix4d& cov,
    Eigen::Vector3d* eigenvalues,
    Eigen::Matrix3d* eigenvectors
) const {
    switch (regularization_method) {
        case RegularizationMethod::NONE: {
            return cov;
        }

        case RegularizationMethod::PLANE: {
            return regularizePlaneCovariance(cov, eigenvalues, eigenvectors);
        }

        case RegularizationMethod::NORMALIZED_MIN_EIG: {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig;
            eig.computeDirect(cov.block<3, 3>(0, 0));

            if (eigenvalues) {
                *eigenvalues = eig.eigenvalues();
            }
            if (eigenvectors) {
                *eigenvectors = eig.eigenvectors();
            }

            Eigen::Vector3d values = eig.eigenvalues() / eig.eigenvalues()[2];
            values = values.array().max(1e-3);

            Eigen::Matrix4d c = Eigen::Matrix4d::Zero();
            c.block<3, 3>(0, 0) = eig.eigenvectors() * values.asDiagonal() * eig.eigenvectors().transpose();
            return c;
        }

        case RegularizationMethod::FROBENIUS: {
            const double lambda = 1e-3;
            Eigen::Matrix3d C = cov.block<3, 3>(0, 0) + lambda * Eigen::Matrix3d::Identity();
            Eigen::Matrix3d C_inv = C.inverse();
            Eigen::Matrix4d C_ = Eigen::Matrix4d::Zero();
            C_.block<3, 3>(0, 0) = (C_inv / C_inv.norm()).inverse();
            return C_;
        }
    }
    std::unreachable();
}

}