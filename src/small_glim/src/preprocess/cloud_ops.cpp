// ============================================================================
//  点云预处理基础操作（preprocess 模块）。
//
//  本 Test 聚焦两个原理性函数：
//
//    TODO 2  computeCovariance         局部协方差（GICP 的核心）
//    TODO 3  regularizePlaneCovariance 平面协方差正则化
//
//  filterByDistance / findNeighbors / voxelDownsampleIndices 已经给全，
//  它们是调库和流程代码，不是本 Test 要考的原理。
// ============================================================================
#include <small_glim/preprocess/cloud_ops.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>

#include <gtsam_points/ann/kdtree.hpp>

namespace small_glim {

std::vector<int> filterByDistance(
    const Eigen::Vector4d* points,
    std::size_t num_points,
    double near_thresh,
    double far_thresh) {
    std::vector<int> indices;
    if (points == nullptr) return indices;
    indices.reserve(num_points);

    const double near_squared = near_thresh * near_thresh;
    const double far_squared = far_thresh * far_thresh;
    for (std::size_t i = 0; i < num_points; ++i) {
        const Eigen::Vector4d& point = points[i];
        if (!point.allFinite()) continue;
        const double squared = point.head<3>().squaredNorm();
        if (squared > near_squared && squared < far_squared) {
            indices.push_back(static_cast<int>(i));
        }
    }
    return indices;
}

std::vector<std::size_t> findNeighbors(
    const Eigen::Vector4d* points,
    std::size_t num_points,
    std::size_t k) {
    if (points == nullptr || num_points == 0 || k == 0) return {};

    const std::size_t k_eff = std::min(k, num_points);
    gtsam_points::KdTree tree(points, static_cast<int>(num_points));
    std::vector<std::size_t> neighbors(num_points * k_eff);

    std::vector<std::size_t> k_indices(k_eff);
    std::vector<double> k_squared_distances(k_eff);
    for (std::size_t i = 0; i < num_points; ++i) {
        tree.knn_search(
            points[i].data(), static_cast<int>(k_eff),
            k_indices.data(), k_squared_distances.data());
        std::copy(
            k_indices.begin(), k_indices.end(),
            neighbors.begin() + static_cast<std::ptrdiff_t>(i * k_eff));
    }
    return neighbors;
}

// ----------------------------------------------------------------------------
// TODO 2：computeCovariance
// ----------------------------------------------------------------------------
//
// 【原理】
//
// GICP 用局部协方差判断一个点周围像平面、线还是点。
//
//   mean = (p_1 + ... + p_k) / k
//   cov  = (Σ p_i p_i^T) / k - mean * mean^T
//
// 等价写法是 Σ(p_i-mean)(p_i-mean)^T / k。第一种少一次减法。
//
// 【契约】
//
//   - neighbors 指向 k 个下标，包含中心点自身；
//   - mean_out 非空时写入均值；
//   - k == 0 或 neighbors == nullptr -> 零矩阵、零均值；
//   - 返回 4x4，和 gtsam_points 的齐次坐标接口一致。
//
// 【例子】
//
//   (0,0,0,1), (2,0,0,1), k=2
//   mean=(1,0,0,1)
//   cov(0,0)=1, cov(1,1)=0, cov(3,3)=0
//
// 【易错】
//
//   除以 k-1        -> 样本协方差，权重偏大
//   不减 mean mean^T -> 协方差平移，配准崩
//   w 分量也当坐标   -> 齐次分量出现假方差
//
// 【测试】
//
//   CloudOps.ComputeCovarianceOfTwoPoints
//   CloudOps.ComputeCovarianceRejectsZeroK
Eigen::Matrix4d computeCovariance(
    const std::vector<Eigen::Vector4d>& points,
    const std::size_t* neighbors,
    std::size_t k,
    Eigen::Vector4d* mean_out) {
    (void)points;
    (void)neighbors;
    (void)k;
    if (mean_out) *mean_out = Eigen::Vector4d::Zero();
    return Eigen::Matrix4d::Zero();
}

std::vector<int> voxelDownsampleIndices(
    const Eigen::Vector4d* points,
    std::size_t num_points,
    double leaf) {
    std::vector<int> indices;
    if (points == nullptr || num_points == 0) return indices;
    if (!(leaf > 0.0)) {
        indices.resize(num_points);
        for (std::size_t i = 0; i < num_points; ++i) {
            indices[i] = static_cast<int>(i);
        }
        return indices;
    }

    using Key = std::tuple<int, int, int>;
    std::map<Key, std::vector<std::size_t>> voxels;
    for (std::size_t i = 0; i < num_points; ++i) {
        const Eigen::Vector4d& point = points[i];
        if (!point.allFinite()) continue;
        const Key key{
            static_cast<int>(std::floor(point.x() / leaf)),
            static_cast<int>(std::floor(point.y() / leaf)),
            static_cast<int>(std::floor(point.z() / leaf))};
        voxels[key].push_back(i);
    }

    indices.reserve(voxels.size());
    for (const auto& [key, members] : voxels) {
        (void)key;
        Eigen::Vector4d centroid = Eigen::Vector4d::Zero();
        for (const std::size_t member : members) centroid += points[member];
        centroid /= static_cast<double>(members.size());

        std::size_t best = members.front();
        double best_squared = (points[best] - centroid).squaredNorm();
        for (const std::size_t member : members) {
            const double squared = (points[member] - centroid).squaredNorm();
            if (squared < best_squared) {
                best_squared = squared;
                best = member;
            }
        }
        indices.push_back(static_cast<int>(best));
    }
    return indices;
}

// ----------------------------------------------------------------------------
// TODO 3：regularizePlaneCovariance
// ----------------------------------------------------------------------------
//
// 【原理】
//
// GICP 希望平面点的协方差在法向很小、切向接近 1。
// 真实点云有噪声，最小特征值很小但不为零，直接做约束数值不稳。
//
// 做法：
//
//   对 3x3 协方差做特征分解；
//   把特征值强制成 (1e-3, 1, 1)；
//   用原特征向量重建。
//
// 这样保留平面朝向，同时把法向权重限制在固定量级。
//
// 【契约】
//
//   - eigenvalues 非空时写原始特征值；
//   - eigenvectors 非空时写原始特征向量；
//   - 用 (1e-3, 1, 1) 重建 4x4，其余为 0。
//
// 【例子】
//
//   cov = I -> diag(1e-3, 1, 1, 0)
//
// 【易错】
//
//   不写 eigenvectors -> 法向丢失
//   特征值顺序反了     -> 法向变切向
//   保留原特征值       -> 正则化没生效
//   (3,3) 没清零       -> 齐次分量带噪声
//
// 【测试】
//
//   CloudOps.RegularizePlaneCovarianceOnIdentity
//   CloudOps.RegularizePlaneCovarianceReturnsEigenSystem
Eigen::Matrix4d regularizePlaneCovariance(
    const Eigen::Matrix4d& cov,
    Eigen::Vector3d* eigenvalues,
    Eigen::Matrix3d* eigenvectors) {
    (void)cov;
    (void)eigenvalues;
    (void)eigenvectors;
    return Eigen::Matrix4d::Zero();
}

} // namespace small_glim
