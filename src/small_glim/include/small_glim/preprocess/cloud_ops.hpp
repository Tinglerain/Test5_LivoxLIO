// 点云预处理基础操作（新人实现）。
#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace small_glim {

/// 距离滤波：保留 near < ||p.xyz|| < far 的有限点，返回原索引。
///
/// 约定：
///   - 只使用前三维，忽略齐次分量；
///   - 非有限点直接丢弃；
///   - 严格不等号：距离恰好等于阈值时不保留；
///   - 返回索引保持输入顺序。
std::vector<int> filterByDistance(
    const Eigen::Vector4d* points,
    std::size_t num_points,
    double near_thresh,
    double far_thresh);

/// 对每个点找最近 k 个邻居（包含自身），返回扁平化索引。
///
/// 约定：
///   - k_eff = min(k, num_points)；
///   - 返回长度 = num_points * k_eff；
///   - 第 i 个点的邻居写在 [i*k_eff, (i+1)*k_eff)；
///   - 结果按距离从近到远。
std::vector<std::size_t> findNeighbors(
    const Eigen::Vector4d* points,
    std::size_t num_points,
    std::size_t k);

/// 用 k 个邻居计算均值和协方差。
///
/// 约定：
///   - neighbors 指向 k 个索引；
///   - mean = sum(p) / k；
///   - cov = sum(p p^T) / k - mean mean^T；
///   - 返回 4x4 矩阵，mean_out 非空时写入均值；
///   - k == 0 时返回零矩阵。
Eigen::Matrix4d computeCovariance(
    const std::vector<Eigen::Vector4d>& points,
    const std::size_t* neighbors,
    std::size_t k,
    Eigen::Vector4d* mean_out = nullptr);

/// 体素下采样：每个体素选一个代表点，返回原索引。
///
/// 约定：
///   - leaf <= 0 或输入为空时返回所有索引；
///   - 体素键 = (floor(x/leaf), floor(y/leaf), floor(z/leaf))；
///   - 每个体素选离体素质心最近的点；
///   - 输出按体素键排序，保证确定性。
std::vector<int> voxelDownsampleIndices(
    const Eigen::Vector4d* points,
    std::size_t num_points,
    double leaf);

/// 平面协方差正则化。
///
/// 约定：
///   - 对 3x3 块做特征分解；
///   - 用固定特征值 (1e-3, 1, 1) 重建；
///   - 返回 4x4 矩阵；
///   - eigenvalues / eigenvectors 非空时写入原始特征值和特征向量。
Eigen::Matrix4d regularizePlaneCovariance(
    const Eigen::Matrix4d& cov,
    Eigen::Vector3d* eigenvalues = nullptr,
    Eigen::Matrix3d* eigenvectors = nullptr);

} // namespace small_glim
