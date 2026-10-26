# Part of Covariance

GICP 的局部几何：由邻居估计 4×4 协方差，再对平面协方差做特征值正则化。

本节涉及的协方差统计意义与特征分解基础见 `Part_of_Basics.md` 第四节。

对应实现：`preprocess/cloud_ops.cpp` 中的 `computeCovariance` 与 `regularizePlaneCovariance`。

---

## 一、局部协方差（TODO 2）

### 1.1 这个函数要做什么

**输入**：

```text
points    点云（std::vector<Eigen::Vector4d>）
neighbors 每个点的 k 个邻居索引（连续存放，std::size_t*）
k         邻居数
```

**输出**：

```text
cov       4×4 协方差矩阵（Eigen::Matrix4d）
mean      均值（Eigen::Vector4d*，可为 nullptr）
```

**在系统中的位置**：`CloudCovarianceEstimation::compute_covariances` 为每个点调用，输出供 GICP 使用。GICP 根据协方差的各向异性决定该点在每个方向上的约束强度。

### 1.2 前置知识：协方差的含义

**一维方差**描述一组数字的分散程度。

**多维协方差矩阵**描述一组向量的分散程度，以及各维度之间的相关性。给定 k 个 3 维向量 `p_1, ..., p_k`：

```text
Cov(i, j) = E[(p_i - mean_i)(p_j - mean_j)]
```

矩阵元素含义：

```text
对角线 Cov(i, i)      第 i 维的方差
非对角线 Cov(i, j)    第 i 维与第 j 维的相关性（对称，Cov(i,j) = Cov(j,i)）
```

**几何直觉**：

```text
点云分布形状       协方差特征
-----------------  ---------------------------
球形               三个方向方差都接近
线形               一个方向方差大，另两个接近 0
平面形             一个方向方差接近 0（法向），另两个明显大于 0
```

GICP 靠协方差区分这三种形状，据此加权。

### 1.3 为什么是 4×4

点用 `Eigen::Vector4d` 齐次坐标存储：`(x, y, z, 1)`，第四维恒为 1。

**齐次坐标的好处**：4×4 齐次变换矩阵（`Isometry3d`）可直接乘 `Vector4d`，把旋转和平移打包成一次乘法。若用 `Vector3d`，变换时要分开处理 `R·p + t`。

**协方差为什么保持 4×4**：`gtsam_points` 的 GICP 因子要求点的坐标和协方差维度一致。

**第四维的物理意义**：所有点的第四维都是 1，方差为 0，所以协方差的 `(3,3)` 元素应为 0。**必须显式清零**，因为浮点运算会有残差。

### 1.4 数学公式

**均值**：

```text
mean = (1/k) · Σ_{i=1..k} p_i
```

**协方差**（两种等价写法）：

写法 A（中心化）：

```text
Cov = (1/k) · Σ_{i=1..k} (p_i - mean)(p_i - mean)^T
```

写法 B（展开）：

```text
Cov = (1/k) · Σ_{i=1..k} p_i · p_i^T - mean · mean^T
```

两者数学等价。写法 B 少做一次减法，工程上更常用。

**除以 k，不是 k-1**。这是**总体协方差**（描述这组数据本身），不是**无偏样本协方差**（用于估计总体）。除以 k-1 会让数值偏大 `k/(k-1)` 倍，GICP 权重随之偏差。

### 1.5 手算例子

两个点：`p_1 = (0,0,0,1)`，`p_2 = (2,0,0,1)`，`k = 2`。

**均值**：

```text
mean = (1/2)·[(0,0,0,1) + (2,0,0,1)] = (1, 0, 0, 1)
```

**协方差（写法 B）**：

```text
Σ p_i · p_i^T = (0,0,0,1)(0,0,0,1)^T + (2,0,0,1)(2,0,0,1)^T
              = [[0,0,0,0],     [[4,0,0,2],
                 [0,0,0,0],   +  [0,0,0,0],
                 [0,0,0,0],      [0,0,0,0],
                 [0,0,0,1]]      [2,0,0,1]]
              = [[4,0,0,2],
                 [0,0,0,0],
                 [0,0,0,0],
                 [2,0,0,2]]

(1/k) · Σ = [[2,0,0,1],
             [0,0,0,0],
             [0,0,0,0],
             [1,0,0,1]]

mean · mean^T = [[1,0,0,1],
                 [0,0,0,0],
                 [0,0,0,0],
                 [1,0,0,1]]

Cov = (1/k)·Σ - mean·mean^T = [[1,0,0,0],
                               [0,0,0,0],
                               [0,0,0,0],
                               [0,0,0,0]]
```

由于所有点的齐次分量均为 1，其中心化偏差恒为 0，所以 `(3,3)` 按公式本身就等于 0。实现中仍应显式清零，避免浮点残差：

```text
Cov = [[1,0,0,0],
       [0,0,0,0],
       [0,0,0,0],
       [0,0,0,0]]
```

`Cov(0,0) = 1`，与单测期望一致。

### 1.6 实现步骤（数学形式）

```text
输入：点云 P，邻居索引 neighbors，数量 k
输出：4×4 协方差 Cov，均值 mean

步骤 1：零邻居处理
    若 k = 0 或 neighbors 为空：
        mean ← (0, 0, 0, 0)
        Cov  ← 4×4 零矩阵
        返回

步骤 2：累加
    S_1 ← Σ_{i=1..k} P[neighbors[i]]           （4 维向量）
    S_2 ← Σ_{i=1..k} P[neighbors[i]] · P[neighbors[i]]^T   （4×4 矩阵）

步骤 3：均值
    mean ← S_1 / k

步骤 4：协方差
    Cov ← S_2 / k - mean · mean^T

步骤 5：清零齐次维
    Cov(3, 3) ← 0
```

**用到哪些 Eigen API**：

| 操作 | API |
|---|---|
| 访问点 | `points[neighbors[i]]` |
| 向量累加 | `sum += p` |
| 外积累加 | `sum_sq += p * p.transpose()` |
| 外积（列向量 × 行向量） | `mean * mean.transpose()` |
| 提取元素 | `cov(3, 3) = 0.0` |

### 1.7 常见错误对照

| 错误 | 现象 | 单测表现 |
|---|---|---|
| 除以 `k-1` | 数值偏大 `k/(k-1)` 倍，GICP 权重偏 | `ComputeCovarianceOfTwoPoints` 失败（k=2 时偏 2 倍） |
| 遗漏 `mean · mean^T` 项 | 协方差整体平移，配准完全错误 | `ComputeCovarianceOfTwoPoints` 失败（`cov(0,0)` 变成 2 而非 1） |
| 把齐次维当作独立坐标 | `Cov(3,3)` 出现虚假方差 | 单测中 `(3,3)` 期望 0 |
| `k = 0` 时访问 `neighbors[0]` | 段错误或未定义行为 | `ComputeCovarianceRejectsZeroK` 崩溃 |
| 用 `Vector3d` 而非 `Vector4d` | 与 `gtsam_points` 接口不匹配 | 编译失败 |
| 累加循环中下标用错（比如 `neighbors[i]` 写成 `i`） | 结果与具体数据无关，看起来「差不多」但错 | 大多数情况下单测会失败 |

### 1.8 验证方法

**单测**：

```text
CloudOps.ComputeCovarianceOfTwoPoints    手算例子
CloudOps.ComputeCovarianceRejectsZeroK   k=0 不崩溃
```

**手算步骤**：用 1.5 节的两个点，验证 `cov(0,0) = 1`，`mean.x() = 1`。

---

## 二、平面协方差正则化（TODO 3）

### 2.1 这个函数要做什么

**输入**：

```text
cov         4×4 协方差
```

**输出**：

```text
cov_reg     正则化后的 4×4 协方差
eigenvalues 原始特征值（Eigen::Vector3d*，可为 nullptr）
eigenvectors 原始特征向量（Eigen::Matrix3d*，可为 nullptr）
```

**在系统中的位置**：`computeCovariance` 之后立即调用：

```text
covs[i] = regularizePlaneCovariance(cov, nullptr, &eigenvectors)
normals[i] ← eigenvectors 的第 0 列（即最小特征值对应的特征向量 = 法向）
```

调用方依赖 `eigenvectors` 获取法向。**不写回 `eigenvectors`，后续所有法向都是垃圾值。** 特征向量的正负号本质上不唯一；`CloudCovarianceEstimation` 在 `points[i] · normal > 0` 时会把法向取反，使结果大致指向原点，这一符号约定不属于 `regularizePlaneCovariance` 的契约。

### 2.2 前置知识：特征分解

**定义**：对于方阵 A，若存在非零向量 `v` 和标量 `λ`，满足 `A·v = λ·v`，则 `v` 是 A 的特征向量，`λ` 是对应特征值。

**几何直觉**：`A·v = λ·v` 意为「A 对 v 的变换不改变方向，只改变长度，λ 是长度缩放因子」。

**实对称矩阵**一定能分解为：

```text
A = V · diag(λ_1, λ_2, λ_3) · V^T
```

- `V` 的三列是三个特征向量 `v_1, v_2, v_3`，互相正交且单位化；
- `λ_1 ≤ λ_2 ≤ λ_3` 是三个特征值（Eigen 的 `SelfAdjointEigenSolver` 升序输出）。

**几何意义**：

```text
特征向量 = 点云分布的主方向
特征值   = 该方向上的方差（分布宽度）
```

**平面点的特征值**：

```text
λ_1 ≈ 0     法向（垂直平面方向几乎无展布）
λ_2 ≈ 1     切向 1
λ_3 ≈ 1     切向 2
```

`v_1`（最小特征值对应的特征向量）就是平面法向。

### 2.3 为什么需要正则化

真实点云有噪声，平面点的协方差可能是：

```text
λ_1 = 0.0001
λ_2 = 1.001
λ_3 = 1.002
```

GICP 中法向方向的约束权重约为 `1/λ_1`。λ_1 很小时，法向权重比切向大 4 个数量级，导致：

- 法向的微小噪声被极度放大；
- 一次配准迭代中，法向约束几乎完全主导；
- 相邻帧之间法向估计的微小差异，会让轨迹剧烈抖动。

**工程做法**：不再使用原始特征值，而是把三个特征值统一替换为 `(1e-3, 1, 1)`。平面朝向（V 不变）保持正确，同时把法向权重限制在固定量级。注意这不是 `max(λ_1, 1e-3)` 式的下限截断：即使原始 λ_1 大于 `1e-3`，也会被替换。

**为什么固定为 `(1e-3, 1, 1)` 而不是保留原始切向特征值**：GICP 的常见做法是把所有平面点统一归一化为「法向 1e-3，切向 1」，让平面约束强度一致，便于优化器收敛。

### 2.4 数学公式

给定 4×4 协方差 `Cov`，取左上 3×3 部分 `A`，做特征分解：

```text
A = V · diag(λ_1, λ_2, λ_3) · V^T
```

用固定特征值重建：

```text
A_reg = V · diag(1e-3, 1, 1) · V^T
```

组装 4×4：

```text
Cov_reg = [[A_reg   0],
           [0       0]]
```

**关键不变量**：`V` 不变。平面朝向不变，只改变各方向的方差。

### 2.5 实现步骤（数学形式）

```text
输入：4×4 协方差 Cov
输出：4×4 协方差 Cov_reg；可能的原始特征值 λ、特征向量 V

步骤 1：取 3×3 部分
    A ← Cov 的左上 3×3

步骤 2：特征分解（实对称矩阵）
    λ_1 ≤ λ_2 ≤ λ_3 以及对应的特征向量 V = [v_1 | v_2 | v_3]
    其中 A = V · diag(λ_1, λ_2, λ_3) · V^T

步骤 3：写回原始结果（若指针非空）
    若 eigenvalues ≠ nullptr：eigenvalues ← (λ_1, λ_2, λ_3)
    若 eigenvectors ≠ nullptr：eigenvectors ← V

步骤 4：用固定特征值重建 3×3
    A_reg ← V · diag(1e-3, 1, 1) · V^T

步骤 5：组装 4×4
    Cov_reg ← 4×4 零矩阵
    Cov_reg 的左上 3×3 ← A_reg
```

**用到哪些 Eigen API**：

| 操作 | API |
|---|---|
| 取左上 3×3 部分 | `cov.block<3, 3>(0, 0)` |
| 实对称特征分解 | `Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>` |
| 快速一次性分解 | `eig.computeDirect(A)` |
| 取特征值 | `eig.eigenvalues()`（升序） |
| 取特征向量 | `eig.eigenvectors()`（列向量） |
| 对角矩阵 | `Eigen::Vector3d(1e-3, 1.0, 1.0).asDiagonal()` |
| 重建 | `V * diag * V.transpose()` |
| 放入 4×4 左上 3×3 | `result.block<3, 3>(0, 0) = A_reg` |

`computeDirect` 用解析解，适合 2×2 和 3×3；`compute` 用迭代，适合大矩阵。3×3 用 `computeDirect` 通常更快，但解析解在接近重根或病态时精度可能略差；本项目单测场景下两者都可用。

### 2.6 手算例子

输入 `Cov = I`（4×4 单位矩阵）。

```text
A = [[1, 0, 0],
     [0, 1, 0],
     [0, 0, 1]]

特征分解：
    λ_1 = λ_2 = λ_3 = 1
    V = I

重建：
    A_reg = I · diag(1e-3, 1, 1) · I
          = diag(1e-3, 1, 1)
          = [[1e-3, 0,    0   ],
             [0,    1,    0   ],
             [0,    0,    1   ]]

组装 4×4：
    Cov_reg = [[1e-3, 0, 0, 0],
               [0,    1, 0, 0],
               [0,    0, 1, 0],
               [0,    0, 0, 0]]
```

期望 `Cov_reg(0,0) = 1e-3`，`Cov_reg(1,1) = 1`，`Cov_reg(2,2) = 1`，`Cov_reg(3,3) = 0`。

### 2.7 常见错误对照

| 错误 | 现象 | 单测表现 |
|---|---|---|
| 不写回 eigenvectors | 调用方拿不到法向，系统层面法向全错 | `RegularizePlaneCovarianceReturnsEigenSystem` 失败（输出未写入） |
| 不写回 eigenvalues | 调用方无法使用原始特征值做后续判断 | `RegularizePlaneCovarianceReturnsEigenSystem` 失败（输出未写入） |
| 特征值顺序写成 `(1, 1, 1e-3)` | 法向变成切向，平面被当作线 | `RegularizePlaneCovarianceOnIdentity` 失败（`(0,0)` 不是 `1e-3`），`RegularizePlaneCovarianceReturnsEigenSystem` 也失败 |
| 不替换特征值，直接用原始 | 正则化没生效，法向权重仍然数值不稳 | `RegularizePlaneCovarianceOnIdentity` 失败 |
| 4×4 的 `(3,3)` 未清零 | 齐次维带噪声 | 单测通过，系统有细微偏差 |
| 对 4×4 整体做特征分解（而非只对左上 3×3） | 结果无几何意义 | `RegularizePlaneCovarianceReturnsEigenSystem` 失败 |
| 重建时用 `V^T · diag · V` 顺序 | 结果不是协方差 | `RegularizePlaneCovarianceOnIdentity` 可能通过（V=I），旋转用例失败 |
| 忘记初始化 4×4 其余元素为 0 | 其余位置可能是垃圾值 | 可能随机失败，系统有偏差 |

### 2.8 验证方法

**单测**：

```text
CloudOps.RegularizePlaneCovarianceOnIdentity          单位阵正则化
CloudOps.RegularizePlaneCovarianceReturnsEigenSystem  旋转非对角协方差；检查原始特征值、第一特征向量方向并验证重建
```

**手算步骤**：

1. 用 2.6 节的单位阵，验证四个对角线元素。
2. 第二个用例把特征值 `(0.01, 0.5, 2.0)` 绕 Z 轴旋转 0.4 rad 再构造协方差。必须原样写回升序的 `eigenvalues`，并让 `eigenvectors.col(0)` 与旋转矩阵第 0 列同向或反向（特征向量符号不唯一，单测检查 `abs(dot)`）；正则化结果应为 `R · diag(1e-3, 1, 1) · R^T` 填回左上 3×3，其余元素为 0。

---

## 三、两个函数在系统中的位置

```text
点云预处理
    → CloudCovarianceEstimation::compute_covariances
        → computeCovariance（TODO 2）        输出 4×4 协方差
        → regularizePlaneCovariance（TODO 3）  正则化，写回法向
    → PointCloudCPU 携带 covs 和 normals
    → GICP 因子读取 covs，计算加权对应点距离
```

**TODO 2 写错的系统现象**：GICP 无法区分平面、线、散点，配准权重全错，轨迹抖动，地图细节模糊。

**TODO 3 写错的系统现象**：法向不稳定，同一面墙在不同帧的估计法向不同，GICP 反复把点云往不同方向拉，轨迹同样抖动，但比 TODO 2 的症状更隐蔽。
