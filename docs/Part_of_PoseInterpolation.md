# Part of Pose Interpolation

给定两个时刻的位姿，求中间时刻的位姿。

本节涉及的位姿表示、四元数基础见 `Part_of_Basics.md` 第三节。

**本函数是已提供的去畸变模块 `CloudDeskewing::deskew` 的关键依赖。** 若本函数写错，已提供的 deskew 输出也会错，系统层面表现为点云扭曲或重影。

对应实现：`common/pose_utils.cpp` 中的 `interpolatePose`。

---

## 一、这个函数要做什么

**输入**：

```text
位姿 a：Eigen::Isometry3d
位姿 b：Eigen::Isometry3d
插值系数 alpha：double
```

**输出**：位姿 c（`Eigen::Isometry3d`），满足：

```text
alpha = 0    →  c = a
alpha = 1    →  c = b
alpha = 0.5  →  c 的位置在 a、b 平移的中点，旋转在 a、b 的最短旋转路径中点
```

**在系统中的位置**：已提供的 `CloudDeskewing::deskew` 对每个时间点调用它，插值出该时刻的 IMU 位姿：

```text
T_world_imu1 = interpolatePose(imu_poses[i], imu_poses[i+1], alpha)
```

其中 `alpha` 由该点时间戳在两个 IMU 采样时刻之间的位置决定。IMU 采样间隔约 5 毫秒，激光点的时间戳是任意值，需要插值。

---

## 二、前置知识：Isometry3d 的内部结构

`Eigen::Isometry3d` 表示刚体变换，内部是一个 4×4 矩阵：

```text
      ┌         ┐
      │  R   t  │
T =   │         │
      │  0   1  │
      └         ┘
```

- `R`：3×3 旋转矩阵（`T.linear()`）
- `t`：3×1 平移向量（`T.translation()`）

其语义是：把 b 系的点 p 变换到 a 系，`p_a = R · p_b + t`。

**不能对 `T.matrix()` 的 16 个元素直接做线性插值**，原因见第四节。

---

## 三、平移插值

平移是欧氏空间中的 3 维向量，可直接线性插值：

```text
t(alpha) = (1 - alpha) · t_a + alpha · t_b
```

**手算**：`t_a = (0,0,0)`，`t_b = (2,0,0)`，`alpha = 0.5`：

```text
t(0.5) = 0.5 · (0,0,0) + 0.5 · (2,0,0) = (1,0,0)
```

---

## 四、旋转为什么不能线性插值

旋转矩阵 3×3 共 9 个元素，但受三个约束：

- 三列必须是单位向量；
- 三列两两正交；
- 行列式为 +1。

**约束后只有 3 个自由度**。对 9 个元素独立插值，得到的矩阵通常不再满足这些约束。

**反例**：`a` 为单位矩阵，`b` 为绕 Z 转 180°：

```text
a.R = [[ 1,  0,  0],       b.R = [[-1,  0,  0],
       [ 0,  1,  0],              [ 0, -1,  0],
       [ 0,  0,  1]]              [ 0,  0,  1]]

alpha = 0.5 逐元素插值 = [[0, 0, 0],
                         [0, 0, 0],
                         [0, 0, 1]]
```

结果不是旋转矩阵：它把任意点 `(x, y, z)` 变成 `(0, 0, z)`，x/y 信息被压掉，行列式为 0。

**欧拉角同样不行**：表示不唯一（`yaw` 与 `yaw+360°` 是同一旋转，但数值相差 360），且存在万向锁。

---

## 五、四元数 slerp

### 5.1 四元数的几何图像

单位四元数 `(x, y, z, w)` 满足 `x² + y² + z² + w² = 1`，是 4 维空间中的单位球面上的点。每个旋转对应球面上的一对「对径点」`q` 与 `-q`。

**关键性质**：所有合法旋转都在这个球面上，而球面上任意两点之间都有「大圆弧」这条最短路径。

### 5.2 slerp 的定义

slerp（Spherical Linear Interpolation，球面线性插值）就是沿球面大圆弧按比例取点：

```text
slerp(q_a, q_b, alpha) = 球面上从 q_a 到 q_b 的大圆弧上，比例 alpha 处的点
```

**几何直觉**：

```text
线性插值：两点连直线 → 中间点不在球面上 → 不是合法旋转
slerp：  两点沿大圆弧 → 中间点始终在球面上 → 始终是合法旋转
```

### 5.3 参数顺序陷阱

Eigen 的 `Quaterniond::slerp` 签名是：

```text
q_a.slerp(t, q_b)   // 返回从 q_a 到 q_b 的比例 t 处
```

参数顺序是 `(插值系数, 终点)`，不是 `(终点, 系数)`。这与数学教科书上的 `slerp(q_a, q_b, t)` 顺序不同。写反了会编译报错（类型不匹配），不会静默出错，但需要留意。

Eigen 的 slerp 内部处理 `q` 与 `-q` 的符号问题，保证走最短路径。

---

## 六、实现步骤（数学形式）

```text
输入：位姿 a、b；系数 alpha
输出：位姿 c

步骤 1：clamp 系数
    t ← clamp(alpha, 0, 1)

步骤 2：平移线性插值
    p_c ← (1 - t) · p_a + t · p_b
    其中 p_a = a.translation()，p_b = b.translation()

步骤 3：旋转四元数插值
    q_a ← 从 a.linear() 构造四元数
    q_b ← 从 b.linear() 构造四元数
    q_c ← slerp(q_a, q_b, t)

步骤 4：组装位姿
    R_c ← q_c 转旋转矩阵
    c.linear() ← R_c
    c.translation() ← p_c
```

**用到哪些 Eigen API**：

| 操作 | API |
|---|---|
| 取平移 | `a.translation()` |
| 取旋转矩阵 | `a.linear()` |
| 从旋转矩阵构造四元数 | `Eigen::Quaterniond q(a.linear())` |
| 四元数 slerp | `q_a.slerp(t, q_b)` |
| 四元数转旋转矩阵 | `q_c.toRotationMatrix()` |
| 组装位姿 | 先 `Eigen::Isometry3d::Identity()`，再分别赋值 `.linear()` 与 `.translation()` |
| clamp | `std::clamp(alpha, 0.0, 1.0)` |

---

## 七、边界处理：alpha 必须 clamp

`deskew` 中 `alpha` 的计算方式是：

```text
alpha = (time - t_i) / (t_{i+1} - t_i)
```

浮点误差可能导致 `alpha` 略小于 0 或略大于 1。

**不 clamp 的后果**：

- 平移外推：误差小，影响不大；
- 旋转外推：slerp 沿大圆弧继续走，可能转出大角度，产生飞点。

所以必须 clamp 到 `[0, 1]`。

---

## 八、常见错误对照

| 错误 | 现象 | 单测表现 |
|---|---|---|
| 对 `a.matrix()` 与 `b.matrix()` 逐元素插值 | 结果非旋转矩阵，点云扭曲、坍缩 | `InterpolateMidpoint` 失败 |
| 用欧拉角分解后插值角度 | 小角度看似正常，绕圈时跳变 | `InterpolateMidpoint` 可能通过，系统里快速段出错 |
| 不 clamp alpha | 帧边缘出现飞点 | `InterpolateClampsAlpha` 失败 |
| 平移与旋转用不同的 alpha（比如旋转用 `1-t`） | 位姿不自洽，点云撕裂 | `InterpolateClampsAlpha` 失败；对称的 `alpha=0.5` 中点用例可能通过 |
| slerp 参数顺序写反 | 编译报错（类型不匹配） | 编译失败 |
| 四元数未归一化 | 非单位四元数不能表示有效旋转，`toRotationMatrix()` 结果无定义 | 正常旋转矩阵构造通常足够精确；显式 `.normalized()` 更稳妥 |
| 忘记设置 `c.translation()` | 输出恒在原点 | `InterpolateMidpoint` 失败 |

---

## 九、验证方法

### 9.1 单测

```text
PoseUtils.InterpolateMidpoint        中点插值
PoseUtils.InterpolateClampsAlpha     alpha 外推被 clamp
```

### 9.2 手算验证 `InterpolateMidpoint`

输入：`a` 为单位位姿，`b` 为「平移 (2,0,0)，绕 Z 转 90°」，`alpha = 0.5`。

```text
步骤 2：p_c = 0.5 · (0,0,0) + 0.5 · (2,0,0) = (1, 0, 0)

步骤 3：
    q_a = (0, 0, 0, 1)                    （单位旋转，w=1）
    q_b = (0, 0, sin45°, cos45°)          （绕 Z 转 90°）
        = (0, 0, √2/2, √2/2)
    q_c = slerp(q_a, q_b, 0.5)
        = (0, 0, sin22.5°, cos22.5°)      （绕 Z 转 45°）

期望输出：
    c.translation() = (1, 0, 0)
    c.linear() 对应绕 Z 转 45°
```

### 9.3 间接验证

```text
Deskew.RotationIsCompensated
```

`deskew` 内部调用 `interpolatePose`。如果插值实现用了逐元素矩阵插值，这个测试会失败；欧拉角在单轴用例上可能碰巧通过，但快速多轴旋转仍会出错。它是 `interpolatePose` 的间接验证，不能覆盖所有错误实现。
