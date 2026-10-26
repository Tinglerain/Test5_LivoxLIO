# Part of Initial Attitude

LIO 初始化时使用重力确定 roll/pitch，再消除剩余的 yaw 自由度。

本节涉及的 IMU 比力与重力基础见 `Part_of_Basics.md` 第二节。

对应实现：`odometry/initial_attitude.cpp` 中的 `gravityAlignment` 与 `yawAlignment`。

---

## 一、重力对齐（TODO 4）

### 1.1 这个函数要做什么

**输入**：

```text
gravity_direction   3 维向量（加速度计累积方向，未归一化）
```

**输出**：

```text
R                   3×3 旋转矩阵，满足 R · (v / |v|) = (0, 0, 1)
```

**在系统中的位置**：默认 `naive_initialization: true` 时，`InitialStateEstimation::initial_pose` 里调用：

```text
init_R ← gravityAlignment(sum_acc)
```

其中 `sum_acc` 是 LIO 启动后初始化窗口（默认约 1 秒）内累积的加速度计归一化方向之和。它代表「IMU 认为的天顶方向」。图优化初始化路径会从 LiDAR 位姿构造 `sum_acc_odom`，但默认路径不使用它。

### 1.2 前置知识：加速度计测的是什么

**加速度计测量的不是物体的加速度，而是比力**：

```text
f = a - g
```

- `a`：物体相对惯性系的加速度
- `g`：重力加速度向量（指向地心）
- `f`：加速度计读数

**静止时** `a = 0`，所以 `f = -g`。`g` 指向地心，`-g` 指向天顶。所以静止时读数方向指向**天顶**，大小约 9.8 m/s²。

本录包的原始 `linear_acceleration` 以 g 为单位，节点先乘 `acc_scale` 再传入初始化；本文按换算后的 m/s² 说明。

**为什么用加速度计做重力对齐**：静止时加速度计测到的方向就是天顶。知道天顶方向就确定了 roll/pitch（因为 yaw 是绕天顶转，不影响天顶方向）。

**理想情况**：IMU 完全水平时读数 `(0, 0, +9.8)`，方向就是 +Z。若 IMU 倾斜，读数偏离 +Z，需要一个旋转把它转回 +Z。

### 1.3 数学推导

**问题**：给定单位向量 `v̂`（重力方向归一化），求 3×3 旋转矩阵 `R`，使得：

```text
R · v̂ = (0, 0, 1)
```

**解法**：求「把 `v̂` 旋转到 `(0, 0, 1)` 的最短弧旋转」。

最短弧旋转的几何含义：

- `v̂` 与 `(0, 0, 1)` 同向 → 单位旋转（无旋转）
- `v̂` 与 `(0, 0, 1)` 反向 → 绕垂直于两者平面的某轴转 180°
- 其他情况 → 绕「`v̂` 与 `(0, 0, 1)` 的叉积方向」转它们之间的夹角

**Eigen 提供现成函数**：

```text
Eigen::Quaterniond::FromTwoVectors(from, to)
```

返回「把 `from` 旋转到 `to` 的最短弧旋转」的四元数。**参数顺序是 `(起点, 终点)`**，写反了得到的是相反的旋转。

**手算例子**：`v̂ = (1, 0, 0)`。

```text
从 (1, 0, 0) 到 (0, 0, 1) 的最短弧旋转：

旋转轴 ← (1, 0, 0) × (0, 0, 1) = (0, -1, 0)   （即 -Y）
旋转角 ← 两个向量的夹角 = 90°

R = 绕 -Y 转 90° = 绕 Y 转 -90°
  = [[cos(-90°), 0, sin(-90°)],    [[0, 0, -1],
     [0,        1, 0        ],  =   [0, 1, 0 ],
     [-sin(-90°),0, cos(-90°)]]     [1, 0, 0 ]]

验证：R · (1, 0, 0) = (0, 0, 1)   ✓
```

### 1.4 实现步骤（数学形式）

```text
输入：向量 v（3 维，未归一化）
输出：3×3 旋转矩阵 R

步骤 1：零向量判断
    若 |v| < 1e-6：
        R ← 3×3 单位矩阵
        返回

步骤 2：归一化
    v̂ ← v / |v|

步骤 3：求最短弧旋转
    q ← 「把 v̂ 转到 (0, 0, 1) 的最短弧旋转」对应的四元数

步骤 4：转旋转矩阵
    R ← q 转 3×3 矩阵

返回 R
```

**用到哪些 Eigen API**：

| 操作 | API |
|---|---|
| 向量模长 | `v.norm()` |
| 归一化 | `v.normalized()` |
| +Z 单位向量 | `Eigen::Vector3d::UnitZ()` |
| 最短弧旋转 | `Eigen::Quaterniond::FromTwoVectors(v_normalized, unit_z)` |
| 四元数转矩阵 | `q.toRotationMatrix()` |

### 1.5 零向量处理

**为什么需要**：`v / |v|` 对零向量会产生 NaN，`FromTwoVectors` 收到 NaN 输入后可能得到 NaN 或无效四元数。

**什么情况下会遇到**：加速度计故障、初始化窗口内累积数据接近 0（概率低）、输入传错。

**返回单位矩阵的理由**：零向量时无法确定天顶方向，保守选择「不旋转」。

### 1.6 常见错误对照

| 错误 | 现象 | 单测表现 |
|---|---|---|
| `FromTwoVectors(UnitZ(), v̂)` 参数反了 | 旋转方向相反，初始姿态错误（测试输入为 `+X` 时被映射到 `-Z`） | `GravityMapsMeasuredDirectionToZ` 失败 |
| 零向量不判断 | 归一化产生 NaN，整条轨迹 NaN | `GravityZeroReturnsIdentity` 失败 |
| 返回 4×4 矩阵 | 与调用方维度不匹配 | 编译失败 |
| 不归一化输入 | Eigen 的 `FromTwoVectors` 内部会归一化，但实现仍应显式归一化，不要依赖库内部行为 | 通常通过 |
| 手写旋转而不是用 `FromTwoVectors` | 逻辑复杂易错 | 视实现而定 |
| 忘记写 `return` 语句 | 编译错误或返回未定义值 | 编译失败或单测失败 |

### 1.7 验证方法

**单测**：

```text
InitialAttitude.GravityIdentityWhenAlreadyUp       输入 (0,0,1)，期望单位矩阵
InitialAttitude.GravityMapsMeasuredDirectionToZ     输入 (1,0,0)，验证 R·v̂ = (0,0,1) 且 det(R)=1
InitialAttitude.GravityZeroReturnsIdentity          输入零向量，期望单位矩阵
```

**手算步骤**：用 1.3 节的 `v = (1,0,0)`，验证 `R · (1,0,0) = (0,0,1)`。

---

## 二、航向对齐（TODO 5）

### 2.1 这个函数要做什么

**输入**：

```text
x_axis_world   3 维向量（IMU 的 x 轴在世界坐标系下的方向）
```

**输出**：

```text
R              3×3 旋转矩阵，把 x_axis_world 的水平投影转到 +X 轴
```

**在系统中的位置**：默认 `align_initial_odom_to_imu: true` 时，紧跟在 `gravityAlignment` 之后调用：

```text
R_yaw ← yawAlignment(R_gravity · (1, 0, 0))
init_R ← R_yaw · R_gravity
```

先重力对齐确定 roll/pitch，再用 `yawAlignment` 消除 yaw 自由度。

### 2.2 前置知识：yaw 为什么自由

重力方向提供 2 个自由度（roll/pitch），绕重力方向的旋转（yaw）不影响重力方向——无论车朝北朝东，重力都朝下。

所以仅凭加速度计无法确定 yaw。**需要一个额外约定**。

**工程约定**：让 IMU 的 x 轴在水平面上的投影对齐世界 +X。

- IMU 的 x 轴方向在 IMU 自己坐标系中是 `(1, 0, 0)`；
- 在世界系中指向某个方向 `x_axis_world`；
- 这个方向投影到水平面（去掉 z 分量），我们希望它指向 +X。

### 2.3 数学推导

**步骤**：

1. 投影到水平面：

   ```text
   heading = (x_axis_world.x, x_axis_world.y, 0)
   ```

2. 计算 heading 相对 +X 的角度：

   ```text
   θ = atan2(heading.y, heading.x)
   ```

3. 旋转 -θ（绕 Z 轴），把 heading 转到 +X。

**为什么用 `atan2(y, x)` 而不是 `atan(y/x)`**：

- `atan(y/x)` 只返回 `(-π/2, π/2)`，无法区分第一象限和第三象限（`(1,1)` 和 `(-1,-1)` 的 `atan(y/x)` 都是 1）。
- `atan2(y, x)` 根据 `y`、`x` 的符号判断象限，返回完整角度 `[-π, π]`。
- `atan(0/0)` 未定义，`atan2(0, 0)` 返回 0（虽然本场景不会遇到 (0,0)，因为我们已经提前判零向量）。

**为什么旋转 `-θ`**：

- `θ` 是 heading 相对 +X 的逆时针角度；
- 要把 heading 转到 +X，需要顺时针转 `θ`，即逆时针转 `-θ`。

**手算例子**：`x_axis_world = (0, 1, 0)`。

```text
步骤 1：heading = (0, 1, 0)
步骤 2：θ = atan2(1, 0) = π/2 = 90°
步骤 3：绕 Z 转 -90°

R = 绕 Z 转 -90°
  = [[cos(-90°), -sin(-90°), 0],   [[0, 1, 0],
     [sin(-90°),  cos(-90°), 0], =  [-1, 0, 0],
     [0,          0,         1]]    [0, 0, 1]]

验证：R · (0, 1, 0) = (1, 0, 0)   ✓
```

### 2.4 实现步骤（数学形式）

```text
输入：向量 x_world（3 维）
输出：3×3 旋转矩阵 R

步骤 1：投影到水平面
    heading ← (x_world.x, x_world.y, 0)

步骤 2：零向量判断
    若 |heading| < 1e-6：
        R ← 3×3 单位矩阵
        返回

步骤 3：计算角度
    θ ← atan2(heading.y, heading.x)

步骤 4：绕 Z 轴旋转 -θ
    R ← 绕 Z 轴转 -θ 的旋转矩阵
```

**用到哪些 Eigen API**：

| 操作 | API |
|---|---|
| 取向量分量 | `v.x()`, `v.y()` |
| 模长 | `heading.norm()` |
| 反正切 | `std::atan2(y, x)` |
| 绕轴旋转 | `Eigen::AngleAxisd(angle, axis)` |
| Z 单位向量 | `Eigen::Vector3d::UnitZ()` |
| 转矩阵 | `angle_axis.toRotationMatrix()` |

`AngleAxisd` 的参数顺序是 `(角度, 轴)`，不是 `(轴, 角度)`。

### 2.5 零向量处理

**什么情况下会遇到**：`x_axis_world` 接近 `(0, 0, ±z)`，即 IMU 的 x 轴竖直（朝上或朝下）。这时无法从 x 轴的水平投影判断 yaw。

**为什么必须判零**：水平投影为零时，输入没有可用的航向信息。虽然常见数学库会为 `atan2(0, 0)` 返回数值结果，但该数值不能代表可观测的航向，因此按契约返回单位矩阵。

**返回单位矩阵的理由**：无法确定 yaw 时，保守选择「不旋转」。

### 2.6 常见错误对照

| 错误 | 现象 | 单测表现 |
|---|---|---|
| `AngleAxisd(θ, ...)` 而非 `AngleAxisd(-θ, ...)` | 旋转方向反了，误差取决于初始航向，地图整体旋转 | `YawMapsHeadingToX` 失败 |
| 不显式构造 `(x, y, 0)` 水平投影 | `atan2(y, x)` 本身不使用 z，当前 `YawIgnoresZComponent` 可能通过；但竖直输入可能被误判为有效航向 | `YawIgnoresZComponent` 可能通过，其他用例取决于零判断 |
| 零向量不判断 | 无有效水平航向仍被当作有效角度处理；若先归一化会产生 NaN | `YawZeroReturnsIdentity` 可能失败 |
| 用 `atan(y/x)` 而非 `atan2(y, x)` | 只覆盖两个象限，x≤0 等角度会算错 | 当前 `YawMapsHeadingToX` 可能通过，其他象限的系统行为错误 |
| `atan2` 参数顺序写反（`atan2(x, y)`） | 角度偏差 90° | `YawMapsHeadingToX` 失败 |
| `AngleAxisd` 参数顺序写反 | 编译失败（类型不匹配） | 编译失败 |
| 忘记归一化 heading | `atan2` 仍可用（只看比例），但不推荐 | 通常通过 |

### 2.7 验证方法

**单测**：

```text
InitialAttitude.YawMapsHeadingToX      输入 (0,1,0)，验证 R·(0,1,0) = (1,0,0)
InitialAttitude.YawIgnoresZComponent   输入 (0,1,9.0)，与 (0,1,0) 结果相同
InitialAttitude.YawZeroReturnsIdentity 输入零向量，期望单位矩阵
```

**手算步骤**：用 2.3 节的 `x_world = (0, 1, 0)`，验证 `R · (0, 1, 0) = (1, 0, 0)`。

---

## 三、两个函数在初始化流程中的位置

```text
LIO 启动
  → 收集约 1 秒的 IMU 数据
  → 累加归一化的加速度方向 → sum_acc
  → gravityAlignment(sum_acc) → 确定 roll/pitch
  → 计算 IMU 的 x 轴在世界系的水平投影
  → yawAlignment(imu_x_world) → 确定 yaw
  → 组合成 init_T_world_imu
  → 作为第一帧初始位姿
  → smoother 从该位姿开始优化
```

以上是默认 `naive_initialization: true` 的路径。若关闭该选项，初始化会走图优化路径，并使用由 LiDAR 位姿累积的 `sum_acc_odom`，但重力/航向函数的契约相同。

**TODO 4 写错的系统现象**：初始 roll/pitch 错，整张地图一开始就倾斜。由于 `init_pose_damping_scale` 把初始位姿「钉」住，后续帧很难把倾斜拉回来。
