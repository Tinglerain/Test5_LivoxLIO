# Part of Deskew

LiDAR 运动畸变补偿：用 IMU 位姿将每个点从采样时刻的 LiDAR 系变换到帧起始时刻的 LiDAR 系。

**本模块已提供完整实现，无需修改。**

源码位置：`preprocess/cloud_deskewing.cpp` 中的 `CloudDeskewing::deskew`（IMU 位姿版本）。

本模块在新人任务中的作用有两个：

1. **集成环境的一部分**：它由 `odometry_estimation.cpp` 在除首帧初始化外的每个点云帧调用，把预处理后的点云变成可供 GICP 配准的去畸变点云。它已提供，系统因此能在你完成 5 个 TODO 之后完整运行。
2. **TODO 1 的集成调用方**：它内部调用 `interpolatePose`。你的 `interpolatePose` 写错时，`deskew` 的输出也会错。`Deskew.RotationIsCompensated` 在中间时刻验证该插值；另外两项主要验证 deskew 的恒等与非法时间行为。

下面详细说明它的实现。理解本文档不要求你写 deskew，但要求你能读懂它的坐标链、时间表和边界处理——它们也是理解 TODO 1 的关键上下文。

---

## 一、运动畸变

LiDAR 非瞬时采集，而是边转边扫，一帧 0.1 秒内各点采样时刻不同。

机器人静止时无影响，所有点在同一姿态下采集。机器人运动时：

```text
假设机器人在 0.1 秒内旋转了 30 度：

帧起始时刻: 机器人朝北
帧结束时刻: 机器人朝东北

点0 在朝北时采集
点10000 在朝东北时采集

若不做补偿，将整帧约两万个原始点视为同一时刻采集，相当于将不同朝向的扫描强行拼接。
结果：墙变为两堵、角变为弧——即运动畸变。
```

去畸变即逆转此过程：对每个点计算其采样时刻的机器人姿态，将点变换至帧起始时刻的姿态下，使整帧点云等效于同一瞬间采集。

---

## 二、坐标链：点的变换路径

目标：将点从"采样时刻的 LiDAR 系"变换到"帧起始时刻的 LiDAR 系"。

中间经世界系中转。完整链路：

```text
采样时刻 LiDAR 系 (p_lidar1)
    |
    | T_imu_lidar       变换至采样时刻 IMU 系
    v
采样时刻 IMU 系
    |
    | T_world_imu1      变换至世界系
    v
世界系
    |
    | T_imu0_world      变换至帧起始 IMU 系
    v
帧起始 IMU 系
    |
    | T_lidar_imu       变换至帧起始 LiDAR 系
    v
帧起始 LiDAR 系 (p_lidar0)
```

合成：

```text
p_lidar0 = T_lidar_imu * T_imu0_world * T_world_imu1 * T_imu_lidar * p_lidar1
```

定义：

```text
T_lidar0_lidar1 = T_lidar_imu * T_imu0_world * T_world_imu1 * T_imu_lidar
```

其中 `T_lidar_imu = T_imu_lidar.inverse()`；`T_imu0_world` 是帧起始时刻 IMU 位姿的逆，具体用哪个 IMU 样本见第五节。

---

### 为什么不能跳过外参

若 LiDAR 与 IMU 完全重合（外参为单位矩阵），可跳过。但实际中二者不重合——据 `params_sensors.yaml`，`T_lidar_imu`（IMU 系到 LiDAR 系）的平移约为 (1.1 cm, 2.3 cm, -4.4 cm)，旋转分量为单位四元数；`T_imu_lidar` 是它的逆。

遗漏 `T_imu_lidar` 或 `T_lidar_imu` 将使整帧偏移一个外参。低速时不明显，高速旋转时地图与轨迹无法对齐。

---

### 乘法顺序的重要性

变换从右往左作用于点。顺序写反一项，点即被送至错误位置。这不是近似误差，而是完全错误。

---

## 三、T_world_imu1 的获取

每个点的时间为 `time = stamp + t`（`stamp` 为帧起始时间，`t` 为相对时间）。

IMU 位姿非每点时刻均有，而是按 IMU 采样率（200 Hz）离散给出。获取步骤：

1. 在 `imu_times` 中找到包含 `time` 的两个相邻样本；恰好落在采样时刻时直接使用该样本
2. 计算插值系数 `alpha = (time - imu_times[cursor]) / (imu_times[cursor+1] - imu_times[cursor])`
3. 用 `interpolatePose` 插值得到 `T_world_imu1`（**这就是 TODO 1 被调用的位置**）
4. 若 `time` 超出最后一个 IMU 样本，直接使用最后一个位姿

游标 `cursor` 无需从头查找——IMU 时间升序，点时间亦升序，维护游标向前推进即可，复杂度 O(n) 而非 O(n²)。

第 3 步是 `deskew` 与 TODO 1 的唯一连接点。`interpolatePose` 写错时，`T_world_imu1` 就会错，整帧点云被送至错误位姿。

---

## 四、时间表：避免逐点插值

原始一帧约两万个点，预处理降采样后通常约一万五千个点；相邻点时间通常相差数十微秒。逐点插值计算量大，且结果几乎相同。

做法是维护时间表：

```text
time_table      间隔超过 time_eps (1e-4 秒) 的时刻
time_indices    每个点对应的 time_table 下标
```

仅当前点与时间表最后时刻相差超过 1e-4 秒时才新建一项。万余个点通常只对应数百个时刻，每个时刻仅计算一次 `T_lidar0_lidar1`。

最后逐点查表：

```text
deskewed[i] = T_lidar0_lidar1[time_indices[i]] * points[i]
```

---

## 五、T_imu0_world 的计算

帧起始时刻对应的 IMU 位姿由首个时间表时刻推进后的 `imu_cursor` 决定，通常是 `imu_poses[0]`，但若首点时间已经晚于若干 IMU 样本，则是 `imu_poses[imu_cursor]`，不能假定游标一定为 0。

注意：`imu_poses[i]` 存储的是 `T_world_imu`（IMU 系到世界系的变换），而需要的是 `T_imu0_world`（世界系到帧起始 IMU 系的变换）。因此必须取逆：

```text
T_imu0_world = imu_poses[imu_cursor].inverse()
```

直接使用 `imu_poses[imu_cursor]` 而不取逆是常见错误，方向相反，点会被送到错误位置。

---

## 六、边界处理

### times 校验

```text
times 为空               -> 返回空
times.front/back 不有限  -> 返回原 points
times.back < times.front -> 返回原 points（倒序）
times.back - times.front > 1.0 秒 -> 返回原 points（跨度过大，数据异常）
```

跨度判断使用 `> 1.0`，不要使用 `>= 1.0`。恰好 1 秒的帧合法。

返回原 points 而非空——数据异常时不做补偿，至少保证点不丢失。

### imu_poses 为空

无 IMU 位姿时无法执行基于位姿的去畸变。此时使用第一个重载（常速度假设版本）兜底，线速度与角速度均传 0，相当于不做补偿。

### 时间超出 IMU 范围

最后一个点的时间可能比最后一个 IMU 样本晚数毫秒（IMU 与 LiDAR 时间戳未完全对齐）。未处理将导致越界。超出时直接使用最后一个 IMU 位姿。

---

## 七、实现错误的影响

**本模块已提供，正常不会被改错。** 下表列出的是「若把已提供的实现改坏」会出现的现象，以及「`interpolatePose` 写错时如何传导到本模块」：

| 错误来源 | 后果 |
|---|---|
| `interpolatePose`（TODO 1）写错 | `T_world_imu1` 错误，整帧点云在快速旋转时扭曲、分层；平移插值错误时点云重影。当前 `deskew` 调用前已 clamp alpha，直接调用插值函数时仍需遵守 clamp 契约 |
| 若 deskew 本身被改动，遗漏 `T_imu_lidar` / `T_lidar_imu` | 整帧偏移一个外参，地图与轨迹无法对齐 |
| 若 deskew 本身被改动，`T_imu0_world` 未取逆 | 方向相反，点被送到错误位置 |
| 若 deskew 本身被改动，乘法顺序错误 | 点被送至错误位置，地图模糊 |
| 若 deskew 本身被改动，时间超出 IMU 范围未兜底 | 越界访问，崩溃 |

新人只需保证 TODO 1 写对，deskew 的其余逻辑由已提供实现负责。

---

## 八、deskew 内部实现步骤（供阅读）

首帧由初始化流程直接变换到 IMU 系，不调用 `deskew`；从第二帧起进入以下流程。以下是已提供实现的逻辑顺序，供阅读理解，不要求实现：

1. 边界校验（空、倒序、跨度）
2. imu_poses 为空的处理
3. 建立时间表与索引
4. 对时间表中每个时刻：推进游标、**调用 `interpolatePose`**、计算 `T_lidar0_lidar1`
5. 逐点查表变换
6. 返回去畸变后的点云

第 4 步是理解本文档与 TODO 1 关系的关键：`deskew` 本身不做插值，它把逐时刻的位姿插值交给 TODO 1。

---

## 九、测试

```text
Deskew.IdentityPosesKeepPoints      位姿全为单位矩阵时点不变
Deskew.RotationIsCompensated        有旋转时点被正确补偿
Deskew.InvalidTimesReturnInput      时间倒序 -> 返回原 points
```

三项测试的定位：

- 第一项为基础测试：IMU 未运动（位姿全为 I）时，去畸变不应改变任何点。该用例实际只检查两个点的 x 坐标，覆盖范围有限。若 `interpolatePose` 写错（例如始终返回单位矩阵），`IdentityPosesKeepPoints` 仍会通过，因为期望恰好是单位矩阵，所以这项不构成对 TODO 1 的严格检验；
- 第二项为核心测试：三个点分别位于帧起始、IMU 姿态中点和帧结束时刻；中间点必须使用 45 度插值姿态补偿。**`interpolatePose` 若用旋转矩阵逐元素插值而非 slerp，此测试会失败。**
- 第三项为边界测试：时间非法时不做补偿，原样返回。与 `interpolatePose` 无关。

因此，`test_deskew.cpp` 的 `Deskew.RotationIsCompensated` 通过中间时刻作为 `interpolatePose` 的间接验证。这是已提供 deskew 在新人任务中的第二个作用。
