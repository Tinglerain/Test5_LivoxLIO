# Part of Overview

本篇建立整体印象，公式与细节在后续文档中展开。

阅读后应能回答以下三个问题：

```text
1. LIO 解决什么问题？
2. 一帧数据进来，经过哪些模块？
3. 哪些已提供，哪些待实现？
```

若无法回答，不要继续阅读后续文档。

---

## 一、LIO 解决什么问题

机器人运动时需知道自身位置与周围环境。仅使用激光雷达（无 IMU）时存在以下问题：

- 激光 10 Hz，两帧之间机器人已移动一段距离；
- 快速旋转时，一帧内点云被"抹开"；
- 相邻帧运动过大时，配准容易失败；
- 某一帧配准错误后，误差持续累积。

IMU 以 200 Hz 采样，远高于激光频率，填补了帧间空档。

LIO 的核心思路：

```text
IMU 负责高频预测和帧内运动补偿
LiDAR 负责用环境几何将位姿"拉回"
```

---

## 二、为什么不能仅做逐帧 ICP

最朴素的做法：以上一帧点云为目标，当前帧配准，累加相对位姿。低速平稳运动下可行，但本录包运动剧烈：

```text
陀螺均值约 4.6 rad/s，峰值接近 34 rad/s
加速度峰值约 6.9 g
```

仅靠"预测 + 一次 ICP"存在两个问题：

1. 预测不准时，ICP 初值偏差过大，直接配错；
2. 某一帧出错后无纠正机制，误差一路累积。

本系统通过三项机制解决：

```text
IMU 预积分      高频状态传播
因子图滑窗      最近几秒的位姿联合优化
GICP + iVox     用局部地图实现更稳定的配准
```

---

## 三、数据流

```text
IMU:   /livox/imu -> 时间戳校验/偏移 -> IMU 预积分 ---------------┐
                                                                  │
LiDAR: /livox/lidar/pointcloud -> TimeKeeper -> CloudPreprocessor │
                                                                  v
                                      ┌──────────────────────────────┐
                                      │ OdometryEstimation           │
                                      │ 去畸变 + IMU/GICP 因子 + 滑窗 │
                                      └──────────────┬───────────────┘
                                                     │
                          ┌──────────────────────────┼─────────────────────┐
                          v                          v                     v
                  /small_glim/odometry      /small_glim/path       AsyncMapping
                  /small_glim/ivox_cloud                           │
                  /small_glim/registered_cloud                     v
                                                          map/mapping_*/
```

总结：IMU 与点云分别进入各自处理链；点云经时间统一、预处理，在里程计中完成去畸变、配准与滑窗优化，随后发布位姿/点云并进入关键帧建图。

---

## 四、各模块功能

### TimeKeeper

TimeKeeper 处理 LiDAR 点云：把每点时间戳统一为"帧时间 + 每点相对帧时间"，并处理相对/绝对时间与单位差异；若点云无逐点时间，则根据历史帧间隔按点序分配伪时间。IMU 不经过 TimeKeeper，只在 IMU 回调中校验时间戳并按 `imu_time_offset` 调整后送入预积分。

### CloudPreprocessor

执行四项操作：降采样（0.05 m 体素；本录包多数帧从约 2 万点降至约 1.5 万点）、距离滤波（移除过近与过远的点）、按时间排序（去畸变要求点按时间有序）、可选离群点/cropbox。

### CloudDeskewing

用 IMU 位姿插值，将每个点从采样时刻变换到帧起始时刻。这是快速旋转时避免地图模糊的关键。**本模块已提供完整实现**，详见 `Part_of_Deskew.md`。

### OdometryEstimation

核心模块：IMU 预积分因子、GICP 配准因子、先验因子、固定滞后滑窗。同时估计位姿、速度、IMU 零偏。

### AsyncMapping

仅处理关键帧（新位姿相对上一关键帧的平移或旋转超过阈值时插入）。插入时将点云变换到世界系，按距离范围滤波后追加到全局点云，并更新关键帧位姿记录；每累计若干关键帧做一次体素下采样。`poses.txt` 随关键帧写入，`mapping.pcd` 只在节点正常退出且地图非空时保存。

---

## 五、已提供的部分

LIO 框架、数据流与周边模块已提供；下节列出的 5 个原理函数仍为占位实现：

```text
RawPoints 字段解析
TimeKeeper 主流程
CloudPreprocessor 主流程
CloudDeskewing 完整实现（已提供，供阅读）
IMU 预积分
初始状态估计主流程（重力/航向对齐待实现）
因子图滑窗优化
GICP / iVox
AsyncMapping 主流程
ROS 节点、TF、launch、参数
```

无需从零实现 SLAM。

---

## 六、待实现的 5 个 TODO

仅涉及与 LIO 原理直接相关的函数：

```text
common/pose_utils.cpp
    interpolatePose           位姿插值

preprocess/cloud_ops.cpp
    computeCovariance         局部协方差
    regularizePlaneCovariance 平面协方差正则化

odometry/initial_attitude.cpp
    gravityAlignment          重力对齐
    yawAlignment              航向对齐
```

已提供、不作为 TODO 的函数：

```text
preprocess/cloud_deskewing.cpp
    CloudDeskewing::deskew    完整运动补偿（已提供）
```

滤波、kNN、体素下采样、关键帧判断、距离裁剪已提供完整实现。这些属于流程代码而非本 Test 考核的原理部分。

5 个待实现的函数各自对应一个原理，题目中的模块说明与验收要求见 `QUESTION.md` 第三节至第五节。`cloud_deskewing.cpp` 中的 `deskew` 依赖 `interpolatePose`，若 `interpolatePose` 写错，去畸变模块的输出也会错；反之 `Deskew.RotationIsCompensated` 可作为 `interpolatePose` 的间接验证；另外两项 deskew 测试也会走各自路径，但不构成对 TODO 1 的严格检验。

---

## 七、本录包的难度

录包运动剧烈：陀螺角速度均值 4.65 rad/s，p99 达 26.7 rad/s，加速度最大 6.9 g。仅做"恒速预测 + 点对点 ICP"将导致轨迹持续漂移。

本系统通过以下机制保持稳定：IMU 高频预积分、加速度计提供重力与竖直运动信息、零偏在线估计、因子图联合优化历史位姿、GICP 比点对点更稳定、关键帧地图避免无限增长。

待实现的 5 个函数是上述链路中的原理性组件，仅当全部正确实现时，系统才能正常工作。

---

## 八、后续阅读

| 目标 | 文档 |
|---|---|
| 录包、字段、坐标系、参数 | `Part_of_Data.md` |
| 5 个函数的实现方式 | 三篇模块文档（见 `docs/README.md`） |
| 去畸变模块的详细解读 | `Part_of_Deskew.md`（已提供） |
| 运行与查看结果 | `Part_of_Run.md` |

建议先阅读 `Part_of_Data.md`，不要直接开始编写代码。
