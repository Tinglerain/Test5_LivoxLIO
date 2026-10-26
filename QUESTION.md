# Test5 LivoxLIO 题目

完整阅读路径见 `docs/README.md`。首次阅读建议从 `docs/Part_of_Basics.md`（前置概念）与 `docs/Part_of_Env.md`（环境搭建）开始。

---

## 一、任务目标

基于给定的 Livox Mid-360 录包（点云 + IMU），运行完整的 LiDAR-IMU 里程计与建图，并实现其中 5 个与 LIO 原理直接相关的模块。

点云去畸变模块 `CloudDeskewing::deskew` 已提供完整实现，不属于本题待实现内容，无需修改。它调用待实现的 `interpolatePose`；理解该模块的坐标链与时间表有助于完成位姿插值，建议通读 `docs/Part_of_Deskew.md`。

系统数据流与状态估计关系（IMU 预积分与点云处理并行提供状态/观测，配准结果进入滑窗优化）：

```text
IMU 高频数据 -> 预积分/位姿序列 ──────────────┐
                                                v
点云 -> 预处理 -> 去畸变（已提供） -> GICP-to-iVox 配准
                                                |
                                                v
                                  IMU 因子 + 配准因子 -> 滑窗优化
                                                |
                                      里程计 -> 关键帧建图/地图
```

滤波、kNN、体素下采样、关键帧判断、距离裁剪等流程代码已提供，无需实现。

## 二、输入

```text
/livox/lidar/pointcloud   sensor_msgs/msg/PointCloud2    10 Hz
/livox/imu                sensor_msgs/msg/Imu            约 200 Hz
```

录包位置：`bag/lunatic_0.mcap`

若工作区未附带录包，从 [Google Drive 下载 mid360_bag](https://drive.google.com/drive/folders/1FJgEmFS2wcuFFTUjiKikhG14Zo43saoj)。rosbag2 目录可直接传给 `./replay.sh`；`.mcap` 文件可放到 `bag/lunatic_0.mcap` 或通过脚本参数指定。使用前以 `ros2 bag info` 确认包含上述两个输入话题。

## 三、需要完成的内容

仅修改以下 3 个文件：

```text
src/small_glim/src/common/pose_utils.cpp
src/small_glim/src/preprocess/cloud_ops.cpp
src/small_glim/src/odometry/initial_attitude.cpp
```

以下文件已提供完整实现，不要修改：

```text
src/small_glim/src/preprocess/cloud_deskewing.cpp
```

需实现 5 个函数：

| 推荐顺序 | 源码 TODO | 文件 | 函数 | 原理 |
|---:|---|---|---|---|
| 1 | TODO 1 | `src/small_glim/src/common/pose_utils.cpp` | `interpolatePose` | 平移线性插值，旋转四元数 slerp |
| 2 | TODO 2 | `src/small_glim/src/preprocess/cloud_ops.cpp` | `computeCovariance` | 由 k 邻居计算均值与 4x4 协方差 |
| 3 | TODO 3 | `src/small_glim/src/preprocess/cloud_ops.cpp` | `regularizePlaneCovariance` | 特征分解后将平面特征值固定为 `(1e-3, 1, 1)` |
| 4 | TODO 4 | `src/small_glim/src/odometry/initial_attitude.cpp` | `gravityAlignment` | 求旋转，将重力方向对齐到 `+Z` |
| 5 | TODO 5 | `src/small_glim/src/odometry/initial_attitude.cpp` | `yawAlignment` | 绕 Z 旋转，将水平航向对齐到 `+X` |

详细原理、公式与易错点见：

```text
docs/Part_of_PoseInterpolation.md   TODO 1
docs/Part_of_Deskew.md              已提供模块（详读）
docs/Part_of_Covariance.md          TODO 2、3
docs/Part_of_Initial_Attitude.md    TODO 4、5
```

### 函数实现错误时的现象

| 推荐顺序 | 函数 | 错误现象 |
|---:|---|---|
| 1 | `interpolatePose` | 位姿插值错误会导致已提供的去畸变输出错误，地图模糊、重影或点云扭曲 |
| 2 | `computeCovariance` | GICP 平面/线判断错误，配准权重异常，轨迹抖动 |
| 3 | `regularizePlaneCovariance` | 法向不稳定，GICP 抖动，地图细节模糊 |
| 4 | `gravityAlignment` | 初始 roll/pitch 错误，轨迹整体倾斜 |

## 四、硬约束

1. **不要修改函数签名和头文件契约。**
2. **不要修改 IMU 预积分、因子图、GICP、iVox、节点骨架。**
3. **不要修改已提供的 `cloud_deskewing.cpp`。**
4. **算法须为纯函数**，不能依赖 ROS 时间或全局状态。
5. **边界须按契约处理**：
   - `interpolatePose` 的 alpha 须 clamp 到 `[0,1]`；
   - `computeCovariance` 除以 `k`，不是 `k-1`；
   - `gravityAlignment` / `yawAlignment` 输入接近零向量时返回单位矩阵。
6. **不要硬编码录包路径、外参或地图尺寸。**

## 五、验收

```bash
source /opt/ros/jazzy/setup.bash
bash check_env.sh
./compile.sh
source install/setup.bash

colcon test --packages-select small_glim
colcon test-result --verbose
```

终端 1：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch small_glim small_glim.launch.py
```

终端 2：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./replay.sh
```

通过标准：

- `small_glim` 的单测全部通过；
- `/small_glim/odometry` 频率与点云频率一致（约 10 Hz），轨迹连续；
- Foxglove 中地图连续生长，无明显重影；
- 正常退出且至少有关键帧时，`map/mapping_*/` 下生成 `mapping.pcd` 和 `poses.txt`。

## 六、提交

- Foxglove 的屏幕录像；
- 报告：
  - 关于这个 Test，你学到了什么？做的过程中压力如何？
  - 你有使用AI辅助吗，如果有，你是如何使用的？
  - 经过这 5 个 Test 的学习，你有什么收获，能否理解视觉组在团队中的作用？

不要提交 `build/`、`install/`、`log/`、`map/mapping_*/` 和原始录包。

把整个.zip发到：

```text
2759879494@qq.com
```
祝顺利 😊😊😊

<div align="right">—— 浙江大学 RoboMaster 算法组</div>

