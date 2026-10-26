# Test5 LivoxLIO

基于 Livox Mid-360 真实录包，完成完整的 LiDAR-IMU 里程计与建图。

LIO 框架、数据流与流程代码已提供。需补全 5 个与 LIO 原理直接相关的函数——位姿插值、协方差、平面正则化、初始姿态对齐。滤波、kNN、下采样、关键帧判断等流程代码已提供，无需修改。

## 录包下载

从 [Google Drive 下载 mid360_bag 录包](https://drive.google.com/drive/folders/1FJgEmFS2wcuFFTUjiKikhG14Zo43saoj)。`.mcap` 文件可放到 `bag/lunatic_0.mcap`，也可以直接把文件或 rosbag2 目录作为参数传给 `replay.sh`。

```text
bag/
└── lunatic_0.mcap       # 可选的默认录包
```

使用前确认录包包含 `/livox/imu` 与 `/livox/lidar/pointcloud`：

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag info <录包文件或目录>
```

---

## 目录结构

```text
Test5_LivoxLIO/
├── QUESTION.md               题目全文
├── README.md                 本文档
├── check_env.sh              环境自检
├── compile.sh                编译脚本
├── replay.sh                 录包播放脚本
├── bag/                      放置 lunatic_0.mcap
├── docs/
│   ├── README.md             文档导读
│   ├── Part_of_Basics.md     前置概念
│   ├── Part_of_Env.md        环境搭建
│   ├── Part_of_Overview.md   整体管线
│   ├── Part_of_Data.md       录包、话题、参数
│   ├── Part_of_PoseInterpolation.md   TODO 1
│   ├── Part_of_Deskew.md              已提供的去畸变模块
│   ├── Part_of_Covariance.md          TODO 2、3
│   ├── Part_of_Initial_Attitude.md    TODO 4、5
│   └── Part_of_Run.md        编译、运行、排错
├── map/                      建图输出
├── video/                    演示视频
└── src/small_glim/           LIO 工程
```

---

## 第一步：配置环境

**依赖一览：**

```text
Ubuntu 24.04 / ROS 2 Jazzy / g++-13
GTSAM 4.3 / gtsam_points 1.2 / PCL
foxglove_bridge / rosbag2 mcap
```

环境搭建的完整步骤、验证方法与常见失败模式见：

```text
docs/Part_of_Env.md
```

环境搭建完成后，在已 source ROS 2 的终端执行自检：

```bash
source /opt/ros/jazzy/setup.bash
bash check_env.sh
```

没有 `[缺失]` 项后继续；`[注意]` 项按输出提示处理。若缺少 GTSAM 或 gtsam_points，按 `docs/Part_of_Env.md` 第五节、第六节处理。

---

## 第二步：编译与确认

```bash
source /opt/ros/jazzy/setup.bash
./compile.sh
source install/setup.bash
```

编译完成后先不要修改代码。先启动节点，确认进程、话题和 Foxglove 连接正常；占位实现下不要回放建图；完整回放至少需要先完成 TODO 2、3，正式验收留到 5 个 TODO 全部完成：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch small_glim small_glim.launch.py
```

打开 Foxglove（`ws://localhost:8765`），连接后确认能看到 `/small_glim/odometry`、`/small_glim/path` 等话题。占位实现阶段只确认节点和 Foxglove 能启动，不以地图质量或完整地图文件作为验收结果。由于协方差/正则化仍是占位实现，回放点云后因子图可能出现 `IndeterminantLinearSystemException` 并退出，这是预期现象，不是环境问题。

演示视频：[registered_cloud](./video/registered_cloud.mp4)。

---

## 第三步：理解待实现内容

若对点云、IMU、刚体变换、协方差、配准、因子图不熟悉，先阅读 `docs/Part_of_Basics.md`，该文档只解释概念，不涉及实现细节。

然后阅读 `docs/Part_of_Overview.md`，理解一帧点云的数据流与模块划分；再阅读 `docs/Part_of_Data.md`，确认录包话题与字段含义。

需实现的 5 个函数位于以下 3 个文件（路径均相对于 Test5 项目根目录）：

| 推荐顺序 | 源码 TODO | 文件 | 函数 | 功能 |
|---:|---|---|---|---|
| 1 | TODO 1 | `src/small_glim/src/common/pose_utils.cpp` | `interpolatePose` | 位姿插值（平移线性 + 旋转 slerp） |
| 2 | TODO 2 | `src/small_glim/src/preprocess/cloud_ops.cpp` | `computeCovariance` | 由邻居计算局部协方差 |
| 3 | TODO 3 | `src/small_glim/src/preprocess/cloud_ops.cpp` | `regularizePlaneCovariance` | 平面协方差特征值正则化 |
| 4 | TODO 4 | `src/small_glim/src/odometry/initial_attitude.cpp` | `gravityAlignment` | 用重力确定 roll/pitch |
| 5 | TODO 5 | `src/small_glim/src/odometry/initial_attitude.cpp` | `yawAlignment` | 用 x 轴投影确定 yaw |


建议按推荐顺序实现表格中的第 1 至第 5 项。位姿插值（TODO 1）虽然只有几行代码，但它是已提供去畸变模块的前置函数，务必先写对。

仅修改上表列出的 3 个源文件；不要修改已提供的 `cloud_deskewing.cpp`、IMU 预积分、因子图、GICP、iVox 及节点骨架。

如果函数没写或写错，可能的结果可见演示视频：

- [interpolatePose](./video/interpolatePose.mp4)
- [computeCovariance](./video/computeCovariance.mp4)
- [gravityAlignment](./video/gravityAlignment.mp4)

---

## 第四步：逐项实现与测试

每项流程：**阅读文档 → 实现函数 → 运行测试 → 通过后进入下一项**。

```bash
source install/setup.bash
colcon test --packages-select small_glim
colcon test-result --verbose
```

4 个单测文件与源码 TODO 标签的对应：

```text
test_pose_utils.cpp       TODO 1
test_deskew.cpp           已提供的 deskew 行为；旋转用例间接验证 TODO 1
test_cloud_ops.cpp        TODO 2、3
test_initial_attitude.cpp TODO 4、5
```

初始状态下，15 个 gtest 用例中共有 8 个失败、7 个通过，属于占位实现的预期结果。完整清单与预期状态见 `docs/Part_of_Run.md` 第三节。注意 `colcon test` 即使有用例失败也可能返回 0，最终以 `colcon test-result --verbose` 为准。

单测失败时，先根据失败测试名定位对应文档，不要凭猜测修改。

---

## 第五步：运行录包验证

单测全部通过后，运行录包查看实际建图效果：

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

Foxglove 中应观察到：地图随运动逐渐生长、墙面轮廓清晰、轨迹连续、回到同一区域时无明显双层墙。录包播放完毕后按 `Ctrl-C` 退出。`mapping.pcd` 只在节点正常退出时一次性写出；`poses.txt` 在运行中随关键帧持续写入并在退出时 flush，仍应正常退出，避免最后的数据未落盘。

效果如下：
[最终效果](./video/normal.mp4)

---

## 输出话题与文件

```text
/small_glim/odometry          nav_msgs/Odometry
/small_glim/path              nav_msgs/Path
/small_glim/registered_cloud  sensor_msgs/PointCloud2
/small_glim/ivox_cloud        sensor_msgs/PointCloud2
/tf                           odom -> imu_link -> lidar_link
```

建图输出目录为 `map/mapping_<时间戳>/`：`poses.txt` 在节点启动时创建、随关键帧持续写入（可能为空），`mapping.pcd` 仅在正常退出且至少有关键帧时生成。

---

## Foxglove

`ws://localhost:8765`。3D 面板 Display frame 选 `odom`，添加：

```text
/small_glim/registered_cloud
/small_glim/ivox_cloud
/small_glim/path
/tf
```

---

## 提交要求

提交内容与报告要求见 `QUESTION.md` 第六节。

不要提交 `build/`、`install/`、`log/`、`map/mapping_*/`、原始录包。

---

<div align="right">—— 浙江大学 RoboMaster 算法组</div>
