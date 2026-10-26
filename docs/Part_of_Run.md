# Part of Run

编译、运行、查看结果、打开地图、问题排查。

---

## 一、环境

```bash
cd <Test5 根目录>
source /opt/ros/jazzy/setup.bash
bash check_env.sh
```

检查项：Ubuntu 版本、ROS_DISTRO、colcon / cmake / ninja、g++-13 / gcc-13、GTSAM 4.3、gtsam_points 1.2、PCL、sensor_msgs / nav_msgs / tf2_ros / foxglove_bridge、rosbag2 mcap、录包与话题。

没有 `[缺失]` 项后继续；`[注意]` 项按输出提示处理。环境搭建的完整步骤、验证方法与常见失败模式见 `Part_of_Env.md`。

缺少 GTSAM 或 gtsam_points 时，按 `Part_of_Env.md` 第五节、第六节处理，不要修改本项目代码绕过。

---

## 二、编译

```bash
source /opt/ros/jazzy/setup.bash
./compile.sh
source install/setup.bash
```

`compile.sh` 固定使用 Ninja 生成器，并传入 `-DCMAKE_BUILD_TYPE=Release -Wno-dev`。

**不要改为 Debug 或默认空构建类型。** small_glim 的因子图优化在未优化编译下性能显著下降，导致 IMU 队列积压、丢帧甚至崩溃。

编译过程中可能出现：

```text
** WARNING ** io features related to pcap will be disabled
```

此为 PCL 提示，表示未编译 pcap 支持，与本项目无关，可忽略。

---

## 三、单元测试

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select small_glim
colcon test-result --verbose
```

新人从基线开始时，5 个待实现函数均为占位实现：15 个 gtest 用例中 7 个通过、8 个失败。**此为预期状态**，不是环境问题。

每完成一个函数，失败数相应减少。目标：`colcon test-result --verbose` 显示 `0 errors, 0 failures`。

单测失败时先查看失败测试名，对应关系：

```text
PoseUtils.*           TODO 1 -> Part_of_PoseInterpolation.md
Deskew.*              已提供的 deskew 测试；仅旋转补偿项间接验证 TODO 1 -> Part_of_Deskew.md
CloudOps.*            TODO 2~3 -> Part_of_Covariance.md
InitialAttitude.*     TODO 4~5 -> Part_of_Initial_Attitude.md
```

完整基线清单：

| 测试文件 | 用例 | 对应实现 | 验证内容 | 基线 |
|---|---|---|---|---|
| `test_pose_utils.cpp` | `PoseUtils.InterpolateMidpoint` | TODO 1 | 平移取中点，旋转插值到 45° | 失败 |
| `test_pose_utils.cpp` | `PoseUtils.InterpolateClampsAlpha` | TODO 1 | `alpha` 被 clamp 到 `[0,1]` | 失败 |
| `test_cloud_ops.cpp` | `CloudOps.ComputeCovarianceOfTwoPoints` | TODO 2 | 两点均值和 `cov(0,0)` | 失败 |
| `test_cloud_ops.cpp` | `CloudOps.ComputeCovarianceRejectsZeroK` | TODO 2 | `k=0` 返回零且不崩溃 | 通过 |
| `test_cloud_ops.cpp` | `CloudOps.RegularizePlaneCovarianceOnIdentity` | TODO 3 | 单位阵正则化后的对角线 | 失败 |
| `test_cloud_ops.cpp` | `CloudOps.RegularizePlaneCovarianceReturnsEigenSystem` | TODO 3 | 输出原始特征值、最小特征值方向，并重建旋转后的协方差 | 失败 |
| `test_initial_attitude.cpp` | `InitialAttitude.GravityIdentityWhenAlreadyUp` | TODO 4 | 输入 `+Z` 返回单位矩阵 | 通过 |
| `test_initial_attitude.cpp` | `InitialAttitude.GravityMapsMeasuredDirectionToZ` | TODO 4 | 输入 `+X` 映射到 `+Z`，且行列式为 `+1` | 失败 |
| `test_initial_attitude.cpp` | `InitialAttitude.GravityZeroReturnsIdentity` | TODO 4 | 零向量返回单位矩阵 | 通过 |
| `test_initial_attitude.cpp` | `InitialAttitude.YawMapsHeadingToX` | TODO 5 | `(0,1,0)` 旋转到 `+X` | 失败 |
| `test_initial_attitude.cpp` | `InitialAttitude.YawIgnoresZComponent` | TODO 5 | `(0,1,0)` 与 `(0,1,9)` 结果相同 | 通过 |
| `test_initial_attitude.cpp` | `InitialAttitude.YawZeroReturnsIdentity` | TODO 5 | 零向量返回单位矩阵 | 通过 |
| `test_deskew.cpp` | `Deskew.IdentityPosesKeepPoints` | 已提供 deskew | 位姿全为单位矩阵时点不变 | 通过 |
| `test_deskew.cpp` | `Deskew.RotationIsCompensated` | 间接验证 TODO 1 | 中间时刻使用 45° 插值姿态补偿 | 失败 |
| `test_deskew.cpp` | `Deskew.InvalidTimesReturnInput` | 已提供 deskew | 时间倒序时返回原 points | 通过 |

`colcon test` 即使有用例失败也可能返回 0，判定时必须以 `colcon test-result --verbose` 为准。4 个测试可执行文件还会各计一条 CTest 记录，因此基线汇总可能显示 `19 tests, 12 failures`；讨论题目用例数时应以 15 个 gtest 用例为准。

只有 `Deskew.RotationIsCompensated` 对 TODO 1 有实质验证，并通过中间时刻检验旋转插值；`IdentityPosesKeepPoints` 也会调用插值，但恒等输入无法暴露常见错误，`InvalidTimesReturnInput` 则在插值前返回。这两项可在 TODO 1 未完成时先通过。

查看某个测试的详细输出：

```bash
colcon test --packages-select small_glim --event-handlers console_direct+
```

---

## 四、启动

终端 1：

```bash
cd <Test5 根目录>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch small_glim small_glim.launch.py
```

此 launch 同时启动 `small_glim_node` 与 `foxglove_bridge`（默认端口 8765）。

确认节点已启动：

```bash
ros2 node list
```

应显示 `/small_glim` 与 `/foxglove_bridge`；还可能出现 `small_glim` 内部 TF 监听器生成的 `/transform_listener_impl_*` 节点，这是正常现象。

不启动 Foxglove：`ros2 launch small_glim small_glim.launch.py use_foxglove:=false`

更换端口：`ros2 launch small_glim small_glim.launch.py foxglove_port:=8888`

---

## 五、播放录包

终端 2：

```bash
cd <Test5 根目录>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./replay.sh
```

默认播放 `bag/lunatic_0.mcap`。可指定 `.mcap` 文件或 rosbag2 目录：`./replay.sh /path/to/other_bag`

**不要同时启动多个 `ros2 bag play`**，否则数据重复输入，地图叠加。

录包播放完毕后，在终端 1 按 `Ctrl-C`。

**`mapping.pcd` 只在节点正常退出时一次性写盘**，直接关闭终端或 kill -9 可能导致地图丢失。`poses.txt` 在运行中随关键帧持续写入，但不能据此认为中途退出没有损失。须使用 Ctrl-C 使节点正常退出。

---

## 六、查看输出

```bash
ros2 topic hz /small_glim/odometry
```

正常：average rate 约 10。初始化完成后，正常情况下每帧点云对应一次输出；处理不过来时可能丢帧。

```bash
ros2 topic echo /small_glim/odometry --once
```

观察位置与姿态是否在变化。

```bash
ros2 run tf2_ros tf2_echo odom imu_link
```

观察 TF 是否持续更新。

若 `odometry` 存在但不动，检查：录包是否正在播放、`/livox/lidar/pointcloud` 的 `header.stamp` 是否正常、是否仅有一个 small_glim 实例。

---

## 七、Foxglove

连接 `ws://localhost:8765`。3D 面板 Display frame 选 `odom`。

添加：

```text
/small_glim/registered_cloud
/small_glim/ivox_cloud
/small_glim/path
/tf
```

`/small_glim/path` 为轨迹线，便于在 3D 面板直接观察。节点额外发布 `nav_msgs/Path`；odometry 数值可在 Raw Messages 或 Plot 面板查看。

查看 odometry 数值：使用 Raw Messages 面板选择 `/small_glim/odometry`，或 Plot 面板绘制 `pose.pose.position.x / y / z`。

### 正确现象

- 地图随运动逐渐生长；
- 墙面、箱子轮廓清晰；
- 轨迹连续，无突然折线；
- 回到同一区域时地图基本重合，无明显双层墙。

### 错误现象

| 现象 | 优先检查项 |
|---|---|
| 无任何显示 | 是否连接 bridge、Display frame 是否正确、录包是否正在播放 |
| 地图仅一帧 | 关键帧阈值、机器人是否运动 |
| 地图重影 | 去畸变（TODO 1 与已提供的 deskew）、外参、时间戳 |
| 轨迹跳变 | 配准、IMU 饱和、关键帧 |
| 地图有飞点 | 距离滤波、无效点 |
| 地图高度方向漂移 | IMU 轴方向、加速度单位、外参 |

---

## 八、建图结果位置

`params_mapping.yaml` 中 `output_root: "map"` 为相对路径，相对于**启动 `ros2 launch` 时的工作目录**。

在 Test5 根目录启动时，结果目录为 `Test5_LivoxLIO/map/mapping_<时间戳>/`。`poses.txt`（关键帧位姿）在节点启动时即创建，可能为空；`mapping.pcd`（全局点云）只在正常退出且地图非空时生成。若没有关键帧或异常退出，不要仅凭目录存在判断建图成功。

`frame_*.pcd` 由 `save_raw_mapping_frames` 控制，默认关闭，不会生成。

```bash
ls -lt map | head
```

---

## 九、打开 mapping.pcd

### 方式 A：Foxglove

先启动 bridge（launch 已包含），再发布 PCD：

```bash
ros2 run pcl_ros pcd_to_pointcloud --ros-args \
  -p file_name:=<Test5 根目录>/map/mapping_xxx/mapping.pcd \
  -p publishing_period_ms:=1000 \
  -p tf_frame:=odom
```

Foxglove 中添加 `/cloud_pcd`，Display frame 选 `odom`。

### 方式 B：pcl_viewer

```bash
sudo apt install pcl-tools
latest=$(ls -td map/mapping_*/ | head -1)
pcl_viewer "${latest}mapping.pcd"
```

### 方式 C：Open3D

Ubuntu 24.04 的系统 Python 启用了 PEP 668，直接 `pip install` 可能被拒绝。建议用虚拟环境：

```bash
python3 -m venv /tmp/test5-open3d
source /tmp/test5-open3d/bin/activate
pip install open3d
python3 - <<'EOF'
import glob
import open3d as o3d

path = sorted(glob.glob('map/mapping_*/mapping.pcd'))[-1]
print(path)
pcd = o3d.io.read_point_cloud(path)
o3d.visualization.draw_geometries([pcd])
EOF
```

---

## 十、常见问题排查

### 启动崩溃

检查：GTSAM / gtsam_points / PCL 是否已安装、是否使用 Release 编译、`ros2 launch` 日志中首个 ERROR。

### 无里程计

```bash
ros2 topic list | grep livox
ros2 topic hz /livox/lidar/pointcloud
ros2 topic hz /livox/imu
```

若录包话题不存在，检查 `params_node.yaml` 中的话题名。

### 有里程计但地图未生成

确认：`enable_mapping` 是否为 true、机器人是否实际运动、`map/mapping_*` 目录是否有写权限。

### 地图重影

按顺序排查：

```text
1. 外参 T_lidar_imu
2. 时间偏移 imu_time_offset / lidar_time_offset
3. 加速度单位 acc_scale
4. 去畸变是否正常（TODO 1 与已提供的 deskew）
5. keyframe 阈值是否过小
```

### 运行中途崩溃

查看日志末尾是否出现 `IMU saturation`、`insufficient number of IMU data between LiDAR scans`、`queue overflow`。常见原因为未使用 Release 编译，处理速度不足导致 IMU 队列溢出。

### 占位实现下回放即退出

若尚未实现 TODO 2、3，`computeCovariance` / `regularizePlaneCovariance` 返回零协方差，GICP 因子可能退化，日志出现 `IndeterminantLinearSystemException`。这属于基线占位实现的预期现象，不是环境问题；先完成 TODO 2、3 并通过对应单测，再回放建图。

### 单测通过但地图质量不佳

单测仅验证函数逻辑，不验证系统整体效果。若插值或时间边界处理不够稳健、或初始化/外参配置不合适，单测可能通过但实际效果不佳。此时须在 Foxglove 中观察现象，参照上方错误现象表排查。

---

## 十一、录屏

提交屏幕录像，内容包含：终端 1 的 `ros2 launch`、终端 2 的 `./replay.sh`、Foxglove 中地图生长与轨迹、终端中 `colcon test` 通过。

不要仅提交静态截图。录像可体现轨迹连续性与地图重影情况。

---

## 十二、关闭所有进程

优先在各自终端按 `Ctrl-C` 正常退出 launch 与 bag play。若终端已关闭但进程仍在，先列出当前用户相关进程并确认 PID：

```bash
ps -u "$USER" -o pid,comm,args
```

从列表中确认属于本次 Test5 的 PID 后，再逐个正常终止：

```bash
kill <PID>
```

清理后重新启动，避免多实例竞争同一话题。
