# Part of Data

录包内容、数据格式、坐标系关系、参数位置。

编写代码前应先理解这些内容，避免大量运行时的调试时间。

---

## 零、使用 Foxglove 查看录包

编译与修改代码前，先查看原始数据，了解点云与 IMU 的基本特征。

本录包**不包含** `/tf`。查看原始数据时只需启动 Foxglove 桥与录包；`small_glim` 会发布自己的 TF，可能干扰对原始数据的观察，因此这一步先不启动它。

终端 A，启动 Foxglove 桥：

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

终端 B，循环播放录包：

```bash
cd Test5_LivoxLIO
source /opt/ros/jazzy/setup.bash
ros2 bag play bag/lunatic_0.mcap --loop
```

浏览器打开 Foxglove，连接 `ws://localhost:8765`。

操作步骤：

1. 添加 **3D** 面板，将 **Fixed frame** 设为 `livox_frame`（录包无 TF，使用消息自身坐标系）。
2. 添加话题 `/livox/lidar/pointcloud`，点大小调小，Decay time 设为 0.1 至 0.2 秒，可观察单一帧点云拖尾。
3. 添加 **Plot** 面板，订阅 `/livox/imu` 的 `linear_acceleration.z`（原始单位 g，加速度模长约 1.0；本录包初始 z 分量约 0.7，乘 `acc_scale` 后约 6.9 m/s²，三轴合成约 9.8 m/s²）与 `angular_velocity.z`（转动时变化明显）。

`/livox/lidar` 类型为 `livox_ros_driver2/msg/CustomMsg`，Foxglove 默认不支持此 schema，无需关注，查看 `/livox/lidar/pointcloud` 即可。

查看后应能确认：

```text
- 点云约 10 Hz，IMU 约 200 Hz
- 雷达安装位置，点云中车体/地面/墙的大致范围
- 机器人旋转速度，IMU 角速度量级
- 点云是否存在明显运动拖影
```

这些直观信息对应后续的距离滤波、去畸变与协方差。

---

## 一、录包

默认位置：`bag/lunatic_0.mcap`。工作区未附带录包时，从 [Google Drive 下载 mid360_bag](https://drive.google.com/drive/folders/1FJgEmFS2wcuFFTUjiKikhG14Zo43saoj)，并先用 `ros2 bag info <录包路径>` 确认包含本节列出的话题。下载得到的 rosbag2 目录可直接传给 `./replay.sh`；若得到 `.mcap` 文件，可放入默认位置或通过脚本参数传入。

```bash
ros2 bag info bag/lunatic_0.mcap
```

应显示：

```text
/livox/imu                sensor_msgs/msg/Imu          约 200 Hz
/livox/lidar              livox_ros_driver2/msg/CustomMsg
/livox/lidar/pointcloud   sensor_msgs/msg/PointCloud2   10 Hz
```

本 Test 使用 `/livox/imu` 与 `/livox/lidar/pointcloud`，不使用 `CustomMsg`，因此不依赖 Livox 驱动包。

---

## 二、PointCloud2 格式

`PointCloud2` 是按字节排布的表结构：

```text
fields       字段列表，每个字段在点内的偏移
point_step   一个点占用的字节数
row_step     一行占用的字节数
width/height 点云尺寸
data         字节数据
```

本录包的字段：

| 字段 | 偏移 | 类型 | 含义 |
|---|---:|---|---|
| `x` | 0 | float32 | 传感器系 x |
| `y` | 4 | float32 | 传感器系 y |
| `z` | 8 | float32 | 传感器系 z |
| `intensity` | 12 | float32 | 反射强度 |
| `tag` | 16 | uint8 | Livox 标签 |
| `line` | 17 | uint8 | 线号 |
| `timestamp` | 18 | float64 | 点的绝对时间，单位 ns |

```text
point_step = 26
height     = 1
width      = 约 20000
frame_id   = livox_frame
```

读取第 `i` 个点的 x：`data[i * point_step + x_offset]`。

注意事项：

- 须按 `fields[].offset` 读取，不要假设偏移始终从 0 开始；
- `point_step` 不一定为 26，更换雷达后会变化；
- `height=1` 时 `width` 即点数；`height>1` 时总数为 `width * height`；
- 不要用 `sizeof(float)` 推断偏移，不同雷达字段顺序可能不同。

---

## 三、每点时间戳

`timestamp` 为每个点的绝对时间，单位纳秒，例如 `1758895117916802366`。

转为秒：`1758895117.916802366`。

帧头 `header.stamp` 为该帧的起始时间，通常与第一个点的时间接近。

相对时间：`relative = timestamp * 1e-9 - header.stamp`，大致落在 `[0, 0.1]`。

必要性：雷达边转边扫，非瞬时采集。快速旋转时帧内各点姿态不同，去畸变需知道每个点的采样时刻。

若点云无逐点时间，TimeKeeper 将按点序分配伪时间。

---

## 四、IMU

话题：`/livox/imu`

```text
orientation             不可信，全 0 / 单位四元数
angular_velocity        rad/s，可用
linear_acceleration     原始值约 1 g，需 acc_scale 换算为 m/s²
```

本系统不使用 orientation，使用 angular_velocity 进行预积分与去畸变，使用 linear_acceleration 估计重力、速度、零偏。

`acc_scale` 位于 `params_node.yaml`：`acc_scale: 9.80665`。录包原始加速度按 g 给出，节点乘以该值换算为 m/s²；更换 IMU 时，若加速度单位不同，须修改此值。

---

## 五、坐标系

```text
odom       里程计坐标系，系统内部连续
imu_link   IMU 坐标系
lidar_link LiDAR 坐标系
```

TF 链：`odom -> imu_link -> lidar_link`

```bash
ros2 run tf2_ros tf2_echo odom imu_link
ros2 run tf2_ros tf2_echo imu_link lidar_link
```

注意：本系统无 `map -> odom`，因为是独立 LIO，不做全局重定位。地图与里程计均在 `odom` 系。Foxglove 显示帧须选 `odom`，不要选 `map`。

---

## 六、外参

IMU 到 LiDAR 的变换位于 `config/params_sensors.yaml`：

```yaml
T_lidar_imu: [0.011, 0.02329, -0.04412, 0.0, 0.0, 0.0, 1.0]
```

格式：`[tx, ty, tz, qx, qy, qz, qw]`。`T_lidar_imu` 表示 IMU 系到 LiDAR 系的变换，即 `p_lidar = T_lidar_imu · p_imu`。代码需要把 LiDAR 点转到 IMU 系时，使用它的逆 `T_imu_lidar = T_lidar_imu^{-1}`。

外参错误的影响：所有点整体偏移/旋转，配准仍可运行但地图与轨迹存在系统偏差，快速运动时偏差更明显。

更换雷达或安装位置时必须修改此项。此值为 IMU 到 LiDAR 的标定外参，按实际安装位置测量或标定得到。

---

## 七、参数文件

位于 `src/small_glim/config/`。

### params_node.yaml

管理话题、坐标系、开关、时间偏移、加速度单位。

```yaml
debug: false
enable_mapping: true
enable_tf_publish: true
imu_time_offset: 0.0
lidar_time_offset: 0.0
acc_scale: 9.80665
imu_sub_topic: "/livox/imu"
lidar_sub_topic: "/livox/lidar/pointcloud"
imu_frame_id: "imu_link"
lidar_frame_id: "lidar_link"
odometry_frame_id: "odom"
```

`imu_time_offset` 加到 IMU 时间戳上；`lidar_time_offset` 在 TimeKeeper 处理前加到帧时间戳上。默认 `autoconf_perpoint_times: true` 且逐点时间为绝对时间时，TimeKeeper 会用首点时间覆盖帧时间戳，因此 `lidar_time_offset` 不能可靠地修正这种逐点时间差；IMU 与 LiDAR 间若存在明显时间偏差，应优先修正数据源或使用 `imu_time_offset` 校准。

### params_sensors.yaml

管理 IMU 噪声、饱和阈值、LiDAR 字段名、逐点时间配置。

```yaml
imu_acc_noise: 0.05
imu_gyro_noise: 0.02
imu_acc_saturation_thresh: 39.0
imu_gyro_saturation_thresh: 34.5
global_shutter_lidar: false
T_lidar_imu: [...]
intensity_field: "intensity"
ring_field: "line"
autoconf_perpoint_times: true
max_scan_duration: 0.2
```

`saturation` 阈值用于剧烈运动场景：IMU 某轴超量程时，系统按 `imu_saturation_mult` 缩放对应轴的协方差（默认 `1.0`，即不额外放大），并把该帧标记为饱和。饱和帧仍参与里程计，但里程计会跳过其局部目标地图插入，并可能暂停目标地图更新；`AsyncMapping` 会跳过 `deskew_imu_saturated` 的帧。本录包中加速度有少量样本超过 `imu_acc_saturation_thresh`，陀螺未达到 `imu_gyro_saturation_thresh`；阈值仍用于保护更剧烈的数据。

### params_preprocess.yaml

管理降采样、距离滤波、离群点、协方差邻居数。

```yaml
distance_near_thresh: 0.3
distance_far_thresh: 30.0
use_random_grid_downsampling: false
downsample_resolution: 0.05
k_correspondences: 10
min_points_after_filter: 100
```

`k_correspondences = 10` 即 computeCovariance 的 k 值。过小则协方差不准确，过大则计算量增加且混入非局部点。

### params_odometry_estimation.yaml

管理初始化、优化窗口、配准类型、iVox、冲击暂停。

```yaml
initialization_window_size: 1.0
smoother_lag: 3.0
registration_type: "GICP"
ivox_resolution: 0.5
ivox_update_delay: 0.5
ivox_impact_pause_duration: 0.5
```

`smoother_lag = 3.0` 为因子图固定滞后窗口大小（秒）。最近 3 秒的位姿联合优化，更早的位姿固定不变。

### params_mapping.yaml

管理建图输出、范围裁剪、关键帧阈值、地图体素。

```yaml
save_raw_mapping_frames: false
output_root: "map"
cloud_range_min: 0.8
cloud_range_max: 20.0
keyframe_trans_thresh: 0.1
keyframe_rot_thresh: 5.0
downsample_every_n_keyframes: 1000
map_voxel_leaf_size: 0.05
```

`keyframe_trans_thresh` 的单位是米，`keyframe_rot_thresh` 的单位是度；两者任一超过阈值即插入关键帧。`downsample_every_n_keyframes` 控制累计多少个关键帧后对全局地图做一次体素下采样。

---

## 八、数据验证

播放录包后，另开终端执行：

```bash
ros2 topic hz /livox/imu                    # 约 200 Hz
ros2 topic hz /livox/lidar/pointcloud       # 约 10 Hz
ros2 topic echo /livox/lidar/pointcloud --once --field header
ros2 topic echo /livox/imu --once
```

正常结果：IMU 约 200 Hz，点云约 10 Hz，frame_id = livox_frame。

---

## 九、常见误区

| 误区 | 实际 |
|---|---|
| 将 PointCloud2 当 float 数组顺序读取 | 须按 fields 偏移与 point_step |
| 使用 `now()` 作为点云时间 | 须使用 `header.stamp` 与每点 `timestamp` |
| 将 odom 当作 map | 本系统仅有 odom，不存在 map |
| 忽略外参 | 外参错误导致地图整体偏移 |
| 将 IMU orientation 当作真值 | 录包中 orientation 不可信 |
| 把原始加速度值直接当作 m/s² | 原始值约为 g，需乘 `acc_scale` 换算 |

下一篇：从 `Part_of_PoseInterpolation.md` 开始逐模块实现。
