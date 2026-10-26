# 录包说明

本任务使用 Livox Mid-360 录包。推荐使用：

```text
lunatic_0.mcap
```

如果工作区未附带录包，从 [Google Drive 下载 mid360_bag](https://drive.google.com/drive/folders/1FJgEmFS2wcuFFTUjiKikhG14Zo43saoj)。在 Test5 根目录、已 source ROS 2 的终端运行 `ros2 bag info <录包路径>` 检查话题；rosbag2 目录可直接通过 `./replay.sh <录包目录>` 播放，`.mcap` 文件可放在本目录并命名为 `lunatic_0.mcap`，也可将文件路径传给脚本。

必需话题：

```text
/livox/lidar/pointcloud   sensor_msgs/msg/PointCloud2
/livox/imu                sensor_msgs/msg/Imu
```

`PointCloud2` 字段约定：

```text
x,y,z        float32
intensity    float32
tag, line    uint8
timestamp    float64，绝对时间，单位 ns
```

不要使用 `/livox/lidar`（`livox_ros_driver2/msg/CustomMsg`）作为输入；
本任务使用同一录包中的 `PointCloud2` 版本，避免依赖 Livox 驱动包。

## 放置方式

直接把 mcap 放到本目录：

```text
bag/lunatic_0.mcap
```

或者建立软链接，避免复制大文件（在 Test5 根目录执行）：

```bash
ln -sfn /path/to/lunatic_0.mcap bag/lunatic_0.mcap
```

## 检查

```bash
ros2 bag info bag/lunatic_0.mcap
```

应看到：

```text
/livox/lidar/pointcloud   sensor_msgs/msg/PointCloud2
/livox/imu                sensor_msgs/msg/Imu
```

## 播放

```bash
./replay.sh
# 或指定路径
./replay.sh /path/to/lunatic_0.mcap
```

`replay.sh` 只做单次播放，不接受 `--loop`，避免地图重复叠加。如需循环观察，可手动执行：

```bash
ros2 bag play bag/lunatic_0.mcap --loop
```

正式验收建议单次播放。
