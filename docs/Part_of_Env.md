# Part of Env

本 Test 的环境搭建。环境不计入考核评分，但环境不通无法运行项目。

本节给出安装顺序、每项的验证方法与常见失败模式。

---

## 一、安装顺序

顺序由依赖关系决定，不可颠倒：

```text
1. 系统基础工具        apt（含 build-essential、git）
2. ROS 2 Jazzy         已安装则跳过
3. PCL                 apt
4. GTSAM 4.3           源码编译
5. gtsam_points 1.2    源码编译，依赖第 4 步
6. ROS 2 依赖包        apt
7. foxglove_bridge     apt
8. 录包                 放到 bag/
```

gtsam_points 的 cmake 会查找 GTSAM 的 config 文件。若第 4 步未完成或安装路径不在默认搜索路径中，第 5 步会在 cmake 配置阶段失败。

---

## 二、系统基础工具

`check_env.sh` 检查的构建工具：

```text
bash
python3
g++-13 / gcc-13
cmake
ninja
pkg-config
colcon
ros2
```

Ubuntu 24.04 的默认 GCC 为 13 系列，但最小安装不一定包含编译器和 make。`check_env.sh` 明确要求 `g++-13`、`gcc-13` 命令存在，因此需要安装构建工具、Git 及其余基础依赖：

```bash
sudo apt install build-essential git cmake ninja-build pkg-config python3-colcon-common-extensions
```

若系统存在多个 gcc 版本，编译 GTSAM 时需显式指定编译器：

```bash
export CC=gcc-13
export CXX=g++-13
```

否则 cmake 可能选到其他版本，导致与后续 gtsam_points 编译时的编译器不一致。

**验证：**

```bash
g++-13 --version
cmake --version
ninja --version
colcon version-check
```

四条命令均应正常输出，无「command not found」。

---

## 三、ROS 2 Jazzy

若已安装 ROS 2 Jazzy，跳过本节。检查方式：

```bash
source /opt/ros/jazzy/setup.bash
echo $ROS_DISTRO
```

输出 `jazzy` 即已安装。未安装时，按 ROS 2 Jazzy 官方 Ubuntu apt 安装文档安装 `ros-jazzy-desktop`（<https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>），再回到本节验证。

安装后每次打开新终端需执行：

```bash
source /opt/ros/jazzy/setup.bash
```

可将该行加入 `~/.bashrc`，避免每次手动 source。

**验证：**

```bash
ros2 --help
```

应输出 ros2 命令帮助，无报错。

---

## 四、PCL

```bash
sudo apt install libpcl-dev ros-jazzy-pcl-ros
```

**验证：**

```bash
pkg-config --modversion pcl_common
```

应输出已安装的 PCL 版本。项目通过 CMake 的 `find_package(PCL REQUIRED)` 查找 PCL，不要求固定为 1.14；`check_env.sh` 使用 `pkg-config` 验证 PCL 是否可发现。

---

## 五、GTSAM 4.3

源码编译，目标版本 `4.3a0`。以下为队伍已验证的命令。

### 5.1 安装依赖

```bash
sudo apt install libomp-dev libboost-all-dev libmetis-dev libfmt-dev libspdlog-dev
```

### 5.2 克隆并构建

```bash
git clone https://github.com/borglab/gtsam
cd gtsam && git checkout 4.3a0
mkdir build && cd build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_WITH_TBB=OFF \
         -DGTSAM_USE_SYSTEM_EIGEN=ON \
         -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
         -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

### 5.3 关键选项说明

| 选项 | 作用 |
|---|---|
| `GTSAM_BUILD_EXAMPLES_ALWAYS=OFF` | 不编译示例代码，缩短构建时间 |
| `GTSAM_BUILD_TESTS=OFF` | 不编译测试，缩短构建时间 |
| `GTSAM_WITH_TBB=OFF` | 不启用 TBB 并行，减少依赖 |
| `GTSAM_USE_SYSTEM_EIGEN=ON` | 使用系统 Eigen，避免 GTSAM 自带 Eigen 与 PCL、gtsam_points 的 Eigen 版本冲突 |
| `GTSAM_BUILD_WITH_MARCH_NATIVE=OFF` | 不针对本机 CPU 指令集优化，保证二进制在其他机器上可运行 |
| `CMAKE_BUILD_TYPE=Release` | Release 构建，与 `compile.sh` 的要求一致；Debug 构建会显著拖慢后续因子图优化 |

### 5.4 安装后验证

```bash
ls /usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake
```

或：

```bash
ls /usr/lib/x86_64-linux-gnu/cmake/GTSAM/GTSAMConfig.cmake
```

两条路径任一存在即可，`check_env.sh` 只检查这两个固定位置。若安装到其他路径，构建时需将安装路径加入 `CMAKE_PREFIX_PATH`：

```bash
export CMAKE_PREFIX_PATH=/your/gtsam/install/path:$CMAKE_PREFIX_PATH
```

但 `check_env.sh` 不会读取 `CMAKE_PREFIX_PATH`，仍会把该安装判为 `[缺失]`。此时要么重新安装到 `/usr/local`，要么把自检中的 GTSAM 路径检查视为已知限制。

### 5.5 常见失败模式

| 现象 | 原因 | 处理 |
|---|---|---|
| cmake 报找不到 Eigen | 系统缺 Eigen 头文件 | `sudo apt install libeigen3-dev` 后重新 cmake |
| make 阶段进程被系统杀死 | 并行编译进程过多，内存不足 | 减少并行数：`make -j4` |
| 安装后找不到 GTSAMConfig.cmake | 安装路径不在默认搜索路径 | 设置 `CMAKE_PREFIX_PATH` |
| `git checkout 4.3a0` 报找不到 tag | 仓库克隆不完整 | `git fetch --tags` 后重试 |

---

## 六、gtsam_points 1.2

源码编译，依赖第 5 步的 GTSAM。以下为队伍已验证的命令，版本为 `v1.2.0`。

### 6.1 克隆并构建

```bash
git clone https://github.com/koide3/gtsam_points
cd gtsam_points && git checkout v1.2.0
mkdir build && cd build
cmake .. -DBUILD_WITH_CUDA=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_MARCH_NATIVE=OFF
make -j$(nproc)
sudo make install
```

### 6.2 关键选项说明

| 选项 | 作用 |
|---|---|
| `BUILD_WITH_CUDA=OFF` | 不启用 CUDA，避免依赖 GPU 与 CUDA Toolkit；本项目未使用 GPU 加速 |
| `CMAKE_BUILD_TYPE=Release` | Release 构建，与 GTSAM 及 `compile.sh` 一致 |
| `BUILD_WITH_MARCH_NATIVE=OFF` | 不针对本机 CPU 指令集优化，保证二进制在其他机器上可运行 |

### 6.3 编译前确认 GTSAM 可被找到

```bash
ls /usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake
```

或已按 5.4 设置 `CMAKE_PREFIX_PATH`。若 cmake 配置阶段报找不到 GTSAM，先回到 5.4 确认 GTSAM 安装位置。

### 6.4 安装后验证

```bash
ls /usr/local/lib/cmake/gtsam_points/gtsam_points-config.cmake
```

或：

```bash
ls /usr/local/lib/cmake/gtsam_points/gtsam_pointsConfig.cmake
```

`check_env.sh` 只检查这两个固定位置；若安装到其他前缀，构建时同样需要把前缀加入 `CMAKE_PREFIX_PATH`，但自检仍会报告缺失。

### 6.5 常见失败模式

| 现象 | 原因 | 处理 |
|---|---|---|
| cmake 报找不到 GTSAM | GTSAM 未装或路径未配置 | 回到第 5 步确认 |
| 编译报 GTSAM 版本不兼容 | GTSAM 与 gtsam_points 版本不匹配 | 按 gtsam_points 官方 README 要求的 GTSAM 版本重装 |
| make 阶段进程被系统杀死 | 并行编译进程过多，内存不足 | 减少并行数：`make -j4` |
| 编译报找不到 PCL | PCL 未装或路径未配置 | 回到第四节确认 |

---

## 七、ROS 2 依赖包

`check_env.sh` 检查的包：

```text
sensor_msgs
nav_msgs
geometry_msgs
tf2
tf2_ros
tf2_geometry_msgs
rclcpp
rclcpp_components
foxglove_bridge
rosbag2_storage_mcap
```

`check_env.sh` 的修复提示为 `sudo apt install ros-$ROS_DISTRO-<包名>`，包名中的下划线需替换为连字符。例如 `tf2_ros` 对应：

```bash
sudo apt install ros-jazzy-tf2-ros
```

一次性安装全部：

```bash
sudo apt install \
  ros-jazzy-sensor-msgs \
  ros-jazzy-nav-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-tf2 \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-rclcpp \
  ros-jazzy-rclcpp-components \
  ros-jazzy-foxglove-bridge \
  ros-jazzy-rosbag2-storage-mcap
```

**验证：**

```bash
ros2 pkg prefix sensor_msgs
```

每条 `ros2 pkg prefix <包名>` 应输出路径，不报错。

---

## 八、录包

从 [Google Drive 下载 mid360_bag 录包](https://drive.google.com/drive/folders/1FJgEmFS2wcuFFTUjiKikhG14Zo43saoj)。先检查下载内容：

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag info <录包文件或目录>
```

应包含 `/livox/imu` 与 `/livox/lidar/pointcloud` 话题。`.mcap` 文件可放到 `bag/lunatic_0.mcap`，rosbag2 目录或其他路径可直接传给播放脚本：

```bash
./replay.sh
./replay.sh /path/to/lunatic_0.mcap
./replay.sh /path/to/rosbag2_directory
```

---

## 九、完整验证

以上全部完成后，在已 source ROS 2 的终端执行：

```bash
source /opt/ros/jazzy/setup.bash
bash check_env.sh
```

应没有 `[缺失]` 项；`[注意]` 项不是安装失败。若有 `[缺失]` 项，按 `check_env.sh` 给出的修复提示处理，或回到本文档对应小节。

`[注意]` 项（如录包未找到）不影响编译，但会影响后续运行验证。

---

## 十、编译验证

确认没有 `[缺失]` 项后，执行：

```bash
source /opt/ros/jazzy/setup.bash
./compile.sh
source install/setup.bash
```

编译成功应输出：

```text
构建完成：<项目根目录>
```

编译失败时，按错误信息定位：

| 错误关键字 | 可能原因 |
|---|---|
| `GTSAMConfig.cmake not found` | GTSAM 未装或未设置 `CMAKE_PREFIX_PATH` |
| `gtsam_points-config.cmake not found` | gtsam_points 未装或未设置 `CMAKE_PREFIX_PATH` |
| `PCL not found` | PCL 未装或版本不匹配 |
| `ninja: command not found` | 未装 ninja |
| `colcon: command not found` | 未装 python3-colcon-common-extensions |

编译通过后，运行单元测试检查基线状态。占位实现下部分用例应当失败，这不是环境问题：

```bash
colcon test --packages-select small_glim
colcon test-result --verbose
```

从新人基线提交开始时，5 个待实现函数为占位实现：15 个 gtest 用例中 7 个通过、8 个失败。完整清单见 `docs/Part_of_Run.md` 第三节。`colcon test` 即使有用例失败也可能返回 0，请以 `colcon test-result --verbose` 的汇总为准。若测试无法启动（如找不到测试可执行文件），说明编译或安装有问题，需回到上表排查。
