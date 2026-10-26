# Test5 文档

## 文档组织

文档按"前置基础 → 环境 → 整体 → 数据 → 逐模块 → 运行"的顺序组织。修改代码前应先完成阅读。

| 顺序 | 文档 | 阅读后应达到的理解 |
|---:|---|---|
| 0 | `Part_of_Basics.md` | 点云、IMU、位姿、协方差、配准、因子图的基本概念 |
| 1 | `Part_of_Env.md` | 环境搭建顺序、验证方法与常见失败模式 |
| 2 | `Part_of_Overview.md` | 理解 LIO 解决的问题、数据流、已提供与待实现的部分 |
| 3 | `Part_of_Data.md` | 理解录包、PointCloud2、IMU、坐标系、外参与各参数文件 |
| 4 | `Part_of_PoseInterpolation.md` | 实现 `interpolatePose`，理解平移插值与四元数 slerp |
| 5 | `Part_of_Covariance.md` | 实现 `computeCovariance` 和 `regularizePlaneCovariance`，理解 GICP 局部几何 |
| 6 | `Part_of_Initial_Attitude.md` | 实现 `gravityAlignment` 和 `yawAlignment`，理解初始 roll/pitch/yaw 的确定方式 |
| 7 | `Part_of_Deskew.md` | 阅读已提供的 `CloudDeskewing::deskew`，理解完整运动补偿链；它是 TODO 1 的集成调用方 |
| 8 | `Part_of_Run.md` | 编译、单测、启动、播放、Foxglove 查看、检查建图输出与排错 |

---

## 建议阅读方式

### 第零遍：补齐前置概念

若对点云、IMU、刚体变换、协方差、配准、因子图不熟悉，先阅读 `Part_of_Basics.md`。该文档不涉及实现细节，只解释这些概念在做什么。

若已熟悉上述内容，可直接进入下一步。

### 环境搭建

阅读 `Part_of_Env.md`，按顺序完成环境安装与验证。若已有现成环境，可跳过源码编译部分。

source ROS 2 后执行 `bash check_env.sh`，确认没有 `[缺失]` 项；若出现 `[注意]`，按提示逐项处理。

### 第一遍：建立整体印象

阅读 `Part_of_Overview.md`，暂不深入公式，先确认以下三个问题：

```text
1. 一帧点云进来，经过哪些模块，最后变成什么？
2. 哪些模块已经实现？
3. 待实现的 5 个函数分别位于哪一步？
```

若无法回答，先不要继续阅读后续文档。

### 第二遍：理解数据

阅读 `Part_of_Data.md`。重点：录包话题、PointCloud2 各字段含义、每点时间戳的必要性、三个坐标系的连接关系、各 yaml 配置文件的用途。

### 第三遍：逐模块实现

不要一次读完全部文档后再写代码。推荐顺序：

```text
Part_of_PoseInterpolation   -> interpolatePose
Part_of_Covariance          -> computeCovariance
                             -> regularizePlaneCovariance
Part_of_Initial_Attitude    -> gravityAlignment
                             -> yawAlignment
```

每项流程相同：

```text
阅读对应文档 -> 实现该文档中的函数 -> 运行单测 -> 通过后阅读下一篇
```

单测失败时，先根据失败测试名定位对应模块文档，不要凭猜测修改。

### 第四遍：阅读已提供的去畸变模块

在完成 5 个 TODO 之后、跑录包之前，阅读 `Part_of_Deskew.md`。该模块已提供完整实现，并在去畸变时调用 `interpolatePose`；理解它能帮助判断 TODO 1 对系统输出的影响。

### 第五遍：运行验证

阅读 `Part_of_Run.md`。先运行单测，再运行录包，在 Foxglove 中查看在线结果，最后检查 `poses.txt` 与 `mapping.pcd`。

---

## 文档之间的关系

```text
Part_of_Basics
      |
      v
Part_of_Env
      |
      v
Part_of_Overview
      |
      v
Part_of_Data
      |
      +--> Part_of_PoseInterpolation
      |          |
      |          v
      |    Part_of_Deskew（已提供，供阅读）
      |
      +--> Part_of_Covariance
      |
      +--> Part_of_Initial_Attitude
      |
      v
Part_of_Run
```

---

## 学习阶段参考

以下为阶段划分，仅供参考，可按实际情况调整：

```text
阶段 1：Basics + Env，完成环境搭建与编译
阶段 2：Overview + Data
阶段 3：PoseInterpolation + Covariance
阶段 4：Initial_Attitude
阶段 5：Deskew（阅读已提供实现）
阶段 6：录包回放、Foxglove 查看、打开地图、撰写报告
```

阶段 1 以环境搭建与前置概念理解为主。若需要建立基线，可按项目 README 第二步在占位实现状态下启动一次；此时只确认节点和 Foxglove 能启动，地图质量不作判断。占位实现下回放点云可能触发因子图 underconstrained 并退出；完整回放至少需要先完成 TODO 2、3，正式验收留到全部 TODO 与单测完成后。

---

## 遇到问题时的排查顺序

```text
1. source /opt/ros/jazzy/setup.bash && bash check_env.sh
2. 确认当前是在基线检查还是 TODO 实现阶段
3. 实现阶段运行 colcon test --packages-select small_glim
4. 根据失败测试名定位对应模块文档，一次只修改一个函数
5. 全部单测通过后运行正式录包验收
6. 录包运行异常时，参照 Part_of_Run 的排错表
```

占位实现时的基线启动只用于确认节点能启动；不要把该阶段的地图质量当作验收结果。实现阶段不要在单测未通过时进行正式建图验收。
