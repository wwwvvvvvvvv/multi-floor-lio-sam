# 基于 LIO-SAM 的多楼层机器人建图仿真项目 README

最近更新：**2026-09-09**。

## 当前开发里程碑

| 阶段 | 状态 | 内容 |
| --- | --- | --- |
| v0.1 | ✅ 完成 | 单机器人单楼层 Gazebo 仿真、LiDAR/IMU 适配、LIO-SAM 建图、回环验证与二维精度初评 |
| v0.2 | ✅ 完成 | LIO-SAM 地图保存、`GlobalMap.pcd` 生成、关机后地图保留、PCD 独立重新加载与 RViz 显示 |
| v0.3 | 🚧 下一阶段 | 基于已知 `GlobalMap.pcd` 的 NDT/GICP 定位与重定位 |

> **当前状态：单楼层 LIO-SAM 建图、回环验证、地图保存与离线加载功能验收通过，二维精度初评完成。**
>
> 已完成自动绕行、回环触发及参与优化的功能验证；二维 ATE RMSE 约 3.98 cm。
> 已通过 `/lio_sam/save_map` 保存完整单楼层 PCD 地图，并在关机重启后完成 `GlobalMap.pcd` 的独立重新发布与 RViz 显示验证。
> 当前精度结果仍使用近似时间戳，尚未完成严格同步的三维评估、重复实验及精度阈值验收；基于已有地图的 NDT/GICP 定位尚待开发。
> 实测数据见第 15.3～15.5 节，阶段验收结论见第 20 节。

## 1. 项目目标

本项目面向硕士毕业论文仿真，目标是构建一套基于 **ROS 2 Humble + Gazebo Fortress（Ignition Gazebo）+ LIO-SAM** 的多楼层移动机器人建图、定位与导航实验平台。

当前阶段聚焦最基础、最关键的一步：

> **单机器人、单楼层环境下稳定运行 LIO-SAM，完成三维建图、回环验证、地图保存与离线加载，并为后续已知地图定位建立基础。**

在此基础上，后续将逐步扩展到：

- 单楼层地图保存与重新加载（已完成基础功能验证）；
- 基于已有地图进行 NDT/GICP 重定位；
- 多楼层分别建图；
- 多机器人共享地图；
- 单楼层自主导航；
- 电梯状态机与跨楼层导航；
- 多机器人运输任务与调度。

## 2. 当前软件环境

- Ubuntu 22.04.5
- ROS 2 Humble
- Gazebo Fortress / Ignition Gazebo 6.x
- LIO-SAM ROS 2 版本
- GTSAM 4.1.1

> 注意：本项目使用的是 **Ignition Gazebo / Gazebo Fortress**，不是 Gazebo Classic。

## 3. 当前工作空间

### 3.1 仿真工作空间

```bash
~/multi_floor_ws
```

主要包含：

```text
multi_floor_sim
lio_sam_sim_adapter
```

其中：

- `multi_floor_sim`：Gazebo 场景、机器人模型、传感器、ROS-Gazebo Bridge、启动文件；
- `lio_sam_sim_adapter`：用于将 Gazebo 雷达点云转换为 LIO-SAM 所需要的点云格式。

### 3.2 LIO-SAM 工作空间

```bash
~/lio_sam_ws
```

LIO-SAM 源码目录：

```bash
~/lio_sam_ws/src/LIO-SAM
```

使用官方 LIO-SAM ROS 2 分支进行修改和编译。

## 4. 已完成的主要工作

### 4.1 建立单楼层 Gazebo 仿真环境

当前场景文件：

```bash
~/multi_floor_ws/src/multi_floor_sim/worlds/floor1.sdf
```

目前场景为一个单楼层室内环境，包含外围墙体、内部墙体以及可供机器人绕行的室内空间。

机器人名称：

```text
thesis_robot
```

机器人为四轮差速/滑移移动平台。

### 4.2 配置机器人传感器

3D LiDAR 安装于 `lidar_link`，模型相对高度约为：

```text
z = 0.34 m
```

主要参数：

```text
水平点数：1024
垂直线数：16
垂直视场：±15°
频率：10 Hz
量程：0.3 ~ 30 m
```

对应 ROS Topic：

```text
/lidar/points
```

IMU 安装于 chassis，模型相对高度约为：

```text
z = 0.20 m
```

对应 ROS Topic：

```text
/imu/data
```

Gazebo 中 IMU nominal 更新频率设置约为 200 Hz。

## 5. ROS-Gazebo Bridge

目前使用 `ros_gz_bridge` 将 Ignition Gazebo 数据桥接到 ROS 2。

主要桥接内容包括：

```text
/imu/data
/lidar/points
/cmd_vel
```

其中 `/cmd_vel` 已经可以由 ROS 2 向 Gazebo 机器人发送速度控制命令。

当前启动文件：

```bash
~/multi_floor_ws/src/multi_floor_sim/launch/floor1_system.launch.py
```

其中包含：Gazebo 仿真启动、ROS-Gazebo Bridge、LiDAR 点云转换节点以及 `base_link -> lidar_link` 静态 TF。

## 6. LiDAR 点云适配器

Gazebo 原始 `PointCloud2` 虽然包含 `x/y/z/intensity/ring` 等字段，但 LIO-SAM 还需要可靠的 `ring` 和每点相对时间 `time`。

因此编写：

```bash
lio_sam_sim_adapter/src/lidar_time_converter.cpp
```

用于生成：

```text
/lio_sam/points
```

最终点云字段为：

```text
x
y
z
intensity
ring
time
```

### 6.1 ring 生成方式

原始雷达点云：

```text
width = 1024
height = 16
```

在转换之前保存：

```cpp
const size_t lidar_width = msg->width;
```

再根据原始二维组织形式生成：

```cpp
ring = i / lidar_width;
```

因此最终得到：

```text
ring = 0 ~ 15
```

已经通过检查脚本确认 16 条线全部存在。

### 6.2 time 生成方式

由于当前 Gazebo 点云没有直接提供 LIO-SAM 所需的每点相对时间，因此仿真阶段按照列索引近似：

```cpp
col = i % lidar_width;
time = col / lidar_width * scan_period;
```

其中：

```text
scan_period = 0.1 s
```

即模拟 10 Hz LiDAR 一帧扫描过程中各个点的相对采样时间。

这一修改后，LIO-SAM 地图质量明显改善。

> 说明：这是当前仿真条件下的时间近似方案，后续论文中应明确说明。

## 7. LIO-SAM 参数配置

当前配置文件：

```bash
~/lio_sam_ws/src/LIO-SAM/config/params.yaml
```

核心参数：

```yaml
pointCloudTopic: "/lio_sam/points"
imuTopic: "/imu/data"

lidarFrame: "lidar_link"
baselinkFrame: "base_link"

odometryFrame: "odom"
mapFrame: "map"

sensor: velodyne

N_SCAN: 16
Horizon_SCAN: 1024
```

### 7.1 LiDAR 与 IMU 外参

LiDAR 高度约 0.34 m，IMU 高度约 0.20 m，因此当前配置：

```yaml
extrinsicTrans: [0.0, 0.0, -0.14]
extrinsicRot: identity
extrinsicRPY: identity
```

当前仿真中 LiDAR 与 IMU 坐标轴方向一致。

## 8. TF 树处理

期望 TF 结构：

```text
map
└── odom
    └── base_link
        └── lidar_link
```

当前 `base_link -> lidar_link` 通过静态 TF 发布：

```text
translation = [0, 0, 0.14]
rotation = identity
```

### 8.1 删除 LIO-SAM 自带 robot_state_publisher

在：

```bash
~/lio_sam_ws/src/LIO-SAM/launch/run.launch.py
```

中删除/注释了 LIO-SAM 自带的 `robot_state_publisher`，避免与仿真机器人 TF 冲突。

### 8.2 关闭重复 odom -> lidar_link TF

在：

```bash
~/lio_sam_ws/src/LIO-SAM/src/mapOptmization.cpp
```

中将：

```cpp
br->sendTransform(trans_odom_to_lidar);
```

注释为：

```cpp
// br->sendTransform(trans_odom_to_lidar);
```

原因是 LIO-SAM 的 TransformFusion 已经负责 `odom -> base_link`，同时仿真中已有 `base_link -> lidar_link`。若再发布 `odom -> lidar_link`，会造成 `lidar_link` 多父节点的 TF 问题。

## 9. IMU 与 LiDAR 时间同步处理

之前出现过：

```text
Waiting for IMU data...
```

以及扫描时间与 IMU 数据覆盖范围不一致的问题。

因此修改：

```bash
~/lio_sam_ws/src/LIO-SAM/src/imageProjection.cpp
```

将点云缓存条件：

```cpp
cloudQueue.size() <= 2
```

改为：

```cpp
cloudQueue.size() <= 5
```

通过增加点云处理延迟，使 IMU 队列更容易覆盖整帧 LiDAR 扫描时间。

同时加入了调试输出，用于观察 scan time range 与 IMU time range。

## 10. 曾解决的主要问题

### 10.1 LIO-SAM 地图严重“炸图”

早期现象包括墙体扇形扩散、地图旋转和点云严重错位。

主要处理：

- 修正 LiDAR `time`；
- 修正 `ring`；
- 检查 IMU/LiDAR 时间覆盖；
- 修正 LiDAR-IMU 外参；
- 修正 TF 树。

目前已经基本消除严重发散。

### 10.2 重复 ros_gz_bridge

曾出现：

```text
/imu/data
Publisher count: 2
```

并引发 IMU 重复、时间戳异常、`dt <= 0` 以及 GTSAM/LIO-SAM 异常。

通过清理重复 bridge 并只启动一个 `floor1_system.launch.py` 解决。

### 10.3 ROS 2 daemon 异常

曾出现 ROS 2 daemon / XMLRPC 错误，可通过：

```bash
ros2 daemon stop
ros2 daemon start
```

恢复。

### 10.4 `/cmd_vel` Bridge 字符串拼接错误

在 launch 中加入 `/cmd_vel` 时曾漏写逗号，导致 Python 将两个字符串拼接，破坏 LiDAR Bridge。

最终正确格式：

```python
'/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
'/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
'/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
```

## 11. 当前固定启动方式

### 终端 1：Gazebo + Bridge + 点云适配器

```bash
source /opt/ros/humble/setup.bash
source ~/multi_floor_ws/install/setup.bash
ros2 launch multi_floor_sim floor1_system.launch.py
```

### 终端 2：LIO-SAM + RViz

```bash
source /opt/ros/humble/setup.bash
source ~/lio_sam_ws/install/setup.bash
ros2 launch lio_sam run.launch.py
```

### 终端 3：自动绕行测试或键盘控制（二选一）

验证回环时，推荐使用已实测的自动绕行脚本。待终端 1、2 启动完成，
机器人位于初始世界原点附近且键盘控制已关闭后运行：

```bash
source /opt/ros/humble/setup.bash
cd ~/multi_floor_ws
python3 scripts/auto_loop_test.py
```

脚本会自动驾驶、监测回环、停车并保存结果，完整说明见第 15.2 节。
需要手动探索时，改用以下键盘控制；不要与自动驾驶同时运行：

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

常用控制：

```text
i    前进
,    后退
j    左转
l    右转
k    停止
```

## 12. 编译方式

仿真工作空间修改后：

```bash
cd ~/multi_floor_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

LIO-SAM 修改后：

```bash
cd ~/lio_sam_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## 13. 当前建图效果

目前单楼层移动建图已经达到：

- LIO-SAM 能稳定启动；
- LiDAR 数据能够正常进入 LIO-SAM；
- IMU 数据能够正常使用；
- `ring = 0 ~ 15` 正常；
- 点云不会出现明显发散；
- 房间主体轮廓能够保持；
- 墙体存在少量厚度/重影，但整体可接受；
- 静止时点云基本稳定。

当前阶段可以认为：

> **单机器人单楼层 LIO-SAM 基础建图链路已经跑通。**

## 14. 已验证的 LIO-SAM Topics

目前主要 Topic：

```text
/lio_sam/imu/path
/lio_sam/mapping/cloud_registered
/lio_sam/mapping/cloud_registered_raw
/lio_sam/mapping/map_global
/lio_sam/mapping/map_local
/lio_sam/mapping/odometry
/lio_sam/mapping/odometry_incremental
/lio_sam/mapping/path
/lio_sam/mapping/trajectory
/lio_sam/mapping/icp_loop_closure_history_cloud
/lio_sam/mapping/loop_closure_constraints
```

## 15. 闭环检测验证

目前已经进行了第一次人工闭环测试：

1. 使用键盘控制机器人在房间内运动；
2. 绕场景行驶并重新靠近已经扫描过的区域；
3. LIO-SAM 地图保持稳定；
4. `/lio_sam/mapping/loop_closure_constraints` 成功输出数据。

输出中已经出现：

```text
ns: loop_edges
type: 5
```

同时包含大量闭环连线点对。

第一次人工测试确认了回环约束生成；结合第 15.3 节后续自动测试，当前验证状态为：

```text
闭环检测功能：已触发
闭环约束生成：正常
回环参与优化：已观察到历史位姿更新，结合源码及无 GPS 输入的运行条件予以验证
```

也就是说：

> **LIO-SAM 已经能够实际生成闭环约束。**

### 15.1 回环触发与参与优化的复核步骤

源码中，`performLoopClosure()` 在 ICP 收敛且分数通过后将约束放入队列，
随后发布 `loop_edges`。`saveKeyFramesAndFactor()` 只有在产生新关键帧后，
才调用 `addLoopFactor()` 并更新 iSAM；`correctPoses()` 再更新历史位姿。
因此，**非空回环连线证明约束已生成，不能单独证明约束已经参与优化**。
历史点云 Topic 有输出也不能证明 ICP 通过，因为它在 ICP 检查之前发布。

当前参数为：回环开启、检测频率 1 Hz、候选半径 15 m、历史时间差超过
30 s、ICP 分数不高于 0.3。新关键帧门槛为平移约 1 m 或转动约 0.2 rad。
这些是当前配置值，运行时可核对实际加载值：

```bash
source /opt/ros/humble/setup.bash
ros2 param get /lio_sam_mapOptimization loopClosureEnableFlag
ros2 param get /lio_sam_mapOptimization historyKeyframeSearchTimeDiff
ros2 param get /lio_sam_mapOptimization historyKeyframeSearchRadius
ros2 param get /lio_sam_mapOptimization historyKeyframeFitnessScore
ros2 param get /lio_sam_mapOptimization gpsTopic
```

按第 11 节启动一套新的 Gazebo 和 LIO-SAM，在开始行驶之前另开终端运行：

```bash
source /opt/ros/humble/setup.bash
cd ~/multi_floor_ws
python3 scripts/verify_loop_closure.py
```

该脚本只订阅话题，不控制机器人或修改参数。默认同时监测
`/odometry/gpsz`，如运行时 `gpsTopic` 不同，使用 `--gps-topic` 指定实际话题。

1. 在可通行区域缓慢绕行，重新经过起点附近已扫描区域，尽量保持点云重叠。
   与历史帧的传感器时间差必须超过 30 秒；仿真较慢时，墙钟等待 30 秒不一定够。
2. 观察 RViz 已配置的 MarkerArray：
   `/lio_sam/mapping/loop_closure_constraints`，黄色连线对应回环边。
   脚本会报告非空连线的数量，而非仅检查 `ns` 和 `type` 字段。
3. 出现连线后，在安全空地继续缓慢移动约 1～2 m，或转动超过约 12°，
   使其产生新关键帧，然后等待数秒。
4. 观察脚本是否报告“历史位姿更新”。脚本按关键帧时间戳比较同一历史帧的
   位置和姿态；只追加新轨迹点不会被算作历史更新。
5. 用 Ctrl+C 结束脚本，查看打印出的结果目录。

结果保存在 `results/loop_年月日_时分秒/`：

- `events.jsonl`：回环边数量、历史位姿修改量及监测状态。
- `summary.json`：本轮证据结论。
- `path_before_first_revision.json` / `path_after_first_revision.json`：
  首次观察到历史修改前后的轨迹（有修改才生成）。
- `path_final.json`：监测结束时的轨迹。

在当前源码、单次独立运行且无 GPS 输入的条件下，同时观察到非空回环约束和
历史位姿更新，支持“回环参与优化”的判断。脚本会检查轨迹重置、多发布者和
监测期间的 GPS 消息；请从本轮行驶之前开始记录，避免漏掉先前事件。
这是话题级证据，并非优化器内部日志，也不是 Ground Truth 精度评价。
历史修改量低于 0.00001 m / 0.00001 rad 时脚本不报告修改，因此没有观察到修改
不能直接判定优化失败；这种情况下需进一步记录 `addLoopFactor()` 和 iSAM 更新日志。

本轮实际自动验证结果见第 15.3 节。

### 15.2 自动绕行测试（无需键盘驾驶）

按第 11 节先启动 Gazebo 和 LIO-SAM，机器人应位于初始世界原点附近。
关闭 `teleop_twist_keyboard`，然后运行：

```bash
source /opt/ros/humble/setup.bash
cd ~/multi_floor_ws
python3 scripts/auto_loop_test.py
```

脚本自带第 15.1 节的回环监测，无需另开监测脚本，也无需重新编译。
它会自动启动专用位姿/时钟桥接，读取 Gazebo 已有的
`/world/floor1/dynamic_pose/info` 和 `/world/floor1/clock`。
模型真实位姿用于路线反馈控制，只发布到 `/loop_test/ground_truth`，
不会写入 `/tf` 或 LIO-SAM 的输入。按模型名称选择位姿，避免将相对坐标的
link 位姿误当成机器人世界位姿。

默认世界坐标路线为：

```text
(0,0) → (5.2,0) → (5.2,2.7) → (-1.2,2.7) → (-1.2,0) → (0,0)
                                                                    ↓
                                            等待 → (1.6,0) → (0,0) → 停车
```

路线绕过 `inner_wall_1`，最大前进速度 0.3 m/s、最大转速 0.45 rad/s。
首次返回后至少等待 5 秒仿真时间，且从本轮开始至少经过 35 秒仿真时间，
随后补走往返段，以便异步生成的回环约束由后续关键帧消费。
最终位置容差为 0.12 m，航向为世界坐标 0 rad，容差 0.06 rad。
实际用时取决于仿真实时率和四轮滑移，不按固定墙钟时间盲走。

可先执行不发布速度指令的预检查：

```bash
python3 scripts/auto_loop_test.py --dry-run
```

运行中按 Ctrl+C 会发送停车指令并保存已有结果。位姿断流、仿真暂停、
LIO-SAM 轨迹断流、控制发布者冲突、接近墙体、路线偏离或长期无运动进展，
也会结束驾驶并停车。默认总超时为 1800 秒墙钟时间。
该固定路线只适用于当前 `floor1.sdf` 静态环境，不是通用避障或导航模块；
调整墙体或添加障碍后需要重新规划。

输出目录为 `results/auto_loop_年月日_时分秒/`，除第 15.1 节文件外还包含：

- `drive_summary.json`：路线是否完成、实际首尾位姿和真实返回偏差。
- `ground_truth.csv`：控制使用的世界坐标 x/y/yaw；时间列为接收墙钟时间和
  最近收到的仿真时钟，非严格同步的 ATE/RPE 评估数据。
- `route.json`：路线、速度和场景检查结果。
- `bridge.log`：测试专用桥接日志。

**路线完成与回环验证是两个结果**：`drive_summary.json` 的 `route_completed`
表示机器人已走完路线；回环是否接受以及历史轨迹是否被修改，请看 `summary.json`。
默认回环参数的时间差为 30 秒，如修改过该参数，需同步调整脚本的等待策略。

离线回归检查：

```bash
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s scripts -p 'test_auto_loop_test.py' -v
```

### 15.3 2026-09-09 自动绕行实测结果

结果目录：`results/auto_loop_20260909_160508/`。
已在 Gazebo Fortress + 当前 LIO-SAM 配置下完成实际试跑，全部 7 个航点到达，
最后自动停车。四项离线回归测试及实际 ROS 输入预检查均通过。

| 项目 | 本轮结果 |
| --- | --- |
| 自动路线 | 完成，包括回原点后的补走往返段 |
| 用时 | 约 143.8 秒墙钟时间，126.6 秒仿真时间（从记录起点到结束） |
| 接受的回环边 | 51 条 |
| 观察到历史位姿更新 | 50 次 |
| GPS 输入消息 | 0 |
| 轨迹重置 / 重复轨迹发布者 | 未观察到 |
| 真实最终位置 | x = 0.1032 m，y = -0.0193 m |
| 真实首尾位置偏差 | 0.1050 m，在 0.12 m 到点容差内 |
| 真实首尾航向偏差 | -0.0562 rad，约 -3.22° |

结论：**自动绕行和停车已跑通；观察到回环约束及历史位姿修改，支持回环参与优化。**
51 条边与 50 次历史修改不是一一对应的计数：边是累计接受的约束，历史修改是
话题层观察到的更新事件，不能据此断言每一条约束都已单独完成优化。
当前候选半径为 15 m，因此绕行过程中也会产生回环，并非只在最终回原点时产生。
上述真实首尾偏差属于驾驶控制的返回偏差，不是 LIO-SAM 的定位误差或 ATE/RPE。

实际加载参数另存为 `active_lio_parameters.yaml`，轨迹及回环事件图为：

![自动绕行轨迹与回环证据](results/auto_loop_20260909_160508/route_and_evidence.png)

### 15.4 本轮二维 ATE / RPE 初步计算

使用第 15.3 节保存的 `path_final.json`（最终优化关键帧）和 `ground_truth.csv`，
在重叠时间段 106.0～220.0 秒内匹配 65 个关键帧。早于 GT 记录的首个关键帧不外推，
予以排除。XY 位置经过 SE(2) 旋转和平移对齐，尺度固定为 1。

| 指标 | RMSE | 最大值 |
| --- | --- | --- |
| 二维 ATE，对齐后 | 3.98 cm | 7.88 cm |
| 未对齐 XY 位置差 | 4.87 cm | 9.43 cm |
| 相邻关键帧 RPE 平移 | 2.22 cm | 6.02 cm |
| 相邻关键帧 RPE 航向 | 0.485° | 1.746° |

RPE 使用 64 对相邻关键帧，时间间隔为 0.5～10.0 秒，中位数 0.7 秒；
**不是固定 1 秒 RPE，也不是每米误差**。这些指标描述最终优化轨迹，
没有关闭回环的对照，不能据此量化回环带来的精度提升。

**本次属于二维、近似时间戳的初步评估**：GT 只保存了 x/y/yaw，其时间列是最近
收到的独立仿真时钟，不是位姿消息原始时间戳。尚不能作为严格同步的完整三维
ATE/RPE 结果，也不能直接与不同采样或不同对齐方法的结果比较。

详细方法、逐帧误差、时间偏移敏感性与误差图见
[评估报告](results/auto_loop_20260909_160508/evaluation_2d/report.md)。
计算脚本的六项回归测试已通过，包括与完整齐次变换矩阵的 RPE 公式交叉检查。

复算命令（不需要 ROS 节点运行；依赖 NumPy，绘图使用 Matplotlib）：

```bash
cd ~/multi_floor_ws
python3 scripts/evaluate_trajectory_2d.py results/auto_loop_20260909_160508
```

### 15.5 2026-09-09 地图保存与离线加载验证

在完成单楼层建图后，通过 LIO-SAM 自带服务保存当前地图：

```bash
source /opt/ros/humble/setup.bash
source ~/lio_sam_ws/install/setup.bash

ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
"{resolution: 0.0, destination: '/lio_sam_maps/floor1'}"
```

本轮服务返回：

```text
success=True
```

保存目录：

```text
~/lio_sam_maps/floor1
```

生成文件及规模如下：

| 文件 | 本轮结果 |
| --- | ---: |
| `CornerMap.pcd` | 13,723 points |
| `SurfMap.pcd` | 152,184 points |
| `GlobalMap.pcd` | 165,907 points |
| `trajectory.pcd` | 162 points |
| `transformations.pcd` | 162 points |

其中 `GlobalMap.pcd` 文件大小约 2.6 MB，`trajectory.pcd` 与 `transformations.pcd` 均包含 162 个关键帧记录，说明保存结果来自完整运动建图过程，而不是仅保存单帧初始点云。

为验证地图是否能够脱离建图过程独立复用，关机重启后确认上述 PCD 文件仍然存在，并使用 ROS 2 Humble 中的 `pcl_ros pcd_to_pointcloud` 重新发布：

```bash
source /opt/ros/humble/setup.bash

ros2 run pcl_ros pcd_to_pointcloud \
--ros-args \
-p file_name:=/home/yez/lio_sam_maps/floor1/GlobalMap.pcd \
-p publish_rate:=10.0
```

当前环境中输出 Topic 为：

```text
/cloud_pcd
```

重新发布后的 `PointCloud2`：

```text
width = 165907
fields = x, y, z, intensity
```

与保存时 `GlobalMap.pcd` 的点数一致，说明 PCD 内容能够被正常读取和重新发布。

需要注意：当前 `pcl_ros` 节点发布的点云 `frame_id` 为 `/base_link`。为了完成 **仅用于离线可视化验证** 的 RViz 显示，本轮临时发布零位姿静态 TF：

```bash
ros2 run tf2_ros static_transform_publisher \
0 0 0 0 0 0 map base_link
```

随后在 RViz 中设置：

```text
Fixed Frame = map
PointCloud2 Topic = /cloud_pcd
```

完整单楼层三维地图能够正常显示。

> **该零位姿静态 TF 仅用于验证保存地图可重新显示，不能作为后续定位系统的正式 `map -> base_link` 变换。** 后续 NDT/GICP 定位模块应根据实时匹配结果估计机器人在已知地图中的位姿，并建立规范的 `map -> odom -> base_link -> lidar_link` TF 链。

因此，本阶段已经验证：

```text
LIO-SAM 建图
    ↓
save_map
    ↓
GlobalMap.pcd
    ↓
关机 / 重启
    ↓
PCD 独立重新发布
    ↓
RViz 离线显示
```

即 **单楼层三维地图的保存与离线重新加载功能已经跑通**。该结果证明地图文件可以作为后续 NDT/GICP 已知地图定位、多楼层地图管理和多机器人共享地图的基础数据，但尚未完成“定位模块实际加载地图并输出机器人位姿”的验证。

## 16. 当前闭环测试的不足

第一次人工测试没有记录真实起点，只能验证是否生成回环约束。
后续自动测试已补上真实轨迹记录、自动返回、历史位姿更新观测及二维 ATE/RPE 初评，
当前不足主要是：

- **时间关联仍为近似**：GT 使用最近收到的独立仿真时钟，未保留位姿原始时间戳。
- **评估维度不完整**：已保存的 GT 为 x/y/yaw，缺少 z/roll/pitch，尚未计算严格三维指标。
- **采样不统一**：使用最终优化关键帧，相邻帧间隔为 0.5～10 秒；当前 RPE 不是固定 1 秒指标。
- **实验次数与对照不足**：仅完成一次自动路线实测，尚无多次重复实验或同路线关闭回环的对照。
- **未预设精度验收阈值**：不能用本轮结果反过来定义合格线；应先结合论文目标制定指标和阈值。

因此，当前可确认功能验收通过、二维精度初评完成，尚不能声称论文最终精度验收通过。
真实返回起点的 10.5 cm 偏差属于驾驶控制结果，不能当作 LIO-SAM 定位误差。

## 17. 下一步工作

自动绕行、起终点真实位姿记录、回环功能验证和二维精度初评已完成。
下一阶段建议按以下顺序推进：

1. 明确论文所需的精度指标、时间/距离采样间隔、对齐方法及验收阈值；
2. 统一 ROS 仿真时间，并同步记录原始时间戳、完整六自由度 GT 和估计轨迹；
3. 在同一路线重复实验，并增加开启／关闭回环的对照；
4. 计算严格同步的三维 ATE、固定间隔 RPE，报告重复实验统计和回环带来的变化；
5. **已完成** LIO-SAM 三维地图保存验证；
6. **已完成** 关机重启后 `GlobalMap.pcd` 的独立重新发布与 RViz 显示验证；
7. 开发基于已有 `GlobalMap.pcd` 的 NDT/GICP 定位与重定位模块；
8. 验证不同初始位姿偏差下的重定位成功率、收敛时间和定位误差；
9. 单楼层定位稳定后，再进入多楼层地图管理、电梯拓扑和多机器人扩展。

## 18. 后续论文总体路线

```text
Gazebo 多楼层建筑
        ↓
3D LiDAR + IMU
        ↓
LIO-SAM
        ↓
各楼层独立三维建图
        ↓
保存各楼层 3D Point Cloud
        ↓
3D 地图重定位（NDT / GICP）
        ↓
提取二维导航地图
        ↓
单楼层导航
        ↓
电梯拓扑连接楼层
        ↓
电梯 FSM
        ↓
多楼层自主导航
        ↓
多机器人共享地图
        ↓
任务分配与路径协调
```

其中 LIO-SAM 主要承担 **建图与建图阶段里程计估计**。

后续机器人在已有地图中运行时，应将“建图”和“定位 / 重定位”分开处理。

## 19. 当前未完成但需要后续处理的问题

### 19.1 `/clock`

自动测试期间已将 `/world/floor1/clock` 桥接到专用话题 `/loop_test/clock`，
用于测试等待和记录。本轮实际加载的 LIO-SAM 参数仍为 `use_sim_time: false`。
尚未将全系统统一为 ROS `/clock` 和：

```text
use_sim_time = true
```

目前 LIO-SAM 可以正常运行，但在后续进行论文级时间误差分析、Ground Truth 对齐时应统一仿真时间。

### 19.2 Ground Truth

自动测试已建立专用 Ground Truth 话题 `/loop_test/ground_truth`，
读取 Gazebo `thesis_robot` 的世界位姿，测试结束后关闭该专用桥接。
本轮已保存 x/y/yaw 供路线控制和二维初评使用。

下一步需保存位姿原始时间戳和完整六自由度姿态，用于严格同步的三维精度评价。
Ground Truth 只用于仿真测试控制和算法评价，不能直接提供给 LIO-SAM 作为定位输入。

### 19.3 地图保存与加载

已完成 LIO-SAM `save_map` 与保存地图离线重新加载验证。

当前正式保存目录：

```text
~/lio_sam_maps/floor1
```

其中主要地图文件：

```text
GlobalMap.pcd          165907 points
SurfMap.pcd            152184 points
CornerMap.pcd           13723 points
trajectory.pcd            162 points
transformations.pcd       162 points
```

关机重启后，`GlobalMap.pcd` 已通过 `pcl_ros pcd_to_pointcloud` 独立发布到 `/cloud_pcd` 并在 RViz 中成功显示，证明保存地图文件可被重复读取和使用。

当前“地图加载”仅完成 **离线读取与可视化验证**。下一阶段需要让 NDT/GICP 定位模块实际加载该地图，并根据实时 LiDAR 扫描计算机器人位姿。

### 19.4 定位模块

建图阶段仍使用 LIO-SAM Mapping；保存地图的离线重新加载已经验证，但尚未形成独立的实时定位节点。

后续需要开发：

```text
Saved Map
   +
Current LiDAR Scan
   ↓
NDT / GICP
   ↓
Global Localization / Relocalization
```

## 20. 当前阶段结论

截至目前，本项目已经完成了从零开始搭建：

```text
Gazebo
  ↓
3D LiDAR + IMU
  ↓
ROS-Gazebo Bridge
  ↓
PointCloud Adapter
  ↓
LIO-SAM
  ↓
三维地图 + 闭环优化
  ↓
save_map
  ↓
GlobalMap.pcd
  ↓
离线重新加载与 RViz 显示
```

整条基础链路。

当前阶段验收结论为：

> **单楼层基础建图、回环、地图保存与离线加载功能验收通过，二维精度初评完成。**

| 验收项 | 当前结论 | 依据或限制 |
| --- | --- | --- |
| 基础建图链路 | 通过阶段性功能验收 | LIO-SAM 稳定运行，单楼层地图无明显发散 |
| 自动绕行与停车 | 通过 | 7 个航点完成并返回起点附近，最后自动停车 |
| 回环触发与参与优化 | 通过功能验证 | 51 条回环约束、50 次历史位姿更新，无 GPS 输入或轨迹重置记录 |
| 二维轨迹精度 | 初评完成 | ATE RMSE 3.98 cm；相邻关键帧 RPE 平移 RMSE 2.22 cm、航向 RMSE 0.485° |
| 论文最终精度验收 | 尚未完成 | 缺少严格同步三维数据、预设阈值、重复实验及回环开关对照 |
| 三维地图保存 | 通过 | `save_map` 成功生成完整 PCD 地图，`GlobalMap.pcd` 为 165,907 points |
| 地图离线重新加载 | 通过 | 关机重启后 PCD 可独立发布并在 RViz 中显示 |
| 已有地图定位 / 重定位 | 待开发 | NDT/GICP 尚未实现，见第 19.4 节 |

上述“通过”适用于当前单机器人、单楼层仿真的功能阶段；
二维精度结果是初步测量，不能直接据此宣称达到论文最终精度要求。

下一阶段不建议立即进入多楼层，而应先完成：

> **严格同步的定量评估与对照实验 → NDT/GICP 已知地图定位 → 单楼层导航 → 多楼层扩展**

将单楼层基础功能彻底做扎实后，再扩展到多楼层与多机器人。

## 21. 维护建议

后续每完成一个阶段，建议持续更新本 README，至少记录：

```text
实验日期
修改文件
修改参数
实验环境
测试路线
测试结果
地图文件及点数
Git commit / tag
发现的问题
解决方法
```

这样在多楼层、多机器人阶段修改较多以后，仍然可以追溯当前稳定版本。
