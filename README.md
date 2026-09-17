最近更新：**2026-09-17**。

## 当前开发里程碑

  ---------------------------------------------------------------------------------------------------------------------------
  阶段                    状态                    内容
  ----------------------- ----------------------- ---------------------------------------------------------------------------
  v0.1                    ✅ 完成                 单机器人单楼层 Gazebo 仿真、LiDAR/IMU 适配、LIO-SAM
                                                  建图、回环验证与二维精度初评

  v0.2                    ✅ 完成                 LIO-SAM 地图保存、`GlobalMap.pcd` 生成、关机后地图保留、PCD 独立重新加载与
                                                  RViz 显示

  v0.3                    ✅ 完成                 基于已知 `GlobalMap.pcd` 的 NDT 定位、`/initialpose` 初始位姿设置与重定位

  v0.3.5                  ✅ 完成                 引入 odometry motion prior 与 quality gate，解决转弯场景下 NDT
                                                  错误局部收敛和连续发散问题

  v0.3.7                  ✅ 完成                 建立标准 `map -> odom -> base_link -> lidar_link` TF
                                                  架构，并完成静止、直行及原地转弯动态验证

  v0.4                    ✅ 完成                 从 `GlobalMap.pcd` 生成二维 OccupancyGrid，完成 Nav2
                                                  `map_server`、全局/局部 costmap、BT Navigator、DWB 与速度平滑接入

  v0.4.1                  ✅ 完成                 修复 `/clock`、NDT quality gate 锁死及 `map -> odom` 旧时间戳问题；TF
                                                  高频刷新约 50 Hz；完成单楼层 Nav2 控制链功能验证

  v0.5                    ✅ 完成                 Nav2 终端姿态优化、固定起终点五次重复实验与单机器人导航初步定量评价

  v0.6                    ✅ 完成                 多楼层地图管理核心完成；Floor1 → Floor2 → Floor1 动态换图与重定位联调通过；
                                                  真实 Floor2、电梯 FSM 与完整跨楼层自主导航尚未完成

  后续                    ⏳ 规划中               多机器人共享地图、任务分配、路径协调、电梯预约与延迟传播调度
  ---------------------------------------------------------------------------------------------------------------------------

> **当前状态：单楼层 LIO-SAM 建图、地图保存、NDT 已知地图定位、标准
> TF、二维导航地图与 Nav2 单机器人导航链路均已完成阶段性功能验证。**
>
> 当前系统使用已保存的 `GlobalMap.pcd` 进行 NDT 已知地图定位，采用
> odometry motion prior 与 quality gate 保证连续运动稳定性；NDT
> 接受结果用于更新 `T_map_odom`，并由 `/odom` 回调以约 50 Hz 刷新
> `map -> odom` 的时间戳，避免低频 NDT 计算造成 TF
> 过期。二维导航层已由三维点云投影生成，Nav2 已接入 global planner、DWB
> controller、velocity smoother 与 BT Navigator。 建图阶段二维 ATE RMSE
> 初评约 3.98 cm；Nav2 已完成一个固定起点/固定目标的五次终端精度重复实验，
> 当前配置下定位 TF yaw 误差约 2.8°、Gazebo 真值 yaw 误差约 2.7°。
> v0.6 已完成多楼层地图动态切换与重定位核心：`/current_floor` 可依次驱动 Nav2
> 二维地图、NDT 三维地图和对应楼层 `/initialpose` 切换，并在连续 2 次
> `NDT ACCEPT` 后进入 `FLOOR READY`。真实 Floor2、电梯 FSM、进出梯控制和完整跨楼层
> 自主导航尚未完成。多目标、多路线、障碍场景、严格同步三维 ATE/RPE 及最终论文级验收仍需继续完善。

## 1. 项目目标

本项目面向硕士毕业论文仿真，目标是构建一套基于 **ROS 2 Humble + Gazebo
Fortress（Ignition Gazebo）+ LIO-SAM**
的多楼层移动机器人建图、定位与导航实验平台。

当前已经完成单楼层从建图、定位到导航控制的基础闭环链路：

> **LIO-SAM 三维建图 -\> 地图保存 -\> NDT 已知地图定位 / 重定位 -\> 标准
> TF -\> 二维导航地图 -\> Nav2 -\> DWB -\> `/cmd_vel`。**

当前定位系统采用已保存的 `GlobalMap.pcd` 作为全局点云地图，利用当前
LiDAR 扫描进行 NDT 配准；同时引入 odometry motion prior 与 quality
gate，提高连续运动和转弯场景下的稳定性。导航阶段不再使用 AMCL，而是由
NDT 提供全局定位修正，由 Nav2 完成路径规划与局部控制。

后续将重点扩展到：

-   Nav2 多目标、多路线与障碍场景的系统化参数评价；
-   严格同步 Ground Truth、完整三维 ATE/RPE 与消融/基线实验；
-   制作并验证真实、独立的 Floor2 三维与二维地图；
-   电梯拓扑、楼层切换与跨楼层导航状态机；
-   多机器人共享地图；
-   多机器人任务分配与路径协调；
-   电梯预约、延迟传播与在线局部修复。

## 2. 当前软件环境

-   Ubuntu 22.04.5
-   ROS 2 Humble
-   Gazebo Fortress / Ignition Gazebo 6.x
-   LIO-SAM ROS 2 版本
-   GTSAM 4.1.1
-   Nav2（ROS 2 Humble）
-   `pointcloud_to_laserscan`

> 注意：本项目使用的是 **Ignition Gazebo / Gazebo Fortress**，不是
> Gazebo Classic。

## 3. 当前工作空间

### 3.1 仿真工作空间

``` bash
~/multi_floor_ws
```

主要包含：

``` text
multi_floor_sim
lio_sam_sim_adapter
known_map_localization
```

其中：

-   `multi_floor_sim`：Gazebo 场景、机器人模型、传感器、ROS-Gazebo
    Bridge、Nav2 参数、二维导航地图接入与一键启动文件；
-   `lio_sam_sim_adapter`：用于将 Gazebo 雷达点云转换为 LIO-SAM
    所需要的点云格式；
-   `known_map_localization`：负责已知地图发布、NDT 定位、`/initialpose`
    重定位、odom motion prior、quality gate、标准 TF 广播及 Ground Truth
    评价。

### 3.2 LIO-SAM 源码现状

历史建图实验使用的是官方 LIO-SAM ROS 2 分支的修改版。当前主目录中已经没有
可直接编译的 `~/lio_sam_ws`，因此本文中涉及 `lio_sam` 建图、回环和
`save_map` 的命令属于历史实验复现说明，在恢复完整 LIO-SAM 工作空间前不能直接执行。

目前保留的修改文件快照位于：

``` bash
~/multi-floor-lio-sam/lio_sam_modified
```

其中保存了 `params.yaml`、`run.launch.py`、`imageProjection.cpp` 和
`mapOptmization.cpp` 的项目修改版本，但该目录不是完整的 LIO-SAM 包，不能单独
`colcon build`。当前已知地图 NDT + Nav2 运行链路不依赖重新启动 LIO-SAM Mapping，
使用保存好的 `GlobalMap.pcd` 和 `lio_sam_sim_adapter` 输出的
`/lio_sam/points` 即可运行。

## 4. 已完成的主要工作

### 4.1 建立单楼层 Gazebo 仿真环境

当前场景文件：

``` bash
~/multi_floor_ws/src/multi_floor_sim/worlds/floor1.sdf
```

目前场景为一个单楼层室内环境，包含外围墙体、内部墙体以及可供机器人绕行的室内空间。

机器人名称：

``` text
thesis_robot
```

机器人为四轮差速/滑移移动平台。

### 4.2 配置机器人传感器

3D LiDAR 安装于 `lidar_link`，模型相对高度约为：

``` text
z = 0.34 m
```

主要参数：

``` text
水平点数：1024
垂直线数：16
垂直视场：±15°
频率：10 Hz
量程：0.3 ~ 30 m
```

对应 ROS Topic：

``` text
/lidar/points
```

IMU 安装于 chassis，模型相对高度约为：

``` text
z = 0.20 m
```

对应 ROS Topic：

``` text
/imu/data
```

Gazebo 中 IMU nominal 更新频率设置约为 200 Hz。

## 5. ROS-Gazebo Bridge

目前使用 `ros_gz_bridge` 将 Ignition Gazebo 数据桥接到 ROS 2。

主要桥接内容包括：

``` text
/imu/data
/lidar/points
/cmd_vel
/odom
```

其中 `/cmd_vel` 已经可以由 ROS 2 向 Gazebo 机器人发送速度控制命令。Nav2
集成运行时还通过 `floor1_nav_all.launch.py` 额外桥接
`/clock`，并统一关键节点使用 `use_sim_time:=true`。

当前启动文件：

``` bash
~/multi_floor_ws/src/multi_floor_sim/launch/floor1_system.launch.py
```

其中包含：Gazebo 仿真启动、ROS-Gazebo Bridge、LiDAR 点云转换节点以及
`base_link -> lidar_link` 静态 TF。

## 6. LiDAR 点云适配器

Gazebo 原始 `PointCloud2` 虽然包含 `x/y/z/intensity/ring` 等字段，但
LIO-SAM 还需要可靠的 `ring` 和每点相对时间 `time`。

因此编写：

``` bash
lio_sam_sim_adapter/src/lidar_time_converter.cpp
```

用于生成：

``` text
/lio_sam/points
```

最终点云字段为：

``` text
x
y
z
intensity
ring
time
```

### 6.1 ring 生成方式

原始雷达点云：

``` text
width = 1024
height = 16
```

在转换之前保存：

``` cpp
const size_t lidar_width = msg->width;
```

再根据原始二维组织形式生成：

``` cpp
ring = i / lidar_width;
```

因此最终得到：

``` text
ring = 0 ~ 15
```

已经通过检查脚本确认 16 条线全部存在。

### 6.2 time 生成方式

由于当前 Gazebo 点云没有直接提供 LIO-SAM
所需的每点相对时间，因此仿真阶段按照列索引近似：

``` cpp
col = i % lidar_width;
time = col / lidar_width * scan_period;
```

其中：

``` text
scan_period = 0.1 s
```

即模拟 10 Hz LiDAR 一帧扫描过程中各个点的相对采样时间。

这一修改后，LIO-SAM 地图质量明显改善。

> 说明：这是当前仿真条件下的时间近似方案，后续论文中应明确说明。

## 7. LIO-SAM 参数配置

当前配置文件：

``` bash
~/multi-floor-lio-sam/lio_sam_modified/config/params.yaml
```

核心参数：

``` yaml
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

``` yaml
extrinsicTrans: [0.0, 0.0, -0.14]
extrinsicRot: identity
extrinsicRPY: identity
```

当前仿真中 LiDAR 与 IMU 坐标轴方向一致。

## 8. TF 树处理

当前定位阶段采用标准移动机器人 TF 结构：

``` text
map
└── odom
    └── base_link
        └── lidar_link
```

各级 TF 分工如下：

-   `map -> odom`：NDT 在接受匹配结果后更新最新 `T_map_odom`
    修正值；在两次 NDT 之间，由 `/odom` 回调以约 50 Hz 使用当前 odometry
    时间戳刷新同一修正值，负责全局纠偏并保证 TF 时间新鲜；
-   `odom -> base_link`：由 `odom_tf_broadcaster` 根据 Gazebo `/odom`
    以约 50 Hz 发布，提供连续、平滑的局部运动；
-   `base_link -> lidar_link`：静态外参，平移
    `[0, 0, 0.14]`，旋转为单位四元数。

NDT 已知地图定位得到：

``` text
T_map_base
```

Gazebo `/odom` 提供：

``` text
T_odom_base
```

因此：

``` text
T_map_odom = T_map_base * inverse(T_odom_base)
```

v0.4.1 已通过 `tf2_echo`、`/tf` 与 `/clock` 联合检测验证：

``` text
NDT 全局修正计算          约 1.5 ~ 2.3 Hz（单次计算约 0.44 ~ 0.66 s）
map -> odom TF 刷新       约 50 Hz
odom -> base_link         约 50 Hz
base_link -> lidar_link    static
```

在最新稳定测试中，`map -> odom` 实测约
`49.98 Hz`，采样窗口内平均、P95、P99 与最大时间延迟均为
`0.0000 s`，`>0.10 s` 样本为 0。该结果用于验证 TF 时间戳刷新机制，不代表
NDT 本体运行在 50 Hz。

并确认完整链路为：

``` text
map
└── odom
    └── base_link
        └── lidar_link
```

### 8.1 建图阶段的 TF 冲突处理

在：

``` bash
~/multi-floor-lio-sam/lio_sam_modified/launch/run.launch.py
```

中删除/注释了 LIO-SAM 自带的 `robot_state_publisher`，避免与仿真机器人
TF 冲突。

### 8.2 关闭重复 `odom -> lidar_link` TF

在：

``` bash
~/multi-floor-lio-sam/lio_sam_modified/src/mapOptmization.cpp
```

中将：

``` cpp
br->sendTransform(trans_odom_to_lidar);
```

注释为：

``` cpp
// br->sendTransform(trans_odom_to_lidar);
```

原因是项目中已经明确使用 `base_link -> lidar_link` 静态外参与独立的
`odom -> base_link` 链路，避免 `lidar_link` 出现多父节点问题。

### 8.3 当前定位阶段 TF 发布策略

早期 NDT 验证版本直接发布：

``` text
map -> base_link
```

v0.3.7 起停止该方式，改为：

``` text
NDT                -> map -> odom
odom_tf_broadcaster -> odom -> base_link
static TF           -> base_link -> lidar_link
```

只有通过 quality gate 的 `NDT ACCEPT` 结果才允许更新
`map -> odom`；`NDT REJECT` 不会污染全局 TF。

### 8.4 v0.4.1：NDT quality gate 与 TF 时间戳修复

Nav2 初次接入时曾出现两类与定位/TF相关的问题：

1.  **quality gate 锁死**：当 odometry prior 已可用时，仍使用"当前 NDT
    结果相对上一次接受位姿的绝对跳变"作为拒绝条件，机器人正常运动后可能持续触发
    `position_jump` / `yaw_jump`。当前逻辑改为：
    -   有有效 odom prior 时，主要检查 NDT 与 odom prediction
        的位置/航向误差；
    -   无 odom prior 时，才使用相对上一接受位姿的 position/yaw jump
        约束。
2.  **`map -> odom` TF 过旧**：NDT 单次计算约需 0.44～0.66
    s，如果直接使用扫描时间戳发布 TF，Nav2 会出现
    `Transform data too old`。当前策略为：
    -   NDT `ACCEPT` 时只更新最新 `T_map_odom` 修正值；
    -   `/odom` 回调以当前 odometry 消息时间戳高频刷新 `map -> odom`；
    -   `/initialpose` 重初始化时清空旧修正，避免重新定位期间广播旧 TF。

该修改使 NDT 可以保持低频全局修正，同时向 Nav2 提供高频且时间新鲜的 TF。

## 9. IMU 与 LiDAR 时间同步处理

之前出现过：

``` text
Waiting for IMU data...
```

以及扫描时间与 IMU 数据覆盖范围不一致的问题。

因此修改：

``` bash
~/multi-floor-lio-sam/lio_sam_modified/src/imageProjection.cpp
```

将点云缓存条件：

``` cpp
cloudQueue.size() <= 2
```

改为：

``` cpp
cloudQueue.size() <= 5
```

通过增加点云处理延迟，使 IMU 队列更容易覆盖整帧 LiDAR 扫描时间。

同时加入了调试输出，用于观察 scan time range 与 IMU time range。

## 10. 曾解决的主要问题

### 10.1 LIO-SAM 地图严重"炸图"

早期现象包括墙体扇形扩散、地图旋转和点云严重错位。

主要处理：

-   修正 LiDAR `time`；
-   修正 `ring`；
-   检查 IMU/LiDAR 时间覆盖；
-   修正 LiDAR-IMU 外参；
-   修正 TF 树。

目前已经基本消除严重发散。

### 10.2 重复 ros_gz_bridge

曾出现：

``` text
/imu/data
Publisher count: 2
```

并引发 IMU 重复、时间戳异常、`dt <= 0` 以及 GTSAM/LIO-SAM 异常。

通过清理重复 bridge 并只启动一个 `floor1_system.launch.py` 解决。

### 10.3 ROS 2 daemon 异常

曾出现 ROS 2 daemon / XMLRPC 错误，可通过：

``` bash
ros2 daemon stop
ros2 daemon start
```

恢复。

### 10.4 `/cmd_vel` Bridge 字符串拼接错误

在 launch 中加入 `/cmd_vel` 时曾漏写逗号，导致 Python
将两个字符串拼接，破坏 LiDAR Bridge。

最终正确格式：

``` python
'/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
'/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
'/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
```

### 10.5 `/clock` 未桥接导致仿真时间停滞

Nav2 接入过程中曾发现 `/clock` 的 `Publisher count: 0`，导致所有
`use_sim_time=true`
节点无法获得持续更新的仿真时间。当前在一键启动文件中增加独立 `/clock`
bridge，并要求只保留一个时钟发布者。

正常状态：

``` text
/clock Publisher count: 1
```

### 10.6 `map -> odom` TF 过旧导致 Nav2 控制失败

早期 Nav2 能生成全局路径，但 controller 输出为零，并报告：

``` text
Transform data too old when converting from odom to map
Unable to transform robot pose into global plan's frame
```

通过第 8.4 节所述的高频 TF 刷新机制解决。

### 10.7 重复节点导致 `/scan` 频率异常

手工调试节点与一键 launch 同时运行时，曾出现多个
`lidar_time_converter`、`pointcloud_to_laserscan`、Nav2/RViz
等重复进程，导致 `/scan` 频率超过 100 Hz。

清理全部旧进程后，只保留一套一键启动系统，当前验证：

``` text
/clock Publisher count: 1
/scan  Publisher count: 1
/scan  frequency ≈ 10 Hz
重名节点：无
```

可使用以下命令快速检查重复节点：

``` bash
ros2 node list | sort | uniq -d
```

### 10.8 `map_server` 生命周期启动时序

一键启动初版中 `map_server` 与其 lifecycle manager 同时启动，曾出现
`map_server = inactive [2]`。已将 lifecycle manager
延后启动以避免竞争；如现场仍出现 inactive，可通过以下命令临时恢复：

``` bash
ros2 lifecycle set /map_server activate
```

最终目标是在冷启动复验中确认该手工步骤不再需要。

## 11. 当前启动方式

### 11.1 建图与回环（历史流程，需先恢复完整 LIO-SAM 工作空间）

以下三终端流程对应 v0.1～v0.2 的建图与回环实验。当前机器缺少
`~/lio_sam_ws`，在重新检出并编译完整 LIO-SAM ROS 2 包之前，终端 2 的命令不可执行；
已保存的历史结果和地图文件不受影响。

**终端 1：Gazebo + Bridge + 点云适配器**

``` bash
source /opt/ros/humble/setup.bash
source ~/multi_floor_ws/install/setup.bash
ros2 launch multi_floor_sim floor1_system.launch.py
```

**终端 2：LIO-SAM + RViz**

``` bash
source /opt/ros/humble/setup.bash
source ~/lio_sam_ws/install/setup.bash
ros2 launch lio_sam run.launch.py
```

**终端 3：自动绕行测试或键盘控制（二选一）**

验证回环时，推荐使用已实测的自动绕行脚本。待终端 1、2 启动完成，
机器人位于初始世界原点附近且键盘控制已关闭后运行：

``` bash
source /opt/ros/humble/setup.bash
cd ~/multi_floor_ws
python3 scripts/auto_loop_test.py
```

脚本会自动驾驶、监测回环、停车并保存结果，完整说明见第 15.2 节。
需要手动探索时，改用以下键盘控制；不要与自动驾驶同时运行：

``` bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

常用控制：

``` text
i    前进
,    后退
j    左转
l    右转
k    停止
```

### 11.2 已知地图定位启动方式

在不重新运行 LIO-SAM Mapping 的情况下，已知地图定位使用：

**终端 1：Gazebo + Bridge + 点云适配器**

``` bash
source /opt/ros/humble/setup.bash
source ~/multi_floor_ws/install/setup.bash
ros2 launch multi_floor_sim floor1_system.launch.py
```

**终端 2：`odom -> base_link` TF**

``` bash
source /opt/ros/humble/setup.bash
source ~/multi_floor_ws/install/setup.bash
ros2 run known_map_localization odom_tf_broadcaster
```

**终端 3：已知地图发布 + NDT + RViz**

``` bash
source /opt/ros/humble/setup.bash
source ~/multi_floor_ws/install/setup.bash
ros2 launch known_map_localization localization_rviz.launch.py
```

随后在 RViz 中通过 `2D Pose Estimate` 向 `/initialpose`
提供初始位姿。定位成功后可通过：

``` bash
ros2 topic echo /localization/status --once
```

查看 `NDT ACCEPT / REJECT`、fitness、odom prior 使用情况和预测误差。

### 11.3 v0.4：单楼层 Nav2 一键启动

当前新增一键集成启动文件：

``` bash
~/multi_floor_ws/src/multi_floor_sim/launch/floor1_nav_all.launch.py
```

编译后可执行：

``` bash
source /opt/ros/humble/setup.bash
source ~/multi_floor_ws/install/setup.bash

ros2 launch multi_floor_sim floor1_nav_all.launch.py
```

该 launch 当前负责启动/组织：

``` text
Gazebo + 基础 Bridge
        ↓
/clock bridge
        ↓
odom -> base_link
        ↓
PointCloud2 -> /scan
        ↓
NDT known-map localization
        ↓
map_server
        ↓
/initialpose（当前 Floor1 默认出生点）
        ↓
Nav2
        ↓
RViz
```

当前 Floor1 默认机器人出生位姿约为
`(x=0, y=0, yaw=0)`，因此一键启动暂按该位姿发布初始值；未来多楼层阶段应改为基于楼层状态与机器人实际先验进行初始化。

推荐启动后执行以下健康检查：

``` bash
ros2 topic info /clock | grep "Publisher count"
ros2 topic info /scan | grep "Publisher count"
timeout 4s ros2 topic hz /scan
ros2 topic echo /localization/status --once
ros2 lifecycle get /map_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 action info /navigate_to_pose
```

正常情况下应满足：

``` text
/clock Publisher count: 1
/scan Publisher count: 1
/scan ≈ 10 Hz
NDT ACCEPT
odom_prior_used = true
map_server / controller_server / planner_server / bt_navigator = active [3]
/navigate_to_pose Action servers: 1
```

## 12. 编译方式

仿真工作空间修改后：

``` bash
cd ~/multi_floor_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

恢复完整 `~/lio_sam_ws` 后，LIO-SAM 修改可按以下方式编译：

``` bash
cd ~/lio_sam_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## 13. 当前建图效果

目前单楼层移动建图已经达到：

-   LIO-SAM 能稳定启动；
-   LiDAR 数据能够正常进入 LIO-SAM；
-   IMU 数据能够正常使用；
-   `ring = 0 ~ 15` 正常；
-   点云不会出现明显发散；
-   房间主体轮廓能够保持；
-   墙体存在少量厚度/重影，但整体可接受；
-   静止时点云基本稳定。

当前阶段可以认为：

> **单机器人单楼层 LIO-SAM 基础建图链路已经跑通。**

## 14. 已验证的 LIO-SAM Topics

目前主要 Topic：

``` text
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

1.  使用键盘控制机器人在房间内运动；
2.  绕场景行驶并重新靠近已经扫描过的区域；
3.  LIO-SAM 地图保持稳定；
4.  `/lio_sam/mapping/loop_closure_constraints` 成功输出数据。

输出中已经出现：

``` text
ns: loop_edges
type: 5
```

同时包含大量闭环连线点对。

第一次人工测试确认了回环约束生成；结合第 15.3
节后续自动测试，当前验证状态为：

``` text
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

当前参数为：回环开启、检测频率 1 Hz、候选半径 15 m、历史时间差超过 30
s、ICP 分数不高于 0.3。新关键帧门槛为平移约 1 m 或转动约 0.2 rad。
这些是当前配置值，运行时可核对实际加载值：

``` bash
source /opt/ros/humble/setup.bash
ros2 param get /lio_sam_mapOptimization loopClosureEnableFlag
ros2 param get /lio_sam_mapOptimization historyKeyframeSearchTimeDiff
ros2 param get /lio_sam_mapOptimization historyKeyframeSearchRadius
ros2 param get /lio_sam_mapOptimization historyKeyframeFitnessScore
ros2 param get /lio_sam_mapOptimization gpsTopic
```

按第 11 节启动一套新的 Gazebo 和 LIO-SAM，在开始行驶之前另开终端运行：

``` bash
source /opt/ros/humble/setup.bash
cd ~/multi_floor_ws
python3 scripts/verify_loop_closure.py
```

该脚本只订阅话题，不控制机器人或修改参数。默认同时监测
`/odometry/gpsz`，如运行时 `gpsTopic` 不同，使用 `--gps-topic`
指定实际话题。

1.  在可通行区域缓慢绕行，重新经过起点附近已扫描区域，尽量保持点云重叠。
    与历史帧的传感器时间差必须超过 30 秒；仿真较慢时，墙钟等待 30
    秒不一定够。
2.  观察 RViz 已配置的 MarkerArray：
    `/lio_sam/mapping/loop_closure_constraints`，黄色连线对应回环边。
    脚本会报告非空连线的数量，而非仅检查 `ns` 和 `type` 字段。
3.  出现连线后，在安全空地继续缓慢移动约 1～2 m，或转动超过约 12°，
    使其产生新关键帧，然后等待数秒。
4.  观察脚本是否报告"历史位姿更新"。脚本按关键帧时间戳比较同一历史帧的
    位置和姿态；只追加新轨迹点不会被算作历史更新。
5.  用 Ctrl+C 结束脚本，查看打印出的结果目录。

结果保存在 `results/loop_年月日_时分秒/`：

-   `events.jsonl`：回环边数量、历史位姿修改量及监测状态。
-   `summary.json`：本轮证据结论。
-   `path_before_first_revision.json` /
    `path_after_first_revision.json`：
    首次观察到历史修改前后的轨迹（有修改才生成）。
-   `path_final.json`：监测结束时的轨迹。

在当前源码、单次独立运行且无 GPS 输入的条件下，同时观察到非空回环约束和
历史位姿更新，支持"回环参与优化"的判断。脚本会检查轨迹重置、多发布者和
监测期间的 GPS 消息；请从本轮行驶之前开始记录，避免漏掉先前事件。
这是话题级证据，并非优化器内部日志，也不是 Ground Truth 精度评价。
历史修改量低于 0.00001 m / 0.00001 rad
时脚本不报告修改，因此没有观察到修改
不能直接判定优化失败；这种情况下需进一步记录 `addLoopFactor()` 和 iSAM
更新日志。

本轮实际自动验证结果见第 15.3 节。

### 15.2 自动绕行测试（无需键盘驾驶）

按第 11 节先启动 Gazebo 和 LIO-SAM，机器人应位于初始世界原点附近。 关闭
`teleop_twist_keyboard`，然后运行：

``` bash
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

``` text
(0,0) → (5.2,0) → (5.2,2.7) → (-1.2,2.7) → (-1.2,0) → (0,0)
                                                                    ↓
                                            等待 → (1.6,0) → (0,0) → 停车
```

路线绕过 `inner_wall_1`，最大前进速度 0.3 m/s、最大转速 0.45 rad/s。
首次返回后至少等待 5 秒仿真时间，且从本轮开始至少经过 35 秒仿真时间，
随后补走往返段，以便异步生成的回环约束由后续关键帧消费。 最终位置容差为
0.12 m，航向为世界坐标 0 rad，容差 0.06 rad。
实际用时取决于仿真实时率和四轮滑移，不按固定墙钟时间盲走。

可先执行不发布速度指令的预检查：

``` bash
python3 scripts/auto_loop_test.py --dry-run
```

运行中按 Ctrl+C 会发送停车指令并保存已有结果。位姿断流、仿真暂停、
LIO-SAM 轨迹断流、控制发布者冲突、接近墙体、路线偏离或长期无运动进展，
也会结束驾驶并停车。默认总超时为 1800 秒墙钟时间。
该固定路线只适用于当前 `floor1.sdf` 静态环境，不是通用避障或导航模块；
调整墙体或添加障碍后需要重新规划。

输出目录为 `results/auto_loop_年月日_时分秒/`，除第 15.1
节文件外还包含：

-   `drive_summary.json`：路线是否完成、实际首尾位姿和真实返回偏差。
-   `ground_truth.csv`：控制使用的世界坐标
    x/y/yaw；时间列为接收墙钟时间和 最近收到的仿真时钟，非严格同步的
    ATE/RPE 评估数据。
-   `route.json`：路线、速度和场景检查结果。
-   `bridge.log`：测试专用桥接日志。

**路线完成与回环验证是两个结果**：`drive_summary.json` 的
`route_completed`
表示机器人已走完路线；回环是否接受以及历史轨迹是否被修改，请看
`summary.json`。 默认回环参数的时间差为 30
秒，如修改过该参数，需同步调整脚本的等待策略。

离线回归检查：

``` bash
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s scripts -p 'test_auto_loop_test.py' -v
```

### 15.3 2026-09-09 自动绕行实测结果

结果目录：`results/auto_loop_20260909_160508/`。 已在 Gazebo Fortress +
当前 LIO-SAM 配置下完成实际试跑，全部 7 个航点到达，
最后自动停车。四项离线回归测试及实际 ROS 输入预检查均通过。

  项目                        本轮结果
  --------------------------- -----------------------------------------------------------
  自动路线                    完成，包括回原点后的补走往返段
  用时                        约 143.8 秒墙钟时间，126.6 秒仿真时间（从记录起点到结束）
  接受的回环边                51 条
  观察到历史位姿更新          50 次
  GPS 输入消息                0
  轨迹重置 / 重复轨迹发布者   未观察到
  真实最终位置                x = 0.1032 m，y = -0.0193 m
  真实首尾位置偏差            0.1050 m，在 0.12 m 到点容差内
  真实首尾航向偏差            -0.0562 rad，约 -3.22°

结论：**自动绕行和停车已跑通；观察到回环约束及历史位姿修改，支持回环参与优化。**
51 条边与 50
次历史修改不是一一对应的计数：边是累计接受的约束，历史修改是
话题层观察到的更新事件，不能据此断言每一条约束都已单独完成优化。
当前候选半径为 15
m，因此绕行过程中也会产生回环，并非只在最终回原点时产生。
上述真实首尾偏差属于驾驶控制的返回偏差，不是 LIO-SAM 的定位误差或
ATE/RPE。

实际加载参数另存为 `active_lio_parameters.yaml`，轨迹及回环事件图为：

![自动绕行轨迹与回环证据](results/auto_loop_20260909_160508/route_and_evidence.png)

### 15.4 本轮二维 ATE / RPE 初步计算

使用第 15.3 节保存的 `path_final.json`（最终优化关键帧）和
`ground_truth.csv`， 在重叠时间段 106.0～220.0 秒内匹配 65
个关键帧。早于 GT 记录的首个关键帧不外推， 予以排除。XY 位置经过 SE(2)
旋转和平移对齐，尺度固定为 1。

  指标                  RMSE      最大值
  --------------------- --------- ---------
  二维 ATE，对齐后      3.98 cm   7.88 cm
  未对齐 XY 位置差      4.87 cm   9.43 cm
  相邻关键帧 RPE 平移   2.22 cm   6.02 cm
  相邻关键帧 RPE 航向   0.485°    1.746°

RPE 使用 64 对相邻关键帧，时间间隔为 0.5～10.0 秒，中位数 0.7 秒；
**不是固定 1 秒 RPE，也不是每米误差**。这些指标描述最终优化轨迹，
没有关闭回环的对照，不能据此量化回环带来的精度提升。

**本次属于二维、近似时间戳的初步评估**：GT 只保存了
x/y/yaw，其时间列是最近
收到的独立仿真时钟，不是位姿消息原始时间戳。尚不能作为严格同步的完整三维
ATE/RPE 结果，也不能直接与不同采样或不同对齐方法的结果比较。

详细方法、逐帧误差、时间偏移敏感性与误差图见
[评估报告](results/auto_loop_20260909_160508/evaluation_2d/report.md)。
计算脚本的六项回归测试已通过，包括与完整齐次变换矩阵的 RPE
公式交叉检查。

复算命令（不需要 ROS 节点运行；依赖 NumPy，绘图使用 Matplotlib）：

``` bash
cd ~/multi_floor_ws
python3 scripts/evaluate_trajectory_2d.py results/auto_loop_20260909_160508
```

### 15.5 2026-09-09 地图保存与离线加载验证

在完成单楼层建图后，通过 LIO-SAM 自带服务保存当前地图：

> 以下为 2026-09-09 实验时使用的历史命令。当前需先恢复并编译完整
> `~/lio_sam_ws`，才能再次调用该服务。

``` bash
source /opt/ros/humble/setup.bash
source ~/lio_sam_ws/install/setup.bash

ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
"{resolution: 0.0, destination: '/lio_sam_maps/floor1'}"
```

本轮服务返回：

``` text
success=True
```

保存目录：

``` text
~/lio_sam_maps/floor1
```

生成文件及规模如下：

  文件                            本轮结果
  ----------------------- ----------------
  `CornerMap.pcd`            13,723 points
  `SurfMap.pcd`             152,184 points
  `GlobalMap.pcd`           165,907 points
  `trajectory.pcd`              162 points
  `transformations.pcd`         162 points

其中 `GlobalMap.pcd` 文件大小约 2.6 MB，`trajectory.pcd` 与
`transformations.pcd` 均包含 162
个关键帧记录，说明保存结果来自完整运动建图过程，而不是仅保存单帧初始点云。

为验证地图是否能够脱离建图过程独立复用，关机重启后确认上述 PCD
文件仍然存在，并使用 ROS 2 Humble 中的 `pcl_ros pcd_to_pointcloud`
重新发布：

``` bash
source /opt/ros/humble/setup.bash

ros2 run pcl_ros pcd_to_pointcloud \
--ros-args \
-p file_name:=/home/yez/lio_sam_maps/floor1/GlobalMap.pcd \
-p publish_rate:=10.0
```

当前环境中输出 Topic 为：

``` text
/cloud_pcd
```

重新发布后的 `PointCloud2`：

``` text
width = 165907
fields = x, y, z, intensity
```

与保存时 `GlobalMap.pcd` 的点数一致，说明 PCD
内容能够被正常读取和重新发布。

需要注意：当前 `pcl_ros` 节点发布的点云 `frame_id` 为
`/base_link`。为了完成 **仅用于离线可视化验证** 的 RViz
显示，本轮临时发布零位姿静态 TF：

``` bash
ros2 run tf2_ros static_transform_publisher \
0 0 0 0 0 0 map base_link
```

随后在 RViz 中设置：

``` text
Fixed Frame = map
PointCloud2 Topic = /cloud_pcd
```

完整单楼层三维地图能够正常显示。

> **该零位姿静态 TF
> 仅用于验证保存地图可重新显示，不能作为后续定位系统的正式
> `map -> base_link` 变换。** 后续 NDT/GICP
> 定位模块应根据实时匹配结果估计机器人在已知地图中的位姿，并建立规范的
> `map -> odom -> base_link -> lidar_link` TF 链。

因此，本阶段已经验证：

``` text
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

即
**单楼层三维地图的保存与离线重新加载功能已经跑通**。该结果证明地图文件可以作为后续
NDT/GICP
已知地图定位、多楼层地图管理和多机器人共享地图的基础数据。在 2026-09-09
完成 v0.2 验证时，尚未完成“定位模块实际加载地图并输出机器人位姿”；该项后来已在
第 15.6～15.8 节的 v0.3～v0.3.7 工作中完成。

### 15.6 已知地图 NDT 定位与 `/initialpose` 重定位

已完成 `known_map_localization` ROS 2 包开发。核心数据流为：

``` text
GlobalMap.pcd
      +
当前 /lio_sam/points
      ↓
     NDT
      ↓
T_map_lidar
      ↓
结合 T_base_lidar
      ↓
T_map_base
```

NDT 属于局部配准方法，对初始位姿敏感，因此通过 RViz `2D Pose Estimate`
向：

``` text
/initialpose
```

提供 `T_map_base` 近似初值，再转换为 LiDAR 初值参与 NDT。

在机器人位于地图原点附近时，精确初值 `(x=0, y=0, yaw=0)`
可恢复到健康匹配状态，典型 fitness 约为：

``` text
0.014 ~ 0.016
```

### 15.7 v0.3.5：odometry motion prior 与 quality gate

早期版本使用上一帧 NDT
位姿作为下一帧初值，直线运动基本正常，但转弯时曾出现错误局部收敛并持续发散。

为此加入：

1.  **odometry motion prior**

``` text
delta_T = inverse(T_odom_base_prev) * T_odom_base_curr
T_map_base_pred = T_map_base_last * delta_T
```

利用 `/odom` 的相对运动预测当前 NDT 初值。

2.  **quality gate**

综合检查：

-   `hasConverged()`
-   fitness
-   位置跳变
-   yaw 跳变
-   与 odom prediction 的位置偏差
-   与 odom prediction 的 yaw 偏差

仅当全部条件满足时：

``` text
NDT ACCEPT
```

否则：

``` text
NDT REJECT
```

并保持上一份有效定位与 odom 参考，不允许错误局部最优继续污染后续帧。

当前生产用 fitness 阈值为：

``` text
0.017
```

### 15.8 v0.3.7：标准 TF 与动态验证

标准 TF 重构后进行了静止、直行与原地转弯验证。

静止状态下典型结果：

``` text
NDT ACCEPT
fitness ≈ 0.014
odom_prior_used = true
```

直行测试中，机器人以约 `0.1 m/s` 前进约 3 s：

``` text
predicted_x ≈ 0.330 m
raw_x       ≈ 0.332 m
prediction_position_error ≈ 3.7 mm
fitness ≈ 0.0144
```

原地转弯测试中，机器人以约 `0.3 rad/s` 转动约 3 s：

``` text
理论转角 ≈ 0.9 rad
NDT yaw  ≈ 0.889 rad
fitness  ≈ 0.0141
odom_prior_used = true
```

本轮转弯过程中未复现旧版连续发散现象。

Ground Truth 由 Gazebo world pose 独立提供；手工 `--once`
采样因时间戳不同仅用于现场检查，正式 RMSE 仍应使用同步 evaluator 结果。

### 15.9 v0.4：二维导航地图与 Nav2 单机器人导航接入

已从 `GlobalMap.pcd` 提取二维导航栅格地图。当前地图参数：

``` text
路径：~/nav_maps/floor1/map.yaml
分辨率：0.05 m
尺寸：411 × 202
origin ≈ (-10.195, -4.940)
```

点云转二维扫描采用 `pointcloud_to_laserscan`：

``` text
输入：/lio_sam/points
输出：/scan
target_frame：lidar_link
高度范围：-0.05 ~ 0.30 m
range：0.35 ~ 20.0 m
/scan 实测频率：约 10 Hz
```

Nav2 使用 `navigation_launch.py`，不启动 AMCL；全局位姿由 NDT
提供。当前主要链路：

``` text
GlobalMap.pcd
     ↓
NDT + odom prior + quality gate
     ↓
map -> odom -> base_link
     ↓
2D OccupancyGrid + /scan
     ↓
Nav2 global planner
     ↓
DWB controller
     ↓
/cmd_vel_nav
     ↓
velocity_smoother
     ↓
/cmd_vel
     ↓
Gazebo robot
```

当前已验证：

-   `/navigate_to_pose` 存在 `/bt_navigator` Action server；
-   `controller_server`、`planner_server`、`bt_navigator` 可进入
    `active [3]`；
-   DWB 能生成非零线速度与角速度；
-   速度经 `velocity_smoother` 后继续发布到 `/cmd_vel`；
-   目标附近可观察到"停止前进 -\> 原地调整最终朝向 -\> 角速度逐步减小至
    0"的控制行为；
-   当前局部控制器为 DWB，尚未进行 TEB/MPPI 等控制器对比。

v0.4 阶段只完成了功能链路验收。v0.5 已补充一个固定起点/固定目标下的
位置误差、航向误差、路径长度和导航时间重复实验；最小障碍距离、轨迹平滑度、
控制振荡、CPU 开销以及多目标/多路线评价仍待补充。

### 15.10 Nav2终端姿态优化与重复性导航实验

在完成 Nav2
单机器人导航功能验证后，进一步针对目标点附近机器人停止精度不足的问题进行了参数优化。

初始实验中，机器人能够完成路径规划和目标到达，但最终航向误差约为
14°，无法满足后续精确停靠和多楼层电梯场景需求。因此对 Nav2
控制参数进行了优化。

#### 15.10.1 参数对照过程与当前配置

初始基线为：

``` yaml
RotateToGoal.scale: 32.0
yaw_goal_tolerance: 0.25
```

首先只把 `RotateToGoal.scale` 从 32 提高到 50，并完成五次固定路线测试。
该组定位 TF yaw 误差平均约 14.1°，Gazebo 真值 yaw 误差平均约 13.5°，
没有解决最终航向误差问题。该参数随后恢复为 32。

当前正式使用的组合是：

``` yaml
RotateToGoal.scale: 32.0
yaw_goal_tolerance: 0.05
```

因此，当前终端航向精度改善主要来自 Goal Checker 航向容差收紧，不能归因于
`RotateToGoal.scale=50`。scale=50 对照记录见
[`experiments/dwb_comparison_20260916_verified/scale50_five_runs.md`](experiments/dwb_comparison_20260916_verified/scale50_five_runs.md)。

#### 15.10.2 Goal Checker参数优化

原始参数：

``` yaml
yaw_goal_tolerance: 0.25
```

对应约 14.3°。

当前参数：

``` yaml
yaw_goal_tolerance: 0.05
```

对应约 2.9°，提高终端姿态约束。`xy_goal_tolerance` 仍保持 0.25 m，
因此本轮主要改善航向精度，并未把位置到达容差收紧到厘米级。

#### 15.10.3 重复性实验

在固定起点 `(0,0,0)` 和固定目标
`(5.455,1.436,-1.487 rad)` 条件下，对当前参数进行了五次有效导航实验。
五次运行的 NavigateToPose action 均返回 `SUCCEEDED`，运行时查询确认参数为
`scale=32`、`yaw_goal_tolerance=0.05`，且未检测到 odom 跳变。

| 指标 | Run01 | Run02 | Run03 | Run04 | Run05 | 平均值 |
|---|---:|---:|---:|---:|---:|---:|
| 导航时间 (s) | 42.169 | 41.491 | 43.959 | 44.209 | 42.091 | 42.784 |
| 初始规划长度 (m) | 5.768 | 5.768 | 5.768 | 5.768 | 5.768 | 5.768 |
| odom 累计路径长度 (m) | 6.365 | 6.283 | 6.378 | 6.407 | 6.311 | 6.349 |
| 路径比 | 1.103 | 1.089 | 1.106 | 1.111 | 1.094 | 1.101 |
| 平均速度 (m/s) | 0.151 | 0.151 | 0.145 | 0.145 | 0.150 | 0.148 |
| 定位 TF 位置误差 (cm) | 20.1 | 19.5 | 19.5 | 19.5 | 21.7 | 20.06 |
| 定位 TF yaw 误差 (°) | 2.8 | 2.7 | 2.8 | 2.8 | 2.9 | 2.80 |
| Gazebo 真值位置误差 (cm) | 21.5 | 20.9 | 22.2 | 22.0 | 22.8 | 21.88 |
| Gazebo 真值 yaw 误差 (°) | 2.7 | 2.6 | 2.7 | 2.7 | 2.8 | 2.70 |

表中“路径长度”按旧基线口径由 `/odom` XY 累计得到；定位 TF 误差和 Gazebo
world pose 真值误差分开报告。完整计算口径、CSV 和失败重试记录见
[`experiments/dwb_yaw005_scale32_20260916/scale32_five_runs.md`](experiments/dwb_yaw005_scale32_20260916/scale32_five_runs.md)。

Run02 的首次尝试在进入导航前遇到 Nav2 planner lifecycle 响应超时，原始失败
记录已保留，表中使用隔离通信域后的有效补测。五次有效导航中均出现过 1～2 次
`Failed to make progress`，随后由行为树恢复并成功到达。因此当前证据支持
“固定路线最终可完成”，但还不能把过程描述为完全无恢复、无异常的稳定导航。

实验结果表明：

1.  当前参数下五次有效实验均能完成固定目标点导航；
2.  终端航向误差由约14°降低至3°以内；
3.  结果为后续电梯停靠与楼层切换实验提供了终端姿态基础，但是否满足正式停靠要求，
    仍需预先定义验收阈值并进行多目标、多路线和进出电梯场景测试。

当前阶段已经完成单楼层导航控制链和终端精度优化验证。

------------------------------------------------------------------------

## 16. 当前闭环测试的不足

第一次人工测试没有记录真实起点，只能验证是否生成回环约束。
后续自动测试已补上真实轨迹记录、自动返回、历史位姿更新观测及二维 ATE/RPE
初评， 当前不足主要是：

-   **时间关联仍为近似**：GT
    使用最近收到的独立仿真时钟，未保留位姿原始时间戳。
-   **评估维度不完整**：已保存的 GT 为 x/y/yaw，缺少
    z/roll/pitch，尚未计算严格三维指标。
-   **采样不统一**：使用最终优化关键帧，相邻帧间隔为 0.5～10 秒；当前
    RPE 不是固定 1 秒指标。
-   **实验次数与对照不足**：仅完成一次自动路线实测，尚无多次重复实验或同路线关闭回环的对照。
-   **未预设精度验收阈值**：不能用本轮结果反过来定义合格线；应先结合论文目标制定指标和阈值。

因此，当前可确认功能验收通过、二维精度初评完成，尚不能声称论文最终精度验收通过。
真实返回起点的 10.5 cm 偏差属于驾驶控制结果，不能当作 LIO-SAM 定位误差。

## 17. 下一步工作

当前单楼层建图、地图保存、NDT 已知地图定位、标准 TF、二维导航地图以及
Nav2 单机器人导航链路已经完成阶段性功能验证。

当前单楼层系统已经完成：LIO-SAM 三维建图、地图保存与加载、NDT 已知地图定位、
标准 TF 维护、Nav2 单机器人导航，以及固定起终点下的终端姿态重复实验。

下一阶段按以下顺序推进：

1.  将已完成的 v0.6 Floor Map Manager 补齐安装/运行依赖并接入现有一键 launch；
2.  制作真实且相互独立的 Floor2 三维/二维地图，验证真实楼层坐标和出梯初始位姿；
3.  完善严格同步 Ground Truth、完整三维 ATE/RPE 和定位消融实验；
4.  开展不同目标点、多路线、多障碍场景导航测试，补充最小障碍距离、轨迹平滑度、
    控制振荡和 CPU 开销；
5.  设计 DWB / TEB / MPPI 控制器对比实验；
6.  建立电梯拓扑与 FSM，验证进梯、楼层切换和出梯后的地图/定位切换；
7.  在多楼层单机器人稳定后，再扩展多机器人共享地图、任务分配、路径协调与电梯调度。

其中论文级定量评估仍需继续补充：

-   全系统统一 ROS 仿真时间与原始时间戳同步；
-   完整六自由度 Ground Truth；
-   多起点、多路线和多目标重复实验；
-   基线 / 消融对照；
-   预先设定的评价指标与验收阈值；
-   Nav2 控制器与导航性能对比实验。

## 18. 后续论文总体路线

``` text
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

后续机器人在已有地图中运行时，应将"建图"和"定位 / 重定位"分开处理。

## 19. 当前仍需后续处理的问题

### 19.1 `/clock`

Nav2 集成阶段已经补充 Ignition/Gazebo `/clock`
bridge，并在当前一键启动链路中统一主要定位/导航节点使用
`use_sim_time = true`。

当前运行时检查要求：

``` text
/clock Publisher count: 1
```

重复启动额外 clock bridge 会造成两个时钟发布者，因此使用
`floor1_nav_all.launch.py` 时不应再手动启动第二个 `/clock` bridge。

论文级同步评价仍需进一步保证 Ground
Truth、NDT、轨迹记录和评价脚本均直接使用同一原始仿真时间戳，而不是"最近一次收到的时钟值"。

### 19.2 Ground Truth

当前已实现：

``` text
Gazebo /world/floor1/dynamic_pose/info
        ↓
world_pose_ground_truth
        ↓
/ground_truth/world_pose
```

该节点按 `thesis_robot` 模型读取 Gazebo world pose，并保留原始 Pose_V
时间戳，用作独立 Ground Truth。

Ground Truth 仅用于算法评价，不参与 NDT 定位输入。

后续仍需完善：

-   全六自由度误差统计；
-   严格时间同步；
-   多次重复实验；
-   固定路线与固定采样策略；
-   论文最终 ATE/RPE 指标。

### 19.3 地图保存与加载

LIO-SAM `save_map`、PCD 离线加载以及 NDT 定位模块实际加载
`GlobalMap.pcd` 均已完成。

当前正式保存目录：

``` text
~/lio_sam_maps/floor1
```

主要地图文件：

``` text
GlobalMap.pcd          165907 points
SurfMap.pcd            152184 points
CornerMap.pcd           13723 points
trajectory.pcd            162 points
transformations.pcd       162 points
```

`GlobalMap.pcd` 已从"仅用于 RViz 离线显示"进一步用于实时 NDT
已知地图定位，并已经生成用于 Nav2 的二维占据栅格地图：

``` text
~/nav_maps/floor1/map.yaml
~/nav_maps/floor1/map.pgm
resolution = 0.05 m
size = 411 × 202
```

当前二维地图由三维点云高度切片/投影得到，已经能够支撑第一轮 Nav2
功能验证；后续仍可针对墙体厚度、离散噪点和障碍物膨胀效果进一步清理和优化。

### 19.4 定位模块

当前已经形成独立实时定位链路：

``` text
Saved GlobalMap.pcd
      +
Current LiDAR Scan
      ↓
NDT
      +
Odometry Motion Prior
      ↓
Quality Gate
      ↓
T_map_base
      ↓
T_map_odom
```

并形成标准 TF：

``` text
map
└── odom
    └── base_link
        └── lidar_link
```

当前已完成静止、直线、原地转弯验证。

仍需在后续论文实验中进一步开展：

-   多次重复测试；
-   更复杂运动轨迹；
-   不同初始位姿偏差下的重定位成功率；
-   baseline / prior-only / gate-only / full method 消融对照；
-   严格同步 Ground Truth 下的最终定位精度统计。

### 19.5 Nav2 单机器人导航

当前 Nav2 已形成独立的单楼层导航链路，不使用 AMCL。主要配置文件：

``` text
~/multi_floor_ws/src/multi_floor_sim/config/nav2_params.yaml
```

当前局部控制器为 DWB，主要数据流为：

``` text
NDT localization + TF
        +
2D map + /scan
        ↓
Nav2 planner / controller / BT navigator
        ↓
/cmd_vel_nav
        ↓
velocity_smoother
        ↓
/cmd_vel
```

当前仍需进一步开展：

-   不同 DWB 参数的系统化 A/B 对照；
-   不同起点和目标点下的到达误差与重复性测试；
-   不同目标点、障碍场景与路线的稳定性测试；
-   DWB 与 TEB/MPPI 等方案的必要性和对比设计；
-   导航评价指标与论文实验方案固化。

### 19.6 v0.6 多楼层地图动态切换与重定位

v0.6 的**多楼层地图管理核心已经完成**，新增：

``` text
src/multi_floor_sim/config/floor_maps.yaml
src/multi_floor_sim/scripts/floor_map_manager.py
```

`floor_maps.yaml` 作为楼层地图注册表，记录每层的 Nav2 `map.yaml`、NDT
`GlobalMap.pcd` 和默认初始位姿。`ndt_localizer` 支持运行时修改 `map_path`：先把新
PCD 加载到临时点云，成功后替换 NDT target，并安全清除上一楼层的 last valid pose、
odom prior 缓存和 `map -> odom` 修正，随后等待新楼层 `/initialpose`。

Floor Map Manager 已实际验证以下流程：

``` text
/current_floor
      ↓
/map_server/load_map 切换二维地图
      ↓
/ndt_localizer/set_parameters 切换 GlobalMap.pcd
      ↓
发布对应楼层 /initialpose
      ↓
监听 /localization/status
      ↓
连续收到 2 次 NDT ACCEPT
      ↓
记录 FLOOR READY 并发布当前楼层信息
```

管理器采用显式状态机：

``` text
IDLE
  → SWITCHING_NAV_MAP
  → SWITCHING_NDT_MAP
  → PUBLISH_INITIAL_POSE
  → WAITING_NDT_READY
  → READY
```

服务阶段和 NDT READY 等待阶段均有超时保护。任何步骤失败时不会把
`current_floor_id` 更新为目标楼层，并会清理 `pending_floor_id`、`pending_cfg`、
`waiting_for_ndt` 和 `ndt_accept_streak`；过期异步响应会被忽略。重复请求当前楼层，
或在切换过程中请求另一楼层，也会被拒绝。

2026-09-17 已在真实 Gazebo + Nav2 + NDT 系统中完成
Floor1 → Floor2 → Floor1 双向切换验证：二维地图和 PCD 均成功切换，换图后自动发布
`/initialpose`，NDT 连续恢复 `ACCEPT`，最终状态为 `accepted=true`、fitness 约
0.014、`odom_prior_used=true`。v0.6 代码已同步到 Git 仓库分支
`v0.6-multifloor`。

该验收证明了**多楼层地图动态切换与重定位链路**，不代表完整多楼层自主导航已经完成。
当前限制为：

-   Floor2 的二维地图和 PCD 仍是 Floor1 的复制占位地图，尚未制作真实独立地图；
-   `floor_map_manager.py` 尚未由 CMake 安装，也未接入现有一键 launch，相关运行依赖
    仍需补充到 `multi_floor_sim/package.xml`；
-   尚未实现电梯 FSM、真实进梯、乘梯和出梯控制；
-   尚未完成真正跨楼层的连续自主导航；
-   尚未进入多机器人任务分配、路径协调与电梯调度阶段。

## 20. 当前阶段结论

截至 v0.6，本项目已经完成：

``` text
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
NDT 已知地图定位
  ↓
/initialpose 重定位
  ↓
odometry motion prior
  ↓
quality gate
  ↓
map -> odom -> base_link -> lidar_link
  ↓
二维 OccupancyGrid + /scan
  ↓
Nav2 global planner
  ↓
DWB + velocity_smoother
  ↓
/cmd_vel
```

整条单机器人、单楼层"建图 -\> 已知地图定位 -\> 导航控制"基础链路，
以及 v0.6 的"楼层请求 -\> 二维/三维地图动态切换 -\> 自动重定位 -\> READY"管理链路。

当前阶段验收结论为：

> **单楼层 LIO-SAM 建图、回环、地图保存、NDT 已知地图定位、标准
> TF、二维导航地图与 Nav2 单机器人导航控制链均已完成阶段性功能验证；v0.6
> 多楼层地图动态切换与重定位核心已经完成真实系统联调。真实 Floor2、电梯 FSM
> 和完整跨楼层自主导航尚未完成。**

  ---------------------------------------------------------------------------------------------------------
  验收项                  当前结论                依据或限制
  ----------------------- ----------------------- ---------------------------------------------------------
  基础建图链路            通过阶段性功能验收      LIO-SAM 稳定运行，单楼层地图无明显发散

  自动绕行与停车          通过                    7 个航点完成并返回起点附近，最后自动停车

  回环触发与参与优化      通过功能验证            51 条回环约束、50 次历史位姿更新，无 GPS
                                                  输入或轨迹重置记录

  建图二维轨迹精度        初评完成                ATE RMSE 3.98 cm；属于近似时间戳二维初评

  三维地图保存            通过                    `GlobalMap.pcd` 为 165,907 points

  地图离线重新加载        通过                    关机重启后 PCD 可独立发布并在 RViz 中显示

  已有地图 NDT 定位 /     通过                    `/initialpose` 可重新初始化，健康 fitness 约 0.014～0.016
  重定位                                          

  odometry motion prior   通过                    连续运动与转弯时可用于 NDT 初值预测

  quality gate            通过                    能拒绝异常匹配；负向阈值测试中拒绝逻辑有效

  标准 TF                 通过                    `map -> odom -> base_link -> lidar_link` 已由
                                                  `view_frames` 验证

  直线动态定位            通过                    NDT ACCEPT，fitness 约 0.0144，prediction error 毫米级

  原地转弯动态定位        通过                    NDT ACCEPT，fitness 约 0.0141，旧版转弯发散未复现

  `map -> odom` TF        通过                    实测约 49.98 Hz；最终稳定测试中 P99/max age 为 0，未出现
  时间新鲜度                                      \>0.10 s 样本

  二维导航地图            通过功能验证            0.05 m，411 × 202，可由 `map_server` 发布

  `/scan` 转换            通过                    单发布者，约 10 Hz

  论文最终精度验收        尚未完成                仍需严格同步三维数据、多起点/多路线重复实验、消融/基线对照及预设阈值

  Nav2 单机器人自主导航   固定路线重复实验完成    `/navigate_to_pose`、DWB、`/cmd_vel_nav`、velocity
                                                  smoother 与 `/cmd_vel` 控制链已接通；固定起终点五次有效
                                                  实验均成功，TF yaw 误差约 2.8°，真值 yaw 误差约 2.7°；
                                                  多目标、多路线及无恢复稳定性仍待评价

  v0.6 楼层地图切换       多楼层地图管理核心完成  Floor1 → Floor2 → Floor1 真实联调通过；具备超时、失败状态清理、
                                                  连续 2 次 NDT ACCEPT 后 READY；Floor2 仍为占位复制地图
  ---------------------------------------------------------------------------------------------------------

下一阶段重点：

> **多目标 Nav2 定量评价 + Floor Map Manager 一键启动接入 -\> 真实独立 Floor2 地图
> -\> 电梯 FSM 与完整跨楼层自主导航 -\> 多机器人任务分配、路径协调与电梯调度扩展**

在进入多机器人之前，优先完成管理器的一键启动接入、真实 Floor2 地图、电梯 FSM
和跨楼层连续导航，同时继续补充论文级定位/导航定量评估。

## 21. 维护建议

后续每完成一个阶段，建议持续更新本 README，至少记录：

``` text
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
