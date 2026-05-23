# agv_navigation (MPPI Navigation)

`agv_navigation` 是导航配置包，负责承载 MPPI 控制器、SLAM、地图和导航启动资源。当前采用在线 SLAM 模式：Gazebo 虚拟雷达发布 `/agv/scan`，`slam_toolbox` 生成 `map -> odom` 坐标变换，MPPI 控制器基于全局规划器生成的路径和当前位姿进行轨迹优化，输出最优的控制速度。

## MPPI 控制器简介

**Model Predictive Path Integral (MPPI)** 是一种基于采样的最优控制方法，通过：

1. **轨迹采样**：在控制空间采样 N 条候选轨迹（batch_size = 2000）
2. **成本评价**：用多个评价器（Critics）计算每条轨迹的成本（避障、路径跟踪、目标到达等）
3. **概率加权**：按 Boltzmann 分布对成本进行指数加权，得到最优控制
4. **执行控制**：发送最优速度命令 `/agv/cmd_vel` 给底盘

相比 DWB 控制器，MPPI 提供：
- **更灵活的采样**：无需预定义速度网格
- **更平滑的轨迹**：基于连续优化，避免离散化导致的抖动
- **更高的实时性**：并行评价，计算延迟 ~50-80ms
- **更易调试**：评价器权重独立配置，易于针对性优化

## 包内容

| 路径 | 作用 |
| --- | --- |
| `config/nav2_params.yaml` | MPPI 和 Nav2 全套参数，包含 MPPI 采样、评价器权重、costmap、AMCL、Global Planner 等 |
| `maps/` | 地图文件目录，当前没有地图文件（首次需建图保存） |
| `launch/mapping.launch.py` | 在线建图入口，启动 `slam_toolbox` 和 `map_saver_server` |
| `launch/navigation.launch.py` | 启动 Nav2 + MPPI 导航栈（不含 SLAM/定位，适合调试） |
| `launch/localization_navigation.launch.py` | 正式导航入口，加载保存地图，启动 AMCL + Nav2 + MPPI |
| `launch/two_agv_localization_navigation.launch.py` | 两车导航入口，为 `agv_01`、`agv_02` 各启动一套带 frame 前缀的 AMCL + Nav2 + MPPI |

## MPPI 核心参数说明

### 采样参数

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `time_steps` | 56 | 预测时间步长（horizon），56 × 0.1s = 5.6s 的预测窗口 |
| `model_dt` | 0.1 | 单步模型时间间隔（秒） |
| `batch_size` | 2000 | 每次迭代采样的候选轨迹数 |
| `iteration_count` | 1 | 每个控制周期的优化迭代次数（1 表示无迭代，直接采样） |

### 速度标准差（噪声）

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `vx_std` | 0.2 | 前进速度的采样标准差 |
| `vy_std` | 0.0 | 横向速度标准差（差分驱动无横向） |
| `wz_std` | 0.4 | 角速度标准差 |

### 速度限制

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `vx_max` | 0.25 | 最大前进速度（m/s） |
| `vx_min` | -0.10 | 最大后退速度（m/s） |
| `vy_max` | 0.0 | 最大横向速度（m/s） |
| `wz_max` | 0.7 | 最大角速度（rad/s） |

### 优化参数

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `temperature` | 0.3 | Boltzmann 分布温度，越小采样越集中在最优解 |
| `gamma` | 0.015 | 学习率（用于迭代优化） |

### 评价器（Critics）

当前启用的评价器（按优先级）：

| 评价器 | 权重 | 作用 |
| --- | --- | --- |
| PathAlignCritic | 14.0 | **最重**：沿全局路径对齐，避免偏离 |
| GoalCritic | 5.0 | 朝向目标位置移动 |
| PreferForwardCritic | 5.0 | 优先前进（减少后退） |
| PathFollowCritic | 5.0 | 跟踪全局路径 |
| ConstraintCritic | 4.0 | 遵守速度和加速度约束 |
| CostCritic | 3.81 | 碰撞代价（collision_cost = 1000000） |
| GoalAngleCritic | 3.0 | 最终朝向角度 |
| PathAngleCritic | 2.0 | 路径切线方向对齐 |

**调整建议**：
- 增加 `PathAlignCritic` 权重 → 更贴近全局路径（但可能不灵活）
- 增加 `CostCritic` 权重 → 更激进地避障（但可能绕路较长）
- 减少 `PreferForwardCritic` 权重 → 允许更多后退（适合狭窄区域）

## 在信息流中的位置

```text
Gazebo /agv/scan
        |
        v
slam_toolbox
        |
        +--> map / odom 相关定位和建图信息
        |
        v
Nav2 Global Planner (NavfnPlanner)
        |
        +--> /plan (全局路径）
        |
        v
MPPI Local Controller
  - 订阅 /plan、/agv/odom、/agv/scan
  - 采样候选轨迹
  - 评价并选择最优控制
        |
        v
agv_scheduler 发送目标点
```

推荐使用两阶段流程：

```text
阶段 1: 建图
agv_bringup/agv_sim.launch.py
        |
        +--> Gazebo + robot_state_publisher + controller + /agv/scan + /agv/odom
        |
agv_navigation/mapping.launch.py
        |
        +--> slam_toolbox 发布 /map 和 map -> odom
        +--> map_saver_server 保存 warehouse.yaml / warehouse.pgm

阶段 2: MPPI 导航
agv_bringup/agv_sim.launch.py
        |
        +--> Gazebo + 机器人底座 + /agv/scan + /agv/odom
        |
agv_navigation/localization_navigation.launch.py
        |
        +--> map_server 加载 warehouse.yaml
        +--> AMCL 使用 /agv/scan 定位
        +--> Nav2 Global Planner 规划全局路径
        +--> MPPI Local Controller 优化局部轨迹 + /agv/cmd_vel
```

## 使用方式

构建并安装导航资源：

```bash
colcon build --symlink-install --packages-select agv_navigation
source install/setup.zsh
```

### 阶段 1: 建图

先启动仿真底座：

```bash
ros2 launch agv_bringup agv_sim.launch.py
```

再启动建图：

```bash
ros2 launch agv_navigation mapping.launch.py
```

用键盘控制小车慢慢跑完整个仓库：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/agv/cmd_vel
```

检查地图是否生成：

```bash
ros2 topic echo --once /map
ros2 run tf2_ros tf2_echo map base_footprint
```

保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f src/agv_navigation/maps/warehouse
```

保存后会得到：

```text
src/agv_navigation/maps/warehouse.yaml
src/agv_navigation/maps/warehouse.pgm
```

### 阶段 2: MPPI 导航

重新启动仿真底座：

```bash
ros2 launch agv_bringup agv_sim.launch.py
```

启动定位和 MPPI 导航：

```bash
ros2 launch agv_navigation localization_navigation.launch.py
```

如果地图不在默认路径，可以显式传入：

```bash
ros2 launch agv_navigation localization_navigation.launch.py \
  map:=$(pwd)/src/agv_navigation/maps/warehouse.yaml
```

检查 MPPI 控制器和定位：

```bash
ros2 action list | grep navigate
ros2 run tf2_ros tf2_echo map base_footprint
```

### 验证 MPPI 控制器正在工作

检查 MPPI 节点是否启动：

```bash
ros2 node list | grep controller
```

应看到 `controller_server` 节点。查看 MPPI 的实时数据：

```bash
# 查看当前速度命令
ros2 topic echo /agv/cmd_vel

# 查看当前里程计
ros2 topic echo /agv/odom

# 查看全局路径
ros2 topic echo /plan

# 查看本地 costmap
ros2 topic echo /local_costmap/costmap_raw
```

### 在 RViz 中手动导航

在 RViz 中使用 `Nav2 Goal` 发布目标点，观察 MPPI 控制器的轨迹优化效果：

```bash
ros2 launch agv_navigation localization_navigation.launch.py
# 同时启动 RViz（可选）
rviz2 -d $(ros2 pkg prefix agv_navigation)/share/agv_navigation/rviz/navigation.rviz
```

或者用命令行发送目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### 两车 MPPI 导航

先启动两车仿真底座：

```bash
ros2 launch agv_bringup two_agv_sim.launch.py gui:=false rviz:=false
```

再启动两车定位和 MPPI 导航：

```bash
ros2 launch agv_navigation two_agv_localization_navigation.launch.py \
  map:=$(pwd)/src/agv_navigation/maps/warehouse.yaml
```

两车仿真默认使用 `localization_mode:=odom`，启动文件会发布固定的
`map -> agv_01_odom` 和 `map -> agv_02_odom`，让 Nav2 的 `map` 坐标和
Gazebo/odom 坐标保持一致。这样适合当前确定性仿真，避免 AMCL 在对称货架
环境里粒子发散导致小车抖动。需要测试粒子滤波定位时可以显式切换：

```bash
ros2 launch agv_navigation two_agv_localization_navigation.launch.py \
  localization_mode:=amcl \
  map:=$(pwd)/src/agv_navigation/maps/warehouse.yaml
```

这个启动文件会从单车 `nav2_params.yaml` 自动生成两份运行时参数，把 MPPI 和 Nav2 的 frame 和话题改成：

```text
agv_01_odom / agv_01_base_footprint / /agv_01/scan / /agv_01/cmd_vel
agv_02_odom / agv_02_base_footprint / /agv_02/scan / /agv_02/cmd_vel
```

对应 action server 为：

```text
/agv_01/navigate_to_pose
/agv_02/navigate_to_pose
```

检查两车定位 TF：

```bash
ros2 run tf2_ros tf2_echo map agv_01_base_footprint
ros2 run tf2_ros tf2_echo map agv_02_base_footprint
```

## MPPI 参数调试指南

### 问题：轨迹抖动（High-Frequency Oscillation）

**症状**：小车沿路径行进时左右摆动

**调试步骤**：
1. 降低 `temperature` → 0.2（采样集中度更高）
2. 增加 `time_steps` → 70（预测窗口更长）
3. 增加 `PathAlignCritic` 权重 → 16.0
4. 减少 `PreferForwardCritic` 权重 → 3.0

### 问题：反应迟缓（Slow Response）

**症状**：小车无法及时躲避动态障碍物

**调试步骤**：
1. 提高 `controller_frequency` → 20.0（控制更频繁）
2. 增加 `batch_size` → 3000（采样更多轨迹）
3. 增加 `CostCritic` 权重 → 5.0（更重视碰撞代价）
4. 降低 `gamma` → 0.01（学习率更小）

### 问题：绕路过长（Over-Conservative）

**症状**：小车在有通道的地方仍然大幅绕路

**调试步骤**：
1. 降低 `CostCritic` 权重 → 2.5
2. 降低 `inflation_radius` → 0.25（局部膨胀半径更小）
3. 增加 `GoalCritic` 权重 → 6.0（更激进地朝向目标）

### 问题：速度不稳定（Velocity Jitter）

**症状**：速度命令在快速变化，导致底盘抖动

**调试步骤**：
1. 启用 `velocity_smoother`（可能已启用，见 nav2_params.yaml）
2. 增加 `smoothing_frequency` → 30.0
3. 降低 `vx_std` 和 `wz_std` → 0.1（减少采样方差）

## 与其他包的关系

- 从 `agv_gazebo` 接收仓库环境中的仿真空间语义。
- 依赖 `agv_description` 中 `lidar_link` 的 Gazebo ray laser 提供 `/agv/scan` 传感器数据。
- 为 `agv_scheduler` 的 `navigate_to_pose` action 提供 MPPI 驱动的导航服务端。
- 由 `agv_bringup` 统一启动。

## 当前待补齐项

- `maps/` 目录初始为空。第一次建图后需要保存 `warehouse.yaml` 和 `warehouse.pgm`。
- `config/nav2_params.yaml` 中的 MPPI 参数已配置完整，但需根据实际仿真效果继续微调。
- 可选：在 `launch/` 中添加专门的 MPPI 参数调试启动文件，支持动态参数覆盖。
- 可选：为 RViz 增加 MPPI 轨迹可视化插件，实时展示采样的候选轨迹。
