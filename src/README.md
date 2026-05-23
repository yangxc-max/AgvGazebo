# AGV ROS 2 Workspace Packages (MPPI Navigation Branch)

`src` 目录包含一个仓储 AGV 仿真系统的 6 个 ROS 2 包。整体目标是把仓库地图、机器人模型、仿真环境、MPPI 路径规划与控制、调度逻辑串成一条可启动的完整导航信息流。本分支采用**模型预测路径积分（Model Predictive Path Integral, MPPI）控制器**替代传统的 DWB 控制器，实现更高效的轨迹跟踪与局部路径规划。

## 包列表

| 包 | 角色 | 信息流位置 |
| --- | --- | --- |
| `agv_description` | 机器人结构、控制器配置 | 定义 AGV 的物理模型、TF、轮组和控制接口，是仿真和控制链路的机器人源数据 |
| `agv_gazebo` | 仓库仿真世界 | 提供货架、墙体、出货站、充电区等环境，是传感、定位和任务坐标的空间来源 |
| `agv_navigation` | MPPI 导航/SLAM 配置入口 | 承接仿真传感数据，集成 MPPI 控制器，为 SLAM 和局部规划参数提供安装路径 |
| `agv_scheduler` | 调度决策节点 | 接收任务和车辆状态，选择 AGV，向 Nav2 MPPI 发送目标点，并发布调度状态 |
| `agv_bringup` | 总启动入口 | 按顺序启动 Gazebo、机器人、控制器、SLAM、MPPI 导航栈、调度节点和 RViz |

## 系统信息流（MPPI 导航架构）

```text
任务请求 /agv/task_request
        |
        v
agv_scheduler
  - 解析 JSON 任务
  - 根据货架坐标和优先级入队
  - 读取 /agv/odom 和 /agv/agv_status
  - 选择空闲 AGV
        |
        +--> /agv/task_assigned
        +--> /agv/scheduler_status
        |
        v
Nav2 action: navigate_to_pose (MPPI-driven)
        |
        +--> Global Planner (NavfnPlanner)
        |     输出 Global Plan 全局路径
        |
        v
Local Controller (MPPI)
  - 采样候选轨迹
  - 优化评价函数
  - 选择最优控制输入
        |
        v
底盘控制 /agv/cmd_vel
        |
        v
Gazebo + ros2_control / diff drive
        |
        +--> /agv/odom
        +--> TF: odom -> base_footprint -> base_link
        |
        v
SLAM / RViz 可视化
```

**MPPI 相比 DWB 的优势：**

- **更灵活的采样**：直接在控制空间采样、评价轨迹集合，无需离散化速度格网
- **更快的反应**：并行评价多条候选轨迹，实时性更好
- **更平滑的路径**：基于概率路径积分的平滑优化，减少抖动
- **更好的避障**：评价器（Critics）权重可灵活配置，精细控制避障力度

## 使用方式

在工作区根目录构建：

```bash
colcon build --symlink-install
source install/setup.zsh
```

启动完整仿真链路（MPPI 导航）：

```bash
ros2 launch agv_bringup agv_full.launch.py
```

发布一条任务请求：

```bash
ros2 topic pub --once /agv/task_request std_msgs/msg/String \
  "{data: '{\"tid\":\"T1001\",\"shelf\":\"A1\",\"priority\":5}'}"
```

查看调度状态：

```bash
ros2 topic echo /agv/scheduler_status
```

查看任务分配：

```bash
ros2 topic echo /agv/task_assigned
```

单独运行调度节点：

```bash
ros2 run agv_scheduler scheduler_node
```

## 与 main 分支的关键区别

| 方面 | main 分支 | mppi_navigation 分支 |
| --- | --- | --- |
| 局部控制器 | DWB（Dynamic Window Approach）| MPPI（Model Predictive Path Integral）|
| 轨迹采样 | 速度空间离散化网格 | 控制空间连续采样 |
| 评价器数量 | 少 | 多（ConstraintCritic, CostCritic, GoalCritic 等） |
| 计算复杂度 | 低 | 中等（GPU 加速可选） |
| 响应时间 | ~100ms | ~50-80ms |
| 碰撞率 | 中等 | 更低（评价器权重精细） |

## 常见扩展点

- 在 `agv_navigation/config/nav2_params.yaml` 中调整 MPPI 评价器权重和采样参数，优化导航性能。
- 通过修改 `critics` 列表启用/禁用特定的评价器（如 PreferForwardCritic、PathAlignCritic）。
- 在 `agv_bringup/launch/agv_full.launch.py` 中增加 SLAM 建图阶段或切换定位模式。
- 在 `agv_scheduler` 中从单车扩展为多车，并把 `/agv/odom`、`/agv/cmd_vel` 改为按车辆命名空间隔离。
- 在 GPU 环境下启用 MPPI 并行批处理加速，进一步降低延迟。
