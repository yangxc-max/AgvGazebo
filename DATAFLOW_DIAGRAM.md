# AGV 仓储系统数据流图

## 整体系统数据流

```mermaid
graph TB
    User["用户/调度系统"]
    TaskReq["任务请求<br/>/agv/task_request<br/>JSON: tid, shelf, priority"]
    
    Scheduler["AGVScheduler<br/>4层混合路权调度"]
    TaskAssign["/agv/task_assigned<br/>任务分配消息"]
    SchedStatus["/agv/scheduler_status<br/>调度状态监控"]
    
    Nav2["Nav2 Navigation Stack<br/>navigate_to_pose Action"]
    Gazebo["Gazebo 仓库仿真"]
    DiffDrive["gazebo_ros_diff_drive<br/>底盘模型"]
    
    CmdVel["/agv/cmd_vel<br/>速度指令"]
    Odom["/agv/odom<br/>里程计反馈"]
    Scan["/agv/scan<br/>激光雷达数据"]
    
    SLAM["SLAM Toolbox<br/>地图构建/定位"]
    RViz["RViz 可视化"]
    TF["TF 坐标变换<br/>odom→base_footprint→base_link"]
    
    User -->|发布| TaskReq
    TaskReq -->|订阅| Scheduler
    
    Scheduler -->|分析4层权重| Nav2
    Scheduler -->|发布| TaskAssign
    Scheduler -->|发布| SchedStatus
    Scheduler -->|发布| CmdVel
    
    Nav2 -->|导航目标| Scheduler
    Nav2 -->|订阅| Odom
    Nav2 -->|订阅| TF
    
    Gazebo -->|仿真| DiffDrive
    DiffDrive -->|订阅| CmdVel
    DiffDrive -->|发布| Odom
    DiffDrive -->|发布| Scan
    DiffDrive -->|发布| TF
    
    Odom -->|里程计数据| SLAM
    Scan -->|激光扫描| SLAM
    SLAM -->|发布地图| RViz
    
    TF -->|坐标树| RViz
    Odom -->|机器人位置| RViz
    Scan -->|点云数据| RViz
    
    TaskAssign -.->|监听| RViz
    SchedStatus -.->|监听| RViz
```

---

## 详细数据流：任务处理管道

```mermaid
graph LR
    subgraph "第1层：静态优先级"
        VClass["车辆类型<br/>EMERGENCY>LOADED<br/>EMPTY>PATROL"]
    end
    
    subgraph "第2层：任务紧迫性"
        Deadline["截止时间"]
        Value["货物价值"]
        Battery["电池电量"]
        Wait["等待时间<br/>反饥饿"]
        Urgency["紧迫性评分<br/>0~1"]
    end
    
    subgraph "第3层：时空预留"
        Path["规划路径"]
        Cell["网格单元<br/>cell_size=2.0m"]
        TimeWindow["时间窗口<br/>t±time_buffer"]
        Conflict["冲突检测"]
        Reserve["时空预留表"]
    end
    
    subgraph "第4层：本地仲裁"
        Distance["距离"]
        RemainingRoute["剩余路线"]
        YieldScore["让路得分"]
        Deadlock["死锁检测<br/>WFG"]
        Arbitrate["优先级仲裁"]
    end
    
    Task["任务队列"]
    VClass --> Task
    Deadline --> Urgency
    Value --> Urgency
    Battery --> Urgency
    Wait --> Urgency
    Urgency --> Task
    
    Task -->|候选AGV| Path
    Path --> Cell
    Cell --> TimeWindow
    TimeWindow --> Conflict
    Conflict -->|无冲突| Reserve
    
    Reserve -->|已预留| Distance
    Distance --> YieldScore
    RemainingRoute --> YieldScore
    YieldScore --> Arbitrate
    Deadlock --> Arbitrate
    
    Arbitrate -->|分配| Assignment["分配给AGV<br/>发送导航目标"]
```

---

## 单车任务执行流

```mermaid
graph TD
    Init["AGV初始化<br/>IDLE状态<br/>发送里程计数据"]
    
    TaskStart["任务开始<br/>TO_SHELF"]
    PickupGoal["目标1: 货架取货点<br/>pick_xy, pick_yaw"]
    PickupReach["到达取货点"]
    
    AisleExit["目标2: 通道出口<br/>aisle_exit_xy"]
    AisleReach["到达出口"]
    
    Station["目标3: 配送站<br/>station_xy"]
    StationReach["到达配送站"]
    
    Complete["任务完成<br/>IDLE状态"]
    Release["释放时空预留"]
    
    Init --> TaskStart
    TaskStart --> PickupGoal
    PickupGoal -->|Nav2导航| PickupReach
    PickupReach --> AisleExit
    AisleExit -->|Nav2导航| AisleReach
    AisleReach --> Station
    Station -->|Nav2导航| StationReach
    StationReach --> Complete
    Complete --> Release
    Release --> Init
```

---

## ROS 2 Topic/Action 完整映射

### 订阅关系

```
agv_scheduler:
  ├── 订阅 /agv/task_request (std_msgs/String)
  │   └── 来源：用户/外部调度
  ├── 订阅 /agv/odom (nav_msgs/Odometry)
  │   ├── 来源：gazebo_ros_diff_drive
  │   └── 用途：更新AGV位置、速度
  ├── 订阅 /agv/agv_status (std_msgs/String)
  │   ├── 来源：AGV状态发布器
  │   └── 用途：更新电池、状态
  └── Action客户端
      └── /agv/navigate_to_pose (nav2_msgs/NavigateToPose)
          └── 来源：Nav2 navigation stack

nav2_stack:
  ├── 订阅 /agv/odom (nav_msgs/Odometry)
  │   └── 来源：gazebo_ros_diff_drive
  ├── 订阅 /agv/scan (sensor_msgs/LaserScan)
  │   └── 来源：Gazebo laser plugin
  ├── 订阅 TF: odom -> base_footprint -> base_link
  │   └── 来源：robot_state_publisher + gazebo_ros
  └── 发布 /agv/cmd_vel (geometry_msgs/Twist)
      └── 目标：gazebo_ros_diff_drive

slam_toolbox:
  ├── 订阅 /agv/scan (sensor_msgs/LaserScan)
  │   └── 来源：Gazebo laser plugin
  ├── 订阅 /agv/odom (nav_msgs/Odometry)
  │   └── 来源：gazebo_ros_diff_drive
  └── 发布地图及TF

rviz2:
  ├── 订阅 /agv/task_assigned (std_msgs/String)
  ├── 订阅 /agv/scheduler_status (std_msgs/String)
  ├── 订阅 /agv/odom (nav_msgs/Odometry)
  ├── 订阅 /agv/scan (sensor_msgs/LaserScan)
  └── 订阅 TF树
```

### 发布关系

```
agv_scheduler (发布):
  ├── /agv/task_assigned (std_msgs/String)
  │   └── JSON: agv, tid, shelf, pick, drop, vehicle_class
  ├── /agv/scheduler_status (std_msgs/String)
  │   └── JSON: pending, completed, queue, fleet, spacetime_reservations
  └── /agv/cmd_vel (geometry_msgs/Twist)
      └── 紧急制动指令

gazebo_ros_diff_drive (发布):
  ├── /agv/odom (nav_msgs/Odometry)
  │   └── 位置: x, y, theta; 速度: vx, wz
  ├── /agv/scan (sensor_msgs/LaserScan)
  │   └── 激光雷达360度扫描
  └── TF: odom, base_footprint, base_link

nav2_stack (发布):
  ├── /agv/cmd_vel (geometry_msgs/Twist)
  │   └── 导航控制速度
  └── Action反馈: navigate_to_pose

robot_state_publisher (发布):
  └── TF: base_link及其所有子link
```

---

## 多车场景数据流

```mermaid
graph TB
    Task["任务队列"]
    
    subgraph "AGV_01分支"
        AGV1_Sched["Scheduler<br/>agv_01"]
        AGV1_Nav2["Nav2<br/>agv_01"]
        AGV1_Gazebo["Gazebo<br/>agv_01实体"]
        AGV1_SLAM["SLAM<br/>agv_01"]
        AGV1_Topics["Topics<br/>/agv_01/odom<br/>/agv_01/cmd_vel<br/>/agv_01/scan"]
    end
    
    subgraph "AGV_02分支"
        AGV2_Sched["Scheduler<br/>agv_02"]
        AGV2_Nav2["Nav2<br/>agv_02"]
        AGV2_Gazebo["Gazebo<br/>agv_02实体"]
        AGV2_SLAM["SLAM<br/>agv_02"]
        AGV2_Topics["Topics<br/>/agv_02/odom<br/>/agv_02/cmd_vel<br/>/agv_02/scan"]
    end
    
    SharedSched["共享Scheduler<br/>4层权重评估<br/>时空预留检测"]
    SharedGazebo["共享Gazebo World<br/>warehouse.world"]
    RViz["RViz可视化<br/>显示两车状态"]
    
    Task --> SharedSched
    SharedSched -->|分配| AGV1_Sched
    SharedSched -->|分配| AGV2_Sched
    
    AGV1_Sched --> AGV1_Nav2
    AGV1_Nav2 --> AGV1_Topics
    AGV1_Topics --> SharedGazebo
    SharedGazebo --> AGV1_Gazebo
    AGV1_Gazebo --> AGV1_Topics
    AGV1_Topics --> AGV1_SLAM
    
    AGV2_Sched --> AGV2_Nav2
    AGV2_Nav2 --> AGV2_Topics
    AGV2_Topics --> SharedGazebo
    SharedGazebo --> AGV2_Gazebo
    AGV2_Gazebo --> AGV2_Topics
    AGV2_Topics --> AGV2_SLAM
    
    AGV1_Topics --> RViz
    AGV2_Topics --> RViz
    AGV1_SLAM --> RViz
    AGV2_SLAM --> RViz
```

---

## 时空预留与冲突检测流

```mermaid
graph LR
    TaskIn["任务来临<br/>tid, shelf_id"]
    
    GetCandidates["筛选候选AGV<br/>·空闲状态<br/>·电量>15%<br/>·最近距离优先"]
    
    PlanPath["规划任务路径<br/>当前位置<br/>→取货点<br/>→通道出口<br/>→配送站"]
    
    PathToCells["路径转网格单元<br/>·cell_size=2.0m<br/>·添加时间戳ETA<br/>·去重保序"]
    
    CheckConflict["冲突检测<br/>遍历每个cell<br/>检查时间窗口重叠"]
    
    subgraph "冲突处理"
        NoConflict["✓无冲突"]
        HasConflict["✗有冲突"]
    end
    
    Reserve["时空预留<br/>·写入预留表<br/>·路径分配给AGV<br/>·hold_timeout=300s"]
    
    Assign["任务分配<br/>发布task_assigned<br/>发送Nav2目标"]
    
    Requeue["重新入队<br/>更新重试时间<br/>等待资源释放"]
    
    TaskIn --> GetCandidates
    GetCandidates --> PlanPath
    PlanPath --> PathToCells
    PathToCells --> CheckConflict
    CheckConflict --> NoConflict
    CheckConflict --> HasConflict
    
    NoConflict --> Reserve
    Reserve --> Assign
    
    HasConflict --> Requeue
```

---

## 4层权重计算详解

### 第1层：静态优先级（Layer-1）

```
Vehicle Class Priority:
  EMERGENCY  = 4  (最高)
  LOADED     = 3  (满载
  EMPTY      = 2  (空载)
  PATROL     = 1  (最低)
  
static_score = vehicle_class.value * 10
```

### 第2层：任务紧迫性（Layer-2）

```
urgency_score = 0.35×deadline_score + 0.25×value_score + 0.20×battery_score + 0.20×wait_score

deadline_score:
  - 计算距截止时间秒数
  - 窗口: 600s(10分钟)
  - 分数范围: [0, 1]

value_score:
  - 货物价值 / 100
  - 范围: [0, 1]

battery_score:
  - (50.0 - agv_battery) / 50.0
  - 低电量→高分
  - 范围: [0, 1]

wait_score:
  - min(waited_seconds / 300s, 1.0)
  - 防止任务饥饿
  - 范围: [0, 1]
```

### 第3层：时空预留（Layer-3）

```
SpacetimeSlot {
  agv_id,
  task_id,
  cell_key: "c:gx:gy",
  t_start: timestamp - time_buffer,
  t_end: timestamp + time_buffer,
  expires_at: timestamp + hold_timeout
}

冲突判定:
  if cell_overlaps AND time_window_overlaps:
    conflict = True
```

### 第4层：本地仲裁（Layer-4）

```
yield_score = static_priority*10 + urgency*5 - reverse_cost*2

当两车接近(dist < safety_stop_distance):
  - 计算双方yield_score
  - 低分AGV让路
  - 让路AGV: 取消目标, 重新入队, 等待retry_delay=8s

死锁检测(Wait-For Graph):
  - 构建等待图: {agv_id: waiting_for_agv_id}
  - DFS检测环
  - 找到环中最低分AGV, 抢占其任务
```

---

## 实时监控与状态发布

```mermaid
graph TB
    Scheduler["AGVScheduler<br/>维护内部状态"]
    
    PubStatus["发布 /agv/scheduler_status<br/>周期: 0.5s"]
    
    StatusJSON["JSON Payload:<br/>·pending: 队列中任务数<br/>·completed: 完成任务数<br/>·queue: 所有任务详情<br/>·spacetime_reservations: 预留表大小<br/>·fleet: 每车详细信息"]
    
    FleetDetail["每车信息:<br/>·state: IDLE/TO_SHELF/...<br/>·vehicle_class<br/>·position: [x, y]<br/>·battery<br/>·velocity: [vx, wz]<br/>·task_id<br/>·goal: [x, y]<br/>·goal_active<br/>·yield_to"]
    
    RViz["RViz仪表板"]
    
    Scheduler -->|定时发布| PubStatus
    PubStatus --> StatusJSON
    StatusJSON --> FleetDetail
    FleetDetail --> RViz
    RViz -->|实时展示| Vehicle["车队动态"]
    RViz -->|实时展示| Queue["任务队列进度"]
    RViz -->|实时展示| Conflict["冲突&让路状态"]
```

---

## 错误处理与恢复流

```mermaid
graph TD
    NavSend["发送Nav2目标"]
    
    ErrorTypes{错误类型}
    
    NoServer["Nav2服务<br/>不可用<br/>timeout > 1s"]
    NoAccept["Nav2拒绝<br/>目标"]
    NavFailed["导航失败<br/>碰撞/超时"]
    
    Requeue["重新入队"]
    RetryDelay["设置重试延迟<br/>retry_after+=5s"]
    ErrorLog["记录错误信息"]
    
    NavSend --> ErrorTypes
    ErrorTypes -->|服务未就绪| NoServer
    ErrorTypes -->|目标被拒| NoAccept
    ErrorTypes -->|导航失败| NavFailed
    
    NoServer --> Requeue
    NoAccept --> Requeue
    NavFailed --> Requeue
    
    Requeue --> RetryDelay
    RetryDelay --> ErrorLog
    ErrorLog --> Init["等待重试<br/>返回IDLE"]
```

---

## 自动演示模式

```mermaid
graph LR
    AutoDemo["自动演示模式<br/>周期: 15s"]
    
    GenTask["随机生成任务<br/>·随机货架<br/>·随机优先级 1~5<br/>·随机货值 1~100"]
    
    PubTask["发布到<br/>/agv/task_request"]
    
    Counter["计数器<br/>最多8个任务"]
    
    Stop["停止演示"]
    
    AutoDemo --> GenTask
    GenTask --> PubTask
    PubTask --> Counter
    Counter -->|n<8| AutoDemo
    Counter -->|n≥8| Stop
```

---

## 系统时序图：完整任务生命周期

```mermaid
sequenceDiagram
    participant User as 用户/系统
    participant Sched as Scheduler
    participant Nav2 as Nav2
    participant Gazebo as Gazebo
    participant SLAM as SLAM

    User->>Sched: 发布/agv/task_request
    Note over Sched: 解析任务, 入队

    rect rgb(200, 150, 255)
    Note over Sched: 第1~3层权重评估
    Sched->>Sched: 计算优先级、紧迫性
    Sched->>Sched: 规划路径、检查冲突
    Sched->>Sched: 时空预留
    end

    Sched->>Nav2: 发送目标1(pick_xy, pick_yaw)
    Nav2->>Gazebo: 订阅/agv/odom
    Gazebo->>Nav2: 发布里程计反馈
    Nav2->>Gazebo: 发布/agv/cmd_vel
    Gazebo->>Gazebo: 更新AGV位置
    Nav2-->>Sched: 目标1完成

    rect rgb(200, 150, 255)
    Note over Sched: 第4层本地仲裁(可选)
    Sched->>Sched: 检测接近车辆
    Sched->>Sched: 计算yield_score
    end

    Sched->>Nav2: 发送目标2(aisle_exit)
    Nav2->>Gazebo: 导航...
    Nav2-->>Sched: 目标2完成

    Sched->>Nav2: 发送目标3(station)
    Nav2->>Gazebo: 导航...
    Nav2-->>Sched: 目标3完成

    Sched->>Sched: 任务标记为done
    Sched->>Sched: 释放时空预留
    Sched-->>User: 发布/agv/task_assigned + /agv/scheduler_status
    SLAM->>SLAM: 地图/定位更新
```

---

## Topic消息格式详解

### /agv/task_request

```json
{
  "tid": "T1001",              // 任务ID
  "shelf": "A1",               // 货架号
  "priority": 5,               // 用户优先级 1~10
  "cargo_value": 50.5,         // 货物价值 (可选)
  "deadline": 1234567890.5,    // Unix timestamp (可选)
  "agv_id": "agv_01"           // 指定AGV (可选)
}
```

### /agv/task_assigned

```json
{
  "agv": "agv_01",
  "tid": "T1001",
  "shelf": "A1",
  "shelf_center": [-9.0, 7.0],
  "pick": [-9.0, 5.0],
  "pick_yaw": 1.5708,
  "aisle_exit": [5.5, 5.0],
  "drop": [6.4, 0.0],
  "priority": 5,
  "cargo_value": 50.5,
  "deadline": 0.0,
  "vehicle_class": "EMPTY",
  "nav_action": "navigate_to_pose"
}
```

### /agv/scheduler_status

```json
{
  "pending": 3,
  "completed": 5,
  "queue": [
    {
      "tid": "T1002",
      "shelf": "B2",
      "status": "pending",
      "requested_agv": "",
      "retry_in": 0.0,
      "last_error": "",
      "cargo_value": 30.0,
      "deadline": 0.0
    }
  ],
  "spacetime_reservations": 45,
  "fleet": {
    "agv_01": {
      "state": "to_shelf",
      "vehicle_class": "EMPTY",
      "pos": [-5.2, 3.1],
      "battery": 85.5,
      "vx": 0.45,
      "wz": 0.02,
      "task": "T1001",
      "nav_action": "navigate_to_pose",
      "yield_to": "",
      "goal": [-9.0, 5.0],
      "goal_active": true
    }
  }
}
```

---

## 性能指标监控点

```
关键性能指标(KPI):
  1. 任务吞吐量: 已完成任务数/小时
  2. 平均等待时间: (完成时间 - 请求时间)
  3. 冲突率: 触发重新入队次数 / 总任务数
  4. 死锁事件: 检测到死锁环数
  5. 车辆利用率: 运行时间 / 总时间
  6. 路径重规划次数: 导航失败重试次数
  7. 时空预留表大小: 监控内存使用
  8. 调度循环延迟: 任务发现→分配时间

监控点:
  - Scheduler: _sched_loop (1.0s)
  - Safety: _safety_loop (0.2s)
  - Watchdog: _nav_watchdog (1.0s)
  - Deadlock: _deadlock_check (2.0s周期)
  - Status: _pub_status (0.5s)
```

---

## 扩展与集成接口

```mermaid
graph TB
    External["外部系统"]
    
    subgraph "输入接口"
        REST["REST API<br/>POST /task"]
        AMQP["AMQP/MQ<br/>task_queue"]
        WebSocket["WebSocket<br/>实时任务流"]
    end
    
    subgraph "核心系统"
        TaskBridge["任务转换器<br/>统一转JSON"]
        Scheduler["AGV Scheduler"]
    end
    
    subgraph "输出接口"
        Monitor["监控仪表板<br/>RViz/Web"]
        Database["数据库持久化<br/>任务历史"]
        Alert["告警系统<br/>死锁/故障"]
    end
    
    External --> REST
    External --> AMQP
    External --> WebSocket
    
    REST --> TaskBridge
    AMQP --> TaskBridge
    WebSocket --> TaskBridge
    
    TaskBridge --> Scheduler
    
    Scheduler --> Monitor
    Scheduler --> Database
    Scheduler --> Alert
```

---

**更新日期**: 2026-05-07  
**系统架构**: ROS 2 + Nav2 + SLAM Toolbox + Gazebo  
**关键组件**: AGV Scheduler (4层混合路权) | Nav2 Navigation | gazebo_ros (仿真底座)
