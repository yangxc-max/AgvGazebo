#!/usr/bin/env python3
"""
AGV warehouse scheduler — Hybrid Right-of-Way Model (四层混合路权)

Layer 1 · Static Priority     — vehicle class baseline (emergency > loaded > empty > patrol)
Layer 2 · Task Urgency         — deadline / value / battery dynamic weight; prevents starvation
Layer 3 · Spacetime Reservation— CBS-inspired (x, y, t) conflict table; each AGV registers
                                  "I will be at cell X at time T"; newcomers must route around
Layer 4 · Local Arbitration    — yield-score = f(priority, reverse-cost, remaining-route);
                                  lower score yields; deadlock detection via Wait-For Graph (WFG)
"""

import json
import math
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Set, Tuple

import yaml
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# Enumerations
# ---------------------------------------------------------------------------

class State(Enum):
    IDLE         = "idle"
    TO_SHELF     = "to_shelf"
    PICKING      = "picking"
    TO_AISLE_EXIT= "to_aisle_exit"
    TO_STATION   = "to_station"
    DELIVERING   = "delivering"
    TO_CHARGE    = "to_charge"
    CHARGING     = "charging"
    WAITING      = "waiting"
    YIELDING     = "yielding"   # NEW: actively yielding to a higher-score peer
    ERROR        = "error"


class VehicleClass(Enum):
    """Layer-1 static priority — higher number = higher base priority."""
    PATROL    = 1   # 巡检车
    EMPTY     = 2   # 空载车
    LOADED    = 3   # 重货车
    EMERGENCY = 4   # 紧急车


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class Task:
    tid: str
    shelf: str
    shelf_center_xy: Tuple[float, float]
    pick_xy: Tuple[float, float]
    pick_yaw: float
    aisle_exit_xy: Tuple[float, float]
    drop_xy: Tuple[float, float]
    priority: int = 1           # user-supplied raw priority
    cargo_value: float = 1.0    # Layer-2: higher value → higher urgency
    deadline: float = 0.0       # Layer-2: unix timestamp, 0 = no deadline
    ts: float = field(default_factory=time.time)
    agv: str = ""
    requested_agv: str = ""
    status: str = "pending"
    retry_after: float = 0.0
    last_error: str = ""
    enqueue_time: float = field(default_factory=time.time)  # starvation guard

    def __lt__(self, other: "Task") -> bool:
        return self.priority > other.priority


@dataclass
class AGVState:
    aid: str
    nav_action: str
    odom_topic: str
    cmd_vel_topic: str
    status_topic: str
    vehicle_class: VehicleClass = VehicleClass.EMPTY
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    state: State = State.IDLE
    battery: float = 100.0
    task: Optional[Task] = None
    vx: float = 0.0
    wz: float = 0.0
    last_odom_ts: float = 0.0
    current_goal_handle: object = None
    current_goal_xy: Optional[Tuple[float, float]] = None
    current_goal_yaw: float = 0.0
    nav_goal_sent_ts: float = 0.0
    nav_goal_accepted_ts: float = 0.0
    # Layer-4 arbitration
    yield_to: str = ""          # aid of the AGV this one is yielding to
    yield_until: float = 0.0    # timestamp when yield expires
    # Layer-3 spacetime path (list of (cell_key, eta) tuples)
    spacetime_path: List[Tuple[str, float]] = field(default_factory=list)


@dataclass(frozen=True)
class ShelfLocation:
    center_xy: Tuple[float, float]
    pick_xy: Tuple[float, float]
    pick_yaw: float = 0.0


# ---------------------------------------------------------------------------
# Layer-3: Spacetime reservation table
# ---------------------------------------------------------------------------

@dataclass
class SpacetimeSlot:
    """One cell claimed by one AGV at a particular time window."""
    agv_id: str
    task_id: str
    t_start: float          # earliest the AGV may be there
    t_end: float            # latest the AGV may be there (+ buffer)
    expires_at: float       # wall-clock expiry for stale cleanup


class SpacetimeTable:
    """
    CBS-inspired shared table: cell_key → list[SpacetimeSlot].
    Thread-safe via the caller's lock (AGVScheduler.lock).
    """

    def __init__(self, cell_size: float = 2.0, time_buffer: float = 4.0):
        self.cell_size   = cell_size
        self.time_buffer = time_buffer          # seconds of padding around slot
        self._table: Dict[str, List[SpacetimeSlot]] = {}

    # ---- helpers -----------------------------------------------------------

    def _cell(self, x: float, y: float) -> str:
        gx = math.floor(x / self.cell_size)
        gy = math.floor(y / self.cell_size)
        return f"c:{gx}:{gy}"

    def _path_cells_eta(
            self,
            waypoints: List[Tuple[float, float]],
            speed: float = 0.5,
            t0: float = 0.0,
    ) -> List[Tuple[str, float]]:
        """Return [(cell_key, eta), …] along a multi-segment path."""
        result: List[Tuple[str, float]] = []
        t = t0
        prev = waypoints[0]
        for wp in waypoints[1:]:
            dist = math.hypot(wp[0] - prev[0], wp[1] - prev[1])
            steps = max(1, int(math.ceil(dist / self.cell_size)))
            for s in range(steps + 1):
                ratio = s / steps
                x = prev[0] + (wp[0] - prev[0]) * ratio
                y = prev[1] + (wp[1] - prev[1]) * ratio
                eta = t + (dist * ratio / speed)
                result.append((self._cell(x, y), eta))
            t += dist / speed
            prev = wp
        # deduplicate while preserving order
        seen: Set[str] = set()
        unique: List[Tuple[str, float]] = []
        for ck, eta in result:
            if ck not in seen:
                seen.add(ck)
                unique.append((ck, eta))
        return unique

    # ---- public API --------------------------------------------------------

    def conflicts(
            self,
            agv_id: str,
            waypoints: List[Tuple[float, float]],
            speed: float = 0.5,
    ) -> List[str]:
        """
        Return list of agv_ids that conflict with this proposed path.
        A conflict exists when another AGV's slot overlaps the same cell
        at an overlapping time window.
        """
        t0 = time.time()
        path = self._path_cells_eta(waypoints, speed, t0)
        conflicting: List[str] = []
        for ck, eta in path:
            for slot in self._table.get(ck, []):
                if slot.agv_id == agv_id:
                    continue
                if slot.t_start - self.time_buffer <= eta <= slot.t_end + self.time_buffer:
                    if slot.agv_id not in conflicting:
                        conflicting.append(slot.agv_id)
        return conflicting

    def reserve(
            self,
            agv_id: str,
            task_id: str,
            waypoints: List[Tuple[float, float]],
            speed: float = 0.5,
            hold: float = 300.0,
    ) -> List[Tuple[str, float]]:
        """
        Register slots for an AGV's planned path.
        Returns the (cell_key, eta) list so the AGV can store it.
        Old slots for this AGV are removed first.
        """
        self.release(agv_id)
        t0 = time.time()
        path = self._path_cells_eta(waypoints, speed, t0)
        exp = t0 + hold
        for ck, eta in path:
            slot = SpacetimeSlot(
                agv_id=agv_id,
                task_id=task_id,
                t_start=eta - self.time_buffer,
                t_end=eta + self.time_buffer,
                expires_at=exp,
            )
            self._table.setdefault(ck, []).append(slot)
        return path

    def release(self, agv_id: str):
        for ck in list(self._table):
            self._table[ck] = [s for s in self._table[ck] if s.agv_id != agv_id]
            if not self._table[ck]:
                del self._table[ck]

    def cleanup_stale(self):
        now = time.time()
        for ck in list(self._table):
            self._table[ck] = [s for s in self._table[ck] if s.expires_at > now]
            if not self._table[ck]:
                del self._table[ck]

    def snapshot(self) -> Dict:
        return {
            ck: [
                {"agv": s.agv_id, "t_start": round(s.t_start, 1),
                 "t_end": round(s.t_end, 1)}
                for s in slots
            ]
            for ck, slots in self._table.items()
        }


# ---------------------------------------------------------------------------
# Layer-2: Task urgency scorer
# ---------------------------------------------------------------------------

class UrgencyScorer:
    """
    Computes a composite urgency weight in [0, 1] for a task.
    Prevents 'big-boss always first, minion starves' by including
    wait-time as a component.
    """

    W_DEADLINE  = 0.35
    W_VALUE     = 0.25
    W_BATTERY   = 0.20
    W_WAIT      = 0.20   # anti-starvation component

    MAX_WAIT_S  = 300.0  # after 5 min a task reaches full wait-score
    DEADLINE_WINDOW = 600.0  # deadlines within 10 min are considered urgent

    @classmethod
    def score(cls, task: Task, agv_battery: float) -> float:
        now = time.time()

        # Deadline urgency (higher when deadline is imminent)
        if task.deadline > 0:
            remaining = max(0.0, task.deadline - now)
            dl_score = 1.0 - min(remaining / cls.DEADLINE_WINDOW, 1.0)
        else:
            dl_score = 0.0

        # Cargo value (normalised to [0,1] assuming max_value ~ 100)
        val_score = min(task.cargo_value / 100.0, 1.0)

        # Battery urgency (low battery → send to charge sooner)
        bat_score = max(0.0, (50.0 - agv_battery) / 50.0)

        # Anti-starvation (longer wait → higher score)
        waited = now - task.enqueue_time
        wait_score = min(waited / cls.MAX_WAIT_S, 1.0)

        return (
            cls.W_DEADLINE * dl_score
            + cls.W_VALUE   * val_score
            + cls.W_BATTERY * bat_score
            + cls.W_WAIT    * wait_score
        )


# ---------------------------------------------------------------------------
# Layer-4: Arbitration & deadlock detection
# ---------------------------------------------------------------------------

class Arbitrator:
    """
    Yield-score computation and Wait-For Graph deadlock detection.
    """

    @staticmethod
    def yield_score(agv: AGVState, task: Optional[Task]) -> float:
        """
        Lower score → this AGV should yield.
        score = static_priority * 10 + task_urgency * 5 - reverse_cost * 2
        """
        static = agv.vehicle_class.value * 10
        urgency = 0.0
        if task:
            urgency = UrgencyScorer.score(task, agv.battery) * 5
        # Rough reverse cost: distance still to travel (more remaining = don't interrupt)
        remaining = 0.0
        if agv.current_goal_xy and task:
            remaining = math.hypot(
                agv.current_goal_xy[0] - agv.x,
                agv.current_goal_xy[1] - agv.y,
            )
        reverse_cost = min(remaining / 10.0, 2.0) * 2  # normalised penalty
        return static + urgency - reverse_cost

    @staticmethod
    def detect_deadlock(wait_for: Dict[str, str]) -> List[List[str]]:
        """
        Given {agv_id: waiting_for_agv_id}, return all cycles (deadlocks).
        Uses DFS with color marking.
        """
        WHITE, GRAY, BLACK = 0, 1, 2
        color: Dict[str, int] = {aid: WHITE for aid in wait_for}
        path: List[str] = []
        cycles: List[List[str]] = []

        def dfs(node: str):
            color[node] = GRAY
            path.append(node)
            nxt = wait_for.get(node)
            if nxt and nxt in color:
                if color[nxt] == GRAY:
                    # found cycle
                    idx = path.index(nxt)
                    cycles.append(list(path[idx:]))
                elif color[nxt] == WHITE:
                    dfs(nxt)
            path.pop()
            color[node] = BLACK

        for node in list(wait_for):
            if color.get(node) == WHITE:
                dfs(node)
        return cycles


# ---------------------------------------------------------------------------
# Main scheduler node
# ---------------------------------------------------------------------------

class AGVScheduler(Node):

    DEFAULT_SHELVES = {
        "A1": ShelfLocation((-9.0,  7.0), (-9.0,  5.0),  math.pi / 2),
        "A2": ShelfLocation((-5.0,  7.0), (-5.0,  5.0),  math.pi / 2),
        "A3": ShelfLocation((-1.0,  7.0), (-1.0,  5.0),  math.pi / 2),
        "A4": ShelfLocation(( 3.0,  7.0), ( 3.0,  5.0),  math.pi / 2),
        "B1": ShelfLocation((-9.0,  3.0), (-9.0,  1.0),  math.pi / 2),
        "B2": ShelfLocation((-5.0,  3.0), (-5.0,  1.0),  math.pi / 2),
        "B3": ShelfLocation((-1.0,  3.0), (-1.0,  1.0),  math.pi / 2),
        "B4": ShelfLocation(( 3.0,  3.0), ( 3.0,  1.0),  math.pi / 2),
        "C1": ShelfLocation((-9.0, -3.0), (-9.0, -1.0), -math.pi / 2),
        "C2": ShelfLocation((-5.0, -3.0), (-5.0, -1.0), -math.pi / 2),
        "C3": ShelfLocation((-1.0, -3.0), (-1.0, -1.0), -math.pi / 2),
        "C4": ShelfLocation(( 3.0, -3.0), ( 3.0, -1.0), -math.pi / 2),
        "D1": ShelfLocation((-9.0, -7.0), (-9.0, -5.0), -math.pi / 2),
        "D2": ShelfLocation((-5.0, -7.0), (-5.0, -5.0), -math.pi / 2),
        "D3": ShelfLocation((-1.0, -7.0), (-1.0, -5.0), -math.pi / 2),
        "D4": ShelfLocation(( 3.0, -7.0), ( 3.0, -5.0), -math.pi / 2),
    }
    DEFAULT_STATION  = (6.4,  0.0)
    DEFAULT_CHARGING = (9.0, -8.0)
    DEFAULT_AISLE_EXIT_X = 5.5

    # Assumed average navigation speed for ETA calculation
    NAV_SPEED = 0.5   # m/s

    def __init__(self):
        super().__init__("agv_scheduler")
        self._declare_params()

        self.shelves, self.station_xy, self.charging_xy = (
            self._load_warehouse_layout())

        self.route_cell_size    = float(self.get_parameter("route_cell_size").value)
        self.route_hold_timeout = float(self.get_parameter("route_hold_timeout").value)
        self.safety_stop_distance = float(self.get_parameter("safety_stop_distance").value)
        self.auto_demo_enabled  = bool(self.get_parameter("auto_demo_enabled").value)
        # Layer-4 yield parameters
        self.yield_duration     = float(self.get_parameter("yield_duration").value)
        self.deadlock_check_period = float(self.get_parameter("deadlock_check_period").value)

        agv_ids       = self._string_list_param("agv_ids", ["agv_01"])
        nav_actions   = self._expanded_param("nav_action_names", ["navigate_to_pose"], len(agv_ids))
        odom_topics   = self._expanded_param("odom_topics",      ["/agv/odom"],       len(agv_ids))
        cmd_vel_topics= self._expanded_param("cmd_vel_topics",   ["/agv/cmd_vel"],    len(agv_ids))
        status_topics = self._expanded_param("status_topics",    ["/agv/agv_status"], len(agv_ids))
        vclass_strs   = self._expanded_param("vehicle_classes",  ["EMPTY"],           len(agv_ids))

        # Layer-3 spacetime table (shared, guarded by self.lock)
        self.stt = SpacetimeTable(
            cell_size=self.route_cell_size,
            time_buffer=self.get_parameter("spacetime_time_buffer").value,
        )

        self.agvs: Dict[str, AGVState] = {}
        self.nav_clients: Dict[str, ActionClient] = {}
        self.cmd_publishers: Dict[str, object] = {}

        for idx, aid in enumerate(agv_ids):
            try:
                vc = VehicleClass[vclass_strs[idx].upper()]
            except KeyError:
                vc = VehicleClass.EMPTY

            agv = AGVState(
                aid=aid,
                nav_action=nav_actions[idx],
                odom_topic=odom_topics[idx],
                cmd_vel_topic=cmd_vel_topics[idx],
                status_topic=status_topics[idx],
                vehicle_class=vc,
            )
            self.agvs[aid] = agv
            self.nav_clients[aid] = ActionClient(self, NavigateToPose, agv.nav_action)
            self.cmd_publishers[aid] = self.create_publisher(Twist, agv.cmd_vel_topic, 10)
            self.create_subscription(
                Odometry,
                agv.odom_topic,
                lambda msg, robot_id=aid: self._on_odom(robot_id, msg),
                10,
            )

        for topic in sorted({agv.status_topic for agv in self.agvs.values()}):
            self.create_subscription(String, topic, self._on_status, 10)

        self.queue:   List[Task] = []
        self.history: List[Task] = []
        self.lock = threading.Lock()
        self._task_cnt = 0
        self._demo_n   = 0

        self.create_subscription(String, "/agv/task_request",  self._on_task, 10)
        self.pub_assign = self.create_publisher(String, "/agv/task_assigned",    10)
        self.pub_sched  = self.create_publisher(String, "/agv/scheduler_status", 10)

        self.create_timer(1.0,  self._sched_loop)
        self.create_timer(0.5,  self._pub_status)
        self.create_timer(0.2,  self._safety_loop)
        self.create_timer(1.0,  self._nav_watchdog)
        self.create_timer(15.0, self._auto_demo)
        self.create_timer(self.deadlock_check_period, self._deadlock_check)

        self.get_logger().info("=" * 60)
        self.get_logger().info("  AGV Scheduler — Hybrid Right-of-Way (4-layer)")
        self.get_logger().info("=" * 60)
        self.get_logger().info(
            f"Shelves: {len(self.shelves)} | "
            f"station=({self.station_xy[0]:.1f},{self.station_xy[1]:.1f}) | "
            f"AGVs: {', '.join(self.agvs)}"
        )

    # -----------------------------------------------------------------------
    # Parameter helpers
    # -----------------------------------------------------------------------

    def _declare_params(self):
        self.declare_parameter("agv_ids",              ["agv_01"])
        self.declare_parameter("nav_action_names",     ["navigate_to_pose"])
        self.declare_parameter("odom_topics",          ["/agv/odom"])
        self.declare_parameter("cmd_vel_topics",       ["/agv/cmd_vel"])
        self.declare_parameter("status_topics",        ["/agv/agv_status"])
        self.declare_parameter("vehicle_classes",      ["EMPTY"])   # Layer-1
        self.declare_parameter("route_cell_size",      2.0)
        self.declare_parameter("route_hold_timeout",   300.0)
        self.declare_parameter("spacetime_time_buffer",4.0)         # Layer-3
        self.declare_parameter("safety_stop_distance", 1.0)
        self.declare_parameter("yield_duration",       8.0)         # Layer-4
        self.declare_parameter("deadlock_check_period",2.0)         # Layer-4
        self.declare_parameter("auto_demo_enabled",    True)
        self.declare_parameter("shelf_layout_file",    "")

    def _load_warehouse_layout(
            self,
    ) -> Tuple[Dict[str, ShelfLocation], Tuple[float, float], Tuple[float, float]]:
        layout_file = str(self.get_parameter("shelf_layout_file").value or "")
        if not layout_file:
            self.get_logger().warn("No shelf_layout_file; using built-in layout")
            return dict(self.DEFAULT_SHELVES), self.DEFAULT_STATION, self.DEFAULT_CHARGING

        with open(layout_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        layout = data.get("warehouse_layout", data)
        raw_shelves = layout.get("shelves", {})
        if not raw_shelves:
            raise ValueError(f"No shelves defined in {layout_file}")

        shelves: Dict[str, ShelfLocation] = {}
        for shelf_id, raw in raw_shelves.items():
            if not isinstance(raw, dict):
                raise ValueError(f"Shelf {shelf_id} must be a mapping")
            center   = self._xy_from_config(raw.get("center"), f"{shelf_id}.center")
            pickup   = self._xy_from_config(raw.get("pickup"), f"{shelf_id}.pickup")
            yaw      = self._yaw_from_config(raw, f"{shelf_id}.pickup_yaw")
            shelves[str(shelf_id)] = ShelfLocation(center, pickup, yaw)

        raw_station = layout.get("station", {})
        station  = self._xy_from_config(
            raw_station.get("dock", raw_station.get("center")), "station.dock")
        charging = self._xy_from_config(
            layout.get("charging", {}).get("center"), "charging.center")

        self.get_logger().info(f"Loaded shelf layout: {layout_file}")
        return shelves, station, charging

    def _xy_from_config(self, value, name: str) -> Tuple[float, float]:
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            raise ValueError(f"{name} must be a two-item [x, y] list")
        return (float(value[0]), float(value[1]))

    def _yaw_from_config(self, raw: dict, name: str) -> float:
        if "pickup_yaw"     in raw: return float(raw["pickup_yaw"])
        if "pickup_yaw_deg" in raw: return math.radians(float(raw["pickup_yaw_deg"]))
        return 0.0

    def _string_list_param(self, name: str, default: List[str]) -> List[str]:
        value = self.get_parameter(name).value
        if value is None:
            return default
        items = [item.strip() for item in (value.split(",") if isinstance(value, str) else value)]
        return [i for i in items if i] or default

    def _expanded_param(self, name: str, default: List[str], count: int) -> List[str]:
        values = self._string_list_param(name, default)
        if len(values) == 1 and count > 1:
            return [values[0]] * count
        if len(values) != count:
            raise ValueError(f"Parameter {name} must have 1 or {count} entries, got {len(values)}")
        return values

    # -----------------------------------------------------------------------
    # Odometry / status callbacks
    # -----------------------------------------------------------------------

    def _on_odom(self, aid: str, msg: Odometry):
        p = msg.pose.pose
        q = p.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        with self.lock:
            agv = self.agvs.get(aid)
            if not agv:
                return
            agv.x   = round(p.position.x, 3)
            agv.y   = round(p.position.y, 3)
            agv.yaw = round(math.atan2(siny, cosy), 3)
            agv.vx  = round(msg.twist.twist.linear.x,  3)
            agv.wz  = round(msg.twist.twist.angular.z, 3)
            agv.last_odom_ts = time.time()

    def _on_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            aid  = data.get("agv_id") or data.get("agv")
            if aid not in self.agvs:
                return
            with self.lock:
                agv = self.agvs[aid]
                agv.state   = State(data.get("state", agv.state.value))
                agv.battery = float(data.get("battery", agv.battery))
        except Exception as exc:
            self.get_logger().warn(f"AGV status parse error: {exc}")

    # -----------------------------------------------------------------------
    # Task ingestion
    # -----------------------------------------------------------------------

    def _on_task(self, msg: String):
        try:
            data = json.loads(msg.data)
            shelf = data.get("shelf", "A1")
            if shelf not in self.shelves:
                self.get_logger().warn(f"Unknown shelf: {shelf}")
                return
            sl = self.shelves[shelf]
            self._task_cnt += 1
            task = Task(
                tid=data.get("tid", f"T{self._task_cnt:04d}"),
                shelf=shelf,
                shelf_center_xy=sl.center_xy,
                pick_xy=sl.pick_xy,
                pick_yaw=sl.pick_yaw,
                aisle_exit_xy=(self.DEFAULT_AISLE_EXIT_X, sl.pick_xy[1]),
                drop_xy=self.station_xy,
                priority=int(data.get("priority", 1)),
                cargo_value=float(data.get("cargo_value", 1.0)),
                deadline=float(data.get("deadline", 0.0)),
                requested_agv=data.get("agv_id", data.get("agv", "")),
            )
            with self.lock:
                self.queue.append(task)
                self._sort_queue_locked()
            self.get_logger().info(
                f"[QUEUE] {task.tid} shelf={shelf} priority={task.priority} "
                f"pending={len(self.queue)}"
            )
        except Exception as exc:
            self.get_logger().error(f"Task parse failed: {exc}")

    def _sort_queue_locked(self):
        """
        Layer-1 + Layer-2 composite sort.
        Effective priority = static_class_bonus + user_priority + urgency_bonus
        Evaluated fresh every scheduling cycle so anti-starvation accrues.
        """
        def key(task: Task) -> float:
            # Find the best candidate AGV for urgency battery query
            best_bat = min((a.battery for a in self.agvs.values()), default=100.0)
            urgency  = UrgencyScorer.score(task, best_bat)
            return -(task.priority + urgency * 3)   # negative → highest first

        self.queue.sort(key=key)

    # -----------------------------------------------------------------------
    # Scheduling loop
    # -----------------------------------------------------------------------

    def _sched_loop(self):
        self.stt.cleanup_stale()
        assignment = None

        with self.lock:
            self._sort_queue_locked()   # refresh Layer-2 scores

            idle = [
                agv for agv in self.agvs.values()
                if agv.state == State.IDLE and agv.battery > 15
            ]
            if not self.queue or not idle:
                return

            for task_idx, task in enumerate(list(self.queue)):
                if task.retry_after > time.time():
                    continue

                candidates = [
                    agv for agv in idle
                    if not task.requested_agv or agv.aid == task.requested_agv
                ]
                # Closest AGV first (within same priority tier)
                candidates.sort(
                    key=lambda a: math.hypot(a.x - task.pick_xy[0], a.y - task.pick_xy[1])
                )

                for agv in candidates:
                    # Layer-3: spacetime conflict check
                    waypoints = [
                        (agv.x, agv.y),
                        task.pick_xy,
                        task.aisle_exit_xy,
                        task.drop_xy,
                    ]
                    blockers = self.stt.conflicts(agv.aid, waypoints, self.NAV_SPEED)
                    if blockers:
                        task.status = f"waiting:{','.join(blockers)}"
                        continue

                    # All clear — reserve and assign
                    path = self.stt.reserve(
                        agv.aid, task.tid, waypoints,
                        self.NAV_SPEED, self.route_hold_timeout,
                    )
                    agv.spacetime_path = path

                    self.queue.pop(task_idx)
                    task.agv    = agv.aid
                    task.status = "running"
                    agv.state   = State.TO_SHELF
                    agv.task    = task
                    assignment  = (agv.aid, task)
                    break

                if assignment:
                    break

        if not assignment:
            return

        aid, task = assignment
        agv = self.agvs[aid]
        self.get_logger().info(
            f"[ASSIGN] {task.tid} → {aid} shelf={task.shelf} "
            f"pick=({task.pick_xy[0]:.1f},{task.pick_xy[1]:.1f})"
        )
        self._pub_assign(agv, task)
        if not self._send_nav(task.pick_xy, task.pick_yaw, task, agv):
            self._return_task_to_queue(agv, task, "Nav2 server not ready")
            return
        with self.lock:
            self.history.append(task)

    # -----------------------------------------------------------------------
    # Navigation
    # -----------------------------------------------------------------------

    def _send_nav(
            self,
            xy: Tuple[float, float],
            yaw: float,
            task: Task,
            agv: AGVState,
    ) -> bool:
        client = self.nav_clients[agv.aid]
        if not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f"[Nav2] {agv.aid} action not ready: {agv.nav_action}")
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(xy[0])
        goal.pose.pose.position.y = float(xy[1])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        with self.lock:
            cur = self.agvs.get(agv.aid)
            if cur:
                cur.current_goal_handle   = None
                cur.current_goal_xy       = xy
                cur.current_goal_yaw      = yaw
                cur.nav_goal_sent_ts      = time.time()
                cur.nav_goal_accepted_ts  = 0.0

        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._nav_accepted(f, task, agv))
        self.get_logger().info(
            f"[Nav2] {agv.aid} goal=({xy[0]:.1f},{xy[1]:.1f},yaw={yaw:.2f}) "
            f"via {agv.nav_action}"
        )
        return True

    def _nav_accepted(self, future, task: Task, agv: AGVState):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._return_task_to_queue(agv, task, f"goal response failed: {exc}")
            return

        if goal_handle is None:
            self._return_task_to_queue(agv, task, "empty goal response")
            return
        if not goal_handle.accepted:
            self._return_task_to_queue(agv, task, "goal rejected")
            return

        with self.lock:
            cur = self.agvs.get(agv.aid)
            if not cur or not cur.task or cur.task.tid != task.tid:
                goal_handle.cancel_goal_async()
                return
            cur.current_goal_handle  = goal_handle
            cur.nav_goal_accepted_ts = time.time()

        goal_handle.get_result_async().add_done_callback(
            lambda f: self._nav_done(f, task, agv)
        )

    def _nav_done(self, future, task: Task, agv: AGVState):
        result = future.result()
        status = getattr(result, "status", None)

        with self.lock:
            cur = self.agvs.get(agv.aid)
            if not cur or not cur.task or cur.task.tid != task.tid:
                return
            cur.current_goal_handle = None

        if status != GoalStatus.STATUS_SUCCEEDED:
            self._return_task_to_queue(agv, task, f"navigation status {status}")
            return

        with self.lock:
            cur = self.agvs[agv.aid]
            if cur.state == State.TO_SHELF:
                cur.state    = State.TO_AISLE_EXIT
                next_xy      = task.aisle_exit_xy
                next_yaw     = 0.0
                next_label   = "aisle exit"
            elif cur.state == State.TO_AISLE_EXIT:
                cur.state    = State.TO_STATION
                next_xy      = self.station_xy
                next_yaw     = 0.0
                next_label   = "station"
            elif cur.state == State.TO_STATION:
                cur.state    = State.IDLE
                cur.task     = None
                cur.current_goal_xy       = None
                cur.nav_goal_sent_ts      = 0.0
                cur.nav_goal_accepted_ts  = 0.0
                task.status  = "done"
                self.stt.release(agv.aid)
                self.get_logger().info(f"[DONE] {task.tid} completed by {agv.aid}")
                return
            else:
                return

        self.get_logger().info(
            f"[ARRIVE] {agv.aid} reached step → heading to {next_label}"
        )
        if not self._send_nav(next_xy, next_yaw, task, agv):
            self._return_task_to_queue(agv, task, f"{next_label} navigation unavailable")

    def _return_task_to_queue(self, agv: AGVState, task: Task, reason: str):
        with self.lock:
            cur = self.agvs.get(agv.aid)
            if cur:
                cur.state                 = State.IDLE
                cur.task                  = None
                cur.current_goal_handle   = None
                cur.current_goal_xy       = None
                cur.nav_goal_sent_ts      = 0.0
                cur.nav_goal_accepted_ts  = 0.0
                cur.yield_to              = ""
                self.stt.release(cur.aid)
            task.status      = "pending"
            task.agv         = ""
            task.last_error  = reason
            task.retry_after = time.time() + 5.0
            if not any(t.tid == task.tid for t in self.queue):
                self.queue.append(task)
                self._sort_queue_locked()
        self._publish_stop(agv.aid)
        self.get_logger().warn(f"[REQUEUE] {task.tid}: {reason}")

    # -----------------------------------------------------------------------
    # Layer-4: Safety loop with arbitration
    # -----------------------------------------------------------------------

    def _safety_loop(self):
        """
        Detect close-proximity pairs.  Instead of always stopping the
        lower-ID robot, compute yield_score for each; the lower-score AGV
        yields.  Yielding AGV cancels its goal and re-queues with a short
        retry delay.
        """
        victims: List[Tuple[str, Task, str]] = []

        with self.lock:
            agvs = list(self.agvs.values())
            now  = time.time()

            for i, left in enumerate(agvs):
                for right in agvs[i + 1:]:
                    if left.state == State.IDLE and right.state == State.IDLE:
                        continue
                    dist = math.hypot(left.x - right.x, left.y - right.y)
                    if dist >= self.safety_stop_distance:
                        continue

                    # Yield decision via Layer-4 score
                    ls = Arbitrator.yield_score(left,  left.task)
                    rs = Arbitrator.yield_score(right, right.task)
                    yielder, holder = (left, right) if ls < rs else (right, left)

                    # Avoid re-triggering if already yielding to this holder
                    if yielder.yield_to == holder.aid and yielder.yield_until > now:
                        continue

                    yielder.yield_to    = holder.aid
                    yielder.yield_until = now + self.yield_duration
                    yielder.state       = State.YIELDING

                    if yielder.task:
                        victims.append((
                            yielder.aid,
                            yielder.task,
                            f"yield to {holder.aid}: dist={dist:.2f}m "
                            f"(score {ls:.1f} < {rs:.1f})",
                        ))

        for aid, task, reason in victims:
            agv = self.agvs[aid]
            if agv.current_goal_handle:
                agv.current_goal_handle.cancel_goal_async()
            self._return_task_to_queue(agv, task, reason)

        # Emergency stop for any moving AGV without a task
        for agv in self.agvs.values():
            if agv.state not in (State.IDLE, State.YIELDING) and agv.task is None:
                self._publish_stop(agv.aid)

    # -----------------------------------------------------------------------
    # Layer-4: Deadlock detection (Wait-For Graph)
    # -----------------------------------------------------------------------

    def _deadlock_check(self):
        """
        Build the Wait-For Graph from current yield_to relationships
        and break any detected cycles by rescheduling the lowest-priority
        AGV in each cycle.
        """
        with self.lock:
            wfg: Dict[str, str] = {
                agv.aid: agv.yield_to
                for agv in self.agvs.values()
                if agv.yield_to
            }

        cycles = Arbitrator.detect_deadlock(wfg)
        if not cycles:
            return

        for cycle in cycles:
            self.get_logger().warn(f"[DEADLOCK] cycle detected: {' → '.join(cycle)}")
            # Break cycle: pick the AGV with the lowest yield_score
            scores = {
                aid: Arbitrator.yield_score(self.agvs[aid], self.agvs[aid].task)
                for aid in cycle
                if aid in self.agvs
            }
            if not scores:
                continue
            loser_id = min(scores, key=scores.__getitem__)
            loser    = self.agvs[loser_id]
            if loser.task:
                self.get_logger().warn(
                    f"[DEADLOCK] breaking cycle — preempting {loser_id} "
                    f"(score={scores[loser_id]:.1f})"
                )
                if loser.current_goal_handle:
                    loser.current_goal_handle.cancel_goal_async()
                self._return_task_to_queue(
                    loser, loser.task,
                    f"deadlock break (cycle: {' → '.join(cycle)})",
                )

    # -----------------------------------------------------------------------
    # Nav watchdog
    # -----------------------------------------------------------------------

    def _nav_watchdog(self):
        victims: List[Tuple[AGVState, Task, str]] = []
        now = time.time()
        with self.lock:
            for agv in self.agvs.values():
                if agv.state == State.IDLE or not agv.task:
                    continue
                if not agv.nav_goal_sent_ts:
                    continue
                pending = (
                    agv.current_goal_handle is None
                    and agv.nav_goal_accepted_ts == 0.0
                    and now - agv.nav_goal_sent_ts > 5.0
                )
                if pending:
                    victims.append((agv, agv.task, "Nav2 goal not accepted within 5 s"))
        for agv, task, reason in victims:
            self._return_task_to_queue(agv, task, reason)

    # -----------------------------------------------------------------------
    # Publishing helpers
    # -----------------------------------------------------------------------

    def _publish_stop(self, aid: str):
        pub = self.cmd_publishers.get(aid)
        if pub:
            pub.publish(Twist())

    def _pub_assign(self, agv: AGVState, task: Task):
        msg = String()
        msg.data = json.dumps({
            "agv": agv.aid,
            "tid": task.tid,
            "shelf": task.shelf,
            "shelf_center": list(task.shelf_center_xy),
            "pick": list(task.pick_xy),
            "pick_yaw": task.pick_yaw,
            "aisle_exit": list(task.aisle_exit_xy),
            "drop": list(task.drop_xy),
            "priority": task.priority,
            "cargo_value": task.cargo_value,
            "deadline": task.deadline,
            "vehicle_class": agv.vehicle_class.name,
            "nav_action": agv.nav_action,
        }, ensure_ascii=False)
        self.pub_assign.publish(msg)

    def _pub_status(self):
        with self.lock:
            done = sum(1 for t in self.history if t.status == "done")
            payload = {
                "pending":   len(self.queue),
                "completed": done,
                "queue": [
                    {
                        "tid": t.tid,
                        "shelf": t.shelf,
                        "status": t.status,
                        "requested_agv": t.requested_agv,
                        "retry_in": max(0.0, round(t.retry_after - time.time(), 1)),
                        "last_error": t.last_error,
                        "cargo_value": t.cargo_value,
                        "deadline": t.deadline,
                    }
                    for t in self.queue
                ],
                "spacetime_reservations": len(self.stt._table),
                "fleet": {
                    aid: {
                        "state":        agv.state.value,
                        "vehicle_class":agv.vehicle_class.name,
                        "pos":          [round(agv.x, 2), round(agv.y, 2)],
                        "battery":      round(agv.battery, 1),
                        "vx":           agv.vx,
                        "wz":           agv.wz,
                        "task":         agv.task.tid if agv.task else None,
                        "nav_action":   agv.nav_action,
                        "yield_to":     agv.yield_to or None,
                        "goal": (
                            [round(agv.current_goal_xy[0], 2),
                             round(agv.current_goal_xy[1], 2)]
                            if agv.current_goal_xy else None
                        ),
                        "goal_active":  agv.current_goal_handle is not None,
                    }
                    for aid, agv in self.agvs.items()
                },
            }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_sched.publish(msg)

    # -----------------------------------------------------------------------
    # Auto demo
    # -----------------------------------------------------------------------

    def _auto_demo(self):
        if not self.auto_demo_enabled or self._demo_n >= 8:
            return
        import random
        shelf    = random.choice(list(self.shelves.keys()))
        priority = random.randint(1, 5)
        value    = round(random.uniform(1.0, 100.0), 1)
        msg = String()
        msg.data = json.dumps({
            "tid":         f"AUTO_{self._demo_n + 1:03d}",
            "shelf":       shelf,
            "priority":    priority,
            "cargo_value": value,
        }, ensure_ascii=False)
        self._on_task(msg)
        self._demo_n += 1


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = AGVScheduler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Scheduler shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
