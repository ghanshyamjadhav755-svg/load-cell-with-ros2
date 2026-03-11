#!/usr/bin/env python3

import time
import enum
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Quaternion
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


def build_pose(frame_id, px, py, pz, qx, qy, qz, qw):
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose.position = Point(x=px, y=py, z=pz)
    ps.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return ps


class State(enum.Enum):
    IDLE = "IDLE"
    WAIT_BEFORE_NAV = "WAIT_BEFORE_NAV"
    NAVIGATE_TO_DELIVERY = "NAVIGATE_TO_DELIVERY"
    WAIT_FOR_UNLOAD = "WAIT_FOR_UNLOAD"
    RETURN_WAIT = "RETURN_WAIT"
    NAVIGATE_TO_START = "NAVIGATE_TO_START"


class LoadCellDeliveryNode(Node):

    def __init__(self):

        super().__init__("load_cell_delivery_node")

        # PARAMETERS
        self.declare_parameter("weight_threshold", 2.0)
        self.declare_parameter("weight_stable_time", 5.0)
        self.declare_parameter("pre_navigate_delay", 5.0)
        self.declare_parameter("post_unload_delay", 5.0)
        self.declare_parameter("nav_server_timeout", 10.0)
        self.declare_parameter("map_frame", "map")

        self.declare_parameter("delivery_x", 0.21)
        self.declare_parameter("delivery_y", 3.99)
        self.declare_parameter("delivery_z", 0.0)
        self.declare_parameter("delivery_qx", 0.0)
        self.declare_parameter("delivery_qy", 0.0)
        self.declare_parameter("delivery_qz", 0.03)
        self.declare_parameter("delivery_qw", 0.99)

        self.declare_parameter("start_x", 0.24)
        self.declare_parameter("start_y", 0.08)
        self.declare_parameter("start_z", 0.0)
        self.declare_parameter("start_qx", 0.0)
        self.declare_parameter("start_qy", 0.0)
        self.declare_parameter("start_qz", -0.13)
        self.declare_parameter("start_qw", 0.99)

        self.declare_parameter("load_cell_topic", "/load_cell_data")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("nav_action", "/navigate_to_pose")

        # GET PARAMETERS
        self.WEIGHT_THRESHOLD_KG = self.get_parameter("weight_threshold").value
        self.WEIGHT_STABLE_TIME_S = self.get_parameter("weight_stable_time").value
        self.PRE_NAVIGATE_DELAY_S = self.get_parameter("pre_navigate_delay").value
        self.POST_UNLOAD_DELAY_S = self.get_parameter("post_unload_delay").value
        self.NAV_SERVER_TIMEOUT_S = self.get_parameter("nav_server_timeout").value
        self.MAP_FRAME = self.get_parameter("map_frame").value

        DELIVERY_X = self.get_parameter("delivery_x").value
        DELIVERY_Y = self.get_parameter("delivery_y").value
        DELIVERY_Z = self.get_parameter("delivery_z").value
        DELIVERY_QX = self.get_parameter("delivery_qx").value
        DELIVERY_QY = self.get_parameter("delivery_qy").value
        DELIVERY_QZ = self.get_parameter("delivery_qz").value
        DELIVERY_QW = self.get_parameter("delivery_qw").value

        START_X = self.get_parameter("start_x").value
        START_Y = self.get_parameter("start_y").value
        START_Z = self.get_parameter("start_z").value
        START_QX = self.get_parameter("start_qx").value
        START_QY = self.get_parameter("start_qy").value
        START_QZ = self.get_parameter("start_qz").value
        START_QW = self.get_parameter("start_qw").value

        TOPIC_LOAD_CELL = self.get_parameter("load_cell_topic").value
        TOPIC_AMCL_POSE = self.get_parameter("amcl_pose_topic").value
        ACTION_NAV = self.get_parameter("nav_action").value

        # POSES
        self._start_pose = build_pose(
            self.MAP_FRAME,
            START_X, START_Y, START_Z,
            START_QX, START_QY, START_QZ, START_QW
        )

        self._delivery_pose = build_pose(
            self.MAP_FRAME,
            DELIVERY_X, DELIVERY_Y, DELIVERY_Z,
            DELIVERY_QX, DELIVERY_QY, DELIVERY_QZ, DELIVERY_QW
        )

        self._start_pose_locked = False

        # STATE VARIABLES
        self._state = State.IDLE
        self._current_weight = 0.0
        self._weight_start_time = None
        self._unload_start_time = None

        self._goal_handle: Optional[ClientGoalHandle] = None
        self._nav_in_flight = False
        self._distance_remaining = 0.0

        self._last_log = {}

        self._cbg = ReentrantCallbackGroup()

        # AMCL SUB
        amcl_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            TOPIC_AMCL_POSE,
            self._amcl_cb,
            amcl_qos,
            callback_group=self._cbg
        )

        # LOAD CELL SUB
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._weight_sub = self.create_subscription(
            Float32,
            TOPIC_LOAD_CELL,
            self._weight_cb,
            sensor_qos,
            callback_group=self._cbg
        )

        # NAV CLIENT
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            ACTION_NAV,
            callback_group=self._cbg
        )

        self._sm_timer = self.create_timer(
            0.1,
            self._sm_tick,
            callback_group=self._cbg
        )

        self.get_logger().info("Load Cell Delivery Node started")

    def _log(self, tag, interval, msg):
        now = time.monotonic()
        if now - self._last_log.get(tag, 0.0) >= interval:
            self._last_log[tag] = now
            self.get_logger().info(msg)

    def _amcl_cb(self, msg):

        if self._start_pose_locked:
            return

        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose

        self._start_pose = ps
        self._start_pose_locked = True

        p = ps.pose.position
        self.get_logger().info(
            f"Start pose locked from AMCL x={p.x:.3f} y={p.y:.3f}"
        )

        self.destroy_subscription(self._amcl_sub)

    def _weight_cb(self, msg):
        self._current_weight = float(msg.data)

    def _sm_tick(self):

        weight = self._current_weight

        # IDLE
        if self._state == State.IDLE:

            self._log(
                "idle",
                2.0,
                f"[IDLE] weight={weight:.2f}kg threshold={self.WEIGHT_THRESHOLD_KG}"
            )

            if weight >= self.WEIGHT_THRESHOLD_KG:

                if self._weight_start_time is None:
                    self._weight_start_time = time.monotonic()

                elapsed = time.monotonic() - self._weight_start_time

                if elapsed >= self.WEIGHT_STABLE_TIME_S:

                    self.get_logger().info(
                        f"[IDLE] Weight stable for {self.WEIGHT_STABLE_TIME_S}s"
                    )

                    self._state = State.WAIT_BEFORE_NAV

                    threading.Thread(
                        target=self._pre_nav_thread,
                        daemon=True
                    ).start()

            else:
                self._weight_start_time = None

        # WAIT FOR UNLOAD
        elif self._state == State.WAIT_FOR_UNLOAD:

            self._log(
                "unload",
                2.0,
                f"[DELIVERY] waiting unload weight={weight:.2f}"
            )

            if weight < self.WEIGHT_THRESHOLD_KG:

                if self._unload_start_time is None:
                    self._unload_start_time = time.monotonic()

                elapsed = time.monotonic() - self._unload_start_time

                if elapsed >= self.WEIGHT_STABLE_TIME_S:

                    self.get_logger().info(
                        f"[DELIVERY] load removed stable for {self.WEIGHT_STABLE_TIME_S}s"
                    )

                    self._state = State.RETURN_WAIT

                    threading.Thread(
                        target=self._return_wait_thread,
                        daemon=True
                    ).start()

            else:
                self._unload_start_time = None

        elif self._state in (State.NAVIGATE_TO_DELIVERY, State.NAVIGATE_TO_START):

            self._log(
                "nav",
                2.0,
                f"[NAV] distance remaining {self._distance_remaining:.2f} m"
            )

    def _pre_nav_thread(self):

        for i in range(int(self.PRE_NAVIGATE_DELAY_S),0,-1):

            self.get_logger().info(
                f"[WAIT] moving in {i} sec"
            )

            time.sleep(1)

        self.get_logger().info("[NAV] sending delivery goal")

        self._state = State.NAVIGATE_TO_DELIVERY

        self._send_nav_goal(
            self._delivery_pose,
            self._on_delivery_reached,
            self._on_nav_failure
        )

    def _return_wait_thread(self):

        for i in range(int(self.POST_UNLOAD_DELAY_S),0,-1):

            self.get_logger().info(
                f"[WAIT] returning in {i} sec"
            )

            time.sleep(1)

        self.get_logger().info("[NAV] sending start goal")

        self._state = State.NAVIGATE_TO_START

        home = self._start_pose
        home.header.stamp = self.get_clock().now().to_msg()

        self._send_nav_goal(
            home,
            self._on_start_reached,
            self._on_nav_failure
        )

    def _send_nav_goal(self, pose, on_success, on_failure):

        if not self._nav_client.wait_for_server(timeout_sec=self.NAV_SERVER_TIMEOUT_S):

            self.get_logger().error("Nav server unavailable")
            on_failure()
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._nav_feedback_cb
        )

        future.add_done_callback(
            lambda f: self._goal_response_cb(f, on_success, on_failure)
        )

    def _goal_response_cb(self, future, on_success, on_failure):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error("Goal rejected")
            on_failure()
            return

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            lambda f: self._result_cb(f, on_success, on_failure)
        )

    def _result_cb(self, future, on_success, on_failure):

        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            on_success()
        else:
            self.get_logger().error("Navigation failed")
            on_failure()

    def _nav_feedback_cb(self, msg):

        self._distance_remaining = msg.feedback.distance_remaining

    def _on_delivery_reached(self):

        self.get_logger().info("Delivery reached waiting unload")

        self._state = State.WAIT_FOR_UNLOAD

    def _on_start_reached(self):

        self.get_logger().info("Start reached cycle completed")

        self._state = State.IDLE
        self._weight_start_time = None
        self._unload_start_time = None

    def _on_nav_failure(self):

        time.sleep(3)
        self._state = State.IDLE


def main(args=None):

    rclpy.init(args=args)

    node = LoadCellDeliveryNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


# #!/usr/bin/env python3

# # ── BEHAVIOUR ─────────────────────────────────────────────────────────────────
# WEIGHT_THRESHOLD_KG   = 2.0    # kg — weight to trigger delivery
# PRE_NAVIGATE_DELAY_S  = 5.0    # s  — delay after weight detected, before moving
# POST_UNLOAD_DELAY_S   = 5.0    # s  — delay after load removed, before returning
# NAV_SERVER_TIMEOUT_S  = 10.0   # s  — how long to wait for Nav2 action server
# MAP_FRAME             = "map"

# # ── DELIVERY POSE ─────────────────────────────────────────────────────────────
# DELIVERY_X  =  0.2158270923694098
# DELIVERY_Y  =  3.9984935107647397
# DELIVERY_Z  =  0.0
# DELIVERY_QX =  0.0
# DELIVERY_QY =  0.0
# DELIVERY_QZ =  0.030388678063095885
# DELIVERY_QW =  0.9995381574735291


# # ── START POSE ────────────────────────────────────────────────────────────────
# #    Robot's home position. Set this from:
# #    ros2 topic echo /amcl_pose --once   (run at the start position)
# START_X  =  0.24343829330124162
# START_Y  =  0.0826174484819945
# START_Z  =  0.0
# START_QX =  0.0
# START_QY =  0.0
# START_QZ =  0.-0.1375539546259112
# START_QW =  0.9904942753831406

# # ── TOPICS ────────────────────────────────────────────────────────────────────
# TOPIC_LOAD_CELL = "/load_cell_data"
# TOPIC_AMCL_POSE = "/amcl_pose"
# ACTION_NAV      = "/navigate_to_pose"


# import time
# import enum
# import threading
# from typing import Optional

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from rclpy.action.client import ClientGoalHandle
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.qos import (
#     QoSProfile, QoSDurabilityPolicy,
#     QoSReliabilityPolicy, QoSHistoryPolicy,
# )

# from std_msgs.msg import Float32
# from geometry_msgs.msg import (
#     PoseStamped, PoseWithCovarianceStamped, Point, Quaternion,
# )
# from nav2_msgs.action import NavigateToPose
# from action_msgs.msg import GoalStatus


# # ─────────────────────────────────────────────────────────────────────────────
# #  Helpers
# # ─────────────────────────────────────────────────────────────────────────────

# def build_pose(frame_id, px, py, pz, qx, qy, qz, qw) -> PoseStamped:
#     ps = PoseStamped()
#     ps.header.frame_id  = frame_id
#     ps.pose.position    = Point(x=px, y=py, z=pz)
#     ps.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
#     return ps


# class State(enum.Enum):
#     IDLE                 = "IDLE"
#     WAIT_BEFORE_NAV      = "WAIT_BEFORE_NAV"
#     NAVIGATE_TO_DELIVERY = "NAVIGATE_TO_DELIVERY"
#     WAIT_FOR_UNLOAD      = "WAIT_FOR_UNLOAD"
#     RETURN_WAIT          = "RETURN_WAIT"
#     NAVIGATE_TO_START    = "NAVIGATE_TO_START"


# # ─────────────────────────────────────────────────────────────────────────────
# #  Node
# # ─────────────────────────────────────────────────────────────────────────────

# class LoadCellDeliveryNode(Node):

#     def __init__(self) -> None:
#         super().__init__("load_cell_delivery_node")

#         # ── build poses from config ───────────────────────────────────────────
#         # Start pose: locked once from /amcl_pose at boot.
#         # If amcl never publishes, START_* config values are used as fallback.
#         self._start_pose: PoseStamped = build_pose(
#             MAP_FRAME,
#             START_X, START_Y, START_Z,
#             START_QX, START_QY, START_QZ, START_QW,
#         )
#         self._start_pose_locked: bool = False   # True after first amcl message

#         self._delivery_pose: PoseStamped = build_pose(
#             MAP_FRAME,
#             DELIVERY_X, DELIVERY_Y, DELIVERY_Z,
#             DELIVERY_QX, DELIVERY_QY, DELIVERY_QZ, DELIVERY_QW,
#         )

#         # ── runtime ───────────────────────────────────────────────────────────
#         self._state             : State                      = State.IDLE
#         self._state_lock        : threading.Lock             = threading.Lock()
#         self._current_weight    : float                      = 0.0
#         self._weight_lock       : threading.Lock             = threading.Lock()
#         self._goal_handle       : Optional[ClientGoalHandle] = None
#         self._nav_in_flight     : bool                       = False
#         self._distance_remaining: float                      = 0.0
#         self._last_log          : dict                       = {}

#         self._cbg = ReentrantCallbackGroup()

#         # ── /amcl_pose — TRANSIENT_LOCAL to get last pose on connect ──────────
#         amcl_qos = QoSProfile(
#             history     = QoSHistoryPolicy.KEEP_LAST,
#             depth       = 1,
#             reliability = QoSReliabilityPolicy.RELIABLE,
#             durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL,
#         )
#         self._amcl_sub = self.create_subscription(
#             PoseWithCovarianceStamped,
#             TOPIC_AMCL_POSE,
#             self._amcl_cb,
#             qos_profile    = amcl_qos,
#             callback_group = self._cbg,
#         )

#         # ── /load_cell_data ───────────────────────────────────────────────────
#         sensor_qos = QoSProfile(
#             history     = QoSHistoryPolicy.KEEP_LAST,
#             depth       = 10,
#             reliability = QoSReliabilityPolicy.BEST_EFFORT,
#             durability  = QoSDurabilityPolicy.VOLATILE,
#         )
#         self._weight_sub = self.create_subscription(
#             Float32,
#             TOPIC_LOAD_CELL,
#             self._weight_cb,
#             qos_profile    = sensor_qos,
#             callback_group = self._cbg,
#         )

#         # ── Nav2 action client ────────────────────────────────────────────────
#         self._nav_client = ActionClient(
#             self, NavigateToPose, ACTION_NAV,
#             callback_group = self._cbg,
#         )

#         # ── 10 Hz tick ────────────────────────────────────────────────────────
#         self._sm_timer = self.create_timer(
#             0.1, self._sm_tick, callback_group=self._cbg
#         )

#         self.get_logger().info(
#             f"\n"
#             f"  Load Cell Delivery Node started\n"
#             f"  --------------------------------\n"
#             f"  Weight threshold : {WEIGHT_THRESHOLD_KG} kg\n"
#             f"  Pre-nav delay    : {PRE_NAVIGATE_DELAY_S} s\n"
#             f"  Post-unload delay: {POST_UNLOAD_DELAY_S} s\n"
#             f"  Delivery pose    : x={DELIVERY_X}  y={DELIVERY_Y}  "
#             f"qz={DELIVERY_QZ}  qw={DELIVERY_QW}\n"
#             f"  Start pose       : x={START_X}  y={START_Y}  "
#             f"qz={START_QZ}  qw={START_QW}\n"
#             f"  Monitoring {TOPIC_LOAD_CELL} ..."
#         )

#     # ─────────────────────────────────────────────────────────────────────────
#     #  Subscriber callbacks
#     # ─────────────────────────────────────────────────────────────────────────

#     def _amcl_cb(self, msg: PoseWithCovarianceStamped) -> None:
#         # Lock start pose ONCE at boot — never update again.
#         # This ensures returning home always goes to the real start position.
#         if self._start_pose_locked:
#             return

#         ps = PoseStamped()
#         ps.header = msg.header
#         ps.pose   = msg.pose.pose
#         self._start_pose        = ps
#         self._start_pose_locked = True

#         p = ps.pose.position
#         q = ps.pose.orientation
#         self.get_logger().info(
#             f"  Start pose locked from /amcl_pose: "
#             f"x={p.x:.4f}  y={p.y:.4f}  qz={q.z:.4f}  qw={q.w:.4f}"
#         )
#         # No longer need this subscriber
#         self.destroy_subscription(self._amcl_sub)

#     def _weight_cb(self, msg: Float32) -> None:
#         with self._weight_lock:
#             self._current_weight = float(msg.data)



#     def _log(self, tag: str, interval: float, msg: str) -> None:
#         now = time.monotonic()
#         if now - self._last_log.get(tag, 0.0) >= interval:
#             self._last_log[tag] = now
#             self.get_logger().info(msg)

#     # ─────────────────────────────────────────────────────────────────────────
#     #  State machine
#     # ─────────────────────────────────────────────────────────────────────────

#     def _transition(self, new: State) -> None:
#         with self._state_lock:
#             self._state = new

#     def _get_state(self) -> State:
#         with self._state_lock:
#             return self._state

#     def _get_weight(self) -> float:
#         with self._weight_lock:
#             return self._current_weight

#     def _sm_tick(self) -> None:
#         state  = self._get_state()
#         weight = self._get_weight()

#         if state == State.IDLE:
#             self._log("idle", 2.0,
#                 f"[IDLE] weight = {weight:.2f} kg  (threshold: {WEIGHT_THRESHOLD_KG} kg)"
#             )
#             if weight >= WEIGHT_THRESHOLD_KG:
#                 self.get_logger().info(
#                     f"[IDLE] Load detected! {weight:.2f} kg — "
#                     f"moving to delivery in {PRE_NAVIGATE_DELAY_S:.0f} s"
#                 )
#                 self._transition(State.WAIT_BEFORE_NAV)
#                 threading.Thread(target=self._pre_nav_thread, daemon=True).start()

#         elif state == State.WAIT_FOR_UNLOAD:
#             self._log("unload", 2.0,
#                 f"[DELIVERY] Waiting for load removal — weight = {weight:.2f} kg"
#             )
#             if weight < WEIGHT_THRESHOLD_KG:
#                 self.get_logger().info(
#                     f"[DELIVERY] Load removed ({weight:.2f} kg) — "
#                     f"returning to start in {POST_UNLOAD_DELAY_S:.0f} s"
#                 )
#                 self._transition(State.RETURN_WAIT)
#                 threading.Thread(target=self._return_wait_thread, daemon=True).start()

#         elif state in (State.NAVIGATE_TO_DELIVERY, State.NAVIGATE_TO_START):
#             self._log("nav", 2.0,
#                 f"[NAV] Navigating to "
#                 f"{'delivery' if state == State.NAVIGATE_TO_DELIVERY else 'start'} — "
#                 f"distance remaining: {self._distance_remaining:.2f} m"
#             )

#     # ─────────────────────────────────────────────────────────────────────────
#     #  Daemon threads
#     # ─────────────────────────────────────────────────────────────────────────

#     def _pre_nav_thread(self) -> None:
#         for remaining in range(int(PRE_NAVIGATE_DELAY_S), 0, -1):
#             self.get_logger().info(
#                 f"[WAIT] Moving to delivery in {remaining} s ..."
#             )
#             time.sleep(1.0)

#         self.get_logger().info("[NAV] Sending robot to delivery location ...")
#         self._transition(State.NAVIGATE_TO_DELIVERY)
#         self._send_nav_goal(
#             pose       = self._delivery_pose,
#             on_success = self._on_delivery_reached,
#             on_failure = self._on_nav_failure,
#             label      = "DELIVERY",
#         )

#     def _return_wait_thread(self) -> None:
#         for remaining in range(int(POST_UNLOAD_DELAY_S), 0, -1):
#             self.get_logger().info(
#                 f"[WAIT] Returning to start in {remaining} s ..."
#             )
#             time.sleep(1.0)

#         self.get_logger().info("[NAV] Sending robot back to start location ...")
#         self._transition(State.NAVIGATE_TO_START)

#         home = self._start_pose
#         home.header.stamp = self.get_clock().now().to_msg()
#         self._send_nav_goal(
#             pose       = home,
#             on_success = self._on_start_reached,
#             on_failure = self._on_nav_failure,
#             label      = "START",
#         )

#     # ─────────────────────────────────────────────────────────────────────────
#     #  Nav2 pipeline
#     # ─────────────────────────────────────────────────────────────────────────

#     def _send_nav_goal(
#         self,
#         pose: PoseStamped,
#         on_success,
#         on_failure,
#         label: str = "",
#     ) -> None:
#         if self._nav_in_flight:
#             self.get_logger().warn("[NAV] Goal already in flight — skipping.")
#             return

#         if not self._nav_client.wait_for_server(timeout_sec=NAV_SERVER_TIMEOUT_S):
#             self.get_logger().error("[NAV] Action server not available!")
#             on_failure()
#             return

#         goal_msg                   = NavigateToPose.Goal()
#         goal_msg.pose              = pose
#         goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

#         self._nav_in_flight      = True
#         self._distance_remaining = 0.0

#         future = self._nav_client.send_goal_async(
#             goal_msg,
#             feedback_callback = self._nav_feedback_cb,
#         )
#         future.add_done_callback(
#             lambda f: self._goal_response_cb(f, on_success, on_failure)
#         )

#     def _goal_response_cb(self, future, on_success, on_failure) -> None:
#         goal_handle: ClientGoalHandle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error("[NAV] Goal rejected by action server.")
#             self._nav_in_flight = False
#             on_failure()
#             return

#         self._goal_handle = goal_handle
#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(
#             lambda f: self._result_cb(f, on_success, on_failure)
#         )

#     def _result_cb(self, future, on_success, on_failure) -> None:
#         self._nav_in_flight      = False
#         self._distance_remaining = 0.0
#         status = future.result().status

#         if status == GoalStatus.STATUS_SUCCEEDED:
#             on_success()
#         else:
#             self.get_logger().error(f"[NAV] Goal failed (status={status})")
#             on_failure()

#     def _nav_feedback_cb(self, feedback_msg) -> None:
#         self._distance_remaining = feedback_msg.feedback.distance_remaining

#     # ─────────────────────────────────────────────────────────────────────────
#     #  Outcome callbacks
#     # ─────────────────────────────────────────────────────────────────────────

#     def _on_delivery_reached(self) -> None:
#         self.get_logger().info(
#             "[NAV] Delivery location reached — waiting for load removal ..."
#         )
#         self._transition(State.WAIT_FOR_UNLOAD)

#     def _on_start_reached(self) -> None:
#         self.get_logger().info(
#             "[NAV] Start location reached — cycle complete, back to IDLE"
#         )
#         self._transition(State.IDLE)

#     def _on_nav_failure(self) -> None:
#         self.get_logger().error("[NAV] Navigation failed — resetting to IDLE in 3 s ...")
#         time.sleep(3.0)
#         self._transition(State.IDLE)

#     # ─────────────────────────────────────────────────────────────────────────
#     #  Shutdown
#     # ─────────────────────────────────────────────────────────────────────────

#     def destroy_node(self) -> None:
#         if self._nav_in_flight and self._goal_handle is not None:
#             self._goal_handle.cancel_goal_async()
#         super().destroy_node()


# # ─────────────────────────────────────────────────────────────────────────────
# #  Entry point
# # ─────────────────────────────────────────────────────────────────────────────

# def main(args=None) -> None:
#     rclpy.init(args=args)
#     node = LoadCellDeliveryNode()

#     executor = MultiThreadedExecutor(num_threads=4)
#     executor.add_node(node)

#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down ...")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()
