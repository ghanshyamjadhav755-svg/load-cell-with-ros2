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

from std_msgs.msg import Float32, String
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
    IDLE                 = "IDLE"
    NAVIGATE_TO_START    = "NAVIGATE_TO_START"
    DOCK_AT_START        = "DOCK_AT_START"        # sent dock_0, waiting DOCKED
    WAIT_FOR_LOAD        = "WAIT_FOR_LOAD"         # weight stable + pre-delay AT dock, then undock
    UNDOCK_AT_START      = "UNDOCK_AT_START"       # sent undock, waiting UNDOCKED -> nav delivery
    NAVIGATE_TO_DELIVERY = "NAVIGATE_TO_DELIVERY"
    DOCK_AT_DELIVERY     = "DOCK_AT_DELIVERY"      # sent dock_1, waiting DOCKED
    WAIT_FOR_UNLOAD      = "WAIT_FOR_UNLOAD"       # weight drops stable + post-delay AT dock, then undock
    UNDOCK_AT_DELIVERY   = "UNDOCK_AT_DELIVERY"    # sent undock, waiting UNDOCKED -> nav start


class LoadCellDeliveryNode(Node):

    def __init__(self):

        super().__init__("load_cell_delivery_node")

        # ── PARAMETERS ────────────────────────────────────────────────────────
        self.declare_parameter("weight_threshold", 2.0)
        self.declare_parameter("weight_stable_time", 5.0)
        self.declare_parameter("pre_navigate_delay", 5.0)   # delay AT dock after load confirmed
        self.declare_parameter("post_unload_delay", 5.0)    # delay AT dock after unload confirmed
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

        # dock topics — must match MultiDockManager
        self.declare_parameter("dock_command_topic", "/align_dock/command")
        self.declare_parameter("dock_status_topic", "/dock_status")

        # ── GET PARAMETERS ────────────────────────────────────────────────────
        self.WEIGHT_THRESHOLD_KG  = self.get_parameter("weight_threshold").value
        self.WEIGHT_STABLE_TIME_S = self.get_parameter("weight_stable_time").value
        self.PRE_NAVIGATE_DELAY_S = self.get_parameter("pre_navigate_delay").value
        self.POST_UNLOAD_DELAY_S  = self.get_parameter("post_unload_delay").value
        self.NAV_SERVER_TIMEOUT_S = self.get_parameter("nav_server_timeout").value
        self.MAP_FRAME            = self.get_parameter("map_frame").value

        DELIVERY_X  = self.get_parameter("delivery_x").value
        DELIVERY_Y  = self.get_parameter("delivery_y").value
        DELIVERY_Z  = self.get_parameter("delivery_z").value
        DELIVERY_QX = self.get_parameter("delivery_qx").value
        DELIVERY_QY = self.get_parameter("delivery_qy").value
        DELIVERY_QZ = self.get_parameter("delivery_qz").value
        DELIVERY_QW = self.get_parameter("delivery_qw").value

        START_X  = self.get_parameter("start_x").value
        START_Y  = self.get_parameter("start_y").value
        START_Z  = self.get_parameter("start_z").value
        START_QX = self.get_parameter("start_qx").value
        START_QY = self.get_parameter("start_qy").value
        START_QZ = self.get_parameter("start_qz").value
        START_QW = self.get_parameter("start_qw").value

        TOPIC_LOAD_CELL    = self.get_parameter("load_cell_topic").value
        TOPIC_AMCL_POSE    = self.get_parameter("amcl_pose_topic").value
        ACTION_NAV         = self.get_parameter("nav_action").value
        TOPIC_DOCK_COMMAND = self.get_parameter("dock_command_topic").value
        TOPIC_DOCK_STATUS  = self.get_parameter("dock_status_topic").value

        # ── POSES ─────────────────────────────────────────────────────────────
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

        # ── STATE VARIABLES ───────────────────────────────────────────────────
        self._state             = State.IDLE
        self._current_weight    = 0.0

        # _weight_start_time: marks when weight first crossed threshold (load)
        # _unload_start_time: marks when weight first dropped below threshold (unload)
        # Both are reset to None whenever the condition breaks before completion.
        self._weight_start_time: Optional[float] = None
        self._unload_start_time: Optional[float] = None

        self._goal_handle: Optional[ClientGoalHandle] = None
        self._nav_in_flight      = False   # guard: one nav goal at a time
        self._distance_remaining = 0.0

        self._last_log = {}

        # ── DOCK EVENT — edge-triggered single-slot buffer ─────────────────────
        # _dock_status_cb writes here only when the value changes.
        # _consume_dock_event() atomically checks + clears — fires exactly once
        # per DOCKED / UNDOCKED regardless of how many SM ticks pass.
        self._dock_event:      Optional[str] = None
        self._dock_event_lock: threading.Lock = threading.Lock()

        self._cbg = ReentrantCallbackGroup()

        # ── QOS PROFILES ──────────────────────────────────────────────────────
        amcl_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Reliable — dock commands and dock status must never be dropped
        reliable_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # ── SUBSCRIPTIONS ─────────────────────────────────────────────────────
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            TOPIC_AMCL_POSE,
            self._amcl_cb,
            amcl_qos,
            callback_group=self._cbg
        )

        self._weight_sub = self.create_subscription(
            Float32,
            TOPIC_LOAD_CELL,
            self._weight_cb,
            sensor_qos,
            callback_group=self._cbg
        )

        # Reliable — every DOCKED / UNDOCKED must arrive
        self._dock_status_sub = self.create_subscription(
            String,
            TOPIC_DOCK_STATUS,
            self._dock_status_cb,
            reliable_qos,
            callback_group=self._cbg
        )

        # ── PUBLISHERS ────────────────────────────────────────────────────────
        # Reliable — dock_0 / dock_1 / undock must not drop
        self._dock_cmd_pub = self.create_publisher(
            String, TOPIC_DOCK_COMMAND, reliable_qos
        )

        # ── NAV CLIENT ────────────────────────────────────────────────────────
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            ACTION_NAV,
            callback_group=self._cbg
        )

        # ── 10 Hz SM TIMER ────────────────────────────────────────────────────
        self._sm_timer = self.create_timer(
            0.1,
            self._sm_tick,
            callback_group=self._cbg
        )

        self.get_logger().info("Load Cell Delivery Node started")

    # ─── logging helper ───────────────────────────────────────────────────────

    def _log(self, tag, interval, msg):
        now = time.monotonic()
        if now - self._last_log.get(tag, 0.0) >= interval:
            self._last_log[tag] = now
            self.get_logger().info(msg)

    # ─── subscriber callbacks ─────────────────────────────────────────────────

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

    # ─── dock event helpers ───────────────────────────────────────────────────

    def _dock_status_cb(self, msg: String):
        """
        Edge-triggered: only update _dock_event when the incoming status
        differs from the current unconsumed event. Prevents repeated 'DOCKED'
        publishes from the hold-zero loop re-triggering the SM after first consumption.
        """
        incoming = msg.data.strip()
        with self._dock_event_lock:
            if self._dock_event != incoming:
                self._dock_event = incoming
                self.get_logger().info(f"[DOCK] event queued: '{incoming}'")

    def _consume_dock_event(self, expected: str) -> bool:
        """
        Atomically check-and-clear the pending dock event.
        Returns True only if pending event matches `expected`.
        Guarantees each DOCKED / UNDOCKED fires exactly once.
        """
        with self._dock_event_lock:
            if self._dock_event == expected:
                self._dock_event = None
                return True
            return False

    def _clear_dock_event(self):
        """Discard any stale pending event before starting a new dock sequence."""
        with self._dock_event_lock:
            self._dock_event = None

    def _peek_dock_event(self) -> Optional[str]:
        with self._dock_event_lock:
            return self._dock_event

    def _send_dock_command(self, cmd: str):
        self._dock_cmd_pub.publish(String(data=cmd))
        self.get_logger().info(f"[DOCK] command sent: '{cmd}'")

    # ─── state machine ────────────────────────────────────────────────────────

    def _sm_tick(self):

        weight = self._current_weight

        # ══ IDLE — auto-kick: navigate to start/pickup station ══
        if self._state == State.IDLE:

            self._log("idle", 3.0, "[IDLE] mission ready — navigating to start")
            self._state = State.NAVIGATE_TO_START
            self._send_nav_goal(
                self._start_pose,
                self._on_start_nav_reached,
                self._on_nav_failure
            )

        # ══ NAVIGATE_TO_START — feedback only, result via callback ══
        elif self._state == State.NAVIGATE_TO_START:

            self._log("nav_start", 2.0,
                      f"[NAV->START] dist={self._distance_remaining:.2f}m")

        # ══ DOCK_AT_START — consume DOCKED edge event ══
        elif self._state == State.DOCK_AT_START:

            self._log("dock0", 2.0,
                      f"[DOCK_0] awaiting DOCKED, pending={self._peek_dock_event()!r}")

            if self._consume_dock_event("DOCKED"):
                self.get_logger().info("[DOCK_0] DOCKED — waiting for load")
                self._weight_start_time = None
                self._state = State.WAIT_FOR_LOAD

        # ══ WAIT_FOR_LOAD ══
        # Phase 1 — weight must be stable >= threshold for WEIGHT_STABLE_TIME_S
        # Phase 2 — remain docked for PRE_NAVIGATE_DELAY_S more (robot stays put)
        # Then     — send undock, robot immediately departs after undock completes
        elif self._state == State.WAIT_FOR_LOAD:

            self._log("load", 2.0,
                      f"[LOAD] weight={weight:.2f}kg threshold={self.WEIGHT_THRESHOLD_KG}")

            if weight >= self.WEIGHT_THRESHOLD_KG:

                if self._weight_start_time is None:
                    self._weight_start_time = time.monotonic()

                elapsed = time.monotonic() - self._weight_start_time
                total_wait = self.WEIGHT_STABLE_TIME_S + self.PRE_NAVIGATE_DELAY_S

                if elapsed < self.WEIGHT_STABLE_TIME_S:
                    # still confirming stability
                    pass

                elif elapsed < total_wait:
                    # load confirmed, holding at dock for pre-delay
                    remaining = total_wait - elapsed
                    self._log("load_delay", 1.0,
                              f"[LOAD] confirmed — undocking in {remaining:.1f}s")

                else:
                    # pre-delay complete — undock now, nav follows immediately after
                    self.get_logger().info(
                        "[LOAD] dock wait complete — sending undock"
                    )
                    self._clear_dock_event()
                    self._send_dock_command("undock")
                    self._state = State.UNDOCK_AT_START

            else:
                # weight dropped before stable — reset and keep waiting
                self._weight_start_time = None

        # ══ UNDOCK_AT_START — consume UNDOCKED, then go directly to delivery ══
        elif self._state == State.UNDOCK_AT_START:

            self._log("undock0", 2.0,
                      f"[UNDOCK_0] awaiting UNDOCKED, pending={self._peek_dock_event()!r}")

            if self._consume_dock_event("UNDOCKED"):
                self.get_logger().info(
                    "[UNDOCK_0] undocked — navigating to delivery"
                )
                self._state = State.NAVIGATE_TO_DELIVERY
                self._send_nav_goal(
                    self._delivery_pose,
                    self._on_delivery_reached,
                    self._on_nav_failure
                )

        # ══ NAVIGATE_TO_DELIVERY — feedback only ══
        elif self._state == State.NAVIGATE_TO_DELIVERY:

            self._log("nav_del", 2.0,
                      f"[NAV->DELIVERY] dist={self._distance_remaining:.2f}m")

        # ══ DOCK_AT_DELIVERY — consume DOCKED edge event ══
        elif self._state == State.DOCK_AT_DELIVERY:

            self._log("dock1", 2.0,
                      f"[DOCK_1] awaiting DOCKED, pending={self._peek_dock_event()!r}")

            if self._consume_dock_event("DOCKED"):
                self.get_logger().info("[DOCK_1] DOCKED — waiting for unload")
                self._unload_start_time = None
                self._state = State.WAIT_FOR_UNLOAD

        # ══ WAIT_FOR_UNLOAD ══
        # Phase 1 — weight must be stable < threshold for WEIGHT_STABLE_TIME_S
        # Phase 2 — remain docked for POST_UNLOAD_DELAY_S more (robot stays put)
        # Then     — send undock, robot immediately departs after undock completes
        elif self._state == State.WAIT_FOR_UNLOAD:

            self._log("unload", 2.0,
                      f"[UNLOAD] weight={weight:.2f}kg threshold={self.WEIGHT_THRESHOLD_KG}")

            if weight < self.WEIGHT_THRESHOLD_KG:

                if self._unload_start_time is None:
                    self._unload_start_time = time.monotonic()

                elapsed = time.monotonic() - self._unload_start_time
                total_wait = self.WEIGHT_STABLE_TIME_S + self.POST_UNLOAD_DELAY_S

                if elapsed < self.WEIGHT_STABLE_TIME_S:
                    # still confirming unload stability
                    pass

                elif elapsed < total_wait:
                    # unload confirmed, holding at dock for post-delay
                    remaining = total_wait - elapsed
                    self._log("unload_delay", 1.0,
                              f"[UNLOAD] confirmed — undocking in {remaining:.1f}s")

                else:
                    # post-delay complete — undock now, nav follows immediately after
                    self.get_logger().info(
                        "[UNLOAD] dock wait complete — sending undock"
                    )
                    self._clear_dock_event()
                    self._send_dock_command("undock")
                    self._state = State.UNDOCK_AT_DELIVERY

            else:
                # weight back above threshold before stable — reset and keep waiting
                self._unload_start_time = None

        # ══ UNDOCK_AT_DELIVERY — consume UNDOCKED, then go directly to start ══
        elif self._state == State.UNDOCK_AT_DELIVERY:

            self._log("undock1", 2.0,
                      f"[UNDOCK_1] awaiting UNDOCKED, pending={self._peek_dock_event()!r}")

            if self._consume_dock_event("UNDOCKED"):
                self.get_logger().info(
                    "[UNDOCK_1] undocked — navigating to start"
                )
                self._state = State.NAVIGATE_TO_START

                home = self._start_pose
                home.header.stamp = self.get_clock().now().to_msg()

                self._send_nav_goal(
                    home,
                    self._on_start_reached,
                    self._on_nav_failure
                )

    # ─── nav pipeline ─────────────────────────────────────────────────────────

    def _send_nav_goal(self, pose, on_success, on_failure):

        if self._nav_in_flight:
            self.get_logger().warn("[NAV] goal already in flight — skipping")
            return

        if not self._nav_client.wait_for_server(timeout_sec=self.NAV_SERVER_TIMEOUT_S):
            self.get_logger().error("Nav server unavailable")
            on_failure()
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self._nav_in_flight      = True
        self._distance_remaining = 0.0

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
            self._nav_in_flight = False
            on_failure()
            return

        self._goal_handle = goal_handle

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            lambda f: self._result_cb(f, on_success, on_failure)
        )

    def _result_cb(self, future, on_success, on_failure):

        self._nav_in_flight      = False
        self._distance_remaining = 0.0

        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            on_success()
        else:
            self.get_logger().error(f"Navigation failed (status={status})")
            on_failure()

    def _nav_feedback_cb(self, msg):
        self._distance_remaining = msg.feedback.distance_remaining

    # ─── nav arrival callbacks ────────────────────────────────────────────────

    def _on_start_nav_reached(self):
        self.get_logger().info("[NAV] reached START — sending dock_0")
        self._clear_dock_event()
        self._send_dock_command("dock_0")
        self._state = State.DOCK_AT_START

    def _on_delivery_reached(self):
        self.get_logger().info("[NAV] reached DELIVERY — sending dock_1")
        self._clear_dock_event()
        self._send_dock_command("dock_1")
        self._state = State.DOCK_AT_DELIVERY

    def _on_start_reached(self):
        self.get_logger().info("Start reached — cycle complete")
        self._state = State.IDLE
        self._weight_start_time = None
        self._unload_start_time = None

    def _on_nav_failure(self):
        time.sleep(3)
        self._state = State.IDLE

    # ─── cleanup ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self._nav_in_flight and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        super().destroy_node()


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