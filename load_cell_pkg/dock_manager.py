#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
import math


# Reliable QoS — must match load_cell_delivery_node on both dock topics
_RELIABLE_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


class MultiDockManager(Node):

    def __init__(self):
        super().__init__('multi_dock_manager')

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('handover_distance', 0.18)
        self.declare_parameter('align_speed', 0.06)
        self.declare_parameter('min_linear_speed', 0.03)
        self.declare_parameter('brake_zone', 0.40)
        self.declare_parameter('angular_gain', 1.2)
        self.declare_parameter('max_angular_speed', 0.14)
        self.declare_parameter('search_speed', 0.09)
        self.declare_parameter('angular_deadband', 0.02)
        self.declare_parameter('angular_tolerance', 3.0)
        self.declare_parameter('undock_speed', -0.06)
        self.declare_parameter('undock_duration', 4.0)
        self.declare_parameter('search_timeout', 120.0)

        # per-state TF staleness limits
        self.declare_parameter('tf_max_age_search', 0.15)   # strict  — must be truly fresh
        self.declare_parameter('tf_max_age_rotate', 0.50)   # relaxed — brief FOV loss ok
        self.declare_parameter('tf_max_age_align',  0.30)   # medium  — some tolerance

        # consecutive TF failures before falling back to SEARCH
        self.declare_parameter('max_consecutive_failures', 5)  # ~0.5s at 10Hz

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- PUB/SUB ----------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Reliable — dock_0/dock_1/undock commands must not be dropped
        self.command_sub = self.create_subscription(
            String, '/align_dock/command', self.command_callback, _RELIABLE_QOS)

        # Reliable — DOCKED/UNDOCKED must always reach the mission node
        self.status_pub = self.create_publisher(String, '/dock_status', _RELIABLE_QOS)

        # ---------------- STATE ----------------
        self.state            = "IDLE"
        self.target_tag       = None
        self.start_time       = None
        self.search_start     = None

        self.consecutive_tf_failures = 0

        self.robot_base_frame = 'base_link'
        self.camera_frame     = 'camera_color_optical_frame'

        self.publish_status("IDLE")
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Multi Dock Manager READY")

    # ------------------------------------------------
    def publish_status(self, status: str):
        self.status_pub.publish(String(data=status))

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def reset_failures(self):
        self.consecutive_tf_failures = 0

    def fallback_to_search(self, reason: str):
        """Common fallback — stop, reset counter, go to SEARCH."""
        self.consecutive_tf_failures = 0
        self.state        = "SEARCH"
        self.search_start = self.get_clock().now()
        self.stop_robot()
        self.publish_status("SEARCHING")
        self.get_logger().warn(f"→ SEARCH fallback: {reason}")

    # ------------------------------------------------
    def command_callback(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd == 'dock_0':
            self.target_tag   = 'tag36h11:0'
            self.state        = "SEARCH"
            self.search_start = self.get_clock().now()
            self.reset_failures()
            self.publish_status("SEARCHING")
            self.get_logger().info("Dock 0 — searching for tag")

        elif cmd == 'dock_1':
            self.target_tag   = 'tag36h11:1'
            self.state        = "SEARCH"
            self.search_start = self.get_clock().now()
            self.reset_failures()
            self.publish_status("SEARCHING")
            self.get_logger().info("Dock 1 — searching for tag")

        elif cmd == 'undock' and self.state == "DOCKED":
            self.state      = "UNDOCK"
            self.start_time = self.get_clock().now()
            self.publish_status("UNDOCKING")
            self.get_logger().info("Undocking started")

        elif cmd == 'stop':
            self.state = "IDLE"
            self.reset_failures()
            self.stop_robot()
            self.publish_status("IDLE")
            self.get_logger().info("Stopped")

    # ------------------------------------------------
    def compute_angular(self, heading_error: float) -> float:
        kp   = self.get_parameter('angular_gain').value
        maxw = self.get_parameter('max_angular_speed').value
        db   = self.get_parameter('angular_deadband').value
        raw  = max(-maxw, min(maxw, kp * heading_error))
        return 0.0 if abs(heading_error) < db else raw

    def compute_linear(self, depth_z: float) -> float:
        """Smooth brake ramp — never drops below min_linear_speed."""
        handover   = self.get_parameter('handover_distance').value
        brake_zone = self.get_parameter('brake_zone').value
        base_speed = self.get_parameter('align_speed').value
        min_speed  = self.get_parameter('min_linear_speed').value
        if depth_z > brake_zone:
            return base_speed
        t = (depth_z - handover) / (brake_zone - handover)   # 1.0→0.0
        return max(min_speed, min(base_speed, min_speed + (base_speed - min_speed) * t))

    def tf_age_sec(self, stamp) -> float:
        now_ns   = self.get_clock().now().nanoseconds
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        return (now_ns - stamp_ns) * 1e-9

    def try_lookup_both(self, max_age_sec: float):
        """
        Returns (z_depth, heading_error) only if BOTH transforms are fresh.

        z_depth       — camera frame Z depth  (reliable stop distance)
        heading_error — base_link atan2 bearing (correct angular control)

        Raises TransformException if:
          - TF lookup fails (tag not in tree)
          - Either transform stamp is older than max_age_sec (stale cache)
        """
        lookup_timeout = rclpy.duration.Duration(seconds=0.05)

        tf_cam = self.tf_buffer.lookup_transform(
            self.camera_frame, self.target_tag,
            rclpy.time.Time(), timeout=lookup_timeout)

        tf_base = self.tf_buffer.lookup_transform(
            self.robot_base_frame, self.target_tag,
            rclpy.time.Time(), timeout=lookup_timeout)

        cam_age  = self.tf_age_sec(tf_cam.header.stamp)
        base_age = self.tf_age_sec(tf_base.header.stamp)

        if cam_age > max_age_sec or base_age > max_age_sec:
            raise TransformException(
                f"Stale TF — cam={cam_age:.2f}s  base={base_age:.2f}s  limit={max_age_sec}s")

        z             = tf_cam.transform.translation.z
        tag_x         = tf_base.transform.translation.x
        tag_y         = tf_base.transform.translation.y
        heading_error = math.atan2(tag_y, tag_x)

        return z, heading_error

    def handle_tf_failure(self, state_name: str, error: TransformException) -> bool:
        """
        Increments consecutive failure counter.
        Returns True  if caller should fall back to SEARCH (threshold reached).
        Returns False if caller should pause in place and wait for next frame.
        """
        max_fail = self.get_parameter('max_consecutive_failures').value
        self.consecutive_tf_failures += 1

        self.get_logger().warn(
            f"[{state_name}] TF fail {self.consecutive_tf_failures}/{max_fail}: {error}",
            throttle_duration_sec=0.5)

        if self.consecutive_tf_failures >= max_fail:
            return True   # caller should fall back
        return False      # caller should pause

    # ------------------------------------------------
    def control_loop(self):

        if self.state == "IDLE":
            return

        # ================= DOCKED — hold zero continuously =================
        if self.state == "DOCKED":
            self.stop_robot()
            return

        # ================= UNDOCK =================
        if self.state == "UNDOCK":
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            if elapsed >= self.get_parameter('undock_duration').value:
                self.state = "IDLE"
                self.stop_robot()
                self.publish_status("UNDOCKED")
                self.get_logger().info("UNDOCKED")
                return
            cmd = Twist()
            cmd.linear.x = self.get_parameter('undock_speed').value
            self.cmd_pub.publish(cmd)
            return

        # ================= SEARCH =================
        # Rotate slowly — strict freshness (0.15s) to confirm tag truly visible
        if self.state == "SEARCH":
            elapsed = (self.get_clock().now() - self.search_start).nanoseconds * 1e-9

            if elapsed >= self.get_parameter('search_timeout').value:
                self.state = "IDLE"
                self.stop_robot()
                self.publish_status("SEARCH_FAILED")
                self.get_logger().error(f"Tag not found after {elapsed:.0f}s — aborting")
                return

            try:
                z, heading_error = self.try_lookup_both(
                    self.get_parameter('tf_max_age_search').value)  # 0.15s strict

                # fresh tag confirmed → switch to ROTATE
                self.reset_failures()
                self.state = "ROTATE"
                self.stop_robot()
                self.publish_status("TAG_FOUND")
                self.get_logger().info(
                    f"Fresh tag z={z:.3f}m  err={math.degrees(heading_error):.1f}° "
                    f"— correcting heading")
                return

            except TransformException:
                pass   # stale or not visible — keep searching, no failure count here

            cmd = Twist()
            cmd.angular.z = self.get_parameter('search_speed').value
            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                f"[SEARCH] {elapsed:.1f}s / {self.get_parameter('search_timeout').value:.0f}s",
                throttle_duration_sec=1.0)
            return

        # ================= ROTATE =================
        # Relaxed staleness (0.5s) — brief FOV loss during rotation is normal
        # Only falls back after max_consecutive_failures missed frames
        if self.state == "ROTATE":
            try:
                z, heading_error = self.try_lookup_both(
                    self.get_parameter('tf_max_age_rotate').value)  # 0.5s relaxed

                self.reset_failures()   # got a good frame, reset counter

            except TransformException as e:
                if self.handle_tf_failure("ROTATE", e):
                    self.fallback_to_search("tag lost/stale in ROTATE")
                else:
                    self.stop_robot()   # pause in place, wait for next frame
                return

            tol_rad = math.radians(self.get_parameter('angular_tolerance').value)

            self.get_logger().info(
                f"[ROTATE] z={z:.3f}m  err={math.degrees(heading_error):.1f}°",
                throttle_duration_sec=0.2)

            if abs(heading_error) <= tol_rad:
                self.reset_failures()
                self.state = "ALIGN"
                self.stop_robot()
                self.publish_status("DOCKING")
                self.get_logger().info(
                    f"Heading aligned ({math.degrees(heading_error):.1f}°) — approaching")
                return

            cmd = Twist()
            cmd.linear.x  = 0.0   # strictly no forward during rotation
            cmd.angular.z = self.compute_angular(heading_error)
            self.cmd_pub.publish(cmd)
            return

        # ================= ALIGN =================
        # Medium staleness (0.3s) — motion blur at close range needs some tolerance
        # Stops in place on single failure, falls back only after sustained loss
        if self.state == "ALIGN":
            try:
                z, heading_error = self.try_lookup_both(
                    self.get_parameter('tf_max_age_align').value)   # 0.3s medium

                self.reset_failures()

            except TransformException as e:
                if self.handle_tf_failure("ALIGN", e):
                    self.fallback_to_search("tag lost/stale in ALIGN")
                else:
                    self.stop_robot()   # pause in place, wait for next frame
                return

            tol_rad  = math.radians(self.get_parameter('angular_tolerance').value)
            handover = self.get_parameter('handover_distance').value

            self.get_logger().info(
                f"[ALIGN] z={z:.3f}m  err={math.degrees(heading_error):.1f}°",
                throttle_duration_sec=0.2)

            # smooth stop — robot already crawling before this triggers
            if z <= handover:
                self.reset_failures()
                self.state = "DOCKED"
                self.stop_robot()
                self.publish_status("DOCKED")
                self.get_logger().info(f"DOCKED — z={z:.3f}m")
                return

            # heading drifted beyond 1.5x tolerance → re-rotate before continuing
            if abs(heading_error) > tol_rad * 1.5:
                self.reset_failures()
                self.state = "ROTATE"
                self.stop_robot()
                self.publish_status("ROTATING")
                self.get_logger().warn(
                    f"Heading drifted {math.degrees(heading_error):.1f}° — re-rotating")
                return

            cmd = Twist()
            cmd.linear.x  = self.compute_linear(z)
            cmd.angular.z = self.compute_angular(heading_error)
            self.cmd_pub.publish(cmd)
            return


def main(args=None):
    rclpy.init(args=args)
    node = MultiDockManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()