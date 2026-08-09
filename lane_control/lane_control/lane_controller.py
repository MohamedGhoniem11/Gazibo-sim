#!/usr/bin/env python3
"""PID-style lane centering controller.

Subscribes to /lane/error, /lane/confidence, /lane/detected from lane_perception
and publishes geometry_msgs/Twist on /cmd_vel.

Steering sign (must match lane_detector convention):
  angular.z = -Kp * error
so a positive error (lane center to the right of image center) produces a
right turn (negative yaw rate in ROS).
"""

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class LaneController(Node):
    """Keep the vehicle centered in the detected lane."""

    def __init__(self) -> None:
        super().__init__('lane_controller')
        self._declare_parameters()
        self._load_parameters()

        self._error: float = 0.0
        self._confidence: float = 0.0
        self._detected: bool = False
        self._last_perception_time: Optional[float] = None
        self._lost_since: Optional[float] = None

        self._integral: float = 0.0
        self._prev_error: float = 0.0
        self._prev_time: Optional[float] = None
        self._smoothed_angular: float = 0.0
        self._ever_detected: bool = False
        self._last_good_angular: float = 0.0

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Float32, '/lane/error', self._on_error, 10)
        self.create_subscription(Float32, '/lane/confidence', self._on_confidence, 10)
        self.create_subscription(Bool, '/lane/detected', self._on_detected, 10)

        period = 1.0 / max(self._control_rate_hz, 1.0)
        self._timer = self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'lane_controller ready (base_speed={self._base_speed}, kp={self._kp})'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('base_speed', 1.5)
        self.declare_parameter('minimum_speed', 0.4)
        self.declare_parameter('maximum_speed', 2.5)
        self.declare_parameter('kp', 1.2)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.15)
        self.declare_parameter('maximum_angular_speed', 0.8)
        self.declare_parameter('confidence_threshold', 0.15)
        self.declare_parameter('lost_lane_timeout', 1.5)
        self.declare_parameter('perception_timeout', 0.75)
        self.declare_parameter('steering_smoothing', 0.4)
        self.declare_parameter('curve_slowdown_factor', 1.5)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('error_deadzone', 0.0)
        # Hold last steer briefly through curve dropouts instead of slamming straight/stop
        self.declare_parameter('lost_hold_steer_time', 1.2)

    def _load_parameters(self) -> None:
        gp = self.get_parameter
        self._base_speed = float(gp('base_speed').value)
        self._minimum_speed = float(gp('minimum_speed').value)
        self._maximum_speed = float(gp('maximum_speed').value)
        self._kp = float(gp('kp').value)
        self._ki = float(gp('ki').value)
        self._kd = float(gp('kd').value)
        self._maximum_angular_speed = float(gp('maximum_angular_speed').value)
        self._confidence_threshold = float(gp('confidence_threshold').value)
        self._lost_lane_timeout = float(gp('lost_lane_timeout').value)
        self._perception_timeout = float(gp('perception_timeout').value)
        self._steering_smoothing = float(gp('steering_smoothing').value)
        self._curve_slowdown_factor = float(gp('curve_slowdown_factor').value)
        self._control_rate_hz = float(gp('control_rate_hz').value)
        self._error_deadzone = float(gp('error_deadzone').value)
        self._lost_hold_steer_time = float(gp('lost_hold_steer_time').value)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _touch_perception(self) -> None:
        self._last_perception_time = self._now()

    def _on_error(self, msg: Float32) -> None:
        self._error = float(msg.data)
        self._touch_perception()

    def _on_confidence(self, msg: Float32) -> None:
        self._confidence = float(msg.data)
        self._touch_perception()

    def _on_detected(self, msg: Bool) -> None:
        self._detected = bool(msg.data)
        self._touch_perception()
        if not self._detected:
            if self._lost_since is None:
                self._lost_since = self._now()
        else:
            self._lost_since = None

    def _reset_pid(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None
        self._smoothed_angular = 0.0

    def _publish_stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def _control_loop(self) -> None:
        now = self._now()

        # Watchdog: stale perception
        if self._last_perception_time is None:
            self._publish_stop()
            return
        if (now - self._last_perception_time) > self._perception_timeout:
            self.get_logger().warn('Perception data stale — stopping', throttle_duration_sec=2.0)
            self._reset_pid()
            self._publish_stop()
            return

        if self._detected and self._confidence >= self._confidence_threshold:
            self._ever_detected = True
        lane_ok = self._detected and self._confidence >= self._confidence_threshold
        if not lane_ok:
            if self._lost_since is None:
                self._lost_since = now
            lost_for = now - self._lost_since
            # Through brief curve dropouts: keep last steer, slow down
            if self._ever_detected and lost_for < self._lost_hold_steer_time:
                cmd = Twist()
                cmd.linear.x = self._minimum_speed
                cmd.angular.z = self._last_good_angular
                self._cmd_pub.publish(cmd)
                return
            if lost_for > self._lost_lane_timeout:
                if not self._ever_detected and lost_for < 8.0:
                    cmd = Twist()
                    cmd.linear.x = self._minimum_speed
                    cmd.angular.z = 0.0
                    self._cmd_pub.publish(cmd)
                    self.get_logger().info(
                        'Seeking lane (straight creep)', throttle_duration_sec=2.0
                    )
                    return
                self.get_logger().warn('Lane lost — stopping', throttle_duration_sec=2.0)
                self._reset_pid()
                self._publish_stop()
                return
            cmd = Twist()
            cmd.linear.x = self._minimum_speed
            cmd.angular.z = 0.0
            self._cmd_pub.publish(cmd)
            return

        # PID on lateral error
        dt = 0.0
        if self._prev_time is not None:
            dt = max(1e-3, now - self._prev_time)
        self._prev_time = now

        error = self._error
        if abs(error) < self._error_deadzone:
            error = 0.0
        # Opposite steer: positive error => turn right (negative z)
        p_term = -self._kp * error

        self._integral += error * dt if dt > 0.0 else 0.0
        # Anti-windup
        max_i = 1.0
        self._integral = max(-max_i, min(max_i, self._integral))
        i_term = -self._ki * self._integral

        d_term = 0.0
        if dt > 0.0:
            d_term = -self._kd * (error - self._prev_error) / dt
        self._prev_error = error

        angular = p_term + i_term + d_term
        angular = max(
            -self._maximum_angular_speed,
            min(self._maximum_angular_speed, angular),
        )

        alpha = float(max(0.0, min(1.0, self._steering_smoothing)))
        self._smoothed_angular = (
            alpha * angular + (1.0 - alpha) * self._smoothed_angular
        )

        # Slow down when steering hard
        speed = self._base_speed / (
            1.0 + self._curve_slowdown_factor * abs(self._smoothed_angular)
        )
        speed = max(self._minimum_speed, min(self._maximum_speed, speed))

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = self._smoothed_angular
        self._last_good_angular = self._smoothed_angular
        self._cmd_pub.publish(cmd)

    def destroy_node(self) -> bool:
        # Always stop the car on shutdown
        try:
            self._publish_stop()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
