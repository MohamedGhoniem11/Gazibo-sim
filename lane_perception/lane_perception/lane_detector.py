#!/usr/bin/env python3
"""Lane detector for Sonoma: yellow paint + white barriers, look-ahead center."""

from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


class LaneDetector(Node):
    def __init__(self) -> None:
        super().__init__('lane_detector')
        self._declare_parameters()
        self._load_parameters()
        self._bridge = CvBridge()
        self._smoothed_error: Optional[float] = None
        self._last_half_width: Optional[float] = None

        self._error_pub = self.create_publisher(Float32, '/lane/error', 10)
        self._confidence_pub = self.create_publisher(Float32, '/lane/confidence', 10)
        self._detected_pub = self.create_publisher(Bool, '/lane/detected', 10)
        self._debug_pub = self.create_publisher(Image, self._debug_image_topic, 10)
        self.create_subscription(
            Image, self._camera_topic, self._on_image, qos_profile_sensor_data
        )
        self.get_logger().info(f'lane_detector on {self._camera_topic}')

    def _declare_parameters(self) -> None:
        self.declare_parameter('camera_topic', '/prius/front_camera/image_raw')
        self.declare_parameter('roi_top_ratio', 0.45)
        self.declare_parameter('roi_bottom_ratio', 0.88)
        self.declare_parameter('lookahead_ratio', 0.28)
        self.declare_parameter('yellow_h_low', 10)
        self.declare_parameter('yellow_h_high', 45)
        self.declare_parameter('yellow_s_low', 30)
        self.declare_parameter('yellow_v_low', 28)
        self.declare_parameter('yellow_v_high', 210)
        self.declare_parameter('hls_l_low', 16)
        self.declare_parameter('hls_s_low', 18)
        self.declare_parameter('barrier_l_low', 140)
        self.declare_parameter('barrier_s_high', 55)
        self.declare_parameter('blur_kernel', 5)
        self.declare_parameter('minimum_lane_pixels', 40)
        self.declare_parameter('min_side_pixels', 30)
        self.declare_parameter('expected_lane_width_ratio', 0.62)
        self.declare_parameter('target_offset_ratio', 0.0)
        self.declare_parameter('smoothing_factor', 0.32)
        self.declare_parameter('confidence_pixel_scale', 600.0)
        self.declare_parameter('debug_enabled', True)
        self.declare_parameter('debug_image_topic', '/lane/debug_image')

    def _load_parameters(self) -> None:
        gp = self.get_parameter
        self._camera_topic = gp('camera_topic').get_parameter_value().string_value
        self._roi_top_ratio = float(gp('roi_top_ratio').value)
        self._roi_bottom_ratio = float(gp('roi_bottom_ratio').value)
        self._lookahead_ratio = float(gp('lookahead_ratio').value)
        self._yellow_h_low = int(gp('yellow_h_low').value)
        self._yellow_h_high = int(gp('yellow_h_high').value)
        self._yellow_s_low = int(gp('yellow_s_low').value)
        self._yellow_v_low = int(gp('yellow_v_low').value)
        self._yellow_v_high = int(gp('yellow_v_high').value)
        self._hls_l_low = int(gp('hls_l_low').value)
        self._hls_s_low = int(gp('hls_s_low').value)
        self._barrier_l_low = int(gp('barrier_l_low').value)
        self._barrier_s_high = int(gp('barrier_s_high').value)
        self._blur_kernel = int(gp('blur_kernel').value)
        self._minimum_lane_pixels = int(gp('minimum_lane_pixels').value)
        self._min_side_pixels = int(gp('min_side_pixels').value)
        self._expected_lane_width_ratio = float(gp('expected_lane_width_ratio').value)
        self._target_offset_ratio = float(gp('target_offset_ratio').value)
        self._smoothing_factor = float(gp('smoothing_factor').value)
        self._confidence_pixel_scale = float(gp('confidence_pixel_scale').value)
        self._debug_enabled = bool(gp('debug_enabled').value)
        self._debug_image_topic = gp('debug_image_topic').get_parameter_value().string_value

    def _on_image(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge: {exc}')
            self._publish_lost()
            return
        if bgr is None or bgr.size == 0:
            self._publish_lost()
            return
        self._publish_result(self._process(bgr), msg.header)

    def _process(self, bgr: np.ndarray) -> dict:
        height, width = bgr.shape[:2]
        top = int(np.clip(self._roi_top_ratio, 0.0, 0.9) * height)
        bottom = int(np.clip(self._roi_bottom_ratio, self._roi_top_ratio + 0.05, 1.0) * height)
        roi = bgr[top:bottom, :]
        rh, rw = roi.shape[:2]

        yellow, barrier = self._masks(roi)
        y_look = int(np.clip(self._lookahead_ratio, 0.05, 0.7) * (rh - 1))

        left_x, right_x, pixels, left_fit, right_fit = self._edges_at(
            yellow, barrier, roi, y_look
        )

        image_center = width / 2.0
        target = image_center + self._target_offset_ratio * (width / 2.0)
        default_half = 0.5 * self._expected_lane_width_ratio * width
        half_w = self._last_half_width if self._last_half_width is not None else default_half

        lane_center: Optional[float] = None
        both = left_x is not None and right_x is not None
        if both and (right_x - left_x) > 0.16 * width:
            lane_center = 0.5 * (left_x + right_x)
            measured = 0.5 * (right_x - left_x)
            if 0.12 * width < measured < 0.42 * width:
                self._last_half_width = measured
        elif left_x is not None and right_x is None:
            lane_center = left_x + half_w
        elif right_x is not None and left_x is None:
            lane_center = right_x - half_w

        if lane_center is not None and not (0.08 * width < lane_center < 0.92 * width):
            lane_center = None

        detected = lane_center is not None and pixels >= self._minimum_lane_pixels
        error = 0.0
        confidence = 0.0
        if detected and lane_center is not None:
            raw = float(np.clip((lane_center - target) / (width / 2.0), -1.0, 1.0))
            confidence = float(np.clip(pixels / self._confidence_pixel_scale, 0.05, 1.0))
            if both:
                confidence = min(1.0, confidence + 0.3)
            else:
                confidence = min(0.85, confidence)
            alpha = float(np.clip(self._smoothing_factor, 0.0, 1.0))
            self._smoothed_error = (
                raw if self._smoothed_error is None
                else alpha * raw + (1.0 - alpha) * self._smoothed_error
            )
            error = float(self._smoothed_error)
        else:
            self._smoothed_error = None
            detected = False

        debug = None
        if self._debug_enabled:
            debug = self._draw(
                bgr, top, bottom, yellow, barrier, left_fit, right_fit,
                left_x, right_x, lane_center, image_center, y_look,
                error, confidence, detected,
            )
        return {'error': error, 'confidence': confidence, 'detected': detected, 'debug': debug}

    def _masks(self, roi: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        k = self._blur_kernel if self._blur_kernel % 2 == 1 else self._blur_kernel + 1
        work = cv2.GaussianBlur(roi, (k, k), 0) if k > 1 else roi
        hsv = cv2.cvtColor(work, cv2.COLOR_BGR2HSV)
        hls = cv2.cvtColor(work, cv2.COLOR_BGR2HLS)
        yellow = cv2.bitwise_or(
            cv2.inRange(
                hsv,
                np.array([self._yellow_h_low, self._yellow_s_low, self._yellow_v_low], np.uint8),
                np.array([self._yellow_h_high, 255, self._yellow_v_high], np.uint8),
            ),
            cv2.inRange(
                hls,
                np.array([self._yellow_h_low, self._hls_l_low, self._hls_s_low], np.uint8),
                np.array([self._yellow_h_high, 220, 255], np.uint8),
            ),
        )
        barrier = cv2.inRange(
            hls,
            np.array([0, self._barrier_l_low, 0], np.uint8),
            np.array([180, 255, self._barrier_s_high], np.uint8),
        )
        # Ignore distant top band for yellow (stadium); keep barriers
        yellow[: int(0.10 * yellow.shape[0]), :] = 0
        kernel = np.ones((3, 3), np.uint8)
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, kernel)
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_CLOSE, kernel)
        barrier = cv2.morphologyEx(barrier, cv2.MORPH_OPEN, kernel)
        return yellow, barrier

    def _edges_at(
        self,
        yellow: np.ndarray,
        barrier: np.ndarray,
        roi: np.ndarray,
        y_look: int,
    ) -> Tuple[Optional[float], Optional[float], int, Optional[np.ndarray], Optional[np.ndarray]]:
        h, w = yellow.shape[:2]
        pixels = int(cv2.countNonZero(yellow)) + int(cv2.countNonZero(barrier)) // 4

        left_fit = self._fit_side(yellow, 0, int(0.58 * w), h, 'left')
        right_fit = self._fit_side(yellow, int(0.42 * w), w, h, 'right')

        left_x = self._eval_fit(left_fit, float(y_look), w)
        right_x = self._eval_fit(right_fit, float(y_look), w)

        # Barrier inner edges at look-ahead row (side strips only — avoid sky)
        if left_x is None:
            left_x = self._barrier_inner(barrier, y_look, 0, int(0.40 * w), side='left')
        if right_x is None:
            right_x = self._barrier_inner(barrier, y_look, int(0.60 * w), w, side='right')

        # Asphalt corridor fallback at look-ahead band
        if left_x is None or right_x is None:
            al, ar = self._asphalt_edges(roi, y_look)
            if left_x is None:
                left_x = al
            if right_x is None:
                right_x = ar

        return left_x, right_x, pixels, left_fit, right_fit

    def _fit_side(
        self, mask: np.ndarray, x0: int, x1: int, height: int, side: str
    ) -> Optional[np.ndarray]:
        region = mask[:, x0:x1]
        ys, xs = np.nonzero(region)
        if len(xs) < self._min_side_pixels:
            return None
        xs = xs + x0
        weights = 0.2 + 0.8 * (ys / max(height - 1, 1))
        try:
            fit = np.polyfit(ys.astype(np.float64), xs.astype(np.float64), 1, w=weights)
        except (np.linalg.LinAlgError, ValueError, TypeError):
            return None
        slope = float(fit[0])
        if abs(slope) > 3.0:
            return None
        if side == 'left' and slope > 1.5:
            return None
        if side == 'right' and slope < -1.5:
            return None
        return fit

    def _barrier_inner(
        self, barrier: np.ndarray, y: int, x0: int, x1: int, side: str
    ) -> Optional[float]:
        y0 = max(0, y - 4)
        y1 = min(barrier.shape[0], y + 5)
        band = barrier[y0:y1, x0:x1]
        col = (band > 0).sum(axis=0)
        hits = np.where(col >= 2)[0]
        if len(hits) < 3:
            return None
        if side == 'left':
            # Inner edge = rightmost barrier pixel in left strip
            return float(x0 + hits[-1])
        return float(x0 + hits[0])

    def _asphalt_edges(
        self, roi: np.ndarray, y_look: int
    ) -> Tuple[Optional[float], Optional[float]]:
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]
        y0 = max(0, y_look - 3)
        y1 = min(h, y_look + 4)
        row = gray[y0:y1].mean(axis=0)
        # Dark asphalt relative to this row
        thr = float(np.percentile(row, 45))
        dark = row < max(35.0, min(thr, 70.0))
        # Largest contiguous dark run
        indexed = np.where(dark)[0]
        if len(indexed) < 20:
            return None, None
        breaks = np.where(np.diff(indexed) > 2)[0]
        segs = np.split(indexed, breaks + 1)
        seg = max(segs, key=len)
        if len(seg) < 0.15 * w:
            return None, None
        return float(seg[0]), float(seg[-1])

    @staticmethod
    def _eval_fit(
        fit: Optional[np.ndarray], y: float, width: int
    ) -> Optional[float]:
        if fit is None:
            return None
        x = float(np.polyval(fit, y))
        if x < -0.02 * width or x > 1.02 * width:
            return None
        return float(np.clip(x, 0.0, width - 1.0))

    def _draw(
        self, bgr, top, bottom, yellow, barrier, left_fit, right_fit,
        left_x, right_x, lane_center, image_center, y_look,
        error, confidence, detected,
    ) -> np.ndarray:
        out = bgr.copy()
        h, w = out.shape[:2]
        rh = bottom - top
        cv2.line(out, (0, top), (w, top), (255, 255, 0), 1)
        cv2.line(out, (0, bottom), (w, bottom), (255, 255, 0), 1)
        overlay = out[top:bottom].copy()
        color = np.zeros_like(overlay)
        color[yellow > 0] = (0, 255, 255)
        color[barrier > 0] = (255, 180, 0)
        out[top:bottom] = cv2.addWeighted(overlay, 0.65, color, 0.35, 0)

        for fit in (left_fit, right_fit):
            if fit is None:
                continue
            ys = np.linspace(0, rh - 1, 16)
            xs = np.polyval(fit, ys)
            pts = np.stack([xs, ys + top], axis=1).astype(np.int32)
            cv2.polylines(out, [pts], False, (0, 255, 0), 2)

        y_abs = int(top + y_look)
        cv2.line(out, (0, y_abs), (w, y_abs), (255, 128, 0), 1)

        def vline(x, bgr_c, label):
            if x is None:
                return
            xi = int(np.clip(x, 0, w - 1))
            cv2.line(out, (xi, top), (xi, bottom - 1), bgr_c, 2)
            cv2.putText(
                out, label, (xi + 3, top + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr_c, 1, cv2.LINE_AA,
            )

        vline(left_x, (0, 200, 0), 'L')
        vline(right_x, (0, 200, 0), 'R')
        vline(lane_center, (0, 0, 255), 'C')
        vline(image_center, (255, 0, 0), 'img')
        status = 'DETECTED' if detected else 'LOST'
        cv2.putText(
            out, f'{status}  err={error:+.3f}  conf={confidence:.2f}',
            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
            (0, 255, 0) if detected else (0, 0, 255), 2, cv2.LINE_AA,
        )
        return out

    def _publish_result(self, result: dict, header) -> None:
        self._error_pub.publish(Float32(data=float(result['error'])))
        self._confidence_pub.publish(Float32(data=float(result['confidence'])))
        self._detected_pub.publish(Bool(data=bool(result['detected'])))
        if self._debug_enabled and result['debug'] is not None:
            try:
                msg = self._bridge.cv2_to_imgmsg(result['debug'], encoding='bgr8')
                msg.header = header
                self._debug_pub.publish(msg)
            except CvBridgeError as exc:
                self.get_logger().error(f'debug: {exc}')

    def _publish_lost(self) -> None:
        self._smoothed_error = None
        self._error_pub.publish(Float32(data=0.0))
        self._confidence_pub.publish(Float32(data=0.0))
        self._detected_pub.publish(Bool(data=False))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneDetector()
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
