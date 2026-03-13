#! /usr/bin/env python3
import os
import threading
import time
import yaml
from math import atan2, cos, pi, sin, sqrt

import cv2
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_interfaces.msg import AirQuality
from trajectory_msgs.msg import JointTrajectoryPoint

from stretch_nav2.robot_navigator import BasicNavigator, TaskResult
from .heatmap import Heatmap
from .plan import Planner

SENSOR_DHT_FAIL = 0x1
SENSOR_SGP_FAIL = 0x2

SIGMA_DEFAULTS = {
    0: 0.15,  # temperature
    1: 0.60,  # humidity
    2: 15.0,  # tvoc
    3: 30.0,  # eco2
}

CI_DEFAULTS = {
    0: 0.08,
    1: 0.30,
    2: 8.0,
    3: 15.0,
}

SLOPE_DEFAULTS = {
    0: 0.02,  # temperature
    1: 0.08,  # humidity
    2: 2.0,   # tvoc
    3: 5.0,   # eco2
}


class Monitor(Node):
    def __init__(self):
        super().__init__("air_quality_monitor")

        self.declare_parameter("map_yaml", "")
        self.declare_parameter("map_pgm", "")
        self.declare_parameter("heatmap_type", 3)

        self.declare_parameter("settle_time", 3.0)
        self.declare_parameter("sample_period_sec", 1.0)
        self.declare_parameter("min_samples", 5)
        self.declare_parameter("max_samples", 10)
        self.declare_parameter("max_collect_sec", 15.0)
        self.declare_parameter("stable_window", 4)
        self.declare_parameter("metric_sigma_threshold", -1.0)
        self.declare_parameter("metric_ci_threshold", -1.0)
        self.declare_parameter("metric_slope_threshold", -1.0)

        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)

        self.declare_parameter("adaptive_stride", 0.6)
        self.declare_parameter("adaptive_top_k", 20)
        self.declare_parameter("adaptive_min_distance", 0.6)
        self.declare_parameter("adaptive_travel_weight", 0.25)
        self.declare_parameter("adaptive_grad_weight", 0.6)
        self.declare_parameter("idw_power", 2.0)
        self.declare_parameter("auto_tune_idw_power", True)
        self.declare_parameter("idw_power_candidates", "0.5,0.7,1.0,1.2,1.5,2.0,2.5,3.0,4.0")

        self.lock = threading.Lock()
        self.is_recording = False
        self.aq_buf = []
        self.current_position = (0.0, 0.0)

        self._joint_client = ActionClient(self, FollowJointTrajectory, "/stretch_controller/follow_joint_trajectory")

        self._read_param()

        self.sub_aq = self.create_subscription(AirQuality, "/wacc/air_quality", self._get_aq, 10)
        self.sub_pose = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._pose_cb, 10
        )

    def _read_param(self) -> None:
        yaml_path = self.get_parameter("map_yaml").get_parameter_value().string_value
        if not yaml_path:
            raise ValueError("map_yaml parameter is empty")

        heatmap_type = self.get_parameter("heatmap_type").get_parameter_value().integer_value
        if heatmap_type not in range(4):
            self.get_logger().warn(f"invalid heatmap_type={heatmap_type}, fallback to 3")
            heatmap_type = 3
        self.heatmap_type = heatmap_type

        with open(yaml_path, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)

        pgm_path = self.get_parameter("map_pgm").get_parameter_value().string_value
        if not pgm_path:
            pgm_filename = config.get("image")
            if not pgm_filename:
                raise ValueError("map_pgm empty and image field missing in yaml")
            yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
            pgm_path = os.path.join(yaml_dir, pgm_filename)

        map_img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if map_img is None:
            raise FileNotFoundError(f"failed to read map image: {pgm_path}")

        self.settle_time = self.get_parameter("settle_time").get_parameter_value().double_value
        self.sample_period_sec = self.get_parameter("sample_period_sec").get_parameter_value().double_value
        self.min_samples = self.get_parameter("min_samples").get_parameter_value().integer_value
        self.max_samples = self.get_parameter("max_samples").get_parameter_value().integer_value
        self.max_collect_sec = self.get_parameter("max_collect_sec").get_parameter_value().double_value
        self.stable_window = self.get_parameter("stable_window").get_parameter_value().integer_value
        if self.sample_period_sec <= 0.0:
            self.get_logger().warn(f"invalid sample_period_sec={self.sample_period_sec}, fallback to 1.0")
            self.sample_period_sec = 1.0

        sigma_param = self.get_parameter("metric_sigma_threshold").get_parameter_value().double_value
        ci_param = self.get_parameter("metric_ci_threshold").get_parameter_value().double_value
        slope_param = self.get_parameter("metric_slope_threshold").get_parameter_value().double_value
        self.metric_sigma_threshold = sigma_param if sigma_param > 0 else SIGMA_DEFAULTS[self.heatmap_type]
        self.metric_ci_threshold = ci_param if ci_param > 0 else CI_DEFAULTS[self.heatmap_type]
        self.metric_slope_threshold = slope_param if slope_param > 0 else SLOPE_DEFAULTS[self.heatmap_type]

        self.initial_x = self.get_parameter("initial_x").get_parameter_value().double_value
        self.initial_y = self.get_parameter("initial_y").get_parameter_value().double_value
        self.initial_yaw = self.get_parameter("initial_yaw").get_parameter_value().double_value
        self.current_position = (self.initial_x, self.initial_y)

        self.adaptive_stride = self.get_parameter("adaptive_stride").get_parameter_value().double_value
        self.adaptive_top_k = self.get_parameter("adaptive_top_k").get_parameter_value().integer_value
        self.adaptive_min_distance = self.get_parameter("adaptive_min_distance").get_parameter_value().double_value
        self.adaptive_travel_weight = (
            self.get_parameter("adaptive_travel_weight").get_parameter_value().double_value
        )
        self.adaptive_grad_weight = (
            self.get_parameter("adaptive_grad_weight").get_parameter_value().double_value
        )
        idw_power = self.get_parameter("idw_power").get_parameter_value().double_value
        auto_tune_idw_power = self.get_parameter("auto_tune_idw_power").get_parameter_value().bool_value
        idw_power_candidates = self._parse_float_list(
            self.get_parameter("idw_power_candidates").get_parameter_value().string_value
        )

        self.planner = Planner(self, map_img, config, start_xy=(self.initial_x, self.initial_y))
        self._validate_initial_pose()
        self.heatmap = Heatmap(
            self,
            self.planner,
            self.planner.get_reachable_map(),
            base_map=map_img,
            map_type=self.heatmap_type,
            idw_power=idw_power,
            auto_tune_idw_power=auto_tune_idw_power,
            idw_power_candidates=idw_power_candidates,
        )

    def _validate_initial_pose(self) -> None:
        if not self.planner.is_world_in_bounds(self.initial_x, self.initial_y):
            raise ValueError(
                f"initial pose ({self.initial_x:.3f}, {self.initial_y:.3f}) is outside map bounds"
            )

        if not self.planner.is_world_free(self.initial_x, self.initial_y, use_safe_map=False):
            nearest = self.planner.nearest_free_world(self.initial_x, self.initial_y, use_safe_map=False)
            if nearest is None:
                raise ValueError(
                    f"initial pose ({self.initial_x:.3f}, {self.initial_y:.3f}) lies on an occupied cell"
                )
            raise ValueError(
                "initial pose "
                f"({self.initial_x:.3f}, {self.initial_y:.3f}) lies on an occupied cell; "
                f"nearest free-space suggestion is ({nearest[0]:.3f}, {nearest[1]:.3f})"
            )

        if self.planner.get_navigation_map() is None:
            raise ValueError(
                "failed to build reachable map from initial pose; "
                "check map origin/resolution and initial_x/initial_y"
            )

        if not self.planner.is_world_free(self.initial_x, self.initial_y, use_safe_map=True):
            nearest_safe = self.planner.nearest_free_world(self.initial_x, self.initial_y, use_safe_map=True)
            if nearest_safe is not None:
                self.get_logger().warn(
                    "initial pose is in raw free space but too close to obstacles for the safety margin; "
                    f"reachable planning will use the nearest safe point ({nearest_safe[0]:.3f}, {nearest_safe[1]:.3f})"
                )

    def _parse_float_list(self, raw: str) -> list[float]:
        vals = []
        for token in raw.split(","):
            token = token.strip()
            if not token:
                continue
            try:
                vals.append(float(token))
            except ValueError:
                self.get_logger().warn(f"invalid float token in idw_power_candidates: {token}")
        if not vals:
            return [1.0, 1.5, 2.0, 2.5, 3.0]
        return sorted(set(max(0.5, x) for x in vals))

    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        # Continuously track real robot position from AMCL so that
        # current_position stays valid even after a failed navigation.
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    def _get_aq(self, msg: AirQuality) -> None:
        if msg.state & SENSOR_DHT_FAIL or msg.state & SENSOR_SGP_FAIL:
            return

        with self.lock:
            if self.is_recording:
                self.aq_buf.append((msg.temperature, msg.humidity, msg.tvoc, msg.eco2))

    def move_arm_to_ready(self) -> None:
        if not self._joint_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("joint trajectory server unavailable")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint_wrist_yaw", "wrist_extension", "joint_lift"]

        point = JointTrajectoryPoint()
        point.positions = [pi, 0.0, 0.5]
        point.time_from_start.sec = 4
        goal.trajectory.points = [point]

        self.get_logger().info("moving arm to ready pose")
        future = self._joint_client.send_goal_async(goal)
        while not future.done():
            time.sleep(0.1)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("arm goal rejected")
            return
        result_future = goal_handle.get_result_async()
        arm_timeout = 15.0
        arm_start = time.monotonic()
        while not result_future.done():
            if time.monotonic() - arm_start > arm_timeout:
                self.get_logger().error("arm trajectory timed out after 15 s, continuing anyway")
                break
            time.sleep(0.1)
        self.get_logger().info("arm reached ready pose")

    def get_waypoints(self, stride_m: float | None = None) -> list[tuple[float, float]]:
        return self.planner.get_waypoints(stride_m=stride_m)

    def _trimmed_mean(self, values: list[float], trim_ratio: float = 0.1) -> float:
        if not values:
            return 0.0
        sorted_vals = sorted(values)
        n = len(sorted_vals)
        k = int(n * trim_ratio)
        if 2 * k >= n:
            return sum(sorted_vals) / n
        trimmed = sorted_vals[k : n - k]
        return sum(trimmed) / len(trimmed)

    def _aggregate_samples(self, samples: list[tuple[float, float, float, float]]) -> tuple[float, float, float, float]:
        channels = list(zip(*samples))
        return tuple(self._trimmed_mean(list(ch), trim_ratio=0.1) for ch in channels)

    def _is_stable(self, samples: list[tuple[float, float, float, float]]) -> bool:
        if len(samples) < self.stable_window:
            return False

        metric_index = self.heatmap_type
        window = [row[metric_index] for row in samples[-self.stable_window :]]
        n = len(window)
        mean = sum(window) / n
        if n == 1:
            return True

        variance = sum((x - mean) ** 2 for x in window) / (n - 1)
        std = sqrt(max(variance, 0.0))
        ci_half_width = 1.96 * std / sqrt(n)
        x_mean = (n - 1) / 2.0
        sxx = sum((i - x_mean) ** 2 for i in range(n))
        if sxx <= 0:
            slope = 0.0
        else:
            slope = sum((i - x_mean) * (window[i] - mean) for i in range(n)) / sxx

        conditions = [
            std <= self.metric_sigma_threshold,
            ci_half_width <= self.metric_ci_threshold,
            abs(slope) <= self.metric_slope_threshold,
        ]
        return sum(conditions) >= 2

    def collect_data(self, pos: tuple[float, float]) -> None:
        self.get_logger().info(f"collecting at {pos}")

        # Prevent moving-phase leakage and wait for local airflow to settle.
        with self.lock:
            self.is_recording = False
            self.aq_buf.clear()
        time.sleep(max(0.0, self.settle_time))

        with self.lock:
            self.is_recording = True
            self.aq_buf.clear()

        start_t = time.monotonic()
        while True:
            # Poll at the sensor update cadence so each loop can observe at most one new sample.
            time.sleep(self.sample_period_sec)
            with self.lock:
                snapshot = list(self.aq_buf)

            n = len(snapshot)
            elapsed = time.monotonic() - start_t

            if n >= self.max_samples:
                break
            if elapsed >= self.max_collect_sec:
                break
            if n >= self.min_samples and self._is_stable(snapshot):
                break

        with self.lock:
            self.is_recording = False
            final_samples = list(self.aq_buf)
            self.aq_buf.clear()

        if not final_samples:
            self.get_logger().error("no valid air quality samples collected")
            return

        agg = self._aggregate_samples(final_samples)
        self.get_logger().info(f"collected {len(final_samples)} samples, robust mean={agg}")
        self.heatmap.save_data((pos, agg))

    def get_adaptive_waypoints(self) -> list[tuple[float, float]]:
        self.heatmap.tune_idw_power()
        candidates = self.planner.get_waypoints(stride_m=self.adaptive_stride)
        selected = self.heatmap.suggest_adaptive_waypoints(
            candidates=candidates,
            top_k=self.adaptive_top_k,
            min_distance_m=self.adaptive_min_distance,
            current_pos=self.current_position,
            travel_weight=self.adaptive_travel_weight,
            grad_weight=self.adaptive_grad_weight,
        )
        return selected

    def generate_heatmap(self):
        self.heatmap.tune_idw_power()
        return self.heatmap.render_heatmap()


def _navigate_to(
    navigator: BasicNavigator,
    x: float,
    y: float,
    timeout_sec: float = 300.0,
    from_pos: tuple[float, float] | None = None,
    target_yaw: float | None = None,
) -> TaskResult:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    if target_yaw is not None:
        yaw = target_yaw
    elif from_pos is not None:
        yaw = atan2(y - from_pos[1], x - from_pos[0])
    else:
        yaw = 0.0
    pose.pose.orientation.z = sin(yaw / 2.0)
    pose.pose.orientation.w = cos(yaw / 2.0)

    navigator.goToPose(pose)
    start_time = navigator.get_clock().now()

    while not navigator.isTaskComplete():
        time.sleep(1.0)
        if navigator.get_clock().now() - start_time > Duration(seconds=timeout_sec):
            navigator.get_logger().error("goToPose() timeout, canceling")
            navigator.cancelTask()
            break

    return navigator.getResult()


def main(args=None):
    rclpy.init(args=args)

    try:
        monitor = Monitor()
    except Exception as err:
        print(err)
        rclpy.shutdown()
        return

    executor = MultiThreadedExecutor()
    executor.add_node(monitor)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    monitor.move_arm_to_ready()

    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = monitor.initial_x
    initial_pose.pose.position.y = monitor.initial_y
    initial_pose.pose.orientation.z = sin(monitor.initial_yaw / 2.0)
    initial_pose.pose.orientation.w = cos(monitor.initial_yaw / 2.0)
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    success = True

    coarse_route = monitor.get_waypoints()
    monitor.get_logger().info(f"coarse route points: {len(coarse_route)}")

    for pt in coarse_route:
        result = _navigate_to(navigator, pt[0], pt[1], timeout_sec=300.0, from_pos=monitor.current_position)
        if result == TaskResult.CANCELED:
            success = False
            break
        if result == TaskResult.FAILED:
            continue
        monitor.collect_data(pt)
        monitor.current_position = pt

    if success:
        adaptive_route = monitor.get_adaptive_waypoints()
        monitor.get_logger().info(f"adaptive route points: {len(adaptive_route)}")

        for pt in adaptive_route:
            result = _navigate_to(navigator, pt[0], pt[1], timeout_sec=300.0, from_pos=monitor.current_position)
            if result == TaskResult.CANCELED:
                success = False
                break
            if result == TaskResult.FAILED:
                continue
            monitor.collect_data(pt)
            monitor.current_position = pt

    _navigate_to(
        navigator,
        monitor.initial_x,
        monitor.initial_y,
        timeout_sec=300.0,
        from_pos=monitor.current_position,
        target_yaw=monitor.initial_yaw,
    )

    if success:
        monitor.generate_heatmap()
    elif monitor.heatmap.data:
        monitor.get_logger().warn("monitoring stopped before completion, rendering partial heatmap")
        monitor.generate_heatmap()
    else:
        monitor.get_logger().warn("monitoring stopped before completion, no data collected")

    navigator.lifecycleShutdown()
    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
