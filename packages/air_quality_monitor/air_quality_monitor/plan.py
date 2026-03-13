#! /usr/bin/env python3
import cv2
import numpy as np

ROBOT_RADIUS = 0.2  # meters
MOVING_STRIDE = 1.0  # meters


class Planner:
    def __init__(self, parent, map_data, config, start_xy: tuple[float, float] = (0.0, 0.0)):
        self.parent = parent
        self.resolution = config["resolution"]
        self.origin = config["origin"]
        self.map_height = map_data.shape[0]
        self.map_width = map_data.shape[1]
        self._start_xy = start_xy

        _, binary_map = cv2.threshold(map_data, 250, 255, cv2.THRESH_BINARY)
        self.binary_map = binary_map
        safe_source = self._erode_for_safety(binary_map)
        self.safe_map = self._extract_reachable_region(safe_source, map_name="safe reachable map")

    def _erode_for_safety(self, map_data: np.ndarray) -> np.ndarray:
        margin = max(1, int(np.ceil(ROBOT_RADIUS / self.resolution)))
        kernel_size = margin * 2 + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        return cv2.erode(map_data, kernel, iterations=1)

    def _extract_reachable_region(self, map_data: np.ndarray, map_name: str) -> np.ndarray | None:
        robot_px = self._world_to_pixel(self._start_xy[0], self._start_xy[1])
        seed_px = self._nearest_free_pixel(map_data, robot_px)
        if seed_px is None:
            if self.parent is None:
                print(f"failed to locate {map_name}")
            else:
                self.parent.get_logger().error(f"failed to locate {map_name}")
            debug = cv2.cvtColor(map_data, cv2.COLOR_GRAY2BGR)
            clipped_robot_px = self._clip_pixel(robot_px)
            cv2.circle(debug, clipped_robot_px, 5, (0, 0, 255), -1)
            cv2.imwrite("debug_robot_location.png", debug)
            return None

        if seed_px != self._clip_pixel(robot_px):
            if self.parent is None:
                print(f"start pose snapped from {robot_px} to nearest free pixel {seed_px} on {map_name}")
            else:
                self.parent.get_logger().warn(
                    f"start pose snapped from pixel {robot_px} to nearest free pixel {seed_px} on {map_name}"
                )

        num_labels, labels = cv2.connectedComponents((map_data > 0).astype(np.uint8))
        if num_labels <= 1:
            if self.parent is None:
                print(f"{map_name} has no reachable connected component")
            else:
                self.parent.get_logger().error(f"{map_name} has no reachable connected component")
            return None

        label = int(labels[seed_px[1], seed_px[0]])
        if label == 0:
            if self.parent is None:
                print(f"failed to identify connected component on {map_name}")
            else:
                self.parent.get_logger().error(f"failed to identify connected component on {map_name}")
            return None

        return np.where(labels == label, 255, 0).astype(np.uint8)

    def get_navigation_map(self):
        if self.safe_map is None:
            if self.parent is None:
                print("safe_map unavailable")
            else:
                self.parent.get_logger().error("safe_map unavailable")
            return None
        return self.safe_map

    def get_reachable_map(self):
        return self.get_navigation_map()

    def _world_to_pixel(self, x: float, y: float) -> tuple[int, int]:
        u = int((x - self.origin[0]) / self.resolution)
        v = self.map_height - 1 - int((y - self.origin[1]) / self.resolution)
        return (u, v)

    def _pixel_to_world(self, u: int, v: int) -> tuple[float, float]:
        x = self.origin[0] + u * self.resolution
        y = self.origin[1] + (self.map_height - v - 1) * self.resolution
        return (x, y)

    def _clip_pixel(self, pixel: tuple[int, int]) -> tuple[int, int]:
        u = int(np.clip(pixel[0], 0, self.map_width - 1))
        v = int(np.clip(pixel[1], 0, self.map_height - 1))
        return (u, v)

    def _nearest_free_pixel(self, mask: np.ndarray, pixel: tuple[int, int]) -> tuple[int, int] | None:
        if mask is None or mask.size == 0:
            return None
        clipped = self._clip_pixel(pixel)
        if mask[clipped[1], clipped[0]] > 0:
            return clipped

        ys, xs = np.where(mask > 0)
        if xs.size == 0:
            return None

        idx = int(np.argmin((xs - clipped[0]) ** 2 + (ys - clipped[1]) ** 2))
        return (int(xs[idx]), int(ys[idx]))

    def is_world_in_bounds(self, x: float, y: float) -> bool:
        u, v = self._world_to_pixel(x, y)
        return 0 <= u < self.map_width and 0 <= v < self.map_height

    def is_world_free(self, x: float, y: float, use_safe_map: bool = False) -> bool:
        if not self.is_world_in_bounds(x, y):
            return False
        u, v = self._world_to_pixel(x, y)
        source = self.safe_map if use_safe_map else self.binary_map
        if source is None:
            return False
        return bool(source[v, u] > 0)

    def nearest_free_world(self, x: float, y: float, use_safe_map: bool = False) -> tuple[float, float] | None:
        source = self.safe_map if use_safe_map else self.binary_map
        if source is None:
            return None
        pixel = self._world_to_pixel(x, y)
        nearest = self._nearest_free_pixel(source, pixel)
        if nearest is None:
            return None
        return self._pixel_to_world(nearest[0], nearest[1])

    def get_waypoints(self, stride_m: float | None = None) -> list[tuple[float, float]]:
        if self.safe_map is None:
            if self.parent is None:
                print("safe_map unavailable")
            else:
                self.parent.get_logger().error("safe_map unavailable")
            return []

        if stride_m is None:
            stride_m = MOVING_STRIDE
        stride = max(1, int(stride_m / self.resolution))

        waypoints_px = []
        for y in range(stride // 2, self.map_height, stride):
            row_points = []
            for x in range(stride // 2, self.map_width, stride):
                if self.safe_map[y, x] == 255:
                    row_points.append((x, y))

            if (y // stride) % 2 == 1:
                row_points.reverse()
            waypoints_px.extend(row_points)

        return [self._pixel_to_world(u, v) for (u, v) in waypoints_px]
