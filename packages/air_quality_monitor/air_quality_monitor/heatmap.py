#! /usr/bin/env python3
from math import sqrt

import cv2
import numpy as np

from .plan import ROBOT_RADIUS


class Heatmap:
    def __init__(
        self,
        parent,
        planner,
        map_data,
        base_map=None,
        map_type=3,
        idw_power=2.0,
        auto_tune_idw_power=False,
        idw_power_candidates=None,
    ):
        self.parent = parent
        self.planner = planner
        self.map = map_data
        self.base_map = map_data if base_map is None else base_map
        self.map_type = map_type
        self.idw_power = max(0.5, float(idw_power))
        self.auto_tune_idw_power = bool(auto_tune_idw_power)
        self.idw_power_candidates = (
            [float(x) for x in idw_power_candidates] if idw_power_candidates else [1.0, 1.5, 2.0, 2.5, 3.0]
        )
        self._last_tuned_sample_count = 0

        # data format: [((x, y), (temp, humid, tvoc, eco2)), ...]
        self.data = []

    def save_data(self, val: tuple[tuple[float, float], tuple[float, ...]]) -> None:
        self.data.append(val)

    def _valid_samples(self) -> list[tuple[tuple[float, float], float]]:
        valid = []
        for pos, sensor_values in self.data:
            if self.map_type >= len(sensor_values):
                self.parent.get_logger().warn(
                    f"skip sample: map_type={self.map_type}, tuple_len={len(sensor_values)}"
                )
                continue
            valid.append((pos, float(sensor_values[self.map_type])))
        return valid

    def _free_mask(self) -> np.ndarray | None:
        if self.map is None:
            return None
        return self.map > 250

    def _world_points_to_pixels(self, points: list[tuple[float, float]]) -> np.ndarray:
        return np.array(
            [self.planner._world_to_pixel(pos[0], pos[1]) for pos in points],
            dtype=np.float32,
        )

    def _predict_from_samples(
        self,
        target_u: int,
        target_v: int,
        sample_px: np.ndarray,
        sample_vals: np.ndarray,
        power: float | None = None,
    ) -> float:
        if power is None:
            power = self.idw_power
        du = sample_px[:, 0] - float(target_u)
        dv = sample_px[:, 1] - float(target_v)
        dists2 = du * du + dv * dv + 1e-6
        weights = 1.0 / np.power(dists2, power / 2.0)
        return float(np.sum(weights * sample_vals) / (np.sum(weights) + 1e-12))

    def tune_idw_power(self, min_samples: int = 8) -> float:
        if not self.auto_tune_idw_power:
            return self.idw_power

        samples = self._valid_samples()
        n = len(samples)
        if n < min_samples:
            return self.idw_power
        if n == self._last_tuned_sample_count:
            return self.idw_power

        sample_px = self._world_points_to_pixels([pos for pos, _ in samples])
        sample_vals = np.array([v for _, v in samples], dtype=np.float32)

        best_power = self.idw_power
        best_mae = float("inf")

        for power in self.idw_power_candidates:
            abs_errors = []
            for i in range(n):
                keep = [j for j in range(n) if j != i]
                train_px = sample_px[keep]
                train_vals = sample_vals[keep]
                target_u, target_v = int(sample_px[i, 0]), int(sample_px[i, 1])
                pred = self._predict_from_samples(target_u, target_v, train_px, train_vals, power)
                abs_errors.append(abs(pred - float(sample_vals[i])))

            mae = float(np.mean(abs_errors)) if abs_errors else float("inf")
            if mae < best_mae:
                best_mae = mae
                best_power = float(power)

        self.idw_power = best_power
        self._last_tuned_sample_count = n
        self.parent.get_logger().info(f"auto tuned idw_power={self.idw_power:.3f}, loo_mae={best_mae:.4f}")
        return self.idw_power

    def _build_idw_scalar_map(
        self,
        samples: list[tuple[tuple[float, float], float]],
    ) -> tuple[np.ndarray, np.ndarray] | tuple[None, None]:
        if self.map is None:
            self.parent.get_logger().error("reachable map is None")
            return None, None
        if not samples:
            return None, None

        map_h, map_w = self.map.shape[:2]
        mask = (self.map > 250).astype(np.uint8) * 255
        ys, xs = np.where(mask > 0)

        if xs.size == 0:
            self.parent.get_logger().error("reachable map has no free-space pixels")
            return None, None

        sample_px = self._world_points_to_pixels([pos for pos, _ in samples])
        sample_vals = np.array([v for _, v in samples], dtype=np.float32)

        scalar = np.full((map_h, map_w), np.nan, dtype=np.float32)
        chunk_size = 50000
        for start in range(0, xs.size, chunk_size):
            end = min(start + chunk_size, xs.size)
            chunk_xs = xs[start:end].astype(np.float32)
            chunk_ys = ys[start:end].astype(np.float32)
            du = sample_px[:, 0:1] - chunk_xs[None, :]
            dv = sample_px[:, 1:2] - chunk_ys[None, :]
            dists2 = du * du + dv * dv + 1e-6
            weights = 1.0 / np.power(dists2, self.idw_power / 2.0)
            numer = (weights * sample_vals[:, None]).sum(axis=0)
            denom = weights.sum(axis=0) + 1e-12
            scalar[ys[start:end], xs[start:end]] = numer / denom

        return scalar, mask

    def suggest_adaptive_waypoints(
        self,
        candidates: list[tuple[float, float]],
        top_k: int = 10,
        min_distance_m: float = 0.8,
        current_pos: tuple[float, float] | None = None,
        travel_weight: float = 0.25,
        grad_weight: float = 0.6,
    ) -> list[tuple[float, float]]:
        if top_k <= 0 or not candidates:
            return []

        samples = self._valid_samples()
        if len(samples) < 3:
            return []

        sample_px = self._world_points_to_pixels([pos for pos, _ in samples])
        sample_vals = np.array([v for _, v in samples], dtype=np.float32)

        cand_arr = np.array(candidates, dtype=np.float32)
        cand_px = self._world_points_to_pixels([(float(c[0]), float(c[1])) for c in cand_arr])
        free_mask = self._free_mask()
        if free_mask is None:
            return []

        scores = []
        delta = max(min_distance_m * 0.5, self.planner.resolution * 2.0)
        delta_px = max(1, int(round(delta / self.planner.resolution)))
        if current_pos is None:
            current_pos = (0.0, 0.0)
        travel_weight = float(np.clip(travel_weight, 0.0, 0.8))
        h, w = free_mask.shape

        for idx, cand in enumerate(cand_arr):
            cu, cv = int(cand_px[idx, 0]), int(cand_px[idx, 1])
            if not (0 <= cu < w and 0 <= cv < h and free_mask[cv, cu]):
                continue

            dists_px = np.sqrt((sample_px[:, 0] - float(cu)) ** 2 + (sample_px[:, 1] - float(cv)) ** 2)
            nearest = float(np.min(dists_px)) * self.planner.resolution
            if nearest < min_distance_m:
                continue

            center = self._predict_from_samples(cu, cv, sample_px, sample_vals)

            neighbors_px = [
                (cu + delta_px, cv),
                (cu - delta_px, cv),
                (cu, cv + delta_px),
                (cu, cv - delta_px),
            ]
            neighbor_diffs = []
            for nu, nv in neighbors_px:
                if 0 <= nu < w and 0 <= nv < h and free_mask[nv, nu]:
                    neighbor_diffs.append(abs(center - self._predict_from_samples(nu, nv, sample_px, sample_vals)))
            grad = max(neighbor_diffs) if neighbor_diffs else 0.0

            travel_dist = float(
                np.sqrt((float(cand[0]) - current_pos[0]) ** 2 + (float(cand[1]) - current_pos[1]) ** 2)
            )
            scores.append((idx, nearest, grad, travel_dist))

        if not scores:
            return []

        nearest_vals = np.array([s[1] for s in scores], dtype=np.float32)
        grad_vals = np.array([s[2] for s in scores], dtype=np.float32)
        travel_vals = np.array([s[3] for s in scores], dtype=np.float32)

        n_min, n_max = float(np.min(nearest_vals)), float(np.max(nearest_vals))
        g_min, g_max = float(np.min(grad_vals)), float(np.max(grad_vals))
        t_min, t_max = float(np.min(travel_vals)), float(np.max(travel_vals))

        grad_weight = float(np.clip(grad_weight, 0.0, 1.0))
        ranked = []
        for idx, nearest, grad, travel_dist in scores:
            nearest_norm = 0.0 if n_max == n_min else (nearest - n_min) / (n_max - n_min)
            grad_norm = 0.0 if g_max == g_min else (grad - g_min) / (g_max - g_min)
            travel_norm = 0.0 if t_max == t_min else (travel_dist - t_min) / (t_max - t_min)
            info_score = grad_weight * grad_norm + (1.0 - grad_weight) * nearest_norm
            score = (1.0 - travel_weight) * info_score - travel_weight * travel_norm
            ranked.append((score, idx))

        ranked.sort(reverse=True)

        selected = []
        for _, idx in ranked:
            point = (float(cand_arr[idx][0]), float(cand_arr[idx][1]))
            if any((point[0] - p[0]) ** 2 + (point[1] - p[1]) ** 2 < min_distance_m**2 for p in selected):
                continue
            selected.append(point)
            if len(selected) >= top_k:
                break

        route = self._order_points_by_path(selected, current_pos)
        return self._two_opt_improve(route, current_pos)

    def _order_points_by_path(
        self,
        points: list[tuple[float, float]],
        start_pos: tuple[float, float],
    ) -> list[tuple[float, float]]:
        if len(points) < 2:
            return points

        remaining = list(points)
        ordered = []
        current = (float(start_pos[0]), float(start_pos[1]))

        while remaining:
            next_idx = min(
                range(len(remaining)),
                key=lambda i: (remaining[i][0] - current[0]) ** 2 + (remaining[i][1] - current[1]) ** 2,
            )
            nxt = remaining.pop(next_idx)
            ordered.append(nxt)
            current = nxt

        return ordered

    def _two_opt_improve(
        self,
        points: list[tuple[float, float]],
        start_pos: tuple[float, float],
    ) -> list[tuple[float, float]]:
        """2-opt local search on an open path starting from start_pos."""
        if len(points) < 4:
            return points

        def _d(a: tuple, b: tuple) -> float:
            return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

        full = [start_pos] + list(points)
        n = len(full)
        improved = True
        while improved:
            improved = False
            for i in range(n - 2):
                for j in range(i + 2, n - 1):
                    before = _d(full[i], full[i + 1]) + _d(full[j], full[j + 1])
                    after = _d(full[i], full[j]) + _d(full[i + 1], full[j + 1])
                    if after < before - 1e-8:
                        full[i + 1 : j + 1] = full[i + 1 : j + 1][::-1]
                        improved = True
        return full[1:]

    def _display_kernel(self) -> np.ndarray:
        margin = max(1, int(np.ceil(ROBOT_RADIUS / self.planner.resolution)))
        kernel_size = margin * 2 + 1
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

    def _display_mask_and_base(self) -> tuple[np.ndarray, np.ndarray] | tuple[None, None]:
        if self.map is None:
            return None, None

        display_mask = (self.map > 250).astype(np.uint8) * 255
        display_mask = cv2.dilate(display_mask, self._display_kernel(), iterations=1)

        if self.base_map is not None:
            raw_free_mask = (self.base_map > 250).astype(np.uint8) * 255
            display_mask = cv2.bitwise_and(display_mask, raw_free_mask)
            display_base = np.zeros_like(self.base_map)
            display_base[display_mask > 0] = self.base_map[display_mask > 0]
        else:
            display_base = display_mask.copy()

        return display_mask, display_base

    def render_heatmap(self) -> None:
        if not self.data:
            self.parent.get_logger().error("data is empty, heatmap rendering failed")
            return

        samples = self._valid_samples()
        if not samples:
            self.parent.get_logger().error("no valid samples available")
            return

        scalar_map, mask = self._build_idw_scalar_map(samples)
        if scalar_map is None or mask is None:
            return

        valid_values = scalar_map[mask > 0]
        if valid_values.size == 0:
            self.parent.get_logger().error("IDW produced no valid pixels")
            return

        low = float(np.percentile(valid_values, 5.0))
        high = float(np.percentile(valid_values, 95.0))
        if high <= low:
            low = float(np.min(valid_values))
            high = float(np.max(valid_values))

        normalized = np.zeros_like(scalar_map, dtype=np.uint8)
        if high > low:
            clipped = np.clip((scalar_map - low) / (high - low), 0.0, 1.0)
            normalized[mask > 0] = (clipped[mask > 0] * 255).astype(np.uint8)

        display_mask, display_base = self._display_mask_and_base()
        if display_mask is None or display_base is None:
            self.parent.get_logger().error("display base generation failed")
            return

        dilated_normalized = cv2.dilate(normalized, self._display_kernel(), iterations=1)
        dilated_normalized = cv2.bitwise_and(dilated_normalized, display_mask)

        heat_layer = cv2.applyColorMap(dilated_normalized, cv2.COLORMAP_JET)
        heat_layer = cv2.bitwise_and(heat_layer, heat_layer, mask=display_mask)
        map_bgr = cv2.cvtColor(display_base, cv2.COLOR_GRAY2BGR)
        result = cv2.addWeighted(map_bgr, 0.6, heat_layer, 0.4, 0)

        cv2.imwrite("heatmap_result.png", result)
        self.parent.get_logger().info("heatmap saved to heatmap_result.png")
