#!/usr/bin/env python3
"""Windows OpenCV prototype for the concentric-circle and cross UAV target.

The detector core is deliberately independent from camera capture so it can be
wrapped by a ROS2 node later without changing the image-processing code.
"""

import argparse
import copy
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np


MODE_HIGH = "HIGH_TARGET"
MODE_TRANSITION = "TRANSITION"
MODE_LOW = "LOW_CROSS"
MODES = (MODE_HIGH, MODE_TRANSITION, MODE_LOW)
MODE_VALUE_TO_NAME = {
    0: "OFF",
    1: MODE_HIGH,
    2: MODE_TRANSITION,
    3: MODE_LOW,
}


def local_fine_data_from_dx_dy(dx: int, dy: int) -> Tuple[int, int]:
    """Map image right/down error to the flight controller's forward/left axes."""
    return -int(dy), -int(dx)


def select_flight_vision_stage(
    visual_descent_active: bool,
    height_cm: Optional[float],
    low_height_threshold_cm: float,
) -> int:
    """Select target detection stage from existing flight-state signals."""
    if not visual_descent_active:
        return 1
    if height_cm is None or height_cm >= low_height_threshold_cm:
        return 2
    return 3


def embedded_config() -> Dict[str, Any]:
    """Self-contained defaults used by the Orange Pi ROS2 entry point."""
    return {
        "camera": {
            "index": 0,
            "width": 640,
            "height": 480,
            "fps": 30,
            "backend": "v4l2",
            "fourcc": "MJPG",
        },
        "preprocess": {
            "clahe_clip_limit": 2.0,
            "clahe_grid_size": 8,
            "blur_kernel": 5,
            "adaptive_block_size": 41,
            "adaptive_c": 7.0,
            "morph_kernel": 3,
            "canny_low": 55,
            "canny_high": 150,
        },
        "circle": {
            "min_radius_ratio": 0.045,
            "max_radius_ratio": 0.46,
            "expected_diameter_ratio": 0.60,
            "diameter_ratio_tolerance": 0.18,
            "max_concentricity_ratio": 0.14,
            "enable_hough": 0,
            "hough_dp": 1.2,
            "hough_min_distance_ratio": 0.08,
            "hough_param1": 120.0,
            "hough_param2": 26.0,
            "min_ellipse_axis_ratio": 0.62,
            "min_contour_points": 24,
            "min_contour_circularity": 0.42,
            "min_edge_support": 0.30,
            "min_confidence": 0.70,
            "enable_cross_guided_recovery": 1,
            "cross_guided_min_visible_fraction": 0.82,
            "outer_edge_search_ratio": 0.20,
            "outer_edge_relative_support": 0.65,
            "require_cross_validation": 1,
            "cross_validation_distance_outer_radius_ratio": 0.20,
            "cross_validation_min_distance_px": 14.0,
        },
        "cross": {
            "hough_threshold": 34,
            "min_line_length_ratio": 0.09,
            "max_line_gap": 18,
            "max_segments": 80,
            "parallel_tolerance_deg": 9.0,
            "orthogonal_tolerance_deg": 14.0,
            "min_stroke_width_px": 4.0,
            "max_stroke_width_ratio": 0.28,
            "max_centerlines": 36,
            "intersection_margin_px": 30.0,
            "intersection_cluster_px": 22.0,
            "ray_length_ratio": 0.08,
            "ray_band_px": 3,
            "min_ray_support": 0.58,
            "min_confidence": 0.52,
        },
        "transition": {
            "inner_min_radius_ratio": 0.10,
            "inner_edge_separation_expected_ratio": 0.125,
            "inner_edge_separation_min_ratio": 0.06,
            "inner_edge_separation_max_ratio": 0.22,
        },
        "filter": {
            "ema_alpha": 0.38,
            "radius_ema_alpha": 0.40,
        },
        "tracking": {
            "acquire_frames": 3,
            "max_missed_frames": 3,
            "max_center_jump_outer_radius_ratio": 0.30,
            "max_center_jump_px": 45.0,
            "max_radius_change_ratio": 0.22,
            "pending_center_tolerance_outer_radius_ratio": 0.22,
            "pending_radius_tolerance_ratio": 0.20,
        },
        "dashboard": {"pane_width": 480, "pane_height": 360},
        "logging": {"interval_seconds": 1.0},
    }


def _file_node_to_value(node: cv2.FileNode) -> Any:
    if node.empty() or node.isNone():
        return None
    if node.isMap():
        return {key: _file_node_to_value(node.getNode(key)) for key in node.keys()}
    if node.isSeq():
        return [_file_node_to_value(node.at(i)) for i in range(node.size())]
    if node.isString():
        return node.string()
    if node.isInt():
        return int(node.real())
    if node.isReal():
        return float(node.real())
    raise ValueError("Unsupported value in OpenCV YAML node: {!r}".format(node.name()))


def config_value(config: Dict[str, Any], dotted_path: str) -> Any:
    value: Any = config
    for part in dotted_path.split("."):
        if not isinstance(value, dict) or part not in value:
            raise KeyError("Missing configuration key: {}".format(dotted_path))
        value = value[part]
    return value


def set_config_value(
    config: Dict[str, Any], dotted_path: str, new_value: Any
) -> None:
    target: Any = config
    parts = dotted_path.split(".")
    for part in parts[:-1]:
        if not isinstance(target, dict) or part not in target:
            raise KeyError("Missing configuration key: {}".format(dotted_path))
        target = target[part]
    if not isinstance(target, dict) or parts[-1] not in target:
        raise KeyError("Missing configuration key: {}".format(dotted_path))
    target[parts[-1]] = new_value


def validate_config(config: Dict[str, Any]) -> None:
    required = (
        "camera.index",
        "camera.width",
        "camera.height",
        "camera.fps",
        "preprocess.clahe_clip_limit",
        "preprocess.clahe_grid_size",
        "preprocess.blur_kernel",
        "preprocess.adaptive_block_size",
        "preprocess.adaptive_c",
        "preprocess.morph_kernel",
        "preprocess.canny_low",
        "preprocess.canny_high",
        "circle.min_radius_ratio",
        "circle.max_radius_ratio",
        "circle.expected_diameter_ratio",
        "circle.diameter_ratio_tolerance",
        "circle.max_concentricity_ratio",
        "circle.enable_hough",
        "circle.hough_dp",
        "circle.hough_min_distance_ratio",
        "circle.hough_param1",
        "circle.hough_param2",
        "circle.min_ellipse_axis_ratio",
        "circle.min_contour_points",
        "circle.min_contour_circularity",
        "circle.min_edge_support",
        "circle.min_confidence",
        "circle.enable_cross_guided_recovery",
        "circle.cross_guided_min_visible_fraction",
        "circle.outer_edge_search_ratio",
        "circle.outer_edge_relative_support",
        "circle.require_cross_validation",
        "circle.cross_validation_distance_outer_radius_ratio",
        "circle.cross_validation_min_distance_px",
        "cross.hough_threshold",
        "cross.min_line_length_ratio",
        "cross.max_line_gap",
        "cross.max_segments",
        "cross.parallel_tolerance_deg",
        "cross.orthogonal_tolerance_deg",
        "cross.min_stroke_width_px",
        "cross.max_stroke_width_ratio",
        "cross.max_centerlines",
        "cross.intersection_margin_px",
        "cross.intersection_cluster_px",
        "cross.ray_length_ratio",
        "cross.ray_band_px",
        "cross.min_ray_support",
        "cross.min_confidence",
        "transition.inner_min_radius_ratio",
        "transition.inner_edge_separation_expected_ratio",
        "transition.inner_edge_separation_min_ratio",
        "transition.inner_edge_separation_max_ratio",
        "filter.ema_alpha",
        "filter.radius_ema_alpha",
        "tracking.acquire_frames",
        "tracking.max_missed_frames",
        "tracking.max_center_jump_outer_radius_ratio",
        "tracking.max_center_jump_px",
        "tracking.max_radius_change_ratio",
        "tracking.pending_center_tolerance_outer_radius_ratio",
        "tracking.pending_radius_tolerance_ratio",
        "dashboard.pane_width",
        "dashboard.pane_height",
        "logging.interval_seconds",
    )
    for key in required:
        config_value(config, key)

    for key in (
        "preprocess.blur_kernel",
        "preprocess.adaptive_block_size",
        "preprocess.morph_kernel",
    ):
        kernel = int(config_value(config, key))
        if kernel < 1 or kernel % 2 == 0:
            raise ValueError("{} must be a positive odd integer".format(key))

    alpha = float(config_value(config, "filter.ema_alpha"))
    if not 0.0 < alpha <= 1.0:
        raise ValueError("filter.ema_alpha must be in (0, 1]")
    radius_alpha = float(config_value(config, "filter.radius_ema_alpha"))
    if not 0.0 < radius_alpha <= 1.0:
        raise ValueError("filter.radius_ema_alpha must be in (0, 1]")
    minimum_visible = float(
        config_value(
            config, "circle.cross_guided_min_visible_fraction"
        )
    )
    if not 0.0 < minimum_visible <= 1.0:
        raise ValueError(
            "circle.cross_guided_min_visible_fraction must be in (0, 1]"
        )
    for key in (
        "circle.outer_edge_search_ratio",
        "circle.outer_edge_relative_support",
        "transition.inner_min_radius_ratio",
        "transition.inner_edge_separation_expected_ratio",
        "transition.inner_edge_separation_min_ratio",
        "transition.inner_edge_separation_max_ratio",
    ):
        value = float(config_value(config, key))
        if not 0.0 < value <= 1.0:
            raise ValueError("{} must be in (0, 1]".format(key))
    if float(
        config_value(
            config, "transition.inner_edge_separation_min_ratio"
        )
    ) >= float(
        config_value(
            config, "transition.inner_edge_separation_max_ratio"
        )
    ):
        raise ValueError(
            "transition inner edge separation min must be below max"
        )


def load_config(path: Path) -> Dict[str, Any]:
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise RuntimeError("Cannot open configuration file: {}".format(path))
    try:
        config = _file_node_to_value(storage.root())
    finally:
        storage.release()

    if not isinstance(config, dict):
        raise ValueError("Configuration root must be a YAML mapping")
    validate_config(config)
    return config


@dataclass
class CircleCandidate:
    center: Tuple[float, float]
    radius: float
    score: float
    source: str


@dataclass
class Detection:
    mode: str
    valid: bool = False
    center: Optional[Tuple[float, float]] = None
    confidence: float = 0.0
    source: str = "none"
    reason: str = "not_processed"
    raw_center: Optional[Tuple[float, float]] = None
    held_center: Optional[Tuple[float, float]] = None
    circles: List[Tuple[Tuple[float, float], float]] = field(default_factory=list)
    segments: List[Tuple[int, int, int, int]] = field(default_factory=list)
    selected_angles: List[float] = field(default_factory=list)
    selected_extents: List[Tuple[float, float]] = field(default_factory=list)
    gray: Optional[np.ndarray] = None
    binary: Optional[np.ndarray] = None
    edges: Optional[np.ndarray] = None
    track_state: str = "SEARCHING"

    def dx_dy(self, frame_shape: Sequence[int]) -> Tuple[Optional[int], Optional[int]]:
        if not self.valid or self.center is None:
            return None, None
        height, width = frame_shape[:2]
        dx = int(round(self.center[0] - width / 2.0))
        dy = int(round(self.center[1] - height / 2.0))
        return dx, dy


class TargetDetector:
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.last_mode: Optional[str] = None
        self.track_center: Optional[np.ndarray] = None
        self.track_radii: Optional[np.ndarray] = None
        self.track_misses = 0
        self.pending_center: Optional[np.ndarray] = None
        self.pending_radii: Optional[np.ndarray] = None
        self.pending_count = 0
        self.low_search_anchor: Optional[np.ndarray] = None

    def reset(self) -> None:
        self.last_mode = None
        self.low_search_anchor = None
        self._clear_track()

    def _clear_track(self) -> None:
        self.track_center = None
        self.track_radii = None
        self.track_misses = 0
        self.pending_center = None
        self.pending_radii = None
        self.pending_count = 0

    def preprocess(self, frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        grid_size = int(config_value(self.config, "preprocess.clahe_grid_size"))
        clahe = cv2.createCLAHE(
            clipLimit=float(config_value(self.config, "preprocess.clahe_clip_limit")),
            tileGridSize=(grid_size, grid_size),
        )
        equalized = clahe.apply(gray)
        blur_kernel = int(config_value(self.config, "preprocess.blur_kernel"))
        blurred = cv2.GaussianBlur(equalized, (blur_kernel, blur_kernel), 0)

        binary = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            int(config_value(self.config, "preprocess.adaptive_block_size")),
            float(config_value(self.config, "preprocess.adaptive_c")),
        )
        morph_kernel = int(config_value(self.config, "preprocess.morph_kernel"))
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (morph_kernel, morph_kernel)
        )
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        edges = cv2.Canny(
            blurred,
            int(config_value(self.config, "preprocess.canny_low")),
            int(config_value(self.config, "preprocess.canny_high")),
        )
        return blurred, binary, edges

    @staticmethod
    def _circle_edge_support(
        edge_support_map: np.ndarray,
        center: Tuple[float, float],
        radius: float,
    ) -> float:
        height, width = edge_support_map.shape[:2]
        angles = np.linspace(0.0, 2.0 * math.pi, 180, endpoint=False)
        x = np.rint(center[0] + radius * np.cos(angles)).astype(np.int32)
        y = np.rint(center[1] + radius * np.sin(angles)).astype(np.int32)
        valid = (x >= 0) & (x < width) & (y >= 0) & (y < height)
        if not np.any(valid):
            return 0.0
        return float(np.mean(edge_support_map[y[valid], x[valid]] > 0))

    @staticmethod
    def _cluster_circle_candidates(
        candidates: List[CircleCandidate],
    ) -> List[CircleCandidate]:
        clustered: List[CircleCandidate] = []
        for candidate in sorted(candidates, key=lambda item: item.score, reverse=True):
            duplicate = False
            for existing in clustered:
                distance = math.hypot(
                    candidate.center[0] - existing.center[0],
                    candidate.center[1] - existing.center[1],
                )
                if (
                    distance <= max(4.0, existing.radius * 0.06)
                    and abs(candidate.radius - existing.radius)
                    <= max(4.0, existing.radius * 0.08)
                ):
                    duplicate = True
                    break
            if not duplicate:
                clustered.append(candidate)
        return clustered

    def _circle_candidates(
        self, gray: np.ndarray, edges: np.ndarray
    ) -> List[CircleCandidate]:
        height, width = gray.shape[:2]
        minimum_dimension = min(height, width)
        min_radius = max(
            5,
            int(
                minimum_dimension
                * float(config_value(self.config, "circle.min_radius_ratio"))
            ),
        )
        max_radius = max(
            min_radius + 1,
            int(
                minimum_dimension
                * float(config_value(self.config, "circle.max_radius_ratio"))
            ),
        )
        candidates: List[CircleCandidate] = []
        edge_support_map = cv2.dilate(
            edges, np.ones((3, 3), dtype=np.uint8), iterations=1
        )

        if int(config_value(self.config, "circle.enable_hough")):
            hough = cv2.HoughCircles(
                gray,
                cv2.HOUGH_GRADIENT,
                dp=float(config_value(self.config, "circle.hough_dp")),
                minDist=max(
                    1.0,
                    minimum_dimension
                    * float(
                        config_value(
                            self.config, "circle.hough_min_distance_ratio"
                        )
                    ),
                ),
                param1=float(config_value(self.config, "circle.hough_param1")),
                param2=float(config_value(self.config, "circle.hough_param2")),
                minRadius=min_radius,
                maxRadius=max_radius,
            )
            if hough is not None:
                for x, y, radius in hough[0]:
                    center = (float(x), float(y))
                    support = self._circle_edge_support(
                        edge_support_map, center, float(radius)
                    )
                    candidates.append(
                        CircleCandidate(center, float(radius), support, "hough")
                    )

        contours, _ = cv2.findContours(
            edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE
        )
        min_points = int(config_value(self.config, "circle.min_contour_points"))
        min_axis_ratio = float(
            config_value(self.config, "circle.min_ellipse_axis_ratio")
        )
        min_circularity = float(
            config_value(self.config, "circle.min_contour_circularity")
        )
        for contour in contours:
            if len(contour) < max(5, min_points):
                continue
            perimeter = cv2.arcLength(contour, True)
            area = abs(cv2.contourArea(contour))
            if perimeter <= 0.0 or area <= 0.0:
                continue
            circularity = min(1.0, 4.0 * math.pi * area / (perimeter * perimeter))
            if circularity < min_circularity:
                continue
            ellipse = cv2.fitEllipse(contour)
            (x, y), (diameter_a, diameter_b), _ = ellipse
            major = max(diameter_a, diameter_b)
            minor = min(diameter_a, diameter_b)
            if major <= 0.0 or minor / major < min_axis_ratio:
                continue
            radius = (major + minor) / 4.0
            if not min_radius <= radius <= max_radius:
                continue
            support = self._circle_edge_support(
                edge_support_map, (x, y), radius
            )
            score = 0.55 * support + 0.25 * (minor / major) + 0.20 * circularity
            candidates.append(
                CircleCandidate((float(x), float(y)), radius, score, "ellipse")
            )

        candidates = self._cluster_circle_candidates(candidates)

        # HoughCircles suppresses circles whose centers are closer than minDist,
        # which is exactly the case for concentric rings. Probe the expected
        # 30/50 radius relationship around the strongest detected circles to
        # recover the suppressed partner without using an expensive minDist=1
        # Hough search.
        expected_ratio = float(
            config_value(self.config, "circle.expected_diameter_ratio")
        )
        ratio_tolerance = float(
            config_value(self.config, "circle.diameter_ratio_tolerance")
        )
        minimum_support = float(
            config_value(self.config, "circle.min_edge_support")
        )
        ratio_start = max(0.1, expected_ratio - ratio_tolerance)
        ratio_end = min(0.95, expected_ratio + ratio_tolerance)
        ratios = np.linspace(ratio_start, ratio_end, 13)
        probes: List[CircleCandidate] = []
        for base in candidates[:16]:
            best_inner: Optional[CircleCandidate] = None
            best_outer: Optional[CircleCandidate] = None
            for ratio in ratios:
                inner_radius = base.radius * float(ratio)
                if inner_radius >= min_radius:
                    support = self._circle_edge_support(
                        edge_support_map, base.center, inner_radius
                    )
                    if best_inner is None or support > best_inner.score:
                        best_inner = CircleCandidate(
                            base.center, inner_radius, support, "radial_probe"
                        )
                outer_radius = base.radius / float(ratio)
                if outer_radius <= max_radius:
                    support = self._circle_edge_support(
                        edge_support_map, base.center, outer_radius
                    )
                    if best_outer is None or support > best_outer.score:
                        best_outer = CircleCandidate(
                            base.center, outer_radius, support, "radial_probe"
                        )
            for probe in (best_inner, best_outer):
                if probe is not None and probe.score >= minimum_support:
                    probes.append(probe)

        return self._cluster_circle_candidates(candidates + probes)

    def _detect_circles(
        self, gray: np.ndarray, binary: np.ndarray, edges: np.ndarray
    ) -> Detection:
        result = Detection(
            mode=MODE_HIGH, gray=gray, binary=binary, edges=edges
        )
        candidates = self._circle_candidates(gray, edges)
        min_support = float(config_value(self.config, "circle.min_edge_support"))
        candidates = [item for item in candidates if item.score >= min_support]
        if len(candidates) < 2:
            result.reason = "fewer_than_two_circle_candidates"
            return result

        expected_ratio = float(
            config_value(self.config, "circle.expected_diameter_ratio")
        )
        ratio_tolerance = float(
            config_value(self.config, "circle.diameter_ratio_tolerance")
        )
        max_concentricity = float(
            config_value(self.config, "circle.max_concentricity_ratio")
        )

        best_pair: Optional[Tuple[CircleCandidate, CircleCandidate]] = None
        best_score = -1.0
        for first in candidates:
            for second in candidates:
                if first is second or first.radius <= second.radius:
                    continue
                outer, inner = first, second
                radius_ratio = inner.radius / outer.radius
                ratio_error = abs(radius_ratio - expected_ratio)
                if ratio_error > ratio_tolerance:
                    continue
                center_distance = math.hypot(
                    outer.center[0] - inner.center[0],
                    outer.center[1] - inner.center[1],
                )
                concentricity = center_distance / max(outer.radius, 1.0)
                if concentricity > max_concentricity:
                    continue
                ratio_score = 1.0 - ratio_error / max(ratio_tolerance, 1e-6)
                center_score = 1.0 - concentricity / max(max_concentricity, 1e-6)
                evidence_score = min(1.0, (outer.score + inner.score) / 2.0)
                pair_score = (
                    0.38 * ratio_score
                    + 0.38 * center_score
                    + 0.24 * evidence_score
                )
                if pair_score > best_score:
                    best_score = pair_score
                    best_pair = (outer, inner)

        min_confidence = float(config_value(self.config, "circle.min_confidence"))
        if best_pair is None:
            result.reason = "no_concentric_pair_with_expected_ratio"
            return result
        if best_score < min_confidence:
            result.reason = "circle_confidence_below_threshold"
            result.confidence = max(0.0, best_score)
            return result

        outer, inner = best_pair
        weight_sum = max(outer.score + inner.score, 1e-6)
        center = (
            (outer.center[0] * outer.score + inner.center[0] * inner.score)
            / weight_sum,
            (outer.center[1] * outer.score + inner.center[1] * inner.score)
            / weight_sum,
        )
        result.valid = True
        result.center = center
        result.raw_center = center
        result.confidence = min(1.0, max(0.0, best_score))
        result.source = "concentric_circles"
        result.reason = "ok"
        result.circles = [
            (outer.center, outer.radius),
            (inner.center, inner.radius),
        ]
        return result

    @staticmethod
    def _radial_edge_profile(
        edges: np.ndarray,
        center: Tuple[float, float],
        minimum_radius: int,
        maximum_radius: int,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        height, width = edges.shape[:2]
        radii = np.arange(
            minimum_radius, maximum_radius + 1, dtype=np.float64
        )
        angles = np.linspace(
            0.0, 2.0 * math.pi, 240, endpoint=False, dtype=np.float64
        )
        cosines = np.cos(angles)
        sines = np.sin(angles)
        sample_x = np.rint(
            center[0] + radii[:, np.newaxis] * cosines[np.newaxis, :]
        ).astype(np.int32)
        sample_y = np.rint(
            center[1] + radii[:, np.newaxis] * sines[np.newaxis, :]
        ).astype(np.int32)
        valid = (
            (sample_x >= 0)
            & (sample_x < width)
            & (sample_y >= 0)
            & (sample_y < height)
        )
        visible_fraction = np.mean(valid, axis=1)
        clipped_x = np.clip(sample_x, 0, width - 1)
        clipped_y = np.clip(sample_y, 0, height - 1)
        edge_support_map = cv2.dilate(
            edges, np.ones((3, 3), dtype=np.uint8), iterations=1
        )
        supported = valid & (edge_support_map[clipped_y, clipped_x] > 0)
        valid_counts = np.maximum(np.sum(valid, axis=1), 1)
        supports = np.sum(supported, axis=1) / valid_counts
        return radii, supports, visible_fraction

    @staticmethod
    def _radial_peak_indices(
        supports: np.ndarray,
        visible_fraction: np.ndarray,
        minimum_support: float,
        minimum_visible_fraction: float,
        maximum_peaks: int = 24,
    ) -> List[int]:
        eligible = [
            index
            for index in range(len(supports))
            if supports[index] >= minimum_support
            and visible_fraction[index] >= minimum_visible_fraction
        ]
        peak_indices: List[int] = []
        for index in sorted(
            eligible, key=lambda item: float(supports[item]), reverse=True
        ):
            if all(abs(index - existing) > 4 for existing in peak_indices):
                peak_indices.append(index)
            if len(peak_indices) >= maximum_peaks:
                break
        return peak_indices

    def _snap_circles_to_outer_edges(
        self,
        edges: np.ndarray,
        center: Tuple[float, float],
        circles: List[Tuple[Tuple[float, float], float]],
    ) -> List[Tuple[Tuple[float, float], float]]:
        if not circles:
            return []
        minimum_dimension = min(edges.shape[:2])
        minimum_radius = max(
            5,
            int(
                minimum_dimension
                * float(config_value(self.config, "circle.min_radius_ratio"))
            ),
        )
        maximum_radius = max(
            minimum_radius + 1,
            int(
                minimum_dimension
                * float(config_value(self.config, "circle.max_radius_ratio"))
            ),
        )
        radii, supports, visible = self._radial_edge_profile(
            edges, center, minimum_radius, maximum_radius
        )
        search_ratio = float(
            config_value(self.config, "circle.outer_edge_search_ratio")
        )
        relative_support = float(
            config_value(
                self.config, "circle.outer_edge_relative_support"
            )
        )
        minimum_support = float(
            config_value(self.config, "circle.min_edge_support")
        )
        minimum_visible = float(
            config_value(
                self.config,
                "circle.cross_guided_min_visible_fraction",
            )
        )
        snapped = []
        for _, nominal_radius in circles:
            lower = nominal_radius * (1.0 - search_ratio)
            upper = nominal_radius * (1.0 + search_ratio)
            indexes = np.flatnonzero(
                (radii >= lower)
                & (radii <= upper)
                & (visible >= minimum_visible)
            )
            if indexes.size == 0:
                snapped.append((center, float(nominal_radius)))
                continue
            local_maximum = float(np.max(supports[indexes]))
            threshold = max(
                minimum_support, relative_support * local_maximum
            )
            supported_indexes = indexes[supports[indexes] >= threshold]
            if supported_indexes.size == 0:
                snapped.append((center, float(nominal_radius)))
                continue
            # The requested overlay radius is the outside edge of the black
            # stroke, never whichever of its two Canny edges scored first.
            outer_index = int(supported_indexes[-1])
            snapped.append((center, float(radii[outer_index])))
        return snapped

    def _detect_circles_from_cross_center(
        self,
        gray: np.ndarray,
        binary: np.ndarray,
        edges: np.ndarray,
        center: Tuple[float, float],
    ) -> Detection:
        """Recover the two rings when cross junctions break ellipse contours.

        The official target connects both rings to the cross. A sharp camera
        can therefore turn each white quadrant into a closed contour while the
        actual rings become disconnected arcs. Once the orthogonal cross has
        supplied a center, scan radial edge support around only that center and
        still require the official 30/50 ratio. This is a constrained fallback,
        not a general Hough search over the scene.
        """
        result = Detection(
            mode=MODE_HIGH, gray=gray, binary=binary, edges=edges
        )
        minimum_dimension = min(edges.shape[:2])
        minimum_radius = max(
            5,
            int(
                minimum_dimension
                * float(config_value(self.config, "circle.min_radius_ratio"))
            ),
        )
        maximum_radius = max(
            minimum_radius + 1,
            int(
                minimum_dimension
                * float(config_value(self.config, "circle.max_radius_ratio"))
            ),
        )
        radii, supports, visible_fraction = self._radial_edge_profile(
            edges, center, minimum_radius, maximum_radius
        )

        minimum_support = float(
            config_value(self.config, "circle.min_edge_support")
        )
        minimum_visible = float(
            config_value(
                self.config,
                "circle.cross_guided_min_visible_fraction",
            )
        )
        peak_indices = self._radial_peak_indices(
            supports,
            visible_fraction,
            minimum_support,
            minimum_visible,
        )
        if len(peak_indices) < 2:
            result.reason = "cross_guided_fewer_than_two_radial_edges"
            return result

        expected_ratio = float(
            config_value(self.config, "circle.expected_diameter_ratio")
        )
        ratio_tolerance = float(
            config_value(self.config, "circle.diameter_ratio_tolerance")
        )
        best_pair: Optional[
            Tuple[CircleCandidate, CircleCandidate]
        ] = None
        best_score = -1.0
        for outer_index in peak_indices:
            for inner_index in peak_indices:
                outer_radius = float(radii[outer_index])
                inner_radius = float(radii[inner_index])
                if outer_radius <= inner_radius:
                    continue
                radius_ratio = inner_radius / outer_radius
                ratio_error = abs(radius_ratio - expected_ratio)
                if ratio_error > ratio_tolerance:
                    continue
                outer_support = float(supports[outer_index])
                inner_support = float(supports[inner_index])
                ratio_score = 1.0 - ratio_error / max(
                    ratio_tolerance, 1e-6
                )
                evidence_score = min(
                    1.0, 0.5 * (outer_support + inner_support)
                )
                pair_score = (
                    0.38 * ratio_score
                    + 0.38
                    + 0.24 * evidence_score
                )
                if pair_score > best_score:
                    best_score = pair_score
                    best_pair = (
                        CircleCandidate(
                            center,
                            outer_radius,
                            outer_support,
                            "cross_guided_radial",
                        ),
                        CircleCandidate(
                            center,
                            inner_radius,
                            inner_support,
                            "cross_guided_radial",
                        ),
                    )

        if best_pair is None:
            result.reason = "cross_guided_no_30_50_radial_pair"
            return result
        minimum_confidence = float(
            config_value(self.config, "circle.min_confidence")
        )
        if best_score < minimum_confidence:
            result.reason = "cross_guided_confidence_below_threshold"
            result.confidence = max(0.0, best_score)
            return result

        outer, inner = best_pair
        result.valid = True
        result.center = center
        result.raw_center = center
        result.confidence = min(1.0, max(0.0, best_score))
        result.source = "cross_guided_radial_circles"
        result.reason = "ok"
        result.circles = [
            (center, outer.radius),
            (center, inner.radius),
        ]
        return result

    def _detect_inner_ring_from_cross_center(
        self,
        gray: np.ndarray,
        binary: np.ndarray,
        edges: np.ndarray,
        center: Tuple[float, float],
        expected_outer_edge_radius: Optional[float] = None,
    ) -> Detection:
        """Mode 2 detector: one inner-ring stroke plus the cross center."""
        result = Detection(
            mode=MODE_TRANSITION,
            gray=gray,
            binary=binary,
            edges=edges,
        )
        minimum_dimension = min(edges.shape[:2])
        minimum_radius = max(
            5,
            int(
                minimum_dimension
                * max(
                    float(
                        config_value(
                            self.config, "circle.min_radius_ratio"
                        )
                    ),
                    float(
                        config_value(
                            self.config,
                            "transition.inner_min_radius_ratio",
                        )
                    ),
                )
            ),
        )
        maximum_radius = max(
            minimum_radius + 1,
            int(
                minimum_dimension
                * float(config_value(self.config, "circle.max_radius_ratio"))
            ),
        )
        radii, supports, visible = self._radial_edge_profile(
            edges, center, minimum_radius, maximum_radius
        )
        minimum_support = float(
            config_value(self.config, "circle.min_edge_support")
        )
        minimum_visible = float(
            config_value(
                self.config,
                "circle.cross_guided_min_visible_fraction",
            )
        )
        peaks = self._radial_peak_indices(
            supports,
            visible,
            minimum_support,
            minimum_visible,
        )
        if len(peaks) < 2:
            result.reason = "inner_ring_fewer_than_two_radial_edges"
            return result

        expected_separation = float(
            config_value(
                self.config,
                "transition.inner_edge_separation_expected_ratio",
            )
        )
        minimum_separation = float(
            config_value(
                self.config,
                "transition.inner_edge_separation_min_ratio",
            )
        )
        maximum_separation = float(
            config_value(
                self.config,
                "transition.inner_edge_separation_max_ratio",
            )
        )
        minimum_confidence = float(
            config_value(self.config, "circle.min_confidence")
        )
        pairs = []
        for outer_index in peaks:
            for inner_index in peaks:
                outer_radius = float(radii[outer_index])
                inner_radius = float(radii[inner_index])
                if outer_radius <= inner_radius:
                    continue
                separation_ratio = (
                    outer_radius - inner_radius
                ) / max(outer_radius, 1.0)
                if not (
                    minimum_separation
                    <= separation_ratio
                    <= maximum_separation
                ):
                    continue
                separation_error = abs(
                    separation_ratio - expected_separation
                )
                separation_tolerance = max(
                    expected_separation - minimum_separation,
                    maximum_separation - expected_separation,
                    1e-6,
                )
                separation_score = max(
                    0.0, 1.0 - separation_error / separation_tolerance
                )
                evidence_score = min(
                    1.0,
                    0.5
                    * (
                        float(supports[outer_index])
                        + float(supports[inner_index])
                    ),
                )
                confidence = (
                    0.55 * separation_score + 0.45 * evidence_score
                )
                if confidence >= minimum_confidence:
                    pairs.append(
                        (
                            outer_radius,
                            inner_radius,
                            confidence,
                        )
                    )

        if not pairs:
            result.reason = "no_supported_inner_ring_edge_pair"
            return result

        if expected_outer_edge_radius is not None:
            selected = min(
                pairs,
                key=lambda item: (
                    abs(item[0] - expected_outer_edge_radius)
                    / max(expected_outer_edge_radius, 1.0),
                    -item[2],
                ),
            )
        else:
            # If both rings remain visible at the Mode 1→2 boundary, the
            # smaller valid stroke is the physical inner ring.
            selected = min(pairs, key=lambda item: (item[0], -item[2]))

        outer_edge_radius, _, confidence = selected
        result.valid = True
        result.center = center
        result.raw_center = center
        result.confidence = confidence
        result.source = "inner_ring_from_cross"
        result.reason = "ok"
        result.circles = [
            (
                center,
                float(outer_edge_radius),
            )
        ]
        return result

    def _detect_high_validated(
        self, gray: np.ndarray, binary: np.ndarray, edges: np.ndarray
    ) -> Detection:
        circle = self._detect_circles(gray, binary, edges)
        cross: Optional[Detection] = None
        original_circle_reason = circle.reason
        if (
            (not circle.valid or circle.center is None or not circle.circles)
            and int(
                config_value(
                    self.config,
                    "circle.enable_cross_guided_recovery",
                )
            )
        ):
            cross = self._detect_cross(gray, binary, edges)
            if cross.valid and cross.center is not None:
                recovered = self._detect_circles_from_cross_center(
                    gray, binary, edges, cross.center
                )
                if recovered.valid:
                    circle = recovered
                else:
                    circle.reason = "{}|{}".format(
                        original_circle_reason, recovered.reason
                    )

        if not circle.valid or circle.center is None or not circle.circles:
            return circle

        outer_radius = max(radius for _, radius in circle.circles)
        maximum_distance = max(
            float(
                config_value(
                    self.config, "circle.cross_validation_min_distance_px"
                )
            ),
            outer_radius
            * float(
                config_value(
                    self.config,
                    "circle.cross_validation_distance_outer_radius_ratio",
                )
            ),
        )
        if (
            cross is None
            or not cross.valid
            or cross.center is None
            or math.hypot(
                cross.center[0] - circle.center[0],
                cross.center[1] - circle.center[1],
            )
            > maximum_distance
        ):
            cross = self._detect_cross(
                gray,
                binary,
                edges,
                expected_center=circle.center,
                maximum_center_distance=maximum_distance,
            )
        circle.segments = cross.segments
        circle.selected_angles = cross.selected_angles
        circle.selected_extents = cross.selected_extents

        if not cross.valid or cross.center is None:
            if int(
                config_value(
                    self.config, "circle.require_cross_validation"
                )
            ):
                circle.valid = False
                circle.center = None
                circle.source = "none"
                circle.reason = "cross_validation_failed:{}".format(
                    cross.reason
                )
                circle.confidence = min(circle.confidence, cross.confidence)
            return circle

        # The line intersection is a better projective center than an ellipse
        # center when the target plane is tilted. Rings validate identity; the
        # cross supplies the final center.
        circle.circles = self._snap_circles_to_outer_edges(
            edges, cross.center, circle.circles
        )
        circle.center = cross.center
        circle.raw_center = cross.center
        circle.confidence = min(circle.confidence, cross.confidence)
        circle.source = (
            "cross_guided_rings_plus_cross"
            if circle.source == "cross_guided_radial_circles"
            else "rings_plus_cross"
        )
        circle.reason = "ok"
        return circle

    @staticmethod
    def _line_angle(segment: Tuple[int, int, int, int]) -> float:
        x1, y1, x2, y2 = segment
        angle = math.atan2(y2 - y1, x2 - x1) % math.pi
        return angle

    @staticmethod
    def _angle_distance(first: float, second: float) -> float:
        difference = abs(first - second) % math.pi
        return min(difference, math.pi - difference)

    @staticmethod
    def _line_intersection(
        first: Tuple[int, int, int, int],
        second: Tuple[int, int, int, int],
    ) -> Optional[Tuple[float, float]]:
        x1, y1, x2, y2 = first
        x3, y3, x4, y4 = second
        denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denominator) < 1e-6:
            return None
        determinant_a = x1 * y2 - y1 * x2
        determinant_b = x3 * y4 - y3 * x4
        x = (
            determinant_a * (x3 - x4)
            - (x1 - x2) * determinant_b
        ) / denominator
        y = (
            determinant_a * (y3 - y4)
            - (y1 - y2) * determinant_b
        ) / denominator
        return float(x), float(y)

    @staticmethod
    def _point_to_segment_distance(
        point: Tuple[float, float], segment: Tuple[int, int, int, int]
    ) -> float:
        px, py = point
        x1, y1, x2, y2 = segment
        vx, vy = x2 - x1, y2 - y1
        length_squared = vx * vx + vy * vy
        if length_squared <= 0:
            return math.hypot(px - x1, py - y1)
        projection = ((px - x1) * vx + (py - y1) * vy) / length_squared
        projection = min(1.0, max(0.0, projection))
        closest_x = x1 + projection * vx
        closest_y = y1 + projection * vy
        return math.hypot(px - closest_x, py - closest_y)

    @staticmethod
    def _ray_support(
        binary: np.ndarray,
        center: Tuple[float, float],
        angle: float,
        length: int,
        band: int,
    ) -> float:
        height, width = binary.shape[:2]
        distances = np.linspace(3.0, float(length), max(length, 2))
        offsets = np.arange(-band, band + 1, dtype=np.float64)
        perpendicular_x = -math.sin(angle)
        perpendicular_y = math.cos(angle)
        sample_x = np.rint(
            center[0]
            + math.cos(angle) * distances[:, np.newaxis]
            + perpendicular_x * offsets[np.newaxis, :]
        ).astype(np.int32)
        sample_y = np.rint(
            center[1]
            + math.sin(angle) * distances[:, np.newaxis]
            + perpendicular_y * offsets[np.newaxis, :]
        ).astype(np.int32)
        valid = (
            (sample_x >= 0)
            & (sample_x < width)
            & (sample_y >= 0)
            & (sample_y < height)
        )
        rows_in_frame = np.any(valid, axis=1)
        if not np.any(rows_in_frame):
            return 0.0
        clipped_x = np.clip(sample_x, 0, width - 1)
        clipped_y = np.clip(sample_y, 0, height - 1)
        supported = valid & (binary[clipped_y, clipped_x] > 0)
        return float(np.mean(np.any(supported, axis=1)[rows_in_frame]))

    @staticmethod
    def _ray_extent(
        binary: np.ndarray,
        center: Tuple[float, float],
        angle: float,
        band: int,
        maximum_length: int,
        allowed_gap: int = 8,
    ) -> float:
        height, width = binary.shape[:2]
        perpendicular_x = -math.sin(angle)
        perpendicular_y = math.cos(angle)
        last_supported = 0.0
        consecutive_gap = 0
        for distance in range(2, maximum_length + 1):
            x = center[0] + math.cos(angle) * distance
            y = center[1] + math.sin(angle) * distance
            if not (0 <= x < width and 0 <= y < height):
                break
            supported = False
            for offset in range(-band, band + 1):
                sx = int(round(x + perpendicular_x * offset))
                sy = int(round(y + perpendicular_y * offset))
                if (
                    0 <= sx < width
                    and 0 <= sy < height
                    and binary[sy, sx] > 0
                ):
                    supported = True
                    break
            if supported:
                last_supported = float(distance)
                consecutive_gap = 0
            else:
                consecutive_gap += 1
                if consecutive_gap > allowed_gap:
                    break
        return last_supported

    @staticmethod
    def _centerline_intersection(
        first: Tuple[float, float, float, float, Tuple[int, int, int, int], Tuple[int, int, int, int]],
        second: Tuple[float, float, float, float, Tuple[int, int, int, int], Tuple[int, int, int, int]],
    ) -> Optional[Tuple[float, float]]:
        first_angle, first_rho = first[0], first[1]
        second_angle, second_rho = second[0], second[1]
        first_nx = -math.sin(first_angle)
        first_ny = math.cos(first_angle)
        second_nx = -math.sin(second_angle)
        second_ny = math.cos(second_angle)
        determinant = first_nx * second_ny - first_ny * second_nx
        if abs(determinant) < 1e-6:
            return None
        x = (
            first_rho * second_ny - first_ny * second_rho
        ) / determinant
        y = (
            first_nx * second_rho - first_rho * second_nx
        ) / determinant
        return float(x), float(y)

    def _build_stroke_centerlines(
        self,
        segments: List[Tuple[int, int, int, int]],
        minimum_dimension: int,
    ) -> List[
        Tuple[
            float,
            float,
            float,
            float,
            Tuple[int, int, int, int],
            Tuple[int, int, int, int],
        ]
    ]:
        """Pair parallel stroke edges and return their medial lines.

        HoughLinesP sees two edge lines for every thick black stroke. Directly
        intersecting those edges produces four corners around the real cross
        center and becomes unstable as line width grows. Pairing the parallel
        edges first makes the result independent of the apparent stroke width.
        """
        parallel_tolerance = math.radians(
            float(config_value(self.config, "cross.parallel_tolerance_deg"))
        )
        minimum_width = float(
            config_value(self.config, "cross.min_stroke_width_px")
        )
        maximum_width = minimum_dimension * float(
            config_value(self.config, "cross.max_stroke_width_ratio")
        )
        minimum_overlap = max(
            5.0,
            minimum_dimension
            * float(config_value(self.config, "cross.min_line_length_ratio"))
            * 0.22,
        )
        prepared = []
        for segment in segments:
            angle = self._line_angle(segment)
            midpoint_x = 0.5 * (segment[0] + segment[2])
            midpoint_y = 0.5 * (segment[1] + segment[3])
            length = math.hypot(
                segment[2] - segment[0], segment[3] - segment[1]
            )
            prepared.append(
                (
                    segment,
                    angle,
                    midpoint_x,
                    midpoint_y,
                    length,
                )
            )

        centerlines = []
        for index, first in enumerate(prepared):
            for second in prepared[index + 1 :]:
                angle_error = self._angle_distance(first[1], second[1])
                if angle_error > parallel_tolerance:
                    continue

                doubled_x = math.cos(2.0 * first[1]) + math.cos(2.0 * second[1])
                doubled_y = math.sin(2.0 * first[1]) + math.sin(2.0 * second[1])
                angle = 0.5 * math.atan2(doubled_y, doubled_x)
                if angle < 0.0:
                    angle += math.pi
                direction_x = math.cos(angle)
                direction_y = math.sin(angle)
                normal_x = -direction_y
                normal_y = direction_x
                rho_first = first[2] * normal_x + first[3] * normal_y
                rho_second = second[2] * normal_x + second[3] * normal_y
                stroke_width = abs(rho_first - rho_second)
                if not minimum_width <= stroke_width <= maximum_width:
                    continue

                first_segment = first[0]
                second_segment = second[0]
                first_projection_a = (
                    first_segment[0] * direction_x
                    + first_segment[1] * direction_y
                )
                first_projection_b = (
                    first_segment[2] * direction_x
                    + first_segment[3] * direction_y
                )
                second_projection_a = (
                    second_segment[0] * direction_x
                    + second_segment[1] * direction_y
                )
                second_projection_b = (
                    second_segment[2] * direction_x
                    + second_segment[3] * direction_y
                )
                first_min = min(first_projection_a, first_projection_b)
                first_max = max(first_projection_a, first_projection_b)
                second_min = min(second_projection_a, second_projection_b)
                second_max = max(second_projection_a, second_projection_b)
                overlap = max(
                    0.0,
                    min(first_max, second_max)
                    - max(first_min, second_min),
                )
                if overlap < minimum_overlap:
                    continue

                angle_score = 1.0 - angle_error / max(
                    parallel_tolerance, 1e-6
                )
                overlap_score = min(
                    1.0, overlap / max(min(first[4], second[4]), 1.0)
                )
                length_score = min(
                    1.0,
                    (first[4] + second[4])
                    / max(0.45 * minimum_dimension, 1.0),
                )
                score = (
                    0.38 * angle_score
                    + 0.34 * overlap_score
                    + 0.28 * length_score
                )
                centerlines.append(
                    (
                        angle,
                        0.5 * (rho_first + rho_second),
                        stroke_width,
                        score,
                        first[0],
                        second[0],
                    )
                )

        deduplicated = []
        for candidate in sorted(
            centerlines, key=lambda item: item[3], reverse=True
        ):
            duplicate = any(
                self._angle_distance(candidate[0], existing[0])
                <= math.radians(3.0)
                and abs(candidate[1] - existing[1])
                <= max(3.0, 0.14 * max(candidate[2], existing[2]))
                for existing in deduplicated
            )
            if not duplicate:
                deduplicated.append(candidate)
            if len(deduplicated) >= max(
                2, int(config_value(self.config, "cross.max_centerlines"))
            ):
                break
        return deduplicated

    def _detect_cross(
        self,
        gray: np.ndarray,
        binary: np.ndarray,
        edges: np.ndarray,
        expected_center: Optional[Tuple[float, float]] = None,
        maximum_center_distance: Optional[float] = None,
    ) -> Detection:
        result = Detection(mode=MODE_LOW, gray=gray, binary=binary, edges=edges)
        height, width = edges.shape[:2]
        minimum_dimension = min(height, width)
        raw_lines = cv2.HoughLinesP(
            edges,
            rho=1.0,
            theta=math.pi / 180.0,
            threshold=int(config_value(self.config, "cross.hough_threshold")),
            minLineLength=max(
                8,
                int(
                    minimum_dimension
                    * float(
                        config_value(
                            self.config, "cross.min_line_length_ratio"
                        )
                    )
                ),
            ),
            maxLineGap=int(config_value(self.config, "cross.max_line_gap")),
        )
        if raw_lines is None:
            result.reason = "no_line_segments"
            return result

        segments = [
            tuple(int(value) for value in line[0]) for line in raw_lines
        ]
        maximum_segments = max(
            2, int(config_value(self.config, "cross.max_segments"))
        )
        if len(segments) > maximum_segments:
            segments = sorted(
                segments,
                key=lambda segment: (
                    (segment[2] - segment[0]) ** 2
                    + (segment[3] - segment[1]) ** 2
                ),
                reverse=True,
            )[:maximum_segments]
        result.segments = segments
        if len(segments) < 2:
            result.reason = "fewer_than_two_line_segments"
            return result

        centerlines = self._build_stroke_centerlines(
            segments, minimum_dimension
        )
        if len(centerlines) < 2:
            result.reason = "fewer_than_two_stroke_centerlines"
            return result

        tolerance = math.radians(
            float(config_value(self.config, "cross.orthogonal_tolerance_deg"))
        )
        segment_margin = float(
            config_value(self.config, "cross.intersection_margin_px")
        )
        ray_length = max(
            8,
            int(
                minimum_dimension
                * float(config_value(self.config, "cross.ray_length_ratio"))
            ),
        )
        ray_band = int(config_value(self.config, "cross.ray_band_px"))
        min_ray_support = float(
            config_value(self.config, "cross.min_ray_support")
        )
        # Adaptive thresholding intentionally removes slowly varying dark
        # regions. Once a 2 cm target stroke grows wider than the adaptive
        # block, that also removes the stroke interior. Otsu restores only the
        # dark support map used to verify centerlines; Hough geometry still
        # comes from Canny edges, so shadows alone cannot create a result.
        _, global_dark = cv2.threshold(
            gray,
            0,
            255,
            cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU,
        )
        support_binary = cv2.bitwise_or(binary, global_dark)

        candidates: List[
            Tuple[Tuple[float, float], float, float, float, float]
        ] = []
        orthogonal_pairs = 0
        in_frame_pairs = 0
        supported_pairs = 0
        for index, first in enumerate(centerlines):
            angle_first = first[0]
            for second in centerlines[index + 1 :]:
                angle_second = second[0]
                angle_distance = self._angle_distance(angle_first, angle_second)
                orthogonal_error = abs(math.pi / 2.0 - angle_distance)
                if orthogonal_error > tolerance:
                    continue
                orthogonal_pairs += 1
                intersection = self._centerline_intersection(first, second)
                if intersection is None:
                    continue
                if not (
                    0.0 <= intersection[0] < width
                    and 0.0 <= intersection[1] < height
                ):
                    continue
                in_frame_pairs += 1
                if (
                    expected_center is not None
                    and maximum_center_distance is not None
                    and math.hypot(
                        intersection[0] - expected_center[0],
                        intersection[1] - expected_center[1],
                    )
                    > maximum_center_distance
                ):
                    continue

                # At the cross junction, the boundary of one stroke disappears
                # across the full width of the other stroke. Ring junctions can
                # split it again. Permit a short centerline extrapolation, then
                # let four-direction dark-pixel support prove the intersection.
                extrapolation_margin = 2.25 * ray_length
                first_margin = max(
                    segment_margin, extrapolation_margin, 0.85 * first[2]
                )
                second_margin = max(
                    segment_margin, extrapolation_margin, 0.85 * second[2]
                )
                if min(
                    self._point_to_segment_distance(intersection, first[4]),
                    self._point_to_segment_distance(intersection, first[5]),
                ) > first_margin or min(
                    self._point_to_segment_distance(intersection, second[4]),
                    self._point_to_segment_distance(intersection, second[5]),
                ) > second_margin:
                    continue

                ray_supports = [
                    self._ray_support(
                        support_binary,
                        intersection,
                        angle_first,
                        ray_length,
                        ray_band,
                    ),
                    self._ray_support(
                        support_binary,
                        intersection,
                        angle_first + math.pi,
                        ray_length,
                        ray_band,
                    ),
                    self._ray_support(
                        support_binary,
                        intersection,
                        angle_second,
                        ray_length,
                        ray_band,
                    ),
                    self._ray_support(
                        support_binary,
                        intersection,
                        angle_second + math.pi,
                        ray_length,
                        ray_band,
                    ),
                ]
                support = min(ray_supports)
                if support < min_ray_support:
                    continue
                supported_pairs += 1
                orthogonal_score = 1.0 - orthogonal_error / max(tolerance, 1e-6)
                mean_support = float(np.mean(ray_supports))
                centerline_score = 0.5 * (first[3] + second[3])
                score = (
                    0.26 * orthogonal_score
                    + 0.34 * support
                    + 0.18 * mean_support
                    + 0.22 * centerline_score
                )
                candidates.append(
                    (
                        intersection,
                        score,
                        angle_first,
                        angle_second,
                        0.5 * (first[2] + second[2]),
                    )
                )

        if not candidates:
            if orthogonal_pairs == 0:
                detail = "no_orthogonal_centerlines"
            elif in_frame_pairs == 0:
                detail = "centerline_intersection_out_of_frame"
            elif supported_pairs == 0:
                detail = "centerline_intersection_not_supported"
            else:
                detail = "no_supported_centerline_intersection"
            result.reason = (
                "no_cross_near_expected_center:{}".format(detail)
                if expected_center is not None
                else detail
            )
            return result

        seed = max(candidates, key=lambda item: item[1])
        cluster_radius = max(
            float(config_value(self.config, "cross.intersection_cluster_px")),
            0.35 * seed[4],
        )
        cluster = [
            item
            for item in candidates
            if math.hypot(
                item[0][0] - seed[0][0], item[0][1] - seed[0][1]
            )
            <= cluster_radius
        ]
        weights = np.array([max(item[1], 1e-6) for item in cluster], dtype=np.float64)
        points = np.array([item[0] for item in cluster], dtype=np.float64)
        center_array = np.average(points, axis=0, weights=weights)
        center = (float(center_array[0]), float(center_array[1]))
        dispersion = float(
            np.average(
                np.linalg.norm(points - center_array, axis=1), weights=weights
            )
        )
        cluster_score = min(1.0, len(cluster) / 4.0)
        dispersion_score = max(0.0, 1.0 - dispersion / max(cluster_radius, 1.0))
        confidence = min(
            1.0,
            0.70 * float(np.average(weights))
            + 0.15 * cluster_score
            + 0.15 * dispersion_score,
        )
        min_confidence = float(config_value(self.config, "cross.min_confidence"))
        if confidence < min_confidence:
            result.reason = "cross_confidence_below_threshold"
            result.confidence = confidence
            return result

        result.valid = True
        result.center = center
        result.raw_center = center
        result.confidence = confidence
        result.source = "stroke_centerline_cross"
        result.reason = "ok"
        result.selected_angles = [seed[2], seed[3]]
        maximum_extent = int(math.hypot(width, height))
        result.selected_extents = []
        for angle in result.selected_angles:
            negative_extent = self._ray_extent(
                support_binary,
                center,
                angle + math.pi,
                ray_band,
                maximum_extent,
            )
            positive_extent = self._ray_extent(
                support_binary,
                center,
                angle,
                ray_band,
                maximum_extent,
            )
            result.selected_extents.append(
                (negative_extent, positive_extent)
            )
        return result

    def _detect_transition(
        self, gray: np.ndarray, binary: np.ndarray, edges: np.ndarray
    ) -> Detection:
        expected_center: Optional[Tuple[float, float]] = None
        maximum_distance: Optional[float] = None
        if self.track_center is not None:
            expected_center = (
                float(self.track_center[0]),
                float(self.track_center[1]),
            )
            maximum_distance = float(
                config_value(self.config, "tracking.max_center_jump_px")
            )

        cross = self._detect_cross(
            gray,
            binary,
            edges,
            expected_center=expected_center,
            maximum_center_distance=maximum_distance,
        )
        cross.mode = MODE_TRANSITION
        if not cross.valid or cross.center is None:
            cross.valid = False
            cross.center = None
            cross.source = "none"
            cross.reason = "inner_ring_requires_cross:{}".format(
                cross.reason
            )
            return cross

        expected_radius = None
        if self.track_radii is not None and len(self.track_radii) > 0:
            expected_radius = float(np.min(self.track_radii))
        inner_ring = self._detect_inner_ring_from_cross_center(
            gray,
            binary,
            edges,
            cross.center,
            expected_outer_edge_radius=expected_radius,
        )
        if not inner_ring.valid or not inner_ring.circles:
            cross.valid = False
            cross.center = None
            cross.source = "none"
            cross.reason = "inner_ring_unavailable:{}".format(
                inner_ring.reason
            )
            cross.confidence = min(
                cross.confidence, inner_ring.confidence
            )
            return cross

        cross.circles = inner_ring.circles
        cross.confidence = min(cross.confidence, inner_ring.confidence)
        cross.source = "transition_inner_ring_plus_cross"
        cross.reason = "ok"
        return cross

    @staticmethod
    def _result_radii(result: Detection) -> Optional[np.ndarray]:
        if not result.circles:
            return None
        radii = sorted((float(radius) for _, radius in result.circles), reverse=True)
        return np.array(radii[:2], dtype=np.float64)

    def _pending_candidate_matches(
        self, center: np.ndarray, radii: Optional[np.ndarray]
    ) -> bool:
        if self.pending_center is None:
            return False
        scale = (
            float(max(radii[0], self.pending_radii[0]))
            if radii is not None and self.pending_radii is not None
            else float(config_value(self.config, "tracking.max_center_jump_px"))
        )
        center_limit = max(
            8.0,
            scale
            * float(
                config_value(
                    self.config,
                    "tracking.pending_center_tolerance_outer_radius_ratio",
                )
            ),
        )
        if np.linalg.norm(center - self.pending_center) > center_limit:
            return False
        if radii is not None and self.pending_radii is not None:
            relative_change = np.max(
                np.abs(radii - self.pending_radii)
                / np.maximum(self.pending_radii, 1.0)
            )
            if relative_change > float(
                config_value(
                    self.config, "tracking.pending_radius_tolerance_ratio"
                )
            ):
                return False
        return True

    def _tracked_candidate_matches(
        self, center: np.ndarray, radii: Optional[np.ndarray]
    ) -> Tuple[bool, str]:
        if self.track_center is None:
            return False, "no_active_track"
        outer_radius = (
            float(self.track_radii[0])
            if self.track_radii is not None
            else 0.0
        )
        center_limit = max(
            float(config_value(self.config, "tracking.max_center_jump_px")),
            outer_radius
            * float(
                config_value(
                    self.config,
                    "tracking.max_center_jump_outer_radius_ratio",
                )
            ),
        )
        center_jump = float(np.linalg.norm(center - self.track_center))
        if center_jump > center_limit:
            return False, "rejected_center_jump_{:.1f}px".format(center_jump)
        if radii is not None and self.track_radii is not None:
            relative_change = float(
                np.max(
                    np.abs(radii - self.track_radii)
                    / np.maximum(self.track_radii, 1.0)
                )
            )
            maximum_change = float(
                config_value(
                    self.config, "tracking.max_radius_change_ratio"
                )
            )
            if relative_change > maximum_change:
                return False, "rejected_radius_jump_{:.0%}".format(
                    relative_change
                )
        return True, "ok"

    def _draw_tracked_geometry(self, result: Detection) -> None:
        if self.track_center is None or self.track_radii is None:
            return
        center = (float(self.track_center[0]), float(self.track_center[1]))
        result.circles = [
            (center, float(radius)) for radius in self.track_radii
        ]

    def _apply_tracking(self, result: Detection) -> Detection:
        raw_center = (
            np.array(result.center, dtype=np.float64)
            if result.valid and result.center is not None
            else None
        )
        raw_radii = self._result_radii(result)
        acquire_frames = max(
            1, int(config_value(self.config, "tracking.acquire_frames"))
        )

        if self.track_center is None:
            if raw_center is None:
                self.pending_center = None
                self.pending_radii = None
                self.pending_count = 0
                result.center = None
                result.track_state = "SEARCHING"
                return result

            if self._pending_candidate_matches(raw_center, raw_radii):
                self.pending_count += 1
                self.pending_center = (
                    0.5 * raw_center + 0.5 * self.pending_center
                )
                if raw_radii is not None and self.pending_radii is not None:
                    self.pending_radii = (
                        0.5 * raw_radii + 0.5 * self.pending_radii
                    )
            else:
                self.pending_center = raw_center
                self.pending_radii = raw_radii
                self.pending_count = 1

            if self.pending_count < acquire_frames:
                result.valid = False
                result.center = None
                result.source = "none"
                result.reason = "acquiring_{}_of_{}".format(
                    self.pending_count, acquire_frames
                )
                result.track_state = "ACQUIRING"
                return result

            self.track_center = self.pending_center.copy()
            self.track_radii = (
                self.pending_radii.copy()
                if self.pending_radii is not None
                else None
            )
            self.track_misses = 0
            self.pending_center = None
            self.pending_radii = None
            self.pending_count = 0
            result.valid = True
            result.center = (
                float(self.track_center[0]),
                float(self.track_center[1]),
            )
            result.track_state = "TRACKING"
            self._draw_tracked_geometry(result)
            return result

        if raw_center is not None:
            matches, reason = self._tracked_candidate_matches(
                raw_center, raw_radii
            )
            if matches:
                center_alpha = float(
                    config_value(self.config, "filter.ema_alpha")
                )
                radius_alpha = float(
                    config_value(self.config, "filter.radius_ema_alpha")
                )
                self.track_center = (
                    center_alpha * raw_center
                    + (1.0 - center_alpha) * self.track_center
                )
                if raw_radii is not None:
                    if self.track_radii is None:
                        self.track_radii = raw_radii
                    else:
                        self.track_radii = (
                            radius_alpha * raw_radii
                            + (1.0 - radius_alpha) * self.track_radii
                        )
                self.track_misses = 0
                result.valid = True
                result.center = (
                    float(self.track_center[0]),
                    float(self.track_center[1]),
                )
                result.track_state = "TRACKING"
                self._draw_tracked_geometry(result)
                return result
            result.reason = reason

        self.track_misses += 1
        maximum_misses = max(
            0, int(config_value(self.config, "tracking.max_missed_frames"))
        )
        if self.track_misses <= maximum_misses:
            result.valid = False
            result.held_center = (
                float(self.track_center[0]),
                float(self.track_center[1]),
            )
            result.center = None
            result.source = "none"
            result.reason = "hold_{}/{}:{}".format(
                self.track_misses, maximum_misses, result.reason
            )
            result.track_state = "HOLD"
            self._draw_tracked_geometry(result)
            return result

        result.valid = False
        result.center = None
        result.held_center = None
        result.source = "none"
        result.reason = "track_lost:{}".format(result.reason)
        result.track_state = "LOST"
        self._clear_track()
        return result

    def detect(self, frame: np.ndarray, mode: str) -> Detection:
        if mode not in MODES:
            raise ValueError("Unknown detection mode: {}".format(mode))
        if frame is None or frame.size == 0:
            raise ValueError("Input frame is empty")

        if self.last_mode != mode:
            preserve_track = (
                self.track_center is not None
                and (
                    (self.last_mode == MODE_HIGH and mode == MODE_TRANSITION)
                    or (
                        self.last_mode == MODE_TRANSITION
                        and mode == MODE_LOW
                    )
                )
            )
            if not preserve_track:
                self._clear_track()
                self.low_search_anchor = None
            elif mode == MODE_TRANSITION:
                # Mode 2 tracks only the physical inner ring. Mode 1 radii are
                # stored [outer, inner], so hand off the smaller outer-edge
                # radius and discard the now irrelevant outer ring.
                if self.track_radii is not None and len(self.track_radii) > 0:
                    self.track_radii = np.array(
                        [float(np.min(self.track_radii))],
                        dtype=np.float64,
                    )
                self.track_misses = 0
                self.pending_center = None
                self.pending_radii = None
                self.pending_count = 0
            elif mode == MODE_LOW:
                # Mode 3 no longer trusts old ring radii, but retaining the
                # confirmed Mode 2 center prevents acquisition on a ring
                # crossing or unrelated right-angle structure.
                self.track_radii = None
                self.track_misses = 0
                self.pending_center = None
                self.pending_radii = None
                self.pending_count = 0
                self.low_search_anchor = self.track_center.copy()
            if mode != MODE_LOW:
                self.low_search_anchor = None
            self.last_mode = mode

        gray, binary, edges = self.preprocess(frame)
        if mode == MODE_HIGH:
            result = self._detect_high_validated(gray, binary, edges)
        elif mode == MODE_LOW:
            expected_center = None
            maximum_distance = None
            if self.track_center is not None:
                expected_center = (
                    float(self.track_center[0]),
                    float(self.track_center[1]),
                )
                maximum_distance = float(
                    config_value(
                        self.config, "tracking.max_center_jump_px"
                    )
                )
            elif self.low_search_anchor is not None:
                expected_center = (
                    float(self.low_search_anchor[0]),
                    float(self.low_search_anchor[1]),
                )
                maximum_distance = 2.0 * float(
                    config_value(
                        self.config, "tracking.max_center_jump_px"
                    )
                )
            result = self._detect_cross(
                gray,
                binary,
                edges,
                expected_center=expected_center,
                maximum_center_distance=maximum_distance,
            )
        else:
            result = self._detect_transition(gray, binary, edges)
        result.mode = mode
        tracked = self._apply_tracking(result)
        if (
            mode == MODE_LOW
            and tracked.valid
            and tracked.center is not None
        ):
            self.low_search_anchor = np.array(
                tracked.center, dtype=np.float64
            )
        return tracked


def detect_commanded_frame(
    detector: TargetDetector,
    frame: np.ndarray,
    mode_value: int,
) -> Detection:
    """Shared 0/1/2/3 dispatch used by Windows Demo 2 and Orange Pi ROS2."""
    if mode_value not in MODE_VALUE_TO_NAME:
        raise ValueError("Unknown commanded vision mode: {}".format(mode_value))
    if mode_value == 0:
        return Detection(
            mode="OFF",
            valid=False,
            reason="vision_off",
            track_state="OFF",
        )
    return detector.detect(frame, MODE_VALUE_TO_NAME[mode_value])


def _line_through_frame(
    center: Tuple[float, float], angle: float, width: int, height: int
) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    length = int(math.hypot(width, height))
    dx = math.cos(angle) * length
    dy = math.sin(angle) * length
    first = (int(round(center[0] - dx)), int(round(center[1] - dy)))
    second = (int(round(center[0] + dx)), int(round(center[1] + dy)))
    return first, second


def draw_overlay(
    frame: np.ndarray,
    result: Detection,
    fps: float,
    control_hint: str = "Sliders: mode/tuning   Keys: R reload  Q quit",
    draw_candidate_segments: bool = True,
) -> np.ndarray:
    overlay = frame.copy()
    height, width = overlay.shape[:2]
    image_center = (int(round(width / 2.0)), int(round(height / 2.0)))
    cv2.drawMarker(
        overlay,
        image_center,
        (255, 180, 0),
        cv2.MARKER_CROSS,
        24,
        2,
        cv2.LINE_AA,
    )

    for center, radius in result.circles:
        cv2.circle(
            overlay,
            (int(round(center[0])), int(round(center[1]))),
            int(round(radius)),
            (255, 100, 0),
            2,
            cv2.LINE_AA,
        )

    if draw_candidate_segments:
        for segment in result.segments:
            cv2.line(
                overlay,
                (segment[0], segment[1]),
                (segment[2], segment[3]),
                (80, 80, 80),
                1,
                cv2.LINE_AA,
            )

    if result.raw_center is not None:
        cv2.circle(
            overlay,
            (int(round(result.raw_center[0])), int(round(result.raw_center[1]))),
            4,
            (0, 180, 255),
            -1,
            cv2.LINE_AA,
        )

    if result.held_center is not None:
        held_center = (
            int(round(result.held_center[0])),
            int(round(result.held_center[1])),
        )
        cv2.drawMarker(
            overlay,
            held_center,
            (0, 190, 255),
            cv2.MARKER_TILTED_CROSS,
            22,
            2,
            cv2.LINE_AA,
        )

    if result.valid and result.center is not None:
        target_center = (
            int(round(result.center[0])),
            int(round(result.center[1])),
        )
        for index, angle in enumerate(result.selected_angles):
            if index < len(result.selected_extents):
                negative_extent, positive_extent = result.selected_extents[index]
                direction_x = math.cos(angle)
                direction_y = math.sin(angle)
                first = (
                    int(round(result.center[0] - direction_x * negative_extent)),
                    int(round(result.center[1] - direction_y * negative_extent)),
                )
                second = (
                    int(round(result.center[0] + direction_x * positive_extent)),
                    int(round(result.center[1] + direction_y * positive_extent)),
                )
            else:
                first, second = _line_through_frame(
                    result.center, angle, width, height
                )
            cv2.line(overlay, first, second, (0, 235, 255), 3, cv2.LINE_AA)
            cv2.circle(overlay, first, 4, (0, 235, 255), -1, cv2.LINE_AA)
            cv2.circle(overlay, second, 4, (0, 235, 255), -1, cv2.LINE_AA)
        cv2.drawMarker(
            overlay,
            target_center,
            (0, 255, 0),
            cv2.MARKER_TILTED_CROSS,
            24,
            2,
            cv2.LINE_AA,
        )
        cv2.arrowedLine(
            overlay,
            image_center,
            target_center,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
            tipLength=0.08,
        )
        if result.mode == MODE_LOW:
            cv2.putText(
                overlay,
                "DETECTED CROSS",
                (target_center[0] + 12, target_center[1] - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (0, 235, 255),
                2,
                cv2.LINE_AA,
            )

    dx, dy = result.dx_dy(frame.shape)
    status_color = (
        (0, 220, 0)
        if result.valid
        else (0, 190, 255)
        if result.track_state in ("ACQUIRING", "HOLD")
        else (0, 0, 255)
    )
    lines = [
        "{}  {}  {}  FPS:{:.1f}".format(
            result.mode,
            "VALID" if result.valid else "INVALID",
            result.track_state,
            fps,
        ),
        "dx:{}  dy:{}  confidence:{:.2f}".format(dx, dy, result.confidence),
        "{} | {}".format(result.source, result.reason),
        control_hint,
    ]
    for index, text in enumerate(lines):
        y = 26 + index * 24
        cv2.putText(
            overlay,
            text,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            status_color if index == 0 else (230, 230, 230),
            2 if index == 0 else 1,
            cv2.LINE_AA,
        )
    return overlay


def _pane_image(
    image: np.ndarray, width: int, height: int, label: str
) -> np.ndarray:
    view = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    cv2.putText(
        view,
        label,
        (12, height - 14),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return view


def _parameter_panel(
    result: Detection,
    fps: float,
    config: Dict[str, Any],
    width: int,
    height: int,
) -> np.ndarray:
    panel = np.full((height, width, 3), (28, 28, 28), dtype=np.uint8)
    status_color = (
        (80, 230, 80)
        if result.valid
        else (0, 200, 255)
        if result.track_state in ("ACQUIRING", "HOLD")
        else (70, 70, 255)
    )
    rows = [
        ("TUNING / STATUS", (0, 255, 255), 0.72),
        (
            "Mode: {}   FPS: {:.1f}".format(result.mode, fps),
            (235, 235, 235),
            0.54,
        ),
        (
            "Track: {}   Valid: {}".format(
                result.track_state, "YES" if result.valid else "NO"
            ),
            status_color,
            0.54,
        ),
        (
            "Confidence: {:.3f}".format(result.confidence),
            (235, 235, 235),
            0.54,
        ),
        (
            "Min confidence: {:.2f}".format(
                float(config_value(config, "circle.min_confidence"))
            ),
            (210, 210, 210),
            0.50,
        ),
        (
            "Circle ratio tolerance: {:.2f}".format(
                float(
                    config_value(
                        config, "circle.diameter_ratio_tolerance"
                    )
                )
            ),
            (210, 210, 210),
            0.50,
        ),
        (
            "Concentricity limit: {:.2f} R".format(
                float(
                    config_value(
                        config, "circle.max_concentricity_ratio"
                    )
                )
            ),
            (210, 210, 210),
            0.50,
        ),
        (
            "Center jump gate: {:.2f} R / {:.0f}px".format(
                float(
                    config_value(
                        config,
                        "tracking.max_center_jump_outer_radius_ratio",
                    )
                ),
                float(config_value(config, "tracking.max_center_jump_px")),
            ),
            (210, 210, 210),
            0.50,
        ),
        (
            "Radius jump gate: {:.0%}".format(
                float(
                    config_value(
                        config, "tracking.max_radius_change_ratio"
                    )
                )
            ),
            (210, 210, 210),
            0.50,
        ),
        (
            "EMA center/radius: {:.2f} / {:.2f}".format(
                float(config_value(config, "filter.ema_alpha")),
                float(config_value(config, "filter.radius_ema_alpha")),
            ),
            (210, 210, 210),
            0.50,
        ),
        (
            "Acquire: {} frames   Hold: {} frames".format(
                int(config_value(config, "tracking.acquire_frames")),
                int(config_value(config, "tracking.max_missed_frames")),
            ),
            (210, 210, 210),
            0.50,
        ),
        ("Use sliders below the window to tune.", (150, 220, 255), 0.48),
        ("R reload YAML   Q/Esc quit", (160, 160, 160), 0.48),
    ]
    y = 26
    for text, color, scale in rows:
        cv2.putText(
            panel,
            text,
            (14, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            1 if scale < 0.65 else 2,
            cv2.LINE_AA,
        )
        y += 24

    reason = "{} | {}".format(result.source, result.reason)
    if len(reason) > 62:
        reason = reason[:59] + "..."
    cv2.putText(
        panel,
        reason,
        (14, height - 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.43,
        (180, 180, 180),
        1,
        cv2.LINE_AA,
    )
    return panel


def render_four_grid(
    frame: np.ndarray,
    result: Detection,
    fps: float,
    config: Dict[str, Any],
) -> np.ndarray:
    pane_width = int(config_value(config, "dashboard.pane_width"))
    pane_height = int(config_value(config, "dashboard.pane_height"))
    overlay = _pane_image(
        draw_overlay(frame, result, fps),
        pane_width,
        pane_height,
        "ANNOTATED",
    )
    binary_source = (
        result.binary
        if result.binary is not None
        else np.zeros(frame.shape[:2], dtype=np.uint8)
    )
    edges_source = (
        result.edges
        if result.edges is not None
        else np.zeros(frame.shape[:2], dtype=np.uint8)
    )
    binary = _pane_image(
        cv2.cvtColor(binary_source, cv2.COLOR_GRAY2BGR),
        pane_width,
        pane_height,
        "BINARY",
    )
    edges = _pane_image(
        cv2.cvtColor(edges_source, cv2.COLOR_GRAY2BGR),
        pane_width,
        pane_height,
        "EDGES",
    )
    panel = _parameter_panel(result, fps, config, pane_width, pane_height)
    return cv2.vconcat(
        [cv2.hconcat([overlay, binary]), cv2.hconcat([edges, panel])]
    )


class StatusLogger:
    def __init__(self, interval_seconds: float):
        self.interval_seconds = interval_seconds
        self.last_time = 0.0
        self.last_state: Optional[Tuple[str, bool, str, str, str]] = None

    def update(
        self, result: Detection, frame_shape: Sequence[int], fps: float
    ) -> None:
        now = time.monotonic()
        state = (
            result.mode,
            result.valid,
            result.track_state,
            result.source,
            result.reason,
        )
        if (
            state != self.last_state
            or now - self.last_time >= self.interval_seconds
        ):
            dx, dy = result.dx_dy(frame_shape)
            payload = {
                "time": time.strftime("%H:%M:%S"),
                "mode": result.mode,
                "valid": result.valid,
                "track_state": result.track_state,
                "dx_px": dx,
                "dy_px": dy,
                "confidence": round(result.confidence, 3),
                "source": result.source,
                "reason": result.reason,
                "fps": round(fps, 1),
            }
            print(json.dumps(payload, ensure_ascii=False), flush=True)
            self.last_state = state
            self.last_time = now


def open_camera(
    camera_index: int, config: Dict[str, Any]
) -> Tuple[cv2.VideoCapture, str]:
    backend_name = str(config_value(config, "camera.backend")).lower()
    backend = cv2.CAP_DSHOW if os.name == "nt" and backend_name == "dshow" else cv2.CAP_ANY
    capture = cv2.VideoCapture(camera_index, backend)
    used_backend = "DirectShow" if backend == cv2.CAP_DSHOW else "Auto"
    if not capture.isOpened() and backend != cv2.CAP_ANY:
        capture.release()
        capture = cv2.VideoCapture(camera_index, cv2.CAP_ANY)
        used_backend = "Auto fallback"
    if not capture.isOpened():
        capture.release()
        raise RuntimeError(
            "Cannot open camera index {} using DirectShow or automatic backend".format(
                camera_index
            )
        )

    capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(config_value(config, "camera.width")))
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(config_value(config, "camera.height")))
    capture.set(cv2.CAP_PROP_FPS, float(config_value(config, "camera.fps")))
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture, used_backend


def _no_op_trackbar(_value: int) -> None:
    return None


def create_dashboard_controls(
    window_name: str, config: Dict[str, Any], mode: str
) -> None:
    mode_position = {
        MODE_HIGH: 0,
        MODE_TRANSITION: 1,
        MODE_LOW: 2,
    }[mode]
    controls = (
        ("Mode 0HIGH 1TRANS 2LOW", mode_position, 2),
        (
            "Min confidence x100",
            int(round(float(config_value(config, "circle.min_confidence")) * 100)),
            95,
        ),
        (
            "Circle ratio tolerance x100",
            int(
                round(
                    float(
                        config_value(
                            config, "circle.diameter_ratio_tolerance"
                        )
                    )
                    * 100
                )
            ),
            30,
        ),
        (
            "Concentricity limit x100R",
            int(
                round(
                    float(
                        config_value(
                            config, "circle.max_concentricity_ratio"
                        )
                    )
                    * 100
                )
            ),
            30,
        ),
        (
            "Center jump gate x100R",
            int(
                round(
                    float(
                        config_value(
                            config,
                            "tracking.max_center_jump_outer_radius_ratio",
                        )
                    )
                    * 100
                )
            ),
            100,
        ),
        (
            "Radius jump gate percent",
            int(
                round(
                    float(
                        config_value(
                            config, "tracking.max_radius_change_ratio"
                        )
                    )
                    * 100
                )
            ),
            60,
        ),
        (
            "Center EMA alpha x100",
            int(round(float(config_value(config, "filter.ema_alpha")) * 100)),
            100,
        ),
        (
            "Acquire consecutive frames",
            int(config_value(config, "tracking.acquire_frames")),
            10,
        ),
    )
    for name, value, maximum in controls:
        cv2.createTrackbar(name, window_name, value, maximum, _no_op_trackbar)


def sync_dashboard_controls(
    window_name: str, config: Dict[str, Any], mode: str
) -> None:
    positions = {
        "Mode 0HIGH 1TRANS 2LOW": {
            MODE_HIGH: 0,
            MODE_TRANSITION: 1,
            MODE_LOW: 2,
        }[mode],
        "Min confidence x100": int(
            round(float(config_value(config, "circle.min_confidence")) * 100)
        ),
        "Circle ratio tolerance x100": int(
            round(
                float(
                    config_value(config, "circle.diameter_ratio_tolerance")
                )
                * 100
            )
        ),
        "Concentricity limit x100R": int(
            round(
                float(config_value(config, "circle.max_concentricity_ratio"))
                * 100
            )
        ),
        "Center jump gate x100R": int(
            round(
                float(
                    config_value(
                        config,
                        "tracking.max_center_jump_outer_radius_ratio",
                    )
                )
                * 100
            )
        ),
        "Radius jump gate percent": int(
            round(
                float(
                    config_value(config, "tracking.max_radius_change_ratio")
                )
                * 100
            )
        ),
        "Center EMA alpha x100": int(
            round(float(config_value(config, "filter.ema_alpha")) * 100)
        ),
        "Acquire consecutive frames": int(
            config_value(config, "tracking.acquire_frames")
        ),
    }
    for name, position in positions.items():
        cv2.setTrackbarPos(name, window_name, position)


def read_dashboard_controls(
    window_name: str, config: Dict[str, Any]
) -> str:
    mode_position = cv2.getTrackbarPos(
        "Mode 0HIGH 1TRANS 2LOW", window_name
    )
    mode = (MODE_HIGH, MODE_TRANSITION, MODE_LOW)[
        min(2, max(0, mode_position))
    ]
    config["circle"]["min_confidence"] = max(
        0.50,
        cv2.getTrackbarPos("Min confidence x100", window_name) / 100.0,
    )
    config["circle"]["diameter_ratio_tolerance"] = max(
        0.03,
        cv2.getTrackbarPos(
            "Circle ratio tolerance x100", window_name
        )
        / 100.0,
    )
    config["circle"]["max_concentricity_ratio"] = max(
        0.03,
        cv2.getTrackbarPos("Concentricity limit x100R", window_name)
        / 100.0,
    )
    config["tracking"]["max_center_jump_outer_radius_ratio"] = max(
        0.05,
        cv2.getTrackbarPos("Center jump gate x100R", window_name)
        / 100.0,
    )
    config["tracking"]["max_radius_change_ratio"] = max(
        0.05,
        cv2.getTrackbarPos("Radius jump gate percent", window_name)
        / 100.0,
    )
    config["filter"]["ema_alpha"] = max(
        0.05,
        cv2.getTrackbarPos("Center EMA alpha x100", window_name)
        / 100.0,
    )
    config["tracking"]["acquire_frames"] = max(
        1,
        cv2.getTrackbarPos("Acquire consecutive frames", window_name),
    )
    return mode


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Detect the two-circle and cross UAV landing target"
    )
    parser.add_argument(
        "--camera",
        type=int,
        default=None,
        help="USB camera index; overrides camera.index in the YAML file",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("vision_config.yaml"),
        help="OpenCV YAML configuration file",
    )
    parser.add_argument(
        "--mode",
        choices=("high", "transition", "low"),
        default="high",
        help="Initial detection mode; GUI users can change it with the mode slider",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Do not create a window; useful for capture smoke tests",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Stop after N frames; 0 means run until the user quits",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    try:
        config = load_config(args.config)
    except Exception as error:
        print("Configuration error: {}".format(error), file=sys.stderr)
        return 2

    camera_index = (
        int(args.camera)
        if args.camera is not None
        else int(config_value(config, "camera.index"))
    )
    try:
        capture, backend_name = open_camera(camera_index, config)
    except Exception as error:
        print("Camera error: {}".format(error), file=sys.stderr)
        return 3

    detector = TargetDetector(config)
    mode = {
        "high": MODE_HIGH,
        "transition": MODE_TRANSITION,
        "low": MODE_LOW,
    }[args.mode]
    logger = StatusLogger(
        float(config_value(config, "logging.interval_seconds"))
    )
    actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = capture.get(cv2.CAP_PROP_FPS)
    print(
        "Camera opened: index={} backend={} resolution={}x{} reported_fps={:.1f}".format(
            camera_index, backend_name, actual_width, actual_height, actual_fps
        ),
        flush=True,
    )
    if not args.headless:
        print(
            "Four-grid dashboard enabled. Use the sliders for mode/tuning; "
            "R=reload config Q/Esc=quit",
            flush=True,
        )

    frame_count = 0
    read_failures = 0
    fps = 0.0
    last_frame_time = time.perf_counter()
    window_name = "UAV Target Vision"
    if not args.headless:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        create_dashboard_controls(window_name, config, mode)
        cv2.resizeWindow(
            window_name,
            int(config_value(config, "dashboard.pane_width")) * 2,
            int(config_value(config, "dashboard.pane_height")) * 2,
        )
    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                read_failures += 1
                print(
                    "Camera frame read failed ({}/10)".format(read_failures),
                    file=sys.stderr,
                    flush=True,
                )
                if read_failures >= 10:
                    return 4
                continue
            read_failures = 0
            frame_count += 1

            now = time.perf_counter()
            instantaneous_fps = 1.0 / max(now - last_frame_time, 1e-6)
            last_frame_time = now
            fps = (
                instantaneous_fps
                if fps <= 0.0
                else 0.12 * instantaneous_fps + 0.88 * fps
            )

            if not args.headless:
                mode = read_dashboard_controls(window_name, config)
            result = detector.detect(frame, mode)
            logger.update(result, frame.shape, fps)

            if not args.headless:
                view = render_four_grid(frame, result, fps, config)
                cv2.imshow(window_name, view)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    break
                if key in (ord("r"), ord("R")):
                    try:
                        config = load_config(args.config)
                        detector = TargetDetector(config)
                        logger = StatusLogger(
                            float(
                                config_value(
                                    config, "logging.interval_seconds"
                                )
                            )
                        )
                        sync_dashboard_controls(window_name, config, mode)
                        print(
                            "Configuration reloaded: {}".format(args.config),
                            flush=True,
                        )
                    except Exception as error:
                        print(
                            "Configuration reload failed; keeping previous values: "
                            "{}".format(error),
                            file=sys.stderr,
                            flush=True,
                        )
                try:
                    if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                        break
                except cv2.error:
                    break

            if args.max_frames > 0 and frame_count >= args.max_frames:
                break
    except KeyboardInterrupt:
        print("Interrupted by user", flush=True)
    finally:
        capture.release()
        cv2.destroyAllWindows()
        print("Camera released; processed {} frames".format(frame_count), flush=True)
    return 0


ROS_PARAMETER_PATHS = (
    "camera.index",
    "camera.width",
    "camera.height",
    "camera.fps",
    "camera.fourcc",
    "preprocess.clahe_clip_limit",
    "preprocess.clahe_grid_size",
    "preprocess.blur_kernel",
    "preprocess.adaptive_block_size",
    "preprocess.adaptive_c",
    "preprocess.morph_kernel",
    "preprocess.canny_low",
    "preprocess.canny_high",
    "circle.min_radius_ratio",
    "circle.max_radius_ratio",
    "circle.expected_diameter_ratio",
    "circle.diameter_ratio_tolerance",
    "circle.max_concentricity_ratio",
    "circle.enable_hough",
    "circle.hough_dp",
    "circle.hough_min_distance_ratio",
    "circle.hough_param1",
    "circle.hough_param2",
    "circle.min_ellipse_axis_ratio",
    "circle.min_contour_points",
    "circle.min_contour_circularity",
    "circle.min_edge_support",
    "circle.min_confidence",
    "circle.enable_cross_guided_recovery",
    "circle.cross_guided_min_visible_fraction",
    "circle.outer_edge_search_ratio",
    "circle.outer_edge_relative_support",
    "circle.require_cross_validation",
    "circle.cross_validation_distance_outer_radius_ratio",
    "circle.cross_validation_min_distance_px",
    "cross.hough_threshold",
    "cross.min_line_length_ratio",
    "cross.max_line_gap",
    "cross.max_segments",
    "cross.parallel_tolerance_deg",
    "cross.orthogonal_tolerance_deg",
    "cross.min_stroke_width_px",
    "cross.max_stroke_width_ratio",
    "cross.max_centerlines",
    "cross.intersection_margin_px",
    "cross.intersection_cluster_px",
    "cross.ray_length_ratio",
    "cross.ray_band_px",
    "cross.min_ray_support",
    "cross.min_confidence",
    "transition.inner_min_radius_ratio",
    "transition.inner_edge_separation_expected_ratio",
    "transition.inner_edge_separation_min_ratio",
    "transition.inner_edge_separation_max_ratio",
    "filter.ema_alpha",
    "filter.radius_ema_alpha",
    "tracking.acquire_frames",
    "tracking.max_missed_frames",
    "tracking.max_center_jump_outer_radius_ratio",
    "tracking.max_center_jump_px",
    "tracking.max_radius_change_ratio",
    "tracking.pending_center_tolerance_outer_radius_ratio",
    "tracking.pending_radius_tolerance_ratio",
    "dashboard.pane_width",
    "dashboard.pane_height",
    "logging.interval_seconds",
)


def ros2_main(argv: Sequence[str]) -> int:
    """Run the self-contained Orange Pi ROS2 node."""
    try:
        import rclpy
        from rcl_interfaces.msg import SetParametersResult
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from std_msgs.msg import Bool, Int16, Int32MultiArray
    except ImportError as error:
        print(
            "ROS2 Python modules are unavailable: {}. "
            "Source your ROS2 setup.bash before starting this mode.".format(error),
            file=sys.stderr,
        )
        return 5

    rclpy.init(args=list(argv))

    class OrangePiVisionNode(Node):
        def __init__(self) -> None:
            super().__init__("land_air_vision")
            self.config = embedded_config()
            for path in ROS_PARAMETER_PATHS:
                default = config_value(self.config, path)
                value = self.declare_parameter(path, default).value
                set_config_value(self.config, path, value)
            validate_config(self.config)

            self.show_dashboard = bool(
                self.declare_parameter("show_dashboard", True).value
            )
            self.opencv_threads = max(
                1, int(self.declare_parameter("opencv_threads", 4).value)
            )
            cv2.setNumThreads(self.opencv_threads)

            self.mode_value = 1
            self.fine_topic = str(
                self.declare_parameter("fine_topic", "/fine_data").value
            )
            self.visual_descent_topic = str(
                self.declare_parameter(
                    "visual_descent_topic", "/visual_descent_active"
                ).value
            )
            self.height_topic = str(
                self.declare_parameter("height_topic", "/height").value
            )
            self.low_height_threshold_cm = float(
                self.declare_parameter(
                    "low_height_threshold_cm", 70.0
                ).value
            )
            if self.low_height_threshold_cm <= 0.0:
                raise ValueError("low_height_threshold_cm must be positive")
            self.visual_descent_active = False
            self.current_height_cm: Optional[float] = None

            self.detector = TargetDetector(self.config)
            self.capture: Optional[cv2.VideoCapture] = None
            self.read_failures = 0
            self.last_log_state: Optional[Tuple[Any, ...]] = None
            self.last_log_time = 0.0
            self.window_name = "UAV Target Vision - ROS2"
            self._open_camera()
            self._set_dashboard_visible(self.show_dashboard)

            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.fine_publisher = self.create_publisher(
                Int32MultiArray, self.fine_topic, qos
            )
            self.visual_descent_subscription = self.create_subscription(
                Bool,
                self.visual_descent_topic,
                self._visual_descent_callback,
                qos,
            )
            self.height_subscription = self.create_subscription(
                Int16, self.height_topic, self._height_callback, qos
            )
            self.add_on_set_parameters_callback(self._parameter_callback)

            timer_period = 1.0 / max(
                1.0, float(config_value(self.config, "camera.fps"))
            )
            self.timer = self.create_timer(timer_period, self._frame_callback)
            self.get_logger().info(
                "Vision ready: camera={} {}x{}@{} mode={}({}) "
                "dashboard={}".format(
                    int(config_value(self.config, "camera.index")),
                    int(config_value(self.config, "camera.width")),
                    int(config_value(self.config, "camera.height")),
                    int(config_value(self.config, "camera.fps")),
                    self.mode_value,
                    MODE_VALUE_TO_NAME[self.mode_value],
                    self.show_dashboard,
                )
            )
            self.get_logger().info(
                "Automatic stages: not descending=outer+inner+cross, "
                "descending/high=inner+cross, descending/low=cross-only; "
                "low threshold={:.1f} cm".format(self.low_height_threshold_cm)
            )

        def _set_dashboard_visible(self, visible: bool) -> None:
            if visible:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(
                    self.window_name,
                    int(config_value(self.config, "camera.width")),
                    int(config_value(self.config, "camera.height")),
                )
            else:
                try:
                    cv2.destroyWindow(self.window_name)
                except cv2.error:
                    pass

        def _open_camera(self) -> None:
            if self.capture is not None:
                self.capture.release()
            index = int(config_value(self.config, "camera.index"))
            capture = cv2.VideoCapture(index, cv2.CAP_V4L2)
            if not capture.isOpened():
                capture.release()
                capture = cv2.VideoCapture(index, cv2.CAP_ANY)
            if not capture.isOpened():
                capture.release()
                raise RuntimeError(
                    "Cannot open /dev/video camera index {}".format(index)
                )
            fourcc = str(config_value(self.config, "camera.fourcc"))
            if len(fourcc) == 4:
                capture.set(
                    cv2.CAP_PROP_FOURCC,
                    cv2.VideoWriter_fourcc(*fourcc),
                )
            capture.set(
                cv2.CAP_PROP_FRAME_WIDTH,
                int(config_value(self.config, "camera.width")),
            )
            capture.set(
                cv2.CAP_PROP_FRAME_HEIGHT,
                int(config_value(self.config, "camera.height")),
            )
            capture.set(
                cv2.CAP_PROP_FPS,
                float(config_value(self.config, "camera.fps")),
            )
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.capture = capture
            self.get_logger().info(
                "Camera opened with V4L2: index={} actual={}x{} fps={:.1f}".format(
                    index,
                    int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                    int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                    capture.get(cv2.CAP_PROP_FPS),
                )
            )

        def _update_flight_vision_stage(self) -> None:
            requested = select_flight_vision_stage(
                self.visual_descent_active,
                self.current_height_cm,
                self.low_height_threshold_cm,
            )
            if requested == self.mode_value:
                return
            previous = self.mode_value
            self.mode_value = requested
            self.get_logger().info(
                "Flight vision stage changed {}({}) -> {}({}), "
                "descent={} height_cm={}".format(
                    previous,
                    MODE_VALUE_TO_NAME[previous],
                    requested,
                    MODE_VALUE_TO_NAME[requested],
                    self.visual_descent_active,
                    self.current_height_cm,
                )
            )

        def _visual_descent_callback(self, message: Bool) -> None:
            self.visual_descent_active = bool(message.data)
            self._update_flight_vision_stage()

        def _height_callback(self, message: Int16) -> None:
            self.current_height_cm = float(message.data)
            self._update_flight_vision_stage()

        def _parameter_callback(self, parameters: Sequence[Any]) -> Any:
            candidate = copy.deepcopy(self.config)
            reopen_camera = False
            recreate_timer = False
            dashboard_value = self.show_dashboard
            low_height_threshold_cm = self.low_height_threshold_cm
            for parameter in parameters:
                if parameter.name in ROS_PARAMETER_PATHS:
                    set_config_value(candidate, parameter.name, parameter.value)
                    if parameter.name.startswith("camera."):
                        reopen_camera = True
                    if parameter.name == "camera.fps":
                        recreate_timer = True
                elif parameter.name == "show_dashboard":
                    dashboard_value = bool(parameter.value)
                elif parameter.name == "low_height_threshold_cm":
                    low_height_threshold_cm = float(parameter.value)
            try:
                validate_config(candidate)
                if low_height_threshold_cm <= 0.0:
                    raise ValueError(
                        "low_height_threshold_cm must be positive"
                    )
            except Exception as error:
                return SetParametersResult(
                    successful=False, reason=str(error)
                )

            self.config = candidate
            self.detector.config = self.config
            self.detector.reset()
            self.low_height_threshold_cm = low_height_threshold_cm
            self._update_flight_vision_stage()
            if reopen_camera:
                try:
                    self._open_camera()
                except Exception as error:
                    return SetParametersResult(
                        successful=False, reason=str(error)
                    )
            if dashboard_value != self.show_dashboard:
                self.show_dashboard = dashboard_value
                self._set_dashboard_visible(self.show_dashboard)
            if recreate_timer and hasattr(self, "timer"):
                self.destroy_timer(self.timer)
                timer_period = 1.0 / max(
                    1.0, float(config_value(self.config, "camera.fps"))
                )
                self.timer = self.create_timer(
                    timer_period, self._frame_callback
                )
            return SetParametersResult(successful=True)

        def _log_result(self, result: Detection, fps: float) -> None:
            now = time.monotonic()
            state = (
                self.mode_value,
                result.valid,
                result.track_state,
                result.reason,
            )
            interval = float(
                config_value(self.config, "logging.interval_seconds")
            )
            if state == self.last_log_state and now - self.last_log_time < interval:
                return
            dx, dy = result.dx_dy(
                (
                    int(config_value(self.config, "camera.height")),
                    int(config_value(self.config, "camera.width")),
                    3,
                )
            )
            self.get_logger().info(
                "mode={} valid={} track={} dx={} dy={} confidence={:.3f} "
                "reason={} fps={:.1f}".format(
                    self.mode_value,
                    result.valid,
                    result.track_state,
                    dx,
                    dy,
                    result.confidence,
                    result.reason,
                    fps,
                )
            )
            self.last_log_state = state
            self.last_log_time = now

        def _frame_callback(self) -> None:
            if self.capture is None:
                return
            started = time.perf_counter()
            ok, frame = self.capture.read()
            if not ok or frame is None:
                self.read_failures += 1
                if self.read_failures == 1 or self.read_failures % 10 == 0:
                    self.get_logger().warning(
                        "Camera read failure {}".format(self.read_failures)
                    )
                if self.read_failures >= 10:
                    try:
                        self._open_camera()
                        self.read_failures = 0
                    except Exception as error:
                        self.get_logger().error(str(error))
                return
            self.read_failures = 0

            result = detect_commanded_frame(
                self.detector, frame, self.mode_value
            )

            processing_fps = 1.0 / max(
                time.perf_counter() - started, 1e-6
            )
            if result.valid:
                dx, dy = result.dx_dy(frame.shape)
                if dx is not None and dy is not None:
                    fine = Int32MultiArray()
                    forward_error, lateral_error = local_fine_data_from_dx_dy(
                        dx, dy
                    )
                    fine.data = [forward_error, lateral_error]
                    self.fine_publisher.publish(fine)
            self._log_result(result, processing_fps)

            if self.show_dashboard:
                annotated = draw_overlay(
                    frame,
                    result,
                    processing_fps,
                    "Auto flight stage 1/2/3   Q/Esc quit",
                    draw_candidate_segments=False,
                )
                cv2.imshow(self.window_name, annotated)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    self.get_logger().info("Dashboard requested shutdown")
                    rclpy.shutdown()

        def destroy_node(self) -> bool:
            if self.capture is not None:
                self.capture.release()
                self.capture = None
            cv2.destroyAllWindows()
            return super().destroy_node()

    node: Optional[OrangePiVisionNode] = None
    try:
        node = OrangePiVisionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        print("ROS2 vision node failed: {}".format(error), file=sys.stderr)
        return 6
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    if "--ros2" in sys.argv[1:]:
        ros_arguments = [argument for argument in sys.argv if argument != "--ros2"]
        sys.exit(ros2_main(ros_arguments))
    sys.exit(main())
