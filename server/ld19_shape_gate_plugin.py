from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


class DetectionLike(Protocol):
    point_count: int
    width_m: float
    height_m: float
    extent_m: float
    aspect_ratio: float


class TrackLike(Protocol):
    hits: int
    peak_displacement_m: float
    speed_m_s: float


@dataclass
class ShapeGatePlugin:
    min_cluster_points: int = 4
    min_cluster_width_m: float = 0.10
    min_cluster_height_m: float = 0.10
    min_cluster_extent_m: float = 0.18
    max_aspect_ratio: float = 4.8
    min_track_hits: int = 3
    min_track_displacement_m: float = 0.18
    min_track_speed_m_s: float = 0.05

    def filter_detections(self, detections: list[DetectionLike]) -> list[DetectionLike]:
        kept: list[DetectionLike] = []
        for detection in detections:
            if detection.point_count < self.min_cluster_points:
                continue
            if detection.width_m < self.min_cluster_width_m and detection.height_m < self.min_cluster_height_m:
                continue
            if detection.extent_m < self.min_cluster_extent_m:
                continue
            if detection.aspect_ratio > self.max_aspect_ratio:
                continue
            kept.append(detection)
        return kept

    def should_emit_track(self, track: TrackLike) -> bool:
        if track.hits < self.min_track_hits:
            return False
        if track.peak_displacement_m < self.min_track_displacement_m:
            return False
        if track.speed_m_s < self.min_track_speed_m_s:
            return False
        return True
