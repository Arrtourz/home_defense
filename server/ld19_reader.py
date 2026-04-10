from __future__ import annotations

import argparse
import json
import math
import struct
import time
import urllib.error
import urllib.request
from dataclasses import dataclass

import serial

try:
    from server.ld19_shape_gate_plugin import ShapeGatePlugin
except ModuleNotFoundError:
    from ld19_shape_gate_plugin import ShapeGatePlugin


HEADER = 0x54
VER_LEN = 0x2C
POINTS_PER_PACKET = 12
FRAME_LENGTH = 47

MIN_DISTANCE_M = 0.12
MAX_DISTANCE_M = 10.0
MIN_INTENSITY = 8
MAX_TRACK_MISSES = 8
TRACK_CONFIRM_HITS = 2

CrcTable = [
    0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25,
    0x8B, 0xC6, 0x11, 0x5C, 0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07,
    0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5, 0x1F, 0x52, 0x85, 0xC8,
    0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
    0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93,
    0x3D, 0x70, 0xA7, 0xEA, 0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90,
    0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62, 0x97, 0xDA, 0x0D, 0x40,
    0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
    0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04,
    0xAA, 0xE7, 0x30, 0x7D, 0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26,
    0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4, 0x7C, 0x31, 0xE6, 0xAB,
    0x05, 0x48, 0x9F, 0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
    0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B, 0x27, 0x6A, 0xBD, 0xF0,
    0x5E, 0x13, 0xC4, 0x89, 0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD,
    0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F, 0xCA, 0x87, 0x50, 0x1D,
    0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
    0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A, 0x67,
    0xC9, 0x84, 0x53, 0x1E, 0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7, 0x5D, 0x10, 0xC7, 0x8A,
    0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
    0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1,
    0x7F, 0x32, 0xE5, 0xA8,
]


@dataclass
class Point:
    angle_deg: float
    distance_m: float
    intensity: int
    x_m: float
    y_m: float


@dataclass
class Packet:
    speed_hz: float
    start_angle_deg: float
    end_angle_deg: float
    timestamp_ms: int
    points: list[Point]


@dataclass
class Detection:
    x_m: float
    y_m: float
    point_count: int
    radius_m: float
    width_m: float
    height_m: float
    extent_m: float
    aspect_ratio: float


@dataclass
class AxisKalman:
    pos: float
    vel: float = 0.0
    p00: float = 1.0
    p01: float = 0.0
    p10: float = 0.0
    p11: float = 1.0

    def predict(self, dt: float, accel_var: float = 1.4) -> None:
        self.pos = self.pos + self.vel * dt
        p00 = self.p00 + dt * (self.p10 + self.p01) + dt * dt * self.p11
        p01 = self.p01 + dt * self.p11
        p10 = self.p10 + dt * self.p11
        p11 = self.p11

        q00 = 0.25 * dt * dt * dt * dt * accel_var
        q01 = 0.5 * dt * dt * dt * accel_var
        q11 = dt * dt * accel_var

        self.p00 = p00 + q00
        self.p01 = p01 + q01
        self.p10 = p10 + q01
        self.p11 = p11 + q11

    def update(self, measurement: float, measurement_var: float = 0.05) -> None:
        innovation = measurement - self.pos
        s = self.p00 + measurement_var
        if s <= 0:
            return

        k0 = self.p00 / s
        k1 = self.p10 / s

        self.pos = self.pos + k0 * innovation
        self.vel = self.vel + k1 * innovation

        p00 = (1 - k0) * self.p00
        p01 = (1 - k0) * self.p01
        p10 = self.p10 - k1 * self.p00
        p11 = self.p11 - k1 * self.p01

        self.p00 = p00
        self.p01 = p01
        self.p10 = p10
        self.p11 = p11


@dataclass
class Track:
    track_id: int
    x_filter: AxisKalman
    y_filter: AxisKalman
    hits: int
    misses: int
    last_update_ms: int
    point_count: int
    radius_m: float
    anchor_x_m: float
    anchor_y_m: float
    peak_displacement_m: float = 0.0

    @property
    def x_m(self) -> float:
        return self.x_filter.pos

    @property
    def y_m(self) -> float:
        return self.y_filter.pos

    @property
    def vx_m_s(self) -> float:
        return self.x_filter.vel

    @property
    def vy_m_s(self) -> float:
        return self.y_filter.vel

    @property
    def speed_m_s(self) -> float:
        return math.hypot(self.vx_m_s, self.vy_m_s)

    def predict(self, now_ms: int) -> None:
        dt = max(0.02, min(0.25, (now_ms - self.last_update_ms) / 1000.0))
        self.x_filter.predict(dt)
        self.y_filter.predict(dt)

    def update(self, detection: Detection, now_ms: int) -> None:
        dt = max(0.02, min(0.25, (now_ms - self.last_update_ms) / 1000.0))
        self.x_filter.predict(dt)
        self.y_filter.predict(dt)
        self.x_filter.update(detection.x_m)
        self.y_filter.update(detection.y_m)
        self.hits += 1
        self.misses = 0
        self.last_update_ms = now_ms
        self.point_count = detection.point_count
        self.radius_m = detection.radius_m
        self.peak_displacement_m = max(
            self.peak_displacement_m,
            math.hypot(self.x_m - self.anchor_x_m, self.y_m - self.anchor_y_m),
        )

    def miss(self) -> None:
        self.misses += 1


def crc8(data: bytes) -> int:
    crc = 0
    for value in data:
        crc = CrcTable[(crc ^ value) & 0xFF]
    return crc


def parse_packet(frame: bytes) -> Packet | None:
    if len(frame) != FRAME_LENGTH:
        return None
    if frame[0] != HEADER or frame[1] != VER_LEN:
        return None
    if crc8(frame[:-1]) != frame[-1]:
        return None

    speed_raw = struct.unpack_from("<H", frame, 2)[0]
    start_angle_deg = struct.unpack_from("<H", frame, 4)[0] / 100.0
    end_angle_deg = struct.unpack_from("<H", frame, 42)[0] / 100.0
    timestamp_ms = struct.unpack_from("<H", frame, 44)[0]

    angle_end = end_angle_deg
    if angle_end < start_angle_deg:
        angle_end += 360.0
    step = 0.0 if POINTS_PER_PACKET <= 1 else (angle_end - start_angle_deg) / (POINTS_PER_PACKET - 1)

    points: list[Point] = []
    offset = 6
    for index in range(POINTS_PER_PACKET):
        distance_mm = struct.unpack_from("<H", frame, offset)[0]
        intensity = frame[offset + 2]
        angle_deg = (start_angle_deg + step * index) % 360.0
        angle_rad = math.radians(angle_deg)
        distance_m = distance_mm / 1000.0
        points.append(
            Point(
                angle_deg=round(angle_deg, 2),
                distance_m=distance_m,
                intensity=intensity,
                x_m=round(math.sin(angle_rad) * distance_m, 4),
                y_m=round(math.cos(angle_rad) * distance_m, 4),
            )
        )
        offset += 3

    return Packet(
        speed_hz=round(speed_raw / 360.0, 2),
        start_angle_deg=start_angle_deg,
        end_angle_deg=end_angle_deg,
        timestamp_ms=timestamp_ms,
        points=points,
    )


def euclidean(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def filter_points(points: list[Point]) -> list[Point]:
    return [
        point
        for point in points
        if MIN_DISTANCE_M <= point.distance_m <= MAX_DISTANCE_M and point.intensity >= MIN_INTENSITY
    ]


def cluster_points(
    points: list[Point],
    *,
    max_gap_m: float,
    min_points: int,
    max_angle_gap_deg: float = 4.0,
) -> list[Detection]:
    if not points:
        return []

    ordered = sorted(points, key=lambda point: point.angle_deg)
    clusters: list[list[Point]] = []
    current = [ordered[0]]

    for point in ordered[1:]:
        previous = current[-1]
        angle_gap = point.angle_deg - previous.angle_deg
        gap_m = euclidean((point.x_m, point.y_m), (previous.x_m, previous.y_m))
        if angle_gap <= max_angle_gap_deg and gap_m <= max_gap_m:
            current.append(point)
            continue
        clusters.append(current)
        current = [point]
    clusters.append(current)

    if len(clusters) > 1:
        first = clusters[0][0]
        last = clusters[-1][-1]
        wrap_angle_gap = (first.angle_deg + 360.0) - last.angle_deg
        wrap_gap_m = euclidean((first.x_m, first.y_m), (last.x_m, last.y_m))
        if wrap_angle_gap <= max_angle_gap_deg and wrap_gap_m <= max_gap_m:
            clusters[0] = clusters[-1] + clusters[0]
            clusters.pop()

    detections: list[Detection] = []
    for cluster in clusters:
        if len(cluster) < min_points:
            continue
        centroid_x = sum(point.x_m for point in cluster) / len(cluster)
        centroid_y = sum(point.y_m for point in cluster) / len(cluster)
        min_x = min(point.x_m for point in cluster)
        max_x = max(point.x_m for point in cluster)
        min_y = min(point.y_m for point in cluster)
        max_y = max(point.y_m for point in cluster)
        width_m = max_x - min_x
        height_m = max_y - min_y
        short_side = max(0.02, min(width_m, height_m))
        long_side = max(width_m, height_m)
        radius = max(
            math.hypot(point.x_m - centroid_x, point.y_m - centroid_y)
            for point in cluster
        )
        detections.append(
            Detection(
                x_m=round(centroid_x, 4),
                y_m=round(centroid_y, 4),
                point_count=len(cluster),
                radius_m=round(radius, 3),
                width_m=round(width_m, 3),
                height_m=round(height_m, 3),
                extent_m=round(max(width_m, height_m), 3),
                aspect_ratio=round(long_side / short_side, 3),
            )
        )
    return detections


class SimpleTracker:
    def __init__(self, *, association_distance_m: float) -> None:
        self.association_distance_m = association_distance_m
        self.tracks: list[Track] = []
        self.next_track_id = 1

    def update(self, detections: list[Detection], now_ms: int) -> list[Track]:
        for track in self.tracks:
            track.predict(now_ms)

        unmatched_tracks = set(range(len(self.tracks)))
        unmatched_detections = set(range(len(detections)))
        matches: list[tuple[int, int]] = []

        while unmatched_tracks and unmatched_detections:
            best_pair: tuple[int, int] | None = None
            best_distance = self.association_distance_m
            for track_index in unmatched_tracks:
                track = self.tracks[track_index]
                for detection_index in unmatched_detections:
                    detection = detections[detection_index]
                    distance = euclidean((track.x_m, track.y_m), (detection.x_m, detection.y_m))
                    if distance > best_distance:
                        continue
                    best_distance = distance
                    best_pair = (track_index, detection_index)
            if best_pair is None:
                break
            matches.append(best_pair)
            unmatched_tracks.discard(best_pair[0])
            unmatched_detections.discard(best_pair[1])

        for track_index, detection_index in matches:
            self.tracks[track_index].update(detections[detection_index], now_ms)

        for track_index in unmatched_tracks:
            self.tracks[track_index].miss()

        self.tracks = [track for track in self.tracks if track.misses <= MAX_TRACK_MISSES]

        for detection_index in unmatched_detections:
            detection = detections[detection_index]
            self.tracks.append(
                Track(
                    track_id=self.next_track_id,
                    x_filter=AxisKalman(detection.x_m),
                    y_filter=AxisKalman(detection.y_m),
                    hits=1,
                    misses=0,
                    last_update_ms=now_ms,
                    point_count=detection.point_count,
                    radius_m=detection.radius_m,
                    anchor_x_m=detection.x_m,
                    anchor_y_m=detection.y_m,
                )
            )
            self.next_track_id += 1

        return self.tracks


class BackgroundSubtractor:
    def __init__(self, bin_count: int = 360) -> None:
        self.bin_count = bin_count
        self.background: list[float | None] = [None] * bin_count
        self.mismatch_count = [0] * bin_count

    def foreground_points(self, points: list[Point]) -> list[Point]:
        foreground: list[Point] = []
        for point in points:
            bin_index = int(point.angle_deg) % self.bin_count
            background_distance = self.background[bin_index]
            if background_distance is None:
                self.background[bin_index] = point.distance_m
                continue

            delta = abs(point.distance_m - background_distance)
            if delta <= 0.16:
                self.background[bin_index] = (background_distance * 0.88) + (point.distance_m * 0.12)
                self.mismatch_count[bin_index] = 0
                continue

            if delta >= 0.28:
                foreground.append(point)
                self.mismatch_count[bin_index] += 1
                if self.mismatch_count[bin_index] > 45:
                    self.background[bin_index] = (background_distance * 0.98) + (point.distance_m * 0.02)
                continue

            self.mismatch_count[bin_index] += 1
            if self.mismatch_count[bin_index] > 12:
                self.background[bin_index] = (background_distance * 0.97) + (point.distance_m * 0.03)
        return foreground


class ScanDiffDetector:
    def __init__(self, bin_count: int = 360) -> None:
        self.bin_count = bin_count
        self.previous: list[float | None] = [None] * bin_count

    def foreground_points(self, points: list[Point]) -> list[Point]:
        foreground: list[Point] = []
        current: list[float | None] = [None] * self.bin_count
        for point in points:
            bin_index = int(point.angle_deg) % self.bin_count
            previous_distance = self.previous[bin_index]
            current[bin_index] = point.distance_m
            if previous_distance is None:
                continue
            if abs(point.distance_m - previous_distance) >= 0.22:
                foreground.append(point)
        self.previous = current
        return foreground


class MotionPipeline:
    def __init__(
        self,
        *,
        detector,
        shape_gate: ShapeGatePlugin,
        cluster_gap_m: float,
        min_cluster_points: int,
        association_distance_m: float,
    ) -> None:
        self.detector = detector
        self.shape_gate = shape_gate
        self.cluster_gap_m = cluster_gap_m
        self.min_cluster_points = min_cluster_points
        self.tracker = SimpleTracker(
            association_distance_m=association_distance_m,
        )

    def update(self, points: list[Point], now_ms: int) -> list[Track]:
        filtered = filter_points(points)
        foreground = self.detector.foreground_points(filtered)
        detections = cluster_points(
            foreground,
            max_gap_m=self.cluster_gap_m,
            min_points=self.min_cluster_points,
        )
        detections = self.shape_gate.filter_detections(detections)
        tracks = self.tracker.update(detections, now_ms)
        return [track for track in tracks if self.shape_gate.should_emit_track(track)]


class ScanAccumulator:
    def __init__(self) -> None:
        self.points: list[Point] = []
        self.packet_speeds_hz: list[float] = []
        self.last_start_angle_deg: float | None = None
        self.last_scan_monotonic: float | None = None
        self.ema_scan_interval_ms: float | None = None

    def add_packet(self, packet: Packet) -> dict[str, object] | None:
        wrapped = (
            self.last_start_angle_deg is not None
            and packet.start_angle_deg + 2.0 < self.last_start_angle_deg
        )

        completed_scan: dict[str, object] | None = None
        if wrapped and self.points:
            completed_scan = self.finalize_scan(packet.timestamp_ms)
            self.points = []
            self.packet_speeds_hz = []

        self.points.extend(packet.points)
        self.packet_speeds_hz.append(packet.speed_hz)
        self.last_start_angle_deg = packet.start_angle_deg
        return completed_scan

    def finalize_scan(self, timestamp_ms: int) -> dict[str, object]:
        now = time.monotonic()
        if self.last_scan_monotonic is not None:
            interval_ms = max(1.0, (now - self.last_scan_monotonic) * 1000.0)
            if self.ema_scan_interval_ms is None:
                self.ema_scan_interval_ms = interval_ms
            else:
                self.ema_scan_interval_ms = (self.ema_scan_interval_ms * 0.65) + (interval_ms * 0.35)
        self.last_scan_monotonic = now

        valid_points = [point for point in self.points if point.distance_m > 0.03]
        scan_rate_hz = 0.0 if not self.ema_scan_interval_ms else 1000.0 / self.ema_scan_interval_ms
        speed_hz = sum(self.packet_speeds_hz) / len(self.packet_speeds_hz) if self.packet_speeds_hz else 0.0
        ordered_points = sorted(valid_points, key=lambda point: point.angle_deg)
        return {
            "ts_ms": int(time.time() * 1000),
            "device_timestamp_ms": timestamp_ms,
            "speed_hz": round(speed_hz, 2),
            "scan_rate_hz": round(scan_rate_hz, 2),
            "point_count": len(self.points),
            "valid_count": len(valid_points),
            "max_distance_m": 12.0,
            "points": [
                {
                    "angle_deg": point.angle_deg,
                    "distance_m": round(point.distance_m, 4),
                    "intensity": point.intensity,
                    "x_m": point.x_m,
                    "y_m": point.y_m,
                }
                for point in ordered_points
            ],
            "_points": ordered_points,
        }


class Ld19Reader:
    def __init__(self, port: str, baudrate: int) -> None:
        self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=1)

    def read_packet(self) -> Packet | None:
        while True:
            header = self.serial.read(1)
            if not header:
                return None
            if header[0] != HEADER:
                continue
            remainder = self.serial.read(FRAME_LENGTH - 1)
            if len(remainder) != FRAME_LENGTH - 1:
                return None
            packet = parse_packet(header + remainder)
            if packet:
                return packet


def post_json(url: str, payload: dict[str, object]) -> bool:
    data = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=3) as response:
            return 200 <= response.status < 300
    except (urllib.error.URLError, TimeoutError) as error:
        print(f"[ld19] post failed: {error}")
        return False


def post_scan(server_url: str, node_id: str, scan: dict[str, object]) -> bool:
    payload = {
        "node_id": node_id,
        "source": "ld19",
        "ts_ms": scan["ts_ms"],
        "speed_hz": scan["speed_hz"],
        "scan_rate_hz": scan["scan_rate_hz"],
        "point_count": scan["point_count"],
        "valid_count": scan["valid_count"],
        "max_distance_m": scan["max_distance_m"],
        "points": scan["points"],
    }
    return post_json(f"{server_url.rstrip('/')}/api/lidar/{node_id}", payload)


def post_targets(server_url: str, node_id: str, ts_ms: int, tracks: list[Track]) -> bool:
    payload = {
        "node_id": node_id,
        "ts_ms": ts_ms,
        "targets": [
            {
                "target_id": track.track_id,
                "x_m": round(track.x_m, 3),
                "y_m": round(track.y_m, 3),
                "speed_m_s": round(track.speed_m_s, 3),
                "gate_m": round(track.radius_m, 3),
            }
            for track in tracks
        ],
    }
    return post_json(f"{server_url.rstrip('/')}/api/radar/{node_id}", payload)


def main() -> int:
    parser = argparse.ArgumentParser(description="Read, parse, visualize, and track LD19 LiDAR scans.")
    parser.add_argument("port", nargs="?", default="COM4")
    parser.add_argument("--baudrate", type=int, default=230400)
    parser.add_argument("--server-url", default="http://127.0.0.1:8000")
    parser.add_argument("--node-id", default="ld19_backyard")
    args = parser.parse_args()

    reader = Ld19Reader(args.port, args.baudrate)
    accumulator = ScanAccumulator()
    motion_pipeline = MotionPipeline(
        detector=BackgroundSubtractor(),
        shape_gate=ShapeGatePlugin(),
        cluster_gap_m=0.50,
        min_cluster_points=3,
        association_distance_m=0.95,
    )
    baseline_pipeline = MotionPipeline(
        detector=ScanDiffDetector(),
        shape_gate=ShapeGatePlugin(
            min_cluster_points=4,
            min_cluster_width_m=0.08,
            min_cluster_height_m=0.08,
            min_cluster_extent_m=0.14,
            max_aspect_ratio=6.0,
            min_track_hits=2,
            min_track_displacement_m=0.12,
            min_track_speed_m_s=0.08,
        ),
        cluster_gap_m=0.42,
        min_cluster_points=3,
        association_distance_m=0.90,
    )

    scan_count = 0
    print(f"[ld19] reading {args.port} @ {args.baudrate}")
    print(f"[ld19] raw scan node: {args.node_id}")
    print(f"[ld19] motion node: {args.node_id}__motion")
    print(f"[ld19] baseline node: {args.node_id}__baseline")

    while True:
        packet = reader.read_packet()
        if packet is None:
            continue

        completed_scan = accumulator.add_packet(packet)
        if not completed_scan:
            continue

        scan_count += 1
        now_ms = int(completed_scan["ts_ms"])
        points = list(completed_scan["_points"])
        motion_tracks = motion_pipeline.update(points, now_ms)
        baseline_tracks = baseline_pipeline.update(points, now_ms)

        raw_ok = post_scan(args.server_url, args.node_id, completed_scan)
        motion_ok = post_targets(args.server_url, f"{args.node_id}__motion", now_ms, motion_tracks)
        baseline_ok = post_targets(args.server_url, f"{args.node_id}__baseline", now_ms, baseline_tracks)

        if scan_count % 10 == 0:
            print(
                "[ld19] scans=%d valid=%d raw=%s motion=%d/%s baseline=%d/%s rate=%.2fHz"
                % (
                    scan_count,
                    completed_scan["valid_count"],
                    "yes" if raw_ok else "no",
                    len(motion_tracks),
                    "yes" if motion_ok else "no",
                    len(baseline_tracks),
                    "yes" if baseline_ok else "no",
                    completed_scan["scan_rate_hz"],
                )
            )


if __name__ == "__main__":
    raise SystemExit(main())
