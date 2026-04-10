import queue
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass

import serial


PORT = "COM4"
BAUD = 115200
WINDOW_WIDTH = 900
WINDOW_HEIGHT = 700
MAX_X_CM = 300.0
MAX_Y_CM = 600.0
TARGET_COLORS = ["#ff5a36", "#2aa8ff", "#ffd23f"]


@dataclass
class Target:
    valid: bool
    x_cm: float
    y_cm: float
    speed_cm_s: float
    gate_mm: int


def parse_plot_line(line: str) -> list[Target] | None:
    if not line.startswith("PLOT,"):
        return None

    parts = line.strip().split(",")
    if len(parts) != 1 + 5 * 3:
        return None

    targets: list[Target] = []
    for index in range(1, len(parts), 5):
        valid = parts[index] == "1"
        x_cm = float(parts[index + 1])
        y_cm = float(parts[index + 2])
        speed_cm_s = float(parts[index + 3])
        gate_mm = int(parts[index + 4])
        targets.append(Target(valid, x_cm, y_cm, speed_cm_s, gate_mm))
    return targets


class RadarViewer:
    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self.data_queue: queue.Queue[list[Target]] = queue.Queue()
        self.latest_targets = [Target(False, 0.0, 0.0, 0.0, 0) for _ in range(3)]
        self.last_frame_at = 0.0
        self.running = True
        self.status_text = f"Connecting to {port} @ {baud}..."

        self.root = tk.Tk()
        self.root.title(f"LD2450 Viewer - {port}")
        self.root.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        self.root.configure(bg="#101418")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.header = tk.Label(
            self.root,
            text="LD2450 Realtime Viewer",
            font=("Segoe UI", 18, "bold"),
            fg="#f3f7fa",
            bg="#101418",
        )
        self.header.pack(pady=(12, 4))

        self.status_label = tk.Label(
            self.root,
            text=self.status_text,
            font=("Consolas", 11),
            fg="#8fb3c9",
            bg="#101418",
        )
        self.status_label.pack(pady=(0, 8))

        self.canvas = tk.Canvas(
            self.root,
            width=WINDOW_WIDTH - 40,
            height=WINDOW_HEIGHT - 190,
            bg="#07111a",
            highlightthickness=0,
        )
        self.canvas.pack(padx=20, pady=8, fill="both", expand=True)

        self.info_labels: list[tk.Label] = []
        info_frame = tk.Frame(self.root, bg="#101418")
        info_frame.pack(fill="x", padx=20, pady=(4, 16))
        for i, color in enumerate(TARGET_COLORS):
            label = tk.Label(
                info_frame,
                text=f"T{i + 1}: no target",
                font=("Consolas", 12),
                fg=color,
                bg="#101418",
                anchor="w",
            )
            label.pack(fill="x")
            self.info_labels.append(label)

        self.serial_thread = threading.Thread(target=self.serial_worker, daemon=True)
        self.serial_thread.start()
        self.root.after(50, self.refresh)

    def serial_worker(self) -> None:
        while self.running:
            try:
                with serial.Serial(self.port, self.baud, timeout=1) as ser:
                    self.status_text = (
                        f"Connected to {self.port} @ {self.baud}. Waiting for PLOT frames..."
                    )
                    while self.running:
                        line = ser.readline().decode("utf-8", errors="ignore").strip()
                        targets = parse_plot_line(line)
                        if targets is not None:
                            self.data_queue.put(targets)
            except Exception as exc:
                self.status_text = f"Serial error: {exc}. Retrying..."
                time.sleep(1.0)

    def close(self) -> None:
        self.running = False
        self.root.destroy()

    def map_x(self, x_cm: float, width: int) -> float:
        half_width = width / 2
        return half_width + (x_cm / MAX_X_CM) * (half_width - 30)

    def map_y(self, y_cm: float, height: int) -> float:
        padded_height = height - 40
        clamped = max(0.0, min(MAX_Y_CM, y_cm))
        return padded_height - (clamped / MAX_Y_CM) * (padded_height - 30)

    def draw_background(self, width: int, height: int) -> None:
        self.canvas.delete("all")
        center_x = width / 2
        bottom_y = height - 20

        self.canvas.create_rectangle(0, 0, width, height, fill="#07111a", outline="")
        self.canvas.create_line(center_x, 20, center_x, bottom_y, fill="#27516d", dash=(4, 6))

        for step in range(1, 6):
            y = bottom_y - step * ((bottom_y - 20) / 6)
            self.canvas.create_line(20, y, width - 20, y, fill="#173447", dash=(2, 8))
            distance = int(MAX_Y_CM * step / 6)
            self.canvas.create_text(
                52,
                y - 8,
                text=f"{distance} cm",
                fill="#5e86a0",
                font=("Consolas", 10),
            )

        self.canvas.create_line(20, bottom_y, width - 20, bottom_y, fill="#6bb6e6", width=2)
        self.canvas.create_text(center_x, bottom_y + 12, text="ESP32 / Radar", fill="#9fdcff", font=("Segoe UI", 10))
        self.canvas.create_text(45, bottom_y + 12, text="-X", fill="#5e86a0", font=("Consolas", 10))
        self.canvas.create_text(width - 45, bottom_y + 12, text="+X", fill="#5e86a0", font=("Consolas", 10))

    def draw_targets(self, width: int, height: int) -> None:
        for index, target in enumerate(self.latest_targets):
            if not target.valid:
                self.info_labels[index].config(text=f"T{index + 1}: no target")
                continue

            x = self.map_x(target.x_cm, width)
            y = self.map_y(target.y_cm, height)
            color = TARGET_COLORS[index]
            radius = 10

            self.canvas.create_oval(x - 18, y - 18, x + 18, y + 18, fill="", outline=color, width=2)
            self.canvas.create_oval(x - radius, y - radius, x + radius, y + radius, fill=color, outline="")
            self.canvas.create_text(
                x,
                y - 22,
                text=f"T{index + 1}",
                fill=color,
                font=("Segoe UI", 10, "bold"),
            )

            self.info_labels[index].config(
                text=(
                    f"T{index + 1}: X={target.x_cm:6.1f} cm  "
                    f"Y={target.y_cm:6.1f} cm  "
                    f"Spd={target.speed_cm_s:6.1f} cm/s  "
                    f"Gate={target.gate_mm:4d} mm"
                )
            )

    def refresh(self) -> None:
        updated = False
        while not self.data_queue.empty():
            self.latest_targets = self.data_queue.get_nowait()
            self.last_frame_at = time.time()
            updated = True

        width = max(self.canvas.winfo_width(), 100)
        height = max(self.canvas.winfo_height(), 100)
        self.draw_background(width, height)
        self.draw_targets(width, height)

        age_ms = int((time.time() - self.last_frame_at) * 1000) if self.last_frame_at else 0
        if updated:
            self.status_text = (
                f"Connected to {self.port} @ {self.baud}  |  last frame {age_ms} ms ago"
            )
        elif self.last_frame_at:
            self.status_text = (
                f"Connected to {self.port} @ {self.baud}  |  last frame {age_ms} ms ago"
            )

        self.status_label.config(text=self.status_text)

        self.root.after(50, self.refresh)

    def run(self) -> None:
        self.root.mainloop()


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else PORT
    RadarViewer(port, BAUD).run()
