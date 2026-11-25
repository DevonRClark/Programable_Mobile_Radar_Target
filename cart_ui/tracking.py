"""Read downloaded telemetry and display the controls tracking plot in Qt."""

import csv
import json
import math
from pathlib import Path

from PySide6.QtCore import QSize, Qt
from PySide6.QtWidgets import QDialog, QFileDialog, QHBoxLayout, QLabel, QPushButton, QVBoxLayout
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure


TARGET = "#b74d14"
MEASURED = "#096b95"
ERROR = "#7047a0"
THROTTLE = "#146d61"
INK = "#15273a"
MUTED = "#485c6e"
GRID = "#dce4eb"

REQUIRED = ("t_us", "target_mps", "speed_mps")
OPTIONAL = ("throttle_pct", "position_m")


def read_recording(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        headers = reader.fieldnames or []
        missing = [name for name in REQUIRED if name not in headers]
        if missing:
            raise ValueError(f"Missing CSV columns: {', '.join(missing)}")

        columns = {name: [] for name in (*REQUIRED, *OPTIONAL) if name in headers}
        for row in reader:
            for name, values in columns.items():
                try:
                    value = int(row[name]) if name == "t_us" else float(row[name])
                    if not math.isfinite(value):
                        raise ValueError
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"Invalid {name} on CSV line {reader.line_num}") from exc
                values.append(value)

    timestamps = columns["t_us"]
    if len(timestamps) < 2 or timestamps[-1] == timestamps[0]:
        raise ValueError("Need at least two different timestamps.")
    if any(right < left for left, right in zip(timestamps, timestamps[1:])):
        raise ValueError("Timestamps go backwards. Use one recording per CSV.")
    return columns


def read_profile(path):
    """Recreate the UI preview using linear interpolation of speed squared."""
    points = json.loads(path.read_text(encoding="utf-8-sig"))["points"]
    positions = [float(point["position_m"]) for point in points]
    speeds = [float(point["speed_mps"]) for point in points]
    if (len(points) < 2 or any(not math.isfinite(v) or v < 0 for v in positions + speeds)
            or any(right <= left for left, right in zip(positions, positions[1:]))):
        raise ValueError("Profile needs increasing positions and nonnegative finite speeds.")

    x, v = [], []
    for x0, x1, v0, v1 in zip(positions, positions[1:], speeds, speeds[1:]):
        for step in range(50):
            fraction = step / 50
            x.append(x0 + fraction * (x1 - x0))
            v.append(math.sqrt((1 - fraction) * v0**2 + fraction * v1**2))
    return x + [positions[-1]], v + [speeds[-1]]


def tracking_rmse(time, target, error):
    # Weight by elapsed time because event rows can occur between periodic samples.
    # Include intervals with a nonzero target at both ends and exclude the coast.
    duration, squared_error = 0.0, 0.0
    for i in range(1, len(time)):
        if target[i - 1] != 0 and target[i] != 0:
            dt = time[i] - time[i - 1]
            duration += dt
            squared_error += dt * (error[i - 1] ** 2 + error[i] ** 2) / 2
    return math.sqrt(squared_error / duration) if duration > 0 else None


def plot_speed(ax, x, target, measured):
    ax.plot(x, target, color=TARGET, linewidth=2.1, linestyle="--", label="Logged setpoint")
    ax.plot(x, measured, color=MEASURED, linewidth=2.0, marker="o", markersize=3.2,
            markeredgewidth=0, label="Measured speed")
    ax.set_ylabel("Speed (m/s)")


def style_axis(ax, title):
    ax.set_title(title, loc="left", fontsize=12, fontweight="bold", color=INK, pad=8)
    ax.set_axisbelow(True)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.tick_params(colors=MUTED, length=0, pad=6)
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.xaxis.label.set_color(INK)
    ax.yaxis.label.set_color(INK)


def draw_tracking(fig, path, columns, profile):
    target, measured = columns["target_mps"], columns["speed_mps"]
    active = [i for i, value in enumerate(target) if value != 0]
    start = active[0] if active else 0
    time = [(stamp - columns["t_us"][start]) / 1e6 for stamp in columns["t_us"]]
    error = [actual - command for actual, command in zip(measured, target)]
    rmse = tracking_rmse(time, target, error)
    has_position = "position_m" in columns and bool(active)
    has_throttle = "throttle_pct" in columns

    fig.clear()
    fig.subplots_adjust(left=0.09, right=0.97, top=0.80, bottom=0.12, hspace=0.61, wspace=0.27)
    grid = fig.add_gridspec(2 if has_position or not has_throttle else 3,
                            2 if has_position else 1)
    speed_ax = fig.add_subplot(grid[0, 0])
    plot_speed(speed_ax, time, target, measured)
    style_axis(speed_ax, "Speed response")
    speed_ax.legend(loc="upper right", frameon=False, fontsize=9)
    time_label = "Time since first nonzero setpoint (s)" if active else "Time since recording start (s)"
    speed_ax.set_xlabel(time_label)

    error_ax = fig.add_subplot(grid[1, :] if has_position and not has_throttle else grid[1, 0],
                               sharex=speed_ax)
    error_ax.axhline(0, color=MUTED, linewidth=0.9)
    if active:
        error_ax.plot(time, error, color=ERROR, linewidth=1.8)
    else:
        error_ax.text(0.5, 0.5, "No nonzero speed setpoint",
                      transform=error_ax.transAxes, ha="center", color=MUTED)
    error_ax.set(xlabel=time_label, ylabel="Error (m/s)")
    style_axis(error_ax, "Tracking error  ·  measured − setpoint")
    time_axes = [speed_ax, error_ax]

    if has_position:
        position_ax = fig.add_subplot(grid[0, 1])
        position = [p - columns["position_m"][start] for p in columns["position_m"][start:]]
        plot_speed(position_ax, position, target[start:], measured[start:])
        position_ax.set_xlabel("Corrected position relative to start (m)")
        style_axis(position_ax, "Speed along the track")
        if profile is not None:
            position_ax.plot(*profile, color=MUTED, linewidth=1.5, linestyle=":",
                             label="UI waypoint profile")
            position_ax.legend(loc="upper right", frameon=False, fontsize=9)

    if has_throttle:
        throttle_ax = fig.add_subplot(grid[1, 1] if has_position else grid[2, 0], sharex=speed_ax)
        throttle_ax.plot(time, columns["throttle_pct"], color=THROTTLE,
                         linewidth=1.8, drawstyle="steps-post")
        throttle_ax.set(xlabel=time_label, ylabel="Throttle (%)")
        style_axis(throttle_ax, "Logged throttle")
        time_axes.append(throttle_ax)

    # Mark the first logged zero after each period with a nonzero setpoint.
    for i in range(1, len(target)):
        if target[i - 1] != 0 and target[i] == 0:
            for ax in time_axes:
                ax.axvline(time[i], color=MUTED, linestyle=":", linewidth=1.15)

    fig.text(0.09, 0.947, "Radar cart speed tracking", fontsize=23, weight="bold", color=INK)
    subtitle = f"{path.name}  |  {len(time)} logged rows"
    if rmse is not None:
        subtitle += f"  |  Active tracking RMSE {rmse:.3f} m/s"
    fig.text(0.09, 0.901, subtitle, fontsize=11, color=MUTED)
    if active:
        fig.text(0.09, 0.038,
                 "Dots are speed samples. Vertical dotted lines mark the first logged zero setpoint after a command.",
                 fontsize=9, color=MUTED)


class TrackingDialog(QDialog):
    def __init__(self, parent, output_root):
        super().__init__(parent)
        self.setWindowTitle("Recording tracking")
        available = parent.screen().availableGeometry().size()
        self.resize(QSize(1180, 820).boundedTo(available - QSize(24, 56)))
        self.output_root = output_root
        self.current_path = None

        layout = QVBoxLayout(self)
        header = QHBoxLayout()
        open_button = QPushButton("Open recording…")
        open_button.clicked.connect(self.choose_recording)
        header.addWidget(open_button)
        self.source = QLabel()
        self.source.setTextFormat(Qt.PlainText)
        self.source.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.source.setWordWrap(True)
        header.addWidget(self.source, 1)
        layout.addLayout(header)

        self.notice = QLabel("Choose a downloaded recording CSV to view its tracking plot.")
        self.notice.setTextFormat(Qt.PlainText)
        self.notice.setWordWrap(True)
        layout.addWidget(self.notice)

        self.canvas = FigureCanvasQTAgg(Figure(figsize=(11.8, 7.5), facecolor="#f4f7fa"))
        self.toolbar = NavigationToolbar2QT(self.canvas, self, coordinates=False)
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas, 1)
        self.toolbar.hide()
        self.canvas.hide()

    def choose_recording(self):
        directory = self.current_path.parent if self.current_path else self.output_root
        path, _ = QFileDialog.getOpenFileName(
            self, "Open downloaded recording", str(directory), "CSV files (*.csv)"
        )
        if path:
            self.load_recording(Path(path))

    def load_recording(self, path):
        try:
            columns = read_recording(path)
        except (OSError, ValueError, OverflowError) as exc:
            self.notice.setText(f"Could not open {path.name}. {exc}")
            self.notice.show()
            return

        profile = None
        note = ""
        # The downloader only saves this sidecar when it can associate it with the run.
        if path.name == "recording.csv" and "position_m" in columns and any(columns["target_mps"]):
            sidecar = path.with_name("profile.json")
            if sidecar.is_file():
                try:
                    profile = read_profile(sidecar)
                except (OSError, ValueError, TypeError, KeyError, OverflowError) as exc:
                    note = f"Waypoint overlay unavailable. Could not read profile.json. {exc}"

        draw_tracking(self.canvas.figure, path, columns, profile)
        self.toolbar.update()
        self.current_path = path
        self.source.setText(str(path))
        self.notice.setText(note)
        self.notice.setVisible(bool(note))
        self.toolbar.show()
        self.canvas.show()
        self.canvas.draw_idle()
