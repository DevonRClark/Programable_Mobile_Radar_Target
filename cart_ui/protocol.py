"""Protocol helpers for the cart UI.

Recording and telemetry parsers come from the bench utilities in archive c9e2cad.
Profile and return validation target the submitted firmware's expanded track map.
"""

import csv
import json
import math
import re
import struct
import zlib

COLUMNS = [
    "event",
    "t_us",
    "seq",
    "mode",
    "target_mps",
    "speed_mps",
    "throttle_pct",
    "pulse_us",
    "wheel_count",
    "total_m",
    "run_m",
    "markers",
    "marker_error_m",
    "flags",
    "log_drops",
    "p_term",
    "integral",
    "meters_per_pulse",
    "calibration_id",
]

POSITION_COLUMNS = [
    "raw_position_m",
    "position_m",
    "position_correction_m",
    "move_cutoff_m",
    "position_epoch",
    "position_direction",
    "position_flags",
    "position_reason",
    "cutoff_source",
]
WHEEL_COLUMNS = [
    "left_wheel_count",
    "marker_left_wheel_count",
    "marker_right_wheel_count",
]


def columns_for_metadata(metadata):
    version = metadata.get("sample_version", 1)
    if version not in (1, 2, 3):
        raise ValueError("Unsupported recording sample version")
    return (
        COLUMNS
        + (POSITION_COLUMNS if version >= 2 else [])
        + (WHEEL_COLUMNS if version >= 3 else [])
    )


def decode_transfer(text, expected_id):
    lines = text.splitlines(keepends=True)
    begin = next(
        (
            i
            for i, line in enumerate(lines)
            if line.rstrip("\r\n") == f"LOG_BEGIN {expected_id}"
        ),
        None,
    )
    if begin is None:
        raise ValueError("Recording start marker missing")
    metadata = None
    samples = []
    checksum = 0
    end = None
    details = {}
    for line in lines[begin + 1 :]:
        line = line.replace("\r\n", "\n")
        if line.startswith("LOG_META "):
            if metadata is not None:
                raise ValueError("Duplicate metadata")
            metadata = json.loads(line[len("LOG_META ") :])
            checksum = zlib.crc32(line.encode(), checksum)
        elif line.startswith("LOG_DETAILS "):
            details = json.loads(line[len("LOG_DETAILS ") :])
            checksum = zlib.crc32(line.encode(), checksum)
        elif line.startswith("CSV,"):
            if not line.endswith("\n"):
                raise ValueError("Truncated sample")
            fields = next(csv.reader([line]))[1:]
            if metadata is None or len(fields) != len(columns_for_metadata(metadata)):
                raise ValueError("Invalid sample columns")
            samples.append(fields)
            checksum = zlib.crc32(line.encode(), checksum)
        elif line.startswith("LOG_END "):
            parts = line.split()
            if len(parts) != 4:
                raise ValueError("Invalid completion marker")
            end = tuple(int(value) for value in parts[1:])
            break
    if metadata is None or end is None:
        raise ValueError("Incomplete transfer; no verified completion marker")
    if (
        end != (expected_id, len(samples), checksum)
        or metadata["id"] != expected_id
        or metadata["count"] != len(samples)
    ):
        raise ValueError("Recording ID, length or transfer checksum mismatch")
    if not samples:
        raise ValueError("Empty recording")
    metadata.update(details)
    metadata["transfer_verified"] = True
    metadata["meters_per_pulse"] = float(samples[0][COLUMNS.index("meters_per_pulse")])
    metadata["calibration_id"] = int(samples[0][COLUMNS.index("calibration_id")])
    metadata["last_sample_stop_flags"] = int(samples[-1][COLUMNS.index("flags")])
    return metadata, samples


LIVE = re.compile(
    r"Live -> \|v\|: ([\d.]+) m/s \| Target: (-?[\d.]+) \| Mags: (\d+) \| (RUNNING|STOPPED) \| Stop: (.*)"
)


def parse_line(line):
    capability = re.fullmatch(r"Return capability -> max_mps=(1\.0|2\.0)", line)
    if capability:
        return "return_capability", {"max_mps": float(capability[1])}
    match = LIVE.fullmatch(line)
    if match:
        values = {
            "speed": float(match[1]),
            "target": float(match[2]),
            "markers": int(match[3]),
            "running": match[4] == "RUNNING",
            "stop": match[5],
        }
        if not all(math.isfinite(values[key]) for key in ("speed", "target")):
            raise ValueError("Invalid live values")
        return "live", values
    if line.startswith("Position -> "):
        fields = dict(re.findall(r"(\w+)=([^\s]+)", line))
        keys = (
            "raw",
            "x",
            "cutoff",
            "epoch",
            "dir",
            "flags",
            "reason",
            "source",
            "request",
            "result",
            "seq",
        )
        if not all(k in fields for k in keys):
            return None
        value = {k: float(fields[k]) for k in ("raw", "x", "cutoff")}
        if not all(math.isfinite(v) for v in value.values()):
            raise ValueError("Invalid position telemetry")
        value.update({k: int(fields[k]) for k in keys if k not in value})
        return "position", value
    if line.startswith("LOGGER "):
        fields = dict(re.findall(r"(\w+)=(\d+)", line))
        keys = ("state", "error", "samples", "flags", "saved_id")
        if not all(k in fields for k in keys):
            return None
        return "logger", {k: int(fields[k]) for k in keys}
    if line.startswith("Odom -> "):
        fields = dict(re.findall(r"(\w+)=([^\s]+)", line))
        required = (
            "count",
            "total",
            "run",
            "m_per_pulse",
            "cal",
            "flags",
            "mode",
            "throttle",
        )
        if not all(k in fields for k in required):
            return None
        values = {
            k: float(fields[k].rstrip("%"))
            for k in ("total", "run", "m_per_pulse", "throttle")
        }
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError("Invalid odometry")
        values.update(
            count=int(fields["count"]),
            cal=int(fields["cal"]),
            flags=int(fields["flags"]),
            mode=fields["mode"],
        )
        return "odom", values
    return None


PI_REPLY = re.compile(
    r"PI_REPLY sequence=(\d+) result=(STATUS|APPLIED|BUSY|INVALID|UNSUPPORTED) kp=([\d.]+) ki=([\d.]+) kd=0 persistence=RAM"
)


def parse_pi(text, sequence):
    for match in PI_REPLY.finditer(text):
        if int(match[1]) == sequence:
            kp, ki = float(match[3]), float(match[4])
            if not all(math.isfinite(x) for x in (kp, ki)) or not (
                0 <= kp <= 50 and 0 <= ki <= 100 and kp + ki > 0
            ):
                raise ValueError("Invalid PI readback")
            return {
                "sequence": sequence,
                "result": match[2],
                "kp": kp,
                "ki": ki,
                "kd": 0,
                "persistence": "RAM",
            }
    return None


PROFILE_LIMITS = {"speed": 10.0, "acceleration": 3.5, "deceleration": 4.0}
PROFILE_RUNTIME_S = 60
PROFILE_REPLY = re.compile(
    r"PROFILE_REPLY sequence=(\d+) result=(\w+) crc=(\d+) count=(\d+) "
    r"received=(\d+) uploading=([01]) duration=([\d.]+) floor=([\d.]+) launch=([\d.]+)"
)


def make_profile(points, floor=1.0, launch=1.0):
    """Validate the actual float32 values sent to the firmware without reshaping the profile."""
    if not 2 <= len(points) <= 32:
        raise ValueError("A profile needs 2 to 32 waypoints.")
    if not all(
        math.isfinite(p["position_m"])
        and 0 <= p["position_m"] <= 26
        and math.isfinite(p["speed_mps"])
        and 0 <= p["speed_mps"] <= PROFILE_LIMITS["speed"]
        for p in points
    ):
        raise ValueError(
            "Infeasible input: waypoint positions or speeds are out of range."
        )
    if not 0.1 <= floor <= launch <= PROFILE_LIMITS["speed"]:
        raise ValueError(
            "Use a minimum moving speed of at least 0.1 m/s and a launch speed at or above the minimum."
        )

    payload = b"".join(
        struct.pack("<ff", p["position_m"], p["speed_mps"]) for p in points
    )
    wire_points = [
        {"position_m": x, "speed_mps": v} for x, v in struct.iter_unpack("<ff", payload)
    ]
    floor, launch = struct.unpack("<ff", struct.pack("<ff", floor, launch))
    if (
        wire_points[0]["position_m"] != 0
        or wire_points[-1]["position_m"] < 0.5
        or wire_points[0]["speed_mps"] != 0
        or wire_points[-1]["speed_mps"] != 0
    ):
        raise ValueError(
            "Start at 0 m, finish at least 0.5 m away, and use zero speed at both endpoints."
        )
    if any(p["speed_mps"] <= 0 for p in wire_points[1:-1]):
        raise ValueError("Use a positive speed at every interior waypoint.")
    if launch > max(p["speed_mps"] for p in wire_points):
        raise ValueError(
            "Lower the launch speed and minimum moving speed to fit the highest waypoint speed."
        )

    duration = 0.0
    for left, right in zip(wire_points, wire_points[1:]):
        distance = right["position_m"] - left["position_m"]
        if distance <= 0:
            raise ValueError(
                "Waypoint positions must increase at the cart's numeric precision."
            )
        v0, v1 = left["speed_mps"], right["speed_mps"]
        if v0 + v1 == 0:
            raise ValueError("A profile needs an interior moving waypoint.")
        acceleration = (v1**2 - v0**2) / (2 * distance)
        if (
            not -PROFILE_LIMITS["deceleration"] - 1e-5
            <= acceleration
            <= PROFILE_LIMITS["acceleration"] + 1e-5
        ):
            raise ValueError(
                "Infeasible input: a ramp exceeds the firmware acceleration limits."
            )
        duration += 2 * distance / (v0 + v1)
    if duration + 4 > PROFILE_RUNTIME_S:
        raise ValueError(
            "Profile exceeds the 60 s firmware limit including stopping allowance."
        )

    return {
        "schema_version": 1,
        "points": wire_points,
        "requested": points,
        "crc32": zlib.crc32(payload),
        "count": len(points),
        "interpolation": "linear speed squared over position",
        "planned_duration_s": duration,
        "runtime_limit_s": PROFILE_RUNTIME_S,
        "launch_and_low_speed_floor_mps": floor,
        "launch_speed_mps": launch,
    }


def query_profile(session, command, expected="STATUS"):
    offset = len(session.text)
    sequence = session.command(command)
    for match in PROFILE_REPLY.finditer(session.text[offset:]):
        if int(match[1]) == sequence and match[2] == expected:
            return {
                "crc": int(match[3]),
                "count": int(match[4]),
                "received": int(match[5]),
                "uploading": match[6] == "1",
                "duration_s": float(match[7]),
                "floor": float(match[8]),
                "launch": float(match[9]),
            }
    raise RuntimeError(
        "Profile request refused or current firmware readback missing. " + command
    )


def profile_matches(reply, manifest):
    return (
        reply["crc"] == manifest["crc32"]
        and reply["count"] == reply["received"] == manifest["count"]
        and not reply["uploading"]
        and abs(reply["duration_s"] - manifest["planned_duration_s"]) <= 0.002
        and abs(reply["floor"] - manifest["launch_and_low_speed_floor_mps"]) <= 0.00051
        and abs(reply["launch"] - manifest["launch_speed_mps"]) <= 0.00051
    )


def upload_profile(session, manifest):
    floor = manifest["launch_and_low_speed_floor_mps"]
    launch = manifest["launch_speed_mps"]
    query_profile(
        session,
        f"profile begin {manifest['count']} {floor:.9g} {launch:.9g}",
        "APPLIED",
    )
    for index, point in enumerate(manifest["points"]):
        query_profile(
            session,
            f"profile point {index} {point['position_m']:.9g} {point['speed_mps']:.9g}",
            "APPLIED",
        )
    applied = query_profile(session, f"profile commit {manifest['crc32']}", "APPLIED")
    status = query_profile(session, "profile status")
    if not all(profile_matches(reply, manifest) for reply in (applied, status)):
        raise ValueError("Committed profile differs from the uploaded settings.")
    session.neutral()


RETURN_MIN_MPS = 0.5
RETURN_LEGACY_MAX_MPS = 1.0
RETURN_MAX_MPS = 2.0


def plan_return(position, goal, speed, margin):
    """Plan the same continuous reverse move within the firmware map and time limit."""
    if not (
        math.isfinite(position)
        and math.isfinite(goal)
        and 0 <= position <= 26
        and 0 <= goal <= 26
        and RETURN_MIN_MPS <= speed <= RETURN_MAX_MPS
        and 0.10 <= margin <= 0.25
    ):
        raise ValueError(
            "Return requires a valid position, goal and reverse speed within the track limits."
        )
    cutoff = goal + margin
    distance = position - cutoff
    if distance < 0.1:
        raise ValueError("Already near or behind home. Align manually.")
    if distance / speed + 1.5 > 60:
        raise ValueError("Return exceeds the firmware time limit.")
    return {"distance_m": distance, "cutoff_m": cutoff, "target_mps": -speed}
