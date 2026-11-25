#pragma once
#include "CartCommon.h"

namespace control {
// Distances are unsigned path lengths: a single Hall sensor cannot sense direction.
class Odometry {
public:
    float meters_per_pulse = cart::DIST_PER_PULSE_M;
    double total_m = 0, run_origin_m = 0;
    uint32_t wheel_count = 0, track_count = 0, calibration_origin = 0;
    uint32_t calibration_id = 0;
    float marker_interval_m = 0, marker_error_m = 0;
    bool marker_valid = false, marker_suspect = false;

    void update(uint32_t wheels, uint32_t markers, uint32_t marker_wheel_count) {
        total_m += static_cast<uint32_t>(wheels-wheel_count) * static_cast<double>(meters_per_pulse);
        wheel_count = wheels;
        const uint32_t events = markers-track_count;
        track_count = markers;
        if (!events) return;
        marker_valid = false;
        marker_suspect = events != 1;
        if (events == 1 && marker_seen) {
            const uint32_t pulses = marker_wheel_count-last_marker_wheel;
            marker_interval_m = pulses * meters_per_pulse;
            marker_error_m = marker_interval_m-1.0f;
            marker_valid = pulses > 0;
            const float threshold = 2*meters_per_pulse > 0.05f ? 2*meters_per_pulse : 0.05f;
            marker_suspect = !marker_valid || fabsf(marker_error_m) > threshold;
        }
        last_marker_wheel = marker_wheel_count;
        marker_seen = events == 1;
    }
    void startRun() { run_origin_m = total_m; }
    float runDistance() const { return static_cast<float>(total_m-run_origin_m); }
    void reset(uint32_t wheels, uint32_t markers) {
        wheel_count = calibration_origin = wheels;
        track_count = markers;
        total_m = run_origin_m = 0;
        clearMarker();
    }
    bool calibrate(float measured_m) {
        const uint32_t pulses = wheel_count-calibration_origin;
        if (!isfinite(measured_m) || measured_m <= 0 || measured_m > 5 || pulses < 10) return false;
        const float next = measured_m/pulses;
        if (!isfinite(next) || next < 0.001f || next > 1.0f) return false;
        meters_per_pulse = next;
        ++calibration_id;
        clearMarker(); // intervals must never mix old/new calibration scales
        return true; // applies only to future increments; no position jump
    }
private:
    bool marker_seen = false;
    uint32_t last_marker_wheel = 0;
    void clearMarker() {
        marker_seen = marker_valid = marker_suspect = false;
        marker_interval_m = marker_error_m = 0;
    }
};
} // namespace control
