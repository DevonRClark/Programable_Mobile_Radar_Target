#pragma once
#include <stdio.h>
#include "CartCommon.h"

namespace experiment {
constexpr size_t MAX_LINE = 256;
#pragma pack(push, 1)
struct Sample {
    int64_t time_us;
    cart::Telemetry packet;
    float p_term, integral;
    uint32_t dropped;
    uint16_t pulse_us;
    uint8_t event;
    cart::PositionSample position = {};
    uint32_t left_wheel_count=0, marker_left_wheel_count=0, marker_right_wheel_count=0;
};
#pragma pack(pop)
static_assert(offsetof(Sample,position)==99, "Legacy sample prefix changed");
static_assert(offsetof(Sample,left_wheel_count)==123 && sizeof(Sample)==135,"Recording format3 ABI changed");
inline int format(char *line, size_t capacity, const Sample &s, unsigned version=3) {
    const auto &p = s.packet;
    const int base = snprintf(line, capacity,
        "CSV,%u,%lld,%lu,%u,%.3f,%.3f,%.2f,%u,%lu,%.3f,%.3f,%lu,%.3f,%u,%lu,%.2f,%.3f,%.7f,%lu\n",
        s.event, static_cast<long long>(s.time_us), static_cast<unsigned long>(p.acknowledged_sequence),
        p.mode, p.target_speed, p.speed_magnitude, p.throttle_percent, s.pulse_us,
        static_cast<unsigned long>(p.wheel_count), p.total_distance_m, p.run_distance_m,
        static_cast<unsigned long>(p.marker_total), p.marker_error_m, p.flags,
        static_cast<unsigned long>(s.dropped), s.p_term, s.integral, p.meters_per_pulse,
        static_cast<unsigned long>(p.calibration_id));
    if(version<2 || base<=0 || static_cast<size_t>(base)>=capacity)return base;
    const auto &x=s.position;
    const int extra=snprintf(line+base-1,capacity-base+1,",%.4f,%.4f,%.4f,%.4f,%lu,%d,%u,%u,%u\n",
        x.raw_m,x.x_m,x.correction_m,x.cutoff_m,static_cast<unsigned long>(x.epoch),x.direction,
        x.flags,x.reason,x.cutoff_source);
    const int length=extra<0 ? extra : base-1+extra;
    if(version<3 || length<=0 || static_cast<size_t>(length)>=capacity)return length;
    const int wheels=snprintf(line+length-1,capacity-length+1,",%lu,%lu,%lu\n",
        static_cast<unsigned long>(s.left_wheel_count),static_cast<unsigned long>(s.marker_left_wheel_count),
        static_cast<unsigned long>(s.marker_right_wheel_count));
    return wheels<0 ? wheels:length-1+wheels;
}
} // namespace experiment
