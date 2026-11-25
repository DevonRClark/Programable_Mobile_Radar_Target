#pragma once
#include "CartCommon.h"

namespace control {
// Direction is operator/command intent, never physically observed by this Hall channel.
class TrackPosition {
public:
    cart::PositionSample value;
    uint32_t wheel=0, markers=0;
    int next_marker=0;
    int departure_marker=-1; // optional one-shot re-entry; never replaces the required next marker
    float scale=cart::DIST_PER_PULSE_M;
    int64_t updated_us=0;
    bool known() const { return value.flags & cart::POS_KNOWN; }
    bool valid() const { return value.flags & cart::POS_VALID; }
    bool manual() const { return value.flags & cart::POS_MANUAL; }
    void invalidate(uint8_t reason) {
        if (valid() || !known()) value.reason=reason;
        value.flags &= ~cart::POS_VALID;
    }
    void home(float x,uint32_t w,uint32_t m,float meters_per_pulse,int64_t now) {
        uint32_t epoch=value.epoch+1;if(!epoch)epoch=1;
        value={};value.raw_m=value.x_m=x;value.epoch=epoch;
        value.flags=cart::POS_KNOWN|cart::POS_VALID;value.reason=cart::POS_OK;
        wheel=w;markers=m;scale=meters_per_pulse;updated_us=now;departure_marker=-1;
    }
    void setDirection(int direction,bool manual=false) {
        value.direction=static_cast<int8_t>(direction);
        if(manual && direction) value.flags|=cart::POS_MANUAL;
        else value.flags&=~cart::POS_MANUAL;
        next_marker=direction>0 ? 0 : static_cast<int>(cart::TRACK_MARKER_COUNT)-1;
        while(next_marker>=0 && next_marker<static_cast<int>(cart::TRACK_MARKER_COUNT)) {
            const float marker=cart::TRACK_FIRST_MARKER_M+next_marker*cart::TRACK_SPACING_M;
            if(direction>0 ? marker>value.x_m+.00001f : marker<value.x_m-.00001f)break;
            next_marker+=direction>0 ? 1:-1;
        }
        departure_marker=-1;
        if(direction && valid()) {
            const int nearest=static_cast<int>(roundf((value.x_m-cart::TRACK_FIRST_MARKER_M)/cart::TRACK_SPACING_M));
            if(nearest>=0 && nearest<static_cast<int>(cart::TRACK_MARKER_COUNT) && nearest!=next_marker &&
               fabsf(value.x_m-(cart::TRACK_FIRST_MARKER_M+nearest*cart::TRACK_SPACING_M))<=cart::POSITION_GATE_M)
                departure_marker=nearest;
        }
    }
    void update(uint32_t w,uint32_t m,uint32_t marker_wheel,float meters_per_pulse,int64_t now) {
        const uint32_t pulses=w-wheel, events=m-markers;
        const int64_t elapsed=now-updated_us;
        wheel=w;markers=m;updated_us=now;
        if(!known())return;
        if(!isfinite(meters_per_pulse) || fabsf(meters_per_pulse-scale)>1e-7f) { invalidate(cart::POS_CAL_CHANGED);return; }
        if(elapsed<0 || pulses>static_cast<uint32_t>(ceilf(15.0f*(elapsed/1000000.0f)/scale))+2u) {
            invalidate(cart::POS_COUNTER);return;
        }
        if((pulses || events) && value.direction==0) { invalidate(cart::POS_UNCOMMANDED);return; }
        const float distance=static_cast<float>(value.direction)*pulses*scale;
        value.raw_m+=distance;value.x_m+=distance;
        if(!valid())return; // no anonymous-marker recovery after losing the reference
        if(departure_marker>=0 && fabsf(value.x_m-(cart::TRACK_FIRST_MARKER_M+departure_marker*cart::TRACK_SPACING_M))>cart::POSITION_GATE_M)
            departure_marker=-1;
        const bool have_marker=next_marker>=0 && next_marker<static_cast<int>(cart::TRACK_MARKER_COUNT);
        const float expected=cart::TRACK_FIRST_MARKER_M+next_marker*cart::TRACK_SPACING_M;
        if(events) {
            const uint32_t after_edge=w-marker_wheel;
            if(events!=1 || after_edge>pulses) { invalidate(cart::POS_MARKER_COUNT);return; }
            const float marker_x=value.x_m-static_cast<float>(value.direction)*after_edge*scale;
            if(departure_marker>=0 && fabsf(marker_x-(cart::TRACK_FIRST_MARKER_M+departure_marker*cart::TRACK_SPACING_M))<=cart::POSITION_GATE_M) {
                // Edge hysteresis/wobble can re-enter the departure magnet. Do not
                // snap position or advance the ordinal expected a metre away.
                departure_marker=-1;value.flags|=cart::POS_MARKER_CHECKED;return;
            }
            departure_marker=-1;
            const float correction=expected-marker_x;
            if(!have_marker || fabsf(correction)>cart::POSITION_GATE_M) { invalidate(cart::POS_MARKER_GATE);return; }
            value.x_m+=correction;value.correction_m=correction;
            value.flags|=cart::POS_MARKER_CHECKED;next_marker+=value.direction;
        } else if(have_marker && value.direction*(value.x_m-expected)>cart::POSITION_GATE_M+scale) {
            invalidate(cart::POS_MARKER_MISSED);
        }
        if(value.x_m < -scale || value.x_m > cart::TRACK_LENGTH_M+scale)invalidate(cart::POS_BOUNDS);
    }
};
} // namespace control
