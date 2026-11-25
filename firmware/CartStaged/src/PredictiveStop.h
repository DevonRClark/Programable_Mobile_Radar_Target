#pragma once
#include "CartCommon.h"
namespace control {
// Forward-only LEFT-wheel neutral-stop fit (8 current traces, observed0.54..3.09m/s).
// Maximum terminal braking is zero throttle/1500us; never opposite throttle.
struct PredictiveStop {
    enum Phase : uint8_t { IDLE, TRACK, DECEL, NEUTRAL, CREEP, SETTLED, FAILED };
    static constexpr float B1=.09721015f, B2=.01851059f;
    static constexpr float MARGIN=.02651515f, TOLERANCE=.04f, RESERVE=.20f, CREEP_SPEED=.5f;
    static constexpr float LEFT_B1=.028697974f, LEFT_B2=.052144188f;
    static constexpr float FIT_RESIDUAL=.010394369f, OBSERVATION_S=.02f;
    // Extra preview above1m/s lets the PI shed drive before terminal neutral.
    // This is a provisional response allowance, not measured braking latency.
    static constexpr float PREVIEW_S=.35f;
    // Candidate22: trim30mm from the terminal release distance.
    // The full model still shapes speed; maximum braking remains immediate0%.
    // Not online learning and not a change to the PI or early preview.
    static constexpr float RELEASE_TRIM_M=.030f;
    Phase phase=IDLE;
    float goal=0, cruise=0, effective=0, predicted=0, remaining=0, creep_origin=0;
    uint8_t attempts=0;
    int64_t neutral_us=0, creep_us=0;
    void start(float endpoint,float speed){*this={};goal=endpoint;cruise=speed;effective=speed;phase=TRACK;}
    static float stoppingDistance(float speed){
        speed=fmaxf(speed,0);
        const float legacy=B1*speed+B2*speed*speed+MARGIN;
        const float current=(LEFT_B1+OBSERVATION_S)*speed+LEFT_B2*speed*speed+MARGIN+FIT_RESIDUAL;
        return fmaxf(legacy,current); // Retain the proven low-speed floor.
    }
    static float inverse(float distance,float b1,float b2,float margin){
        const float d=fmaxf(distance-margin,0);
        return 2*d/(sqrtf(b1*b1+4*b2*d)+b1);
    }
    static float allowedSpeed(float distance){
        // Inverse of the maximum of two increasing curves is their minimum.
        return fminf(inverse(distance,B1,B2,MARGIN),
                     inverse(distance,LEFT_B1+OBSERVATION_S,LEFT_B2,MARGIN+FIT_RESIDUAL));
    }
    float previewDistance() const {return RESERVE+fmaxf(cruise-1,0)*PREVIEW_S;}
    static float releaseDistance(float speed){return fmaxf(MARGIN,stoppingDistance(speed)-RELEASE_TRIM_M);}
    void release(int64_t now){phase=NEUTRAL;neutral_us=now;effective=0;}
    void update(float x,float speed,bool quiet,int64_t now,bool shape_speed=true){
        remaining=goal-x;predicted=stoppingDistance(speed);
        if(phase==IDLE || phase==SETTLED || phase==FAILED)return;
        if(!isfinite(x)||!isfinite(speed)||remaining < -TOLERANCE){phase=FAILED;effective=0;return;}
        if(phase==NEUTRAL){
            effective=0;
            if(quiet && now-neutral_us>=cart::WHEEL_STOP_US){
                if(fabsf(remaining)<=TOLERANCE)phase=SETTLED;
                else if(attempts<2){phase=CREEP;++attempts;creep_us=now;creep_origin=x;effective=fminf(cruise,CREEP_SPEED);}
                else phase=FAILED;
            }
            return;
        }
        if(phase==CREEP){
            predicted=stoppingDistance(speed);
            // At rest the full-speed stopping curve would prohibit correcting a
            // small shortfall. Advance at most one encoder step, then settle.
            if(now-creep_us>=2000000 || x-creep_origin>=.8f*cart::LEFT_DIST_PER_PULSE_M ||
                remaining<=TOLERANCE/2 || (speed>.05f && remaining<=predicted-MARGIN)){release(now);return;}
            effective=fminf(cruise,CREEP_SPEED);return;
        }
        if(remaining<=releaseDistance(speed) && speed>.05f){release(now);return;}
        if(remaining<=TOLERANCE){release(now);return;}
        if(!shape_speed){effective=cruise;phase=TRACK;return;}
        const float cap=fmaxf(fminf(cruise,CREEP_SPEED),allowedSpeed(remaining-previewDistance()));
        effective=fminf(cruise,cap);phase=effective<cruise-.0001f ? DECEL:TRACK;
    }
};
} // namespace control
