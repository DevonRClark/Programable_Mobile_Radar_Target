#pragma once
#include "CartCommon.h"
#include "Odometry.h"
#include "TrackPosition.h"
#include "PredictiveStop.h"
#include "VelocityProfile.h"

namespace control {
struct Input {
    uint32_t session = 0, sequence = 0;
    float target = 0;
    bool command = false;
    uint8_t kind = cart::SPEED;
    float throttle_percent = 0, distance_m = 0;
    uint32_t duration_ms = 0;
};

inline bool decode(const uint8_t *bytes, int length, Input &out) {
    if (!bytes || length < static_cast<int>(sizeof(cart::Header))) return false;
    cart::Header h;
    memcpy(&h, bytes, sizeof(h));
    Input p;
    if (length == sizeof(cart::Heartbeat) && cart::validHeader(h, cart::HEARTBEAT)) {
        cart::Heartbeat hb;
        memcpy(&hb, bytes, sizeof(hb));
        p.session = hb.session;
    } else if (length == sizeof(cart::Command) && cart::validHeader(h, cart::COMMAND)) {
        cart::Command cmd;
        memcpy(&cmd, bytes, sizeof(cmd));
        if (!cart::validCommand(cmd)) return false;
        p.session = cmd.session;
        p.sequence = cmd.sequence;
        p.target = cart::clampf(cmd.target_speed, -cart::MAX_SPEED_MPS, cart::MAX_SPEED_MPS);
        p.command = true;
        p.kind = cmd.kind;
        p.throttle_percent = cmd.throttle_percent;
        p.distance_m = cmd.distance_m;
        p.duration_ms = cmd.duration_ms;
    } else {
        return false;
    }
    if (p.session == 0) return false;
    out = p;
    return true;
}

struct Sensors {
    cart::WheelSample wheel;
    cart::WheelSample left_wheel;
    cart::WheelSample right_wheel; // Physical diagnostic snapshot, independent of selected wheel.
    uint32_t track_right_wheel_count = 0;
    uint32_t track_left_wheel_count = 0;
    uint32_t track_total = 0; // debounced, lifetime count; unsigned wrap is allowed
    uint32_t track_wheel_count = 0; // wheel count latched inside the track ISR
};

class Engine {
public:
    explicit Engine(bool can_drive, bool bench_profile = false) : drive_enabled(can_drive), bench(bench_profile) {}
    cart::CommandHistory history;
    cart::PIController pi;
    Odometry odometry;
    TrackPosition position;
    PredictiveStop approach;
    VelocityProfile profile;
    uint32_t profile_sequence=0;
    uint8_t profile_result=cart::PROFILE_QUERIED;
    float profile_origin=0, profile_nominal=0;
    bool profile_launch=false;
    uint32_t creep_wheel_count=0;
    uint32_t position_sequence=0;
    uint8_t position_result=cart::POS_STATUS;
    float move_distance=0;
    int move_direction=0;
    bool fast_reverse_move=false;
    uint8_t mode = cart::SPEED;
    float fixed_throttle = 0;
    int64_t test_deadline_us = 0;
    uint32_t start_wheel_count = 0;
    bool timed_speed = false;
    bool maintenance = false;
    bool motion_blocked = false; // manual recording never admits motor commands
    bool tuning_blocked = false; // control-owned recorder/maintenance gate
    uint32_t pi_sequence=0;
    uint8_t pi_result=cart::PI_QUERIED;
    bool running = false, require_zero = true, faulted = false;
    float target = 0, speed = 0;
    uint8_t reason = cart::STOP_REARM_REQUIRED;
    uint32_t magnets = 0;
    int last_direction = 0;
    int64_t last_received_us = 0, started_us = 0, stopped_us = 0;

    bool receive(const Input &p, int64_t received_us, int64_t now, const Sensors &s) {
        if (now < received_us || now - received_us >= cart::LINK_TIMEOUT_US) return false;
        if (history.setSession(p.session)) {
            profile.clear();
            stop(cart::STOP_REMOTE_RESTART, now);
            require_zero = true;
        }
        last_received_us = received_us; // queue delay never refreshes an old packet
        if (!p.command || !history.accept(p.session, p.sequence)) return true;
        updateSensors(now,s);
        const bool zero = p.kind == cart::STOP || (p.kind == cart::SPEED && p.target == 0);
        if (zero) {
            stop(cart::STOP_USER_CMD, now);
            require_zero = faulted;
            // Leaving RX_OVERFLOW here makes the dongle demand re-arming again
            // on the next telemetry frame, even though this zero was ACKed.
            if (!faulted && (reason == cart::STOP_REARM_REQUIRED ||
                            reason == cart::STOP_RX_OVERFLOW ||
                            reason == cart::STOP_REMOTE_RESTART ||
                            reason == cart::STOP_CMD_TIMEOUT)) reason = cart::STOP_NONE;
            return true;
        }
        if(cart::isProfileConfig(p.kind)) {
            profile_sequence=p.sequence;
            const bool quiet=!s.wheel.seen || (now>=s.wheel.last_us && now-s.wheel.last_us>=cart::WHEEL_STOP_US);
            if(!bench)profile_result=cart::PROFILE_UNSUPPORTED;
            else if(running || maintenance || tuning_blocked || motion_blocked || !quiet)profile_result=cart::PROFILE_BUSY;
            else if(p.kind==cart::PROFILE_STATUS)profile_result=cart::PROFILE_QUERIED;
            else {
                bool ok=p.kind==cart::PROFILE_BEGIN ? profile.begin(p.duration_ms,p.target,p.distance_m):
                    p.kind==cart::PROFILE_POINT ? profile.point(p.duration_ms,p.distance_m,p.target):profile.commit(p.duration_ms);
                profile_result=ok ? cart::PROFILE_APPLIED:cart::PROFILE_INVALID;
                if(ok){require_zero=true;pi.reset();}
            }
            return true;
        }
        if (p.kind>=cart::HOME && p.kind<=cart::POSITION_DIRECTION) {
            position_sequence=p.sequence;
            const bool quiet=!s.wheel.seen || (now>=s.wheel.last_us && now-s.wheel.last_us>=cart::WHEEL_STOP_US);
            if(!bench)position_result=cart::POS_UNSUPPORTED;
            else if(p.kind==cart::POSITION_STATUS)position_result=cart::POS_STATUS;
            else if(running || maintenance || tuning_blocked || motion_blocked || !quiet)position_result=cart::POS_BUSY;
            else if(p.kind==cart::HOME && isfinite(p.distance_m) && p.distance_m>=0 && p.distance_m<=cart::TRACK_LENGTH_M) {
                position.home(p.distance_m,s.wheel.count,s.track_total,odometry.meters_per_pulse,now);
                position_result=cart::POS_APPLIED;require_zero=true;
            } else if(p.kind==cart::POSITION_CLEAR) {
                position.invalidate(cart::POS_CLEARED);position_result=cart::POS_APPLIED;require_zero=true;
            } else if(p.kind==cart::POSITION_DIRECTION && position.valid() && (p.target==-1 || p.target==0 || p.target==1)) {
                position.setDirection(static_cast<int>(p.target),p.target!=0);
                position_result=cart::POS_APPLIED;require_zero=true;
            } else position_result=cart::POS_INVALID;
            return true;
        }
        if (p.kind==cart::PI_SET || p.kind==cart::PI_STATUS) {
            pi_sequence=p.sequence;
            const bool quiet=!s.wheel.seen || (now>=s.wheel.last_us && now-s.wheel.last_us>=cart::WHEEL_STOP_US);
            // A refused update must not stop, reset, or perturb an existing run.
            if (!bench) pi_result=cart::PI_UNSUPPORTED;
            else if (running || maintenance || tuning_blocked || !quiet) pi_result=cart::PI_BUSY;
            else if (p.kind==cart::PI_STATUS) pi_result=cart::PI_QUERIED;
            else if (!cart::validPiGains(p.distance_m,p.throttle_percent) || p.target!=0 || p.duration_ms!=0) pi_result=cart::PI_INVALID;
            else {
                pi.kp=p.distance_m; pi.ki=p.throttle_percent; pi.reset();
                require_zero=true; reason=cart::STOP_REARM_REQUIRED;
                pi_result=cart::PI_APPLIED;
            }
            return true;
        }
        if (cart::isLogCommand(p.kind)) return true;
        if (motion_blocked) {
            stop(cart::STOP_REARM_REQUIRED, now); require_zero = true; return true;
        }
        if (maintenance) {
            stop(cart::STOP_REARM_REQUIRED, now); require_zero = true;
            return true;
        }
        if (p.kind == cart::ODOM_RESET || p.kind == cart::CALIBRATE) {
            const bool quiet = !s.wheel.seen || now-s.wheel.last_us >= cart::WHEEL_STOP_US;
            if (!bench || running || !quiet) {
                stop(cart::STOP_INVALID_COMMAND, now);
                reason = cart::STOP_INVALID_COMMAND;
                require_zero = true;
            } else if (p.kind == cart::ODOM_RESET) {
                odometry.reset(s.wheel.count, s.track_total);
                position.invalidate(cart::POS_CLEARED);
                reason = cart::STOP_ODOM_RESET;
            } else {
                reason = odometry.calibrate(p.distance_m) ? cart::STOP_CALIBRATED : cart::STOP_CALIBRATION_INVALID;
                if(reason==cart::STOP_CALIBRATED)position.invalidate(cart::POS_CAL_CHANGED);
            }
            return true;
        }
        if (!bench && p.kind != cart::SPEED) {
            stop(cart::STOP_UNSUPPORTED, now);
            reason = cart::STOP_UNSUPPORTED;
            require_zero = true;
            return true;
        }
        if (!bench && p.kind == cart::SPEED && p.duration_ms) {
            stop(cart::STOP_UNSUPPORTED, now); reason = cart::STOP_UNSUPPORTED; require_zero = true; return true;
        }
        if (p.kind == cart::SPEED && p.duration_ms && (p.duration_ms < 50 || p.duration_ms > cart::TIMED_SPEED_MAX_MS)) {
            stop(cart::STOP_INVALID_COMMAND, now); reason = cart::STOP_INVALID_COMMAND; require_zero = true; return true;
        }
        if (bench && (fabsf(p.target) > (p.kind==cart::APPROACH ? cart::APPROACH_MAX_SPEED_MPS : cart::BENCH_MAX_SPEED_MPS) ||
            (p.kind == cart::THROTTLE && (!isfinite(p.throttle_percent) ||
             fabsf(p.throttle_percent) > cart::BENCH_MAX_FIXED_THROTTLE || p.duration_ms < 50 || p.duration_ms > cart::FIXED_THROTTLE_MAX_MS)))) {
            stop(cart::STOP_INVALID_COMMAND, now);
            reason = cart::STOP_INVALID_COMMAND;
            require_zero = true;
            return true;
        }
        if(p.kind==cart::PROFILE_RUN && (running || !position.valid() || !profile.ready() || (p.duration_ms!=profile.crc || p.target!=profile.floor || p.distance_m!=profile.launch) ||
            position.value.x_m+profile.length()>cart::TRACK_LENGTH_M-.25f)) {
            stop(cart::STOP_MOVE_REFUSED,now);reason=cart::STOP_MOVE_REFUSED;require_zero=true;return true;
        }
        if(p.kind==cart::MOVE || p.kind==cart::APPROACH) {
            const float endpoint=position.value.x_m+(p.target>0 ? p.distance_m : -p.distance_m);
            if(running || !position.valid() || fabsf(p.target)>(p.kind==cart::APPROACH ? cart::APPROACH_MAX_SPEED_MPS : cart::moveSpeedLimit(p.target)) || p.target==0 ||
                !isfinite(p.distance_m) || p.distance_m<=0 || p.distance_m>cart::MOVE_MAX_DISTANCE_M ||
                endpoint<0 || endpoint>cart::TRACK_LENGTH_M ||
                (p.kind==cart::APPROACH && (!cart::validCommand(cart::Command{cart::makeHeader(cart::COMMAND),p.session,p.sequence,p.target,p.distance_m,p.throttle_percent,p.duration_ms,p.kind}) || endpoint>cart::TRACK_LENGTH_M-.25f || p.distance_m/p.target+7>cart::APPROACH_MAX_RUNTIME_US/1000000.0f))) {
                stop(cart::STOP_MOVE_REFUSED,now);reason=cart::STOP_MOVE_REFUSED;require_zero=true;return true;
            }
        }
        if (running && (p.kind != mode || (p.kind == cart::SPEED && (p.duration_ms > 0) != timed_speed))) {
            stop(cart::STOP_REARM_REQUIRED, now);
            require_zero = true;
            return true;
        }
        if (!drive_enabled || require_zero || faulted) {
            stop(cart::STOP_REARM_REQUIRED, now);
            reason = cart::STOP_REARM_REQUIRED;
            require_zero = true;
            return true;
        }
        // A target update queued before a limit stop must not start another run.
        // A new run requires a new command received after that stop.
        if (!running && (reason == cart::STOP_TRACK_DONE || reason == cart::STOP_MAX_RUNTIME || reason == cart::STOP_TEST_LIMIT || reason == cart::STOP_DISTANCE_CUTOFF || reason == cart::STOP_POSITION_INVALID || reason == cart::STOP_AT_TARGET || reason==cart::STOP_APPROACH_FAILED || reason==cart::STOP_RETURN_OVERSPEED) &&
            received_us <= stopped_us) return true;
        const float demand = p.kind==cart::PROFILE_RUN ? profile.launch : p.kind == cart::THROTTLE ? p.throttle_percent : p.target;
        if (demand == 0) { stop(cart::STOP_USER_CMD, now); return true; }
        const int direction = demand > 0 ? 1 : -1;
        if (last_direction != 0 && direction != last_direction &&
            (running || now - stopped_us < cart::WHEEL_STOP_US ||
             (s.wheel.seen && now - s.wheel.last_us < cart::WHEEL_STOP_US))) {
            stop(cart::STOP_DIRECTION_CHANGE, now);
            reason = cart::STOP_DIRECTION_CHANGE;
            return true;
        }
        const bool fresh_run = !running;
        if (!running) {
            pi.reset();
            started_us = now;
            track_baseline = s.track_total;
            start_wheel_count = s.wheel.count;
            odometry.startRun();
            magnets = 0;
            reason = cart::STOP_NONE;
        }
        if(fresh_run) {
            position.setDirection(direction,false);
            fast_reverse_move = p.kind == cart::MOVE && p.target < -cart::MOVE_MAX_SPEED_MPS;
            position.value.cutoff_source=0;position.value.cutoff_m=0;
            if(p.kind==cart::PROFILE_RUN){
                profile_origin=position.value.x_m;profile_launch=true;profile_nominal=0;
                move_distance=profile.length();move_direction=1;position.value.cutoff_m=profile_origin+move_distance;
                approach.start(position.value.cutoff_m,profile.peak());
            }
            if(p.kind==cart::MOVE || p.kind==cart::APPROACH) {
                move_distance=p.distance_m;move_direction=direction;
                position.value.cutoff_m=position.value.x_m+direction*p.distance_m;
                if(p.kind==cart::APPROACH)approach.start(position.value.cutoff_m,p.target);
            }
        }
        running = true;
        mode = p.kind;
        fixed_throttle = p.throttle_percent;
        timed_speed = mode == cart::SPEED && p.duration_ms > 0;
        if (mode == cart::THROTTLE || timed_speed) {
            int64_t proposed = now+static_cast<int64_t>(p.duration_ms)*1000;
            const int64_t maximum = started_us+cart::MAX_RUNTIME_US;
            if (timed_speed && !fresh_run && test_deadline_us < proposed) proposed = test_deadline_us;
            test_deadline_us = proposed < maximum ? proposed : maximum;
            if (mode == cart::THROTTLE) pi.reset();
        }
        target = p.kind==cart::PROFILE_RUN ? profile.launch:p.target;
        if(fast_reverse_move) limitReverseTarget();
        last_direction = direction;
        return true;
    }

    void service(int64_t now, const Sensors &s, bool pi_due, float dt) {
        updateSensors(now,s);
        speed = cart::speedMagnitude(s.wheel, now, odometry.meters_per_pulse);
        if (running) {
            magnets = s.track_total - track_baseline;
            if (now - last_received_us >= cart::LINK_TIMEOUT_US) {
                stop(cart::STOP_CMD_TIMEOUT, now);
                require_zero = true;
            } else if (now - started_us >= ((mode==cart::APPROACH || mode==cart::PROFILE_RUN) ? cart::APPROACH_MAX_RUNTIME_US:cart::MAX_RUNTIME_US)) {
                stop(cart::STOP_MAX_RUNTIME, now);
            } else if((mode==cart::MOVE || mode==cart::APPROACH || mode==cart::PROFILE_RUN) && !position.valid()) {
                stop(cart::STOP_POSITION_INVALID,now);require_zero=true;
            } else if((mode==cart::APPROACH || mode==cart::PROFILE_RUN) && odometry.runDistance()>move_distance+PredictiveStop::TOLERANCE) {
                stop(cart::STOP_APPROACH_FAILED,now);require_zero=true;
            } else if(mode==cart::MOVE && fast_reverse_move && moveRemaining()<=REVERSE_SLOW_ZONE_M && speed>cart::MOVE_MAX_SPEED_MPS) {
                stop(cart::STOP_RETURN_OVERSPEED,now);require_zero=true;
            } else if(mode==cart::MOVE && (move_direction*(position.value.x_m-position.value.cutoff_m)>=0 || odometry.runDistance()>=move_distance)) {
                position.value.cutoff_source=move_direction*(position.value.x_m-position.value.cutoff_m)>=0 ? 1:2;
                stop(cart::STOP_DISTANCE_CUTOFF,now);require_zero=true;
            } else if (bench && (mode == cart::THROTTLE || timed_speed) && now >= test_deadline_us) {
                stop(cart::STOP_TEST_LIMIT, now);
            } else if (bench && mode!=cart::MOVE && mode!=cart::APPROACH && mode!=cart::PROFILE_RUN && odometry.runDistance() >= (mode==cart::THROTTLE ? cart::FIXED_THROTTLE_MAX_DISTANCE_M :
                timed_speed ? cart::TIMED_SPEED_MAX_DISTANCE_M : cart::BENCH_MAX_DISTANCE_M)) {
                stop(cart::STOP_TEST_LIMIT, now);
            } else if (bench && !((mode==cart::APPROACH || mode==cart::PROFILE_RUN) && approach.phase==PredictiveStop::NEUTRAL) && feedbackMissing(now, s)) {
                stop(cart::STOP_MOTION_TIMEOUT, now);
                require_zero = true;
            } else if (!bench && magnets >= cart::TRACK_MAGNET_LIMIT) {
                stop(cart::STOP_TRACK_DONE, now);
            }
        }
        if(running && mode==cart::MOVE && fast_reverse_move) limitReverseTarget();
        if(running && (mode==cart::APPROACH || mode==cart::PROFILE_RUN)) {
            const auto before=approach.phase;
            const bool quiet=!s.wheel.seen || now-s.wheel.last_us>=cart::WHEEL_STOP_US;
            approach.update(position.value.x_m,speed,quiet,now,mode!=cart::PROFILE_RUN);
            if(approach.phase==PredictiveStop::CREEP && before!=approach.phase){creep_wheel_count=s.wheel.count;pi.reset();}
            if(approach.phase==PredictiveStop::NEUTRAL){pi.reset();position.value.cutoff_source=1;}
            target=approach.effective;
            if(mode==cart::PROFILE_RUN){
                profile_launch=false;
                if(approach.phase==PredictiveStop::TRACK || approach.phase==PredictiveStop::DECEL){
                    bool descending=false;unsigned segment=0;
                    profile_nominal=profile.speedAt(position.value.x_m-profile_origin,descending,segment);
                    // Explicit launch/low-speed floor: a sparse Hall cannot launch at v(0)=0.
                    profile_launch=segment==0 && profile_nominal<profile.launch;
                    const float desired=fmaxf(profile_launch ? profile.launch:profile.floor,profile_nominal);
                    approach.phase=descending ? PredictiveStop::DECEL:PredictiveStop::TRACK;
                    // The same PI speed loop tracks rising AND falling targets.
                    // Only terminal stopping/fault paths force immediate neutral.
                    target=desired;
                } else {profile_nominal=0;}
            }
            if(approach.phase==PredictiveStop::SETTLED){stop(cart::STOP_AT_TARGET,now);require_zero=true;}
            else if(approach.phase==PredictiveStop::FAILED){stop(cart::STOP_APPROACH_FAILED,now);require_zero=true;}
        }
        if(!running && !position.manual() && now-stopped_us>=cart::WHEEL_STOP_US &&
            (!s.wheel.seen || now-s.wheel.last_us>=cart::WHEEL_STOP_US))position.value.direction=0;
        if (running && pi_due && (mode == cart::SPEED || mode==cart::MOVE || mode==cart::APPROACH || mode==cart::PROFILE_RUN)) pi.step(target, speed, dt, outputLimit());
    }

    void holdForMaintenance(int64_t now) {
        stop(cart::STOP_REARM_REQUIRED, now);
        require_zero = true; reason = cart::STOP_REARM_REQUIRED;
    }
    void overflow(int64_t now) {
        stop(cart::STOP_RX_OVERFLOW, now);
        reason = cart::STOP_RX_OVERFLOW;
        require_zero = true;
    }
    void pwmFailure(int64_t now) {
        stop(cart::STOP_REARM_REQUIRED, now);
        reason = cart::STOP_REARM_REQUIRED;
        faulted = true;
        require_zero = true;
    }
    float outputLimit() const {
        if (!bench) return 100.0f;
        return (mode==cart::MOVE || mode==cart::APPROACH || mode==cart::PROFILE_RUN) ? cart::MOVE_OUTPUT_CAP : mode == cart::THROTTLE ? cart::BENCH_MAX_FIXED_THROTTLE :
            timed_speed ? cart::TIMED_PI_MAX_THROTTLE : cart::BENCH_MAX_THROTTLE;
    }
    float throttle() const {
        if (!running) return 0;
        const float value = mode == cart::THROTTLE ? fixed_throttle : pi.throttle;
        const float limit = outputLimit();
        return bench ? cart::clampf(value, -limit, limit) : value;
    }
    cart::Telemetry telemetry() const {
        cart::Telemetry t = {};
        t.header = cart::makeHeader(cart::TELEMETRY);
        t.session = history.session;
        t.acknowledged_sequence = history.sequence;
        t.speed_magnitude = speed;
        t.target_speed = target;
        t.track_magnets = magnets;
        t.running = running ? 1 : 0;
        t.stop_reason = reason;
        t.mode = mode;
        t.throttle_percent = throttle();
        t.wheel_count = odometry.wheel_count;
        t.total_distance_m = static_cast<float>(odometry.total_m);
        t.run_distance_m = odometry.runDistance();
        t.marker_interval_m = odometry.marker_interval_m;
        t.marker_error_m = odometry.marker_error_m;
        t.meters_per_pulse = odometry.meters_per_pulse;
        t.calibration_id = odometry.calibration_id;
        t.marker_total = odometry.track_count;
        t.flags = (running ? cart::FLAG_RUNNING : 0) | (require_zero ? cart::FLAG_REARM : 0) |
            (faulted ? cart::FLAG_FAULT : 0) | (odometry.marker_valid ? cart::FLAG_MARKER_VALID : 0) |
            (odometry.marker_suspect ? cart::FLAG_MARKER_SUSPECT : 0) |
            (odometry.calibration_id ? cart::FLAG_CALIBRATED : 0) |
            (bench && drive_enabled ? cart::FLAG_FAST_REVERSE : 0);
        return t;
    }

    cart::PositionTelemetry positionTelemetry() const {
        return {cart::makeHeader(cart::POSITION),history.session,history.sequence,position_sequence,position.value,position_result};
    }
private:
    // Provisional command taper, independent of the forward-only stopping fit.
    static constexpr float REVERSE_SLOW_ZONE_M=.75f;
    static constexpr float REVERSE_FINAL_MPS=.75f;
    static constexpr float REVERSE_TAPER_MPS2=.5f;
    float moveRemaining() const {
        return fminf(move_direction*(position.value.cutoff_m-position.value.x_m),
                     move_distance-odometry.runDistance());
    }
    void limitReverseTarget() {
        const float taper_distance=fmaxf(moveRemaining()-REVERSE_SLOW_ZONE_M,0);
        const float cap=sqrtf(REVERSE_FINAL_MPS*REVERSE_FINAL_MPS+2*REVERSE_TAPER_MPS2*taper_distance);
        // Position corrections may advance the taper, but never raise the target again.
        target=-fminf(-target,cap);
    }
    void updateSensors(int64_t now,const Sensors &s) {
        odometry.update(s.wheel.count,s.track_total,s.track_wheel_count);
        position.update(s.wheel.count,s.track_total,s.track_wheel_count,odometry.meters_per_pulse,now);
    }
    const bool drive_enabled;
    const bool bench;
    bool feedbackMissing(int64_t now, const Sensors &s) const {
        const bool creeping=(mode==cart::APPROACH || mode==cart::PROFILE_RUN) && approach.phase==PredictiveStop::CREEP;
        if (s.wheel.count == (creeping ? creep_wheel_count:start_wheel_count))
            return now-(creeping ? approach.creep_us:started_us) >= 1500000;
        int64_t timeout = s.wheel.period_us > 0 && s.wheel.period_us < cart::WHEEL_STOP_US ? 3*s.wheel.period_us : 1000000;
        if (timeout < 200000) timeout = 200000;
        if (timeout > 1000000) timeout = 1000000;
        return now-s.wheel.last_us >= timeout;
    }
    uint32_t track_baseline = 0;
    void stop(uint8_t why, int64_t now) {
        if (running) {
            if((mode==cart::APPROACH || mode==cart::PROFILE_RUN) && why!=cart::STOP_AT_TARGET)approach.phase=PredictiveStop::FAILED;
            reason = why;
            stopped_us = now;
        }
        running = false;
        profile_launch=false;
        target = 0;
        pi.reset();
        fixed_throttle = 0;
    }
};

// PI advances only on periodic deadlines. Safety checks run on every wake.
struct Deadline {
    int64_t next_us, previous_us;
    const int64_t period_us;
    explicit Deadline(int64_t now, int64_t period = 10000)
        : next_us(now), previous_us(now - period), period_us(period) {}
    bool advance(int64_t now, float &dt) {
        if (now < next_us) return false;
        dt = (now - previous_us) / 1000000.0f;
        previous_us = now;
        next_us += ((now - next_us) / period_us + 1) * period_us;
        return true;
    }
    int64_t remaining(int64_t now) const { return next_us - now; }
};
inline uint32_t ticksForWait(int64_t remaining_us, uint32_t tick_hz) {
    if (remaining_us <= 0) return 1;
    return static_cast<uint32_t>((remaining_us * tick_hz + 999999) / 1000000);
}
} // namespace control
