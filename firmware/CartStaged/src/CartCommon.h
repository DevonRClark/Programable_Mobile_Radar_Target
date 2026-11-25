#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>
#include <limits>
#include <ctype.h>

// Identical copy in both sketch folders. ESP32-to-ESP32, little-endian wire format.
namespace cart {
constexpr uint16_t MAGIC = 0xC471;
constexpr uint8_t VERSION = 3;
constexpr float BENCH_MAX_SPEED_MPS = 10.0f;
constexpr float BENCH_MAX_THROTTLE = 100.0f; // full physical PWM range
constexpr float BENCH_MAX_FIXED_THROTTLE = 100.0f;
#ifndef CART_PI_SLEW_PERCENT_PER_S
#define CART_PI_SLEW_PERCENT_PER_S 300.0f
#endif
constexpr float PI_SLEW_PERCENT_PER_S = CART_PI_SLEW_PERCENT_PER_S; // trial;100 for historical fixtures
static_assert(PI_SLEW_PERCENT_PER_S>0,"Positive PI slew rate required");
constexpr uint8_t LOG_EVENT_SLEW_LIMITED = 0x04; // sampled PI state; low event bit2
constexpr float TIMED_PI_MAX_THROTTLE = 100.0f; // independent of fixed-throttle experiments
constexpr uint32_t FIXED_THROTTLE_MAX_MS = 2000;
constexpr float FIXED_THROTTLE_MAX_DISTANCE_M = 25.0f;
constexpr float BENCH_MAX_DISTANCE_M = 26.0f;
constexpr float MAX_SPEED_MPS = 10.0f;
// Radar target fitted: six measured3m pushes each111 pulses; user-approved boot calibration.
constexpr float RIGHT_DIST_PER_PULSE_M = 3.0f / 111.0f;
constexpr float LEFT_DIST_PER_PULSE_M = 7.0f / 264.0f;
constexpr float DIST_PER_PULSE_M = LEFT_DIST_PER_PULSE_M;
constexpr uint32_t TIMED_SPEED_MAX_MS = 3000;
constexpr float TIMED_SPEED_MAX_DISTANCE_M = 15.0f;
constexpr int64_t WHEEL_DEBOUNCE_US = 1000;
constexpr int64_t TRACK_DEBOUNCE_US = 20000;
constexpr int64_t WHEEL_STOP_US = 2000000;
constexpr int64_t LINK_TIMEOUT_US = 500000;
constexpr int64_t MAX_RUNTIME_US = 60000000;
constexpr int64_t APPROACH_MAX_RUNTIME_US = 60000000;
constexpr uint32_t TRACK_MAGNET_LIMIT = 3;
constexpr float TRACK_FIRST_MARKER_M=.792f, TRACK_SPACING_M=1.0f, TRACK_LENGTH_M=26.0f;
constexpr unsigned TRACK_MARKER_COUNT=26;
constexpr float POSITION_GATE_M=.10f, MOVE_MAX_SPEED_MPS=1.0f, MOVE_MAX_DISTANCE_M=26.0f;
constexpr float REVERSE_MOVE_MAX_SPEED_MPS=2.0f;
inline float moveSpeedLimit(float target) { return target < 0 ? REVERSE_MOVE_MAX_SPEED_MPS : MOVE_MAX_SPEED_MPS; }
constexpr float APPROACH_MAX_SPEED_MPS=10.0f;
constexpr float MOVE_OUTPUT_CAP=100.0f;
constexpr unsigned PROFILE_MAX_POINTS=32;
constexpr float PROFILE_SPEED_FLOOR_MPS=1.0f; // operator-requested minimum moving profile target; terminal stop/creep override
constexpr float PROFILE_MAX_DISTANCE_M=26, PROFILE_MAX_ACCEL_MPS2=3.5f, PROFILE_MAX_DECEL_MPS2=4;

enum CommandKind : uint8_t { SPEED = 0, STOP = 1, THROTTLE = 2, MOVE = 3, ODOM_RESET = 4, CALIBRATE = 5, LOG_ARM = 6, LOG_OFF = 7, LOG_STATUS = 8, LOG_SAVE = 9, PI_SET = 10, PI_STATUS = 11, LOG_HAND = 12, LOG_FINISH = 13, HOME = 14, POSITION_STATUS = 15, POSITION_CLEAR = 16, POSITION_DIRECTION = 17, APPROACH = 18, PROFILE_BEGIN=19, PROFILE_POINT=20, PROFILE_COMMIT=21, PROFILE_STATUS=22, PROFILE_RUN=23 };
enum TelemetryFlags : uint8_t { FLAG_RUNNING = 1, FLAG_REARM = 2, FLAG_FAULT = 4,
    FLAG_MARKER_VALID = 8, FLAG_MARKER_SUSPECT = 16, FLAG_CALIBRATED = 32, FLAG_FAST_REVERSE = 64 };
enum PacketType : uint8_t { HEARTBEAT = 1, COMMAND = 2, TELEMETRY = 3, PI_REPLY = 6, POSITION = 7, PROFILE_REPLY=8 };
enum StopReason : uint8_t {
    STOP_NONE = 0, STOP_USER_CMD = 1, STOP_TRACK_DONE = 2,
    STOP_CMD_TIMEOUT = 3, STOP_MAX_RUNTIME = 4, STOP_RX_OVERFLOW = 5,
    STOP_REMOTE_RESTART = 6, STOP_DIRECTION_CHANGE = 7, STOP_REARM_REQUIRED = 8,
    STOP_UNSUPPORTED = 9, STOP_INVALID_COMMAND = 10, STOP_MOTION_TIMEOUT = 11,
    STOP_TEST_LIMIT = 12, STOP_ODOM_RESET = 13, STOP_CALIBRATED = 14,
    STOP_CALIBRATION_INVALID = 15, STOP_DISTANCE_CUTOFF = 16, STOP_POSITION_INVALID = 17, STOP_MOVE_REFUSED = 18, STOP_AT_TARGET = 19, STOP_APPROACH_FAILED = 20, STOP_RETURN_OVERSPEED = 21
};

constexpr float DEFAULT_PI_KP=10.0f, DEFAULT_PI_KI=20.0f;
constexpr float MAX_PI_KP=50.0f, MAX_PI_KI=100.0f;
enum PiResult : uint8_t { PI_QUERIED=0, PI_APPLIED=1, PI_BUSY=2, PI_INVALID=3, PI_UNSUPPORTED=4 };
inline bool validPiGains(float kp,float ki) {
    return isfinite(kp) && isfinite(ki) && kp>=0 && kp<=MAX_PI_KP && ki>=0 && ki<=MAX_PI_KI && (kp>0 || ki>0);
}
enum ProfileResult : uint8_t { PROFILE_QUERIED=0, PROFILE_APPLIED=1, PROFILE_BUSY=2, PROFILE_INVALID=3, PROFILE_UNSUPPORTED=4 };
enum PositionReason : uint8_t { POS_UNHOMED=0, POS_OK=1, POS_UNCOMMANDED=2, POS_COUNTER=3,
    POS_MARKER_COUNT=4, POS_MARKER_GATE=5, POS_MARKER_MISSED=6, POS_BOUNDS=7, POS_CAL_CHANGED=8, POS_CLEARED=9 };
enum PositionResult : uint8_t { POS_STATUS=0, POS_APPLIED=1, POS_BUSY=2, POS_INVALID=3, POS_UNSUPPORTED=4 };
enum PositionFlags : uint8_t { POS_KNOWN=1, POS_VALID=2, POS_MANUAL=4, POS_MARKER_CHECKED=8 };
#pragma pack(push, 1)
struct PositionSample {
    float raw_m=0, x_m=0, correction_m=0, cutoff_m=0;
    uint32_t epoch=0;
    int8_t direction=0;
    uint8_t flags=0, reason=POS_UNHOMED, cutoff_source=0;
};
struct Header { uint16_t magic; uint8_t version; uint8_t type; };
struct Heartbeat { Header header; uint32_t session; };
struct Command {
    Header header;
    uint32_t session;
    uint32_t sequence;
    float target_speed;
    float distance_m = 0;
    float throttle_percent = 0;
    uint32_t duration_ms = 0;
    uint8_t kind = SPEED;
};
// Separate reply preserves telemetry/sample layouts and existing recordings.
struct PiReply { Header header; uint32_t session, sequence; float kp,ki; uint8_t result; };
struct ProfileReply { Header header; uint32_t session,sequence,crc; float duration; uint8_t count,received,uploading,result; float speed_floor=PROFILE_SPEED_FLOOR_MPS,launch_speed=PROFILE_SPEED_FLOOR_MPS; };
static_assert(sizeof(ProfileReply)==32,"Profile reply ABI changed");
struct Telemetry {
    Header header;
    uint32_t session;
    uint32_t acknowledged_sequence;
    float speed_magnitude;
    float target_speed;
    uint32_t track_magnets;
    uint8_t running;
    uint8_t stop_reason;
    uint8_t mode = SPEED;
    float throttle_percent = 0;
    uint32_t wheel_count = 0;
    float total_distance_m = 0;
    float run_distance_m = 0;
    float marker_interval_m = 0;
    float marker_error_m = 0;
    float meters_per_pulse = DIST_PER_PULSE_M;
    uint8_t flags = 0;
    uint32_t calibration_id = 0;
    uint32_t marker_total = 0;
    uint8_t log_state = 0, log_error = 0;
    uint16_t log_samples = 0;
    uint32_t log_flags = 0, log_saved_id = 0;
};
struct PositionTelemetry {
    Header header; uint32_t session, acknowledged_sequence, request_sequence;
    PositionSample position; uint8_t result;
};
#pragma pack(pop)

static_assert(sizeof(PositionSample)==24 && sizeof(PositionTelemetry)==41, "Position layout changed");
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "Protocol requires IEEE-754 32-bit floats");
static_assert(sizeof(Header) == 4 && sizeof(Heartbeat) == 8, "Wire layout changed");
static_assert(sizeof(Command) == 29 && sizeof(Telemetry) == 76, "Wire layout changed");

inline Header makeHeader(PacketType type) { return {MAGIC, VERSION, type}; }
inline bool validHeader(const Header &h, PacketType type) {
    return h.magic == MAGIC && h.version == VERSION && h.type == type;
}
inline float clampf(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
}
inline bool newerSequence(uint32_t candidate, uint32_t previous) {
    const uint32_t distance = candidate - previous;
    return distance != 0 && distance < 0x80000000u;
}

// Captured under the ISR/task spinlock. A first edge has no measurable period.
struct WheelSample { bool seen = false; int64_t last_us = 0; int64_t period_us = 0; uint32_t count = 0; };
inline float speedMagnitude(const WheelSample &wheel, int64_t now_us, float meters_per_pulse = DIST_PER_PULSE_M) {
    const int64_t age = now_us - wheel.last_us;
    if (!wheel.seen || wheel.period_us <= 0 || age < 0 || age >= WHEEL_STOP_US) {
        return 0.0f;
    }
    // Hold the actual edge-to-edge estimate until the next edge is overdue;
    // then decay it. Do not divide by a tiny age immediately after an edge.
    const int64_t denominator = age > wheel.period_us ? age : wheel.period_us;
    return meters_per_pulse * 1000000.0f / static_cast<float>(denominator);
}

struct PIController {
    // First data-guided tuning candidate; powered validation still required.
    // Kp: throttle percentage points / (m/s); Ki: percentage points / metre.
    // Default Kp/Ki = 0.5 s; Ki20 candidate reduced measured5m/s overshoot.
    float kp = DEFAULT_PI_KP;
    float ki = DEFAULT_PI_KI;
    float integral = 0.0f;
    float throttle = 0.0f;
    bool slew_limited = false;

    void reset() { integral = 0.0f; throttle = 0.0f; slew_limited = false; }
    float step(float target, float measured_magnitude, float dt_seconds, float output_limit = 100.0f) {
        if (!isfinite(target) || !isfinite(measured_magnitude) ||
            !isfinite(dt_seconds) || dt_seconds <= 0 || target == 0) {
            reset();
            return 0.0f;
        }
        // Bound recovery after a scheduling delay; normal operation uses actual dt.
        const float dt = dt_seconds < 0.1f ? dt_seconds : 0.1f;
        const float error = fabsf(target) - measured_magnitude;
        const float p = kp * error;
        const float candidate = clampf(integral + error * dt, -20.0f, 20.0f);
        const float proposed = p + ki * candidate;
        if ((proposed >= 0.0f && proposed <= output_limit) ||
            (proposed > output_limit && error < 0.0f) ||
            (proposed < 0.0f && error > 0.0f)) {
            integral = candidate;
        }
        const float magnitude = clampf(p + ki * integral, 0.0f, output_limit);
        const float requested = target > 0 ? magnitude : -magnitude;
        const float max_change = PI_SLEW_PERCENT_PER_S * dt;
        throttle = clampf(requested, throttle - max_change, throttle + max_change);
        slew_limited = fabsf(requested-throttle) > .0001f;
        return throttle;
    }
};

// Tracks commands independently of run state: retries cannot restart a finished run.
struct CommandHistory {
    uint32_t session = 0;
    uint32_t sequence = 0;
    bool setSession(uint32_t next) {
        if (next == session) return false;
        session = next;
        sequence = 0;
        return true;
    }
    bool accept(uint32_t remote_session, uint32_t next_sequence) {
        if (remote_session != session || next_sequence == 0) return false;
        if (sequence != 0 && !newerSequence(next_sequence, sequence)) return false;
        sequence = next_sequence;
        return true;
    }
};

enum LineResult : uint8_t { LINE_PENDING, LINE_READY, LINE_REJECTED };
struct LineBuffer {
    char bytes[97] = {};
    size_t length = 0;
    bool rejected = false;
    LineResult push(char ch) {
        if (ch == '\r') return LINE_PENDING; // CRLF serial monitors.
        if (ch == '\n') {
            bytes[length] = '\0';
            const bool bad = rejected;
            length = 0;
            rejected = false;
            return bad ? LINE_REJECTED : LINE_READY;
        }
        if (rejected) return LINE_PENDING;
        if ((static_cast<unsigned char>(ch) < 32 && ch != '\t') || length == 96) {
            rejected = true; // Discard the whole line, never execute its prefix.
        } else {
            bytes[length++] = ch;
        }
        return LINE_PENDING;
    }
};
inline bool parseSpeed(const char *text, float &speed) {
    errno = 0;
    char *end = nullptr;
    const float parsed = strtof(text, &end);
    if (end == text || !isfinite(parsed) || errno == ERANGE) return false;
    while (*end != '\0' && isspace(static_cast<unsigned char>(*end))) ++end;
    if (*end != '\0') return false;
    speed = clampf(parsed, -MAX_SPEED_MPS, MAX_SPEED_MPS);
    return true;
}
inline bool isStop(const Command &c) {
    return c.kind == STOP || (c.kind == SPEED && c.target_speed == 0);
}
inline bool isLogCommand(uint8_t kind) {
    return (kind >= LOG_ARM && kind <= LOG_SAVE) || kind == LOG_HAND || kind == LOG_FINISH;
}
inline bool isProfileConfig(uint8_t kind){return kind>=PROFILE_BEGIN && kind<=PROFILE_STATUS;}
inline bool validCommand(const Command &c) {
    if (!c.session || !c.sequence || c.kind > PROFILE_RUN || !isfinite(c.target_speed) ||
        !isfinite(c.distance_m) || !isfinite(c.throttle_percent)) return false;
    switch (c.kind) {
        case PROFILE_BEGIN: return c.target_speed>=.1f && c.target_speed<=MAX_SPEED_MPS && c.distance_m>=c.target_speed && c.distance_m<=MAX_SPEED_MPS && c.throttle_percent==0 && c.duration_ms>=2 && c.duration_ms<=PROFILE_MAX_POINTS;
        case PROFILE_POINT: return c.throttle_percent==0 && c.duration_ms<PROFILE_MAX_POINTS && c.target_speed>=0 && c.target_speed<=MAX_SPEED_MPS && c.distance_m>=0 && c.distance_m<=PROFILE_MAX_DISTANCE_M;
        case PROFILE_RUN: return c.target_speed>=.1f && c.target_speed<=MAX_SPEED_MPS && c.distance_m>=c.target_speed && c.distance_m<=MAX_SPEED_MPS && c.throttle_percent==0;
        case PROFILE_COMMIT: return c.target_speed==0 && c.distance_m==0 && c.throttle_percent==0;
        case PROFILE_STATUS: return c.target_speed==0 && c.distance_m==0 && c.throttle_percent==0 && c.duration_ms==0;
        // PI_SET reuses otherwise inactive payload slots: distance_m=Kp,
        // throttle_percent=Ki. No motion fields or wire-size changes.
        case HOME: return c.target_speed==0 && c.duration_ms==0 && c.throttle_percent==0 && c.distance_m>=0 && c.distance_m<=TRACK_LENGTH_M;
        case POSITION_DIRECTION: return (c.target_speed==0 || c.target_speed==1 || c.target_speed==-1) && c.duration_ms==0 && c.distance_m==0 && c.throttle_percent==0;
        case POSITION_STATUS: case POSITION_CLEAR: return c.target_speed==0 && c.duration_ms==0 && c.distance_m==0 && c.throttle_percent==0;
        case PI_SET: return c.target_speed==0 && c.duration_ms==0 && validPiGains(c.distance_m,c.throttle_percent);
        case PI_STATUS: return c.target_speed==0 && c.duration_ms==0 && c.distance_m==0 && c.throttle_percent==0;
        case STOP: return c.target_speed == 0 && c.distance_m == 0 && c.throttle_percent == 0 && c.duration_ms == 0;
        case SPEED: return fabsf(c.target_speed) <= MAX_SPEED_MPS && c.distance_m == 0 && c.throttle_percent == 0 && (c.duration_ms == 0 || (c.target_speed != 0 && c.duration_ms >= 50 && c.duration_ms <= TIMED_SPEED_MAX_MS));
        case THROTTLE: return c.target_speed == 0 && c.distance_m == 0 && fabsf(c.throttle_percent) <= BENCH_MAX_FIXED_THROTTLE && c.duration_ms >= 50 && c.duration_ms <= FIXED_THROTTLE_MAX_MS;
        case APPROACH: return c.target_speed>=.5f && c.target_speed<=APPROACH_MAX_SPEED_MPS && c.distance_m>=.5f && c.distance_m<=MOVE_MAX_DISTANCE_M && c.duration_ms==0 && c.throttle_percent==0;
        case MOVE: return fabsf(c.target_speed) > 0 && fabsf(c.target_speed) <= moveSpeedLimit(c.target_speed) && c.distance_m > 0 && c.distance_m <= MOVE_MAX_DISTANCE_M && c.throttle_percent == 0 && c.duration_ms == 0;
        case LOG_ARM: return c.target_speed==0 && c.distance_m==0 && c.throttle_percent==0 && (c.duration_ms==0 || c.duration_ms==10 || c.duration_ms==50);
        case LOG_HAND: case LOG_FINISH: case LOG_OFF: case LOG_STATUS: case LOG_SAVE:
        case ODOM_RESET: return c.target_speed == 0 && c.distance_m == 0 && c.throttle_percent == 0 && c.duration_ms == 0;
        case CALIBRATE: return c.target_speed == 0 && c.distance_m > 0 && c.distance_m <= 5 && c.throttle_percent == 0 && c.duration_ms == 0;
        default: return false;
    }
}
inline const char *kindName(uint8_t kind) {
    switch(kind) {
        case PROFILE_BEGIN:return "PROFILE_BEGIN"; case PROFILE_POINT:return "PROFILE_POINT"; case PROFILE_COMMIT:return "PROFILE_COMMIT"; case PROFILE_STATUS:return "PROFILE_STATUS"; case PROFILE_RUN:return "PROFILE_RUN";
        case APPROACH: return "APPROACH";
        case HOME: return "HOME";
        case POSITION_STATUS: return "POSITION_STATUS";
        case POSITION_CLEAR: return "POSITION_CLEAR";
        case POSITION_DIRECTION: return "POSITION_DIRECTION";
        case PI_SET: return "PI_SET";
        case PI_STATUS: return "PI_STATUS";
        case LOG_HAND: return "LOG_HAND";
        case LOG_FINISH: return "LOG_FINISH";
        case LOG_ARM: return "LOG_ARM";
        case LOG_OFF: return "LOG_OFF";
        case LOG_STATUS: return "LOG_STATUS";
        case LOG_SAVE: return "LOG_SAVE";
        case SPEED: return "SPEED";
        case STOP: return "STOP";
        case THROTTLE: return "THROTTLE";
        case MOVE: return "MOVE";
        case ODOM_RESET: return "ODOM_RESET";
        case CALIBRATE: return "CALIBRATE";
        default: return "INVALID";
    }
}
inline const char *skipSpace(const char *p) {
    while (*p && isspace(static_cast<unsigned char>(*p))) ++p;
    return p;
}
inline bool readNumber(const char *&p, float &value) {
    p = skipSpace(p);
    char *end = nullptr;
    errno = 0;
    const float parsed = strtof(p, &end);
    if (end == p || errno == ERANGE || !isfinite(parsed) || (*end && !isspace(static_cast<unsigned char>(*end)))) return false;
    value = parsed;
    p = end;
    return true;
}
inline bool readUnsigned(const char *&p,uint32_t &value){
    p=skipSpace(p);if(*p<'0'||*p>'9')return false;errno=0;char *end=nullptr;
    unsigned long n=strtoul(p,&end,10);
    if(end==p || errno==ERANGE || n>UINT32_MAX || (*end && !isspace(static_cast<unsigned char>(*end))))return false;
    p=end;value=static_cast<uint32_t>(n);return true;
}
// Parses into a temporary; rejected lines cannot execute a valid-looking prefix.
inline bool parseCommand(const char *text, Command &out) {
    const char *p = skipSpace(text);
    Command c = {};
    c.header = makeHeader(COMMAND);
    c.session = c.sequence = 1; // placeholders; the dongle owns actual identity
    float seconds = 0;
    if (!strncmp(p,"profile",7) && isspace(static_cast<unsigned char>(p[7]))) {
        p=skipSpace(p+7);
        if(!strncmp(p,"begin",5) && isspace(static_cast<unsigned char>(p[5]))){p+=5;c.kind=PROFILE_BEGIN;c.target_speed=PROFILE_SPEED_FLOOR_MPS;if(!readUnsigned(p,c.duration_ms)||(*skipSpace(p)&&!readNumber(p,c.target_speed)))return false;c.distance_m=c.target_speed;if(*skipSpace(p)&&!readNumber(p,c.distance_m))return false;}
        else if(!strncmp(p,"point",5) && isspace(static_cast<unsigned char>(p[5]))){p+=5;c.kind=PROFILE_POINT;if(!readUnsigned(p,c.duration_ms)||!readNumber(p,c.distance_m)||!readNumber(p,c.target_speed))return false;}
        else if(!strncmp(p,"commit",6) && isspace(static_cast<unsigned char>(p[6]))){p+=6;c.kind=PROFILE_COMMIT;if(!readUnsigned(p,c.duration_ms))return false;}
        else if(!strncmp(p,"run",3) && isspace(static_cast<unsigned char>(p[3]))){p+=3;c.kind=PROFILE_RUN;c.target_speed=PROFILE_SPEED_FLOOR_MPS;if(!readUnsigned(p,c.duration_ms)||(*skipSpace(p)&&!readNumber(p,c.target_speed)))return false;c.distance_m=c.target_speed;if(*skipSpace(p)&&!readNumber(p,c.distance_m))return false;}
        else if(!strcmp(p,"status")){p+=6;c.kind=PROFILE_STATUS;}
        else return false;
    } else if (!strncmp(p,"home",4) && (!p[4] || isspace(static_cast<unsigned char>(p[4])))) {
        p+=4; c.kind=HOME; if (*skipSpace(p) && !readNumber(p,c.distance_m)) return false;
    } else if (!strncmp(p,"position",8) && isspace(static_cast<unsigned char>(p[8]))) {
        p=skipSpace(p+8);
        if (!strncmp(p,"status",6) && !*skipSpace(p+6)) { c.kind=POSITION_STATUS; p+=6; }
        else if (!strncmp(p,"invalidate",10) && !*skipSpace(p+10)) { c.kind=POSITION_CLEAR; p+=10; }
        else if (!strncmp(p,"direction",9) && isspace(static_cast<unsigned char>(p[9]))) {
            p+=9;c.kind=POSITION_DIRECTION;if(!readNumber(p,c.target_speed))return false;
        } else return false;
    } else if (strncmp(p,"pi",2)==0 && isspace(static_cast<unsigned char>(p[2]))) {
        p=skipSpace(p+2);
        if (!strncmp(p,"status",6) && !*skipSpace(p+6)) { c.kind=PI_STATUS; p+=6; }
        else if (!strncmp(p,"defaults",8) && !*skipSpace(p+8)) { c.kind=PI_SET; c.distance_m=DEFAULT_PI_KP; c.throttle_percent=DEFAULT_PI_KI; p+=8; }
        else if (!strncmp(p,"set",3) && isspace(static_cast<unsigned char>(p[3]))) {
            p+=3; c.kind=PI_SET;
            if (!readNumber(p,c.distance_m) || !readNumber(p,c.throttle_percent)) return false;
        } else return false;
    } else if (strncmp(p, "log", 3) == 0 && isspace(static_cast<unsigned char>(p[3]))) {
        p = skipSpace(p+3);
        const char *names[] = {"arm", "off", "status", "save", "hand", "finish"};
        const uint8_t kinds[] = {LOG_ARM, LOG_OFF, LOG_STATUS, LOG_SAVE, LOG_HAND, LOG_FINISH};
        bool found = false;
        for (unsigned i = 0; i < 6; ++i) {
            const size_t n = strlen(names[i]);
            if (strncmp(p, names[i], n) == 0 && (!p[n] || isspace(static_cast<unsigned char>(p[n])))) {
                c.kind = kinds[i]; p += n;
                if (c.kind==LOG_ARM && *skipSpace(p)) {
                    float hz=0; if(!readNumber(p,hz) || (hz!=10 && hz!=50)) return false;
                    c.duration_ms=static_cast<uint32_t>(hz); // stopped-only log rate, not motion duration
                }
                found = true; break;
            }
        }
        if (!found) return false;
    } else if (strncmp(p, "speed", 5) == 0 && isspace(static_cast<unsigned char>(p[5]))) {
        p += 5; c.kind = SPEED;
        if (!readNumber(p, c.target_speed)) return false;
        if (*skipSpace(p)) {
            if (!readNumber(p, seconds) || seconds < 0.05f || seconds*1000 > TIMED_SPEED_MAX_MS) return false;
            c.duration_ms = static_cast<uint32_t>(lroundf(seconds*1000));
        }
    } else if (strncmp(p, "throttle", 8) == 0 && isspace(static_cast<unsigned char>(p[8]))) {
        p += 8; c.kind = THROTTLE;
        if (!readNumber(p, c.throttle_percent) || !readNumber(p, seconds) || seconds < 0.05f || seconds > 2) return false;
        c.duration_ms = static_cast<uint32_t>(lroundf(seconds * 1000));
    } else if (!strncmp(p,"approach",8) && isspace(static_cast<unsigned char>(p[8]))) {
        p+=8;c.kind=APPROACH;if(!readNumber(p,c.distance_m)||!readNumber(p,c.target_speed))return false;
    } else if (strncmp(p, "move", 4) == 0 && isspace(static_cast<unsigned char>(p[4]))) {
        p += 4; c.kind = MOVE;
        if (!readNumber(p, c.distance_m) || !readNumber(p, c.target_speed)) return false;
    } else if (strncmp(p, "calibrate", 9) == 0 && isspace(static_cast<unsigned char>(p[9]))) {
        p += 9; c.kind = CALIBRATE;
        if (!readNumber(p, c.distance_m)) return false;
    } else if (strncmp(p, "odom", 4) == 0 && isspace(static_cast<unsigned char>(p[4]))) {
        p = skipSpace(p+4);
        if (strncmp(p, "reset", 5) != 0 || (*skipSpace(p+5))) return false;
        p += 5; c.kind = ODOM_RESET;
    } else {
        if (!readNumber(p, c.target_speed)) return false; // legacy numeric speed alias
        c.kind = c.target_speed == 0 ? STOP : SPEED;
    }
    if (*skipSpace(p) || fabsf(c.target_speed) > ((c.kind==APPROACH || c.kind==PROFILE_POINT) ? APPROACH_MAX_SPEED_MPS : BENCH_MAX_SPEED_MPS) || !validCommand(c)) return false;
    out = c;
    return true;
}
inline const char *stopReasonName(uint8_t reason) {
    switch (reason) {
        case STOP_DISTANCE_CUTOFF: return "DISTANCE CUTOFF";
        case STOP_POSITION_INVALID: return "POSITION INVALID";
        case STOP_AT_TARGET: return "AT TARGET";
        case STOP_RETURN_OVERSPEED: return "RETURN TOO FAST NEAR HOME";
        case STOP_APPROACH_FAILED: return "APPROACH FAILED";
        case STOP_MOVE_REFUSED: return "MOVE REFUSED";
        case STOP_NONE: return "NONE";
        case STOP_USER_CMD: return "USER CMD";
        case STOP_TRACK_DONE: return "TRACK DONE";
        case STOP_CMD_TIMEOUT: return "LINK TIMEOUT";
        case STOP_MAX_RUNTIME: return "MAX RUNTIME";
        case STOP_RX_OVERFLOW: return "RX OVERFLOW";
        case STOP_REMOTE_RESTART: return "DONGLE RESTART";
        case STOP_DIRECTION_CHANGE: return "DIRECTION CHANGE";
        case STOP_REARM_REQUIRED: return "ENTER ZERO TO ARM";
        case STOP_UNSUPPORTED: return "MOVE NOT IMPLEMENTED";
        case STOP_INVALID_COMMAND: return "COMMAND REFUSED";
        case STOP_MOTION_TIMEOUT: return "MOTION FEEDBACK TIMEOUT";
        case STOP_TEST_LIMIT: return "TEST LIMIT";
        case STOP_ODOM_RESET: return "ODOMETRY RESET";
        case STOP_CALIBRATED: return "CALIBRATION APPLIED";
        case STOP_CALIBRATION_INVALID: return "CALIBRATION REFUSED";
        default: return "UNKNOWN";
    }
}
} // namespace cart
