#pragma once
#include <stdlib.h>
#include "ExperimentLog.h"

namespace experiment {
constexpr size_t CAPACITY = 896, BASELINE = 50;
enum State : uint8_t { OFF, ARMED, RECORDING, COAST, FROZEN, SAVED };
enum Error : uint8_t { OK, BUILD_DISABLED, BUSY, NO_MEMORY, NOT_MOUNTED, STORAGE_FULL, IO_ERROR, CORRUPT, INVALID_REQUEST };
enum RecordFlags : uint32_t { INCOMPLETE = 1, TRUNCATED = 2, QUEUE_LOSS = 4, CONTROL_FAULT = 8 };
class Recorder {
public:
    // Smaller worker-allocated blocks avoid a large contiguous heap requirement.
    static constexpr size_t BLOCK_SAMPLES=128, BLOCK_COUNT=CAPACITY/BLOCK_SAMPLES;
    Sample *blocks[BLOCK_COUNT] = {};
    Sample *buffer = nullptr; // first block; non-null only after successful allocation
    Sample &slot(size_t index) const { return blocks[index/BLOCK_SAMPLES][index%BLOCK_SAMPLES]; }
    State state = OFF;
    bool enabled = false, auto_save = false, manual = false;
    uint16_t log_hz = 50; // selected while stopped; retained with this recording
    uint32_t manual_origin = 0;
    uint32_t flags = 0, saved_id = 0;
    uint8_t error = OK;
    size_t count = 0, start = 0;
    int64_t stopped_at = 0;
    uint32_t first_drops = 0;
    bool active() const { return enabled && (state == ARMED || state == RECORDING || state == COAST); }
    ~Recorder() { for(auto block:blocks)free(block); }
    Recorder() = default;
    Recorder(const Recorder &) = delete;
    Recorder &operator=(const Recorder &) = delete;
    using Allocator = void *(*)(size_t,size_t);
    bool arm(Allocator allocate = calloc) { // worker only: control is parked in neutral maintenance
        if (state == FROZEN || active()) { error = BUSY; return false; }
        if (!buffer) {
            for(size_t i=0;i<BLOCK_COUNT;++i) {
                blocks[i]=static_cast<Sample *>(allocate(BLOCK_SAMPLES,sizeof(Sample)));
                if(!blocks[i]) {
                    for(auto &block:blocks){free(block);block=nullptr;}
                    error=NO_MEMORY;return false;
                }
            }
            buffer=blocks[0];
        }
        manual = false; log_hz = 50;
        count = start = 0; flags = 0; error = OK; saved_id = 0;
        enabled = auto_save = true; state = ARMED;
        return true;
    }
    bool armManual(uint32_t wheel_count) {
        if (!arm()) return false;
        manual = true; log_hz = 10; manual_origin = wheel_count;
        return true;
    }
    bool finishManual(bool quiet) {
        if (!manual || state != RECORDING || !quiet) { error = BUSY; return false; }
        freeze();
        return true;
    }
    void off() {
        if(state==OFF){enabled=auto_save=false;return;} // Idempotent: retain saved-buffer identity.
        if (active() && count) flags |= INCOMPLETE;
        enabled = auto_save = false;
        state = count && state != SAVED ? FROZEN : OFF;
    }
    void freeze(bool incomplete = false) {
        if (incomplete) flags |= INCOMPLETE;
        state = FROZEN;
    }
    const Sample &at(size_t i) const { return slot((start+i)%CAPACITY); }
    void sample(const Sample &s, bool quiet) { // control task only
        if (!active()) return;
        if (manual && s.packet.running) { freeze(true); flags |= CONTROL_FAULT; return; }
        if (state == ARMED && (s.packet.running || (manual && s.packet.wheel_count != manual_origin))) {
            state = RECORDING; first_drops = s.dropped;
        }
        if (state == COAST && s.packet.running) { freeze(true); return; }
        if (!manual && state == RECORDING && !s.packet.running) { state = COAST; stopped_at = s.time_us; }
        if (count == CAPACITY) { flags |= TRUNCATED | INCOMPLETE; freeze(); return; }
        slot((start+count)%CAPACITY) = s;
        ++count;
        if (state == ARMED && count > (manual ? 10u : BASELINE)) { start = (start+1)%CAPACITY; --count; }
        if (s.dropped != first_drops && state != ARMED) flags |= QUEUE_LOSS;
        if (s.packet.flags & cart::FLAG_FAULT) flags |= CONTROL_FAULT;
        if (state == COAST && ((quiet && s.time_us-stopped_at >= 2000000) || s.time_us-stopped_at >= 7000000)) freeze();
    }
};
} // namespace experiment
