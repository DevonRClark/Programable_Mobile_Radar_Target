#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace firmware {
// A shared 32-bit value with explicit ESP32 task/ISR synchronization. This
// does not depend on the compiler's std::atomic lock-free guarantees.
// Each value owns its lock. Only the integer access occurs while it is held;
// never call these methods while holding another application spinlock.
class SharedU32 {
public:
    explicit SharedU32(uint32_t initial = 0) : value_(initial) {}
    SharedU32(const SharedU32 &) = delete;
    SharedU32 &operator=(const SharedU32 &) = delete;

    uint32_t load() const {
        portENTER_CRITICAL(&mux_);
        const uint32_t result = value_;
        portEXIT_CRITICAL(&mux_);
        return result;
    }
    void store(uint32_t value) {
        portENTER_CRITICAL(&mux_);
        value_ = value;
        portEXIT_CRITICAL(&mux_);
    }
    uint32_t exchange(uint32_t value) {
        portENTER_CRITICAL(&mux_);
        const uint32_t previous = value_;
        value_ = value;
        portEXIT_CRITICAL(&mux_);
        return previous;
    }
    void increment() {
        portENTER_CRITICAL(&mux_);
        ++value_;
        portEXIT_CRITICAL(&mux_);
    }
    void IRAM_ATTR incrementFromISR() {
        portENTER_CRITICAL_ISR(&mux_);
        ++value_;
        portEXIT_CRITICAL_ISR(&mux_);
    }

private:
    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    uint32_t value_;
};
} // namespace firmware
