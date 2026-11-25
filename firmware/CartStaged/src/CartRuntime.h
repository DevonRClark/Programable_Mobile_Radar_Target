#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <esp_arduino_version.h>
#include <esp_idf_version.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "SharedU32.h"
#include "Settings.h"
#include "ControlCore.h"
#include "ProbeProtocol.h"
#include "LogRadio.h"
#include "ExperimentLog.h"
#if CART_ENABLE_EXPERIMENT_LOG && CART_EXPERIMENT_PROFILE == 1 && CART_TEST_STAGE >= 2
#define CART_RECORDER_AVAILABLE 1
#include <LittleFS.h>
#include "RunStorage.h"
#else
#define CART_RECORDER_AVAILABLE 0
#endif

#if ESP_ARDUINO_VERSION_MAJOR < 3 || !defined(CONFIG_IDF_TARGET_ESP32)
#error "The CART requires a classic ESP32 board with Arduino-ESP32 Core 3.x."
#endif
#if defined(CONFIG_FREERTOS_UNICORE) && CONFIG_FREERTOS_UNICORE
#error "This staged CART build requires the classic ESP32 dual-core configuration."
#endif

namespace firmware {
enum Phase : uint32_t { BOOT, RX_QUEUE, SENSORS, CONTROL, PWM_WRITE, PUBLISH, WAIT, OVERRUN_WAIT, MAINTENANCE };
struct Diagnostics {
    SharedU32 loop_count{0}, loop_ms{0}, control_count{0}, control_ms{0};
    SharedU32 pi_count{0}, early_wakes{0}, work_last_us{0}, work_max_us{0};
    SharedU32 report_stack_hwm{0};
    SharedU32 phase{BOOT}, overruns{0}, stack_hwm{0}, pwm_calls{0}, pwm_fault{0};
    SharedU32 pwm_last_us{0}, pwm_max_us{0}, pwm_slow{0}, log_dropped{0};
    SharedU32 rx_raw{0}, rx_drop{0}, wrong_sender{0}, callback_bad{0};
    SharedU32 valid{0}, parse_bad{0}, stale_input{0}, refused{0};
    SharedU32 tx_queued{0}, tx_error{0}, tx_ok{0}, tx_fail{0}, tx_busy{0};
    SharedU32 wheel_raw{0}, wheel_accepted{0}, track_raw{0}, track_accepted{0};
    SharedU32 session{0}, ack{0}, running{0}, magnets{0}, stop_reason{0};
    SharedU32 frame_age_ms{UINT32_MAX};
} diag;

// Each operation protects only one integer, separately from the sensor lock.
inline void bump(SharedU32 &value) { value.increment(); }
inline void IRAM_ATTR bumpFromISR(SharedU32 &value) { value.incrementFromISR(); }
inline uint32_t get(const SharedU32 &value) { return value.load(); }
inline void phase(Phase p) { diag.phase.store(p); }

struct RxEvent {
    int length;
    int64_t received_us;
    uint8_t bytes[sizeof(cart::Command)];
};
struct Frame { cart::Telemetry packet; int64_t generated_us; cart::PositionTelemetry position = {}; };
struct SensorState {
    control::Sensors sample;
    bool track_seen = false;
    int64_t last_track_us = 0;
};
QueueHandle_t rx_queue = nullptr, telemetry_queue = nullptr, tx_status_queue = nullptr, log_queue = nullptr;
TaskHandle_t control_task = nullptr;
portMUX_TYPE sensor_mux = portMUX_INITIALIZER_UNLOCKED;
SensorState sensor_state;
SharedU32 rx_overflow{0};
control::Engine engine(settings::DRIVE, settings::BENCH); // owned exclusively by control task
probe::State radio_probe;                // owned exclusively by loop in stage 1
QueueHandle_t profile_reply_queue = nullptr;
cart::ProfileReply profile_outgoing = {};
QueueHandle_t pi_reply_queue = nullptr; // one stopped-only configuration response
cart::PiReply pi_outgoing = {}; // stable until send completion
bool pwm_ready = false;                 // set before control-task creation
bool tx_in_flight = false;              // owned exclusively by loop
uint32_t last_tx_ms = 0;
uint32_t last_pwm_duty = UINT32_MAX;     // setup, then exclusively control task
cart::PositionTelemetry position_outgoing = {};
uint32_t last_position_tx_ms=0;
cart::Telemetry outgoing = {};         // held unchanged until send completion

control::Sensors readSensors() {
    control::Sensors copy;
    phase(SENSORS);
    portENTER_CRITICAL(&sensor_mux);
    copy = sensor_state.sample;
    portEXIT_CRITICAL(&sensor_mux);
    copy.right_wheel=copy.wheel;
    copy.track_right_wheel_count=copy.track_wheel_count;
    if(settings::LEFT_FEEDBACK) {
        copy.wheel=copy.left_wheel;
        copy.track_wheel_count=copy.track_left_wheel_count;
    }
    return copy;
}

void IRAM_ATTR wheelISR() {
    if(!settings::LEFT_FEEDBACK)bumpFromISR(diag.wheel_raw);
    const int64_t now = esp_timer_get_time();
    bool accepted = false;
    portENTER_CRITICAL_ISR(&sensor_mux);
    cart::WheelSample &wheel = sensor_state.sample.wheel;
    const int64_t period = now - wheel.last_us;
    if (!wheel.seen || period >= cart::WHEEL_DEBOUNCE_US) {
        wheel.period_us = wheel.seen && period < cart::WHEEL_STOP_US ? period : 0;
        wheel.last_us = now;
        wheel.seen = true;
        ++wheel.count;
        accepted = true;
    }
    portEXIT_CRITICAL_ISR(&sensor_mux);
    if (accepted && !settings::LEFT_FEEDBACK) bumpFromISR(diag.wheel_accepted);
}

// Independent left capture; no wakeup, allocation, logging or controller update in ISR.
void IRAM_ATTR leftWheelISR() {
    if(settings::LEFT_FEEDBACK)bumpFromISR(diag.wheel_raw);
    bool accepted=false;
    const int64_t now=esp_timer_get_time();
    portENTER_CRITICAL_ISR(&sensor_mux);
    auto &wheel=sensor_state.sample.left_wheel;
    const int64_t period=now-wheel.last_us;
    if(!wheel.seen || period>=cart::WHEEL_DEBOUNCE_US) {
        wheel.period_us=wheel.seen && period<cart::WHEEL_STOP_US ? period:0;
        wheel.last_us=now;wheel.seen=true;++wheel.count;accepted=true;
    }
    portEXIT_CRITICAL_ISR(&sensor_mux);
    if(accepted && settings::LEFT_FEEDBACK)bumpFromISR(diag.wheel_accepted);
}

void IRAM_ATTR trackISR() {
    bumpFromISR(diag.track_raw);
    const int64_t now = esp_timer_get_time();
    bool accepted = false;
    portENTER_CRITICAL_ISR(&sensor_mux);
    if (!sensor_state.track_seen || now - sensor_state.last_track_us >= cart::TRACK_DEBOUNCE_US) {
        sensor_state.track_seen = true;
        sensor_state.last_track_us = now;
        ++sensor_state.sample.track_total;
        sensor_state.sample.track_wheel_count = sensor_state.sample.wheel.count;
        sensor_state.sample.track_left_wheel_count = sensor_state.sample.left_wheel.count;
        accepted = true;
    }
    portEXIT_CRITICAL_ISR(&sensor_mux);
    if (accepted) bumpFromISR(diag.track_accepted);
    // Wake on each debounced track edge. The task stops on the third run edge.
    if (accepted && control_task != nullptr) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(control_task, &woken);
        if (woken) portYIELD_FROM_ISR();
    }
}

bool receiveLogRadio(const uint8_t *data, int length);

// Wi-Fi task: validate only envelope/sender, copy into a bounded queue, notify.
// Only per-counter/latch critical sections; no sensor lock, logging, PWM,
// peer creation, or controller state here. Queue/notify calls occur unlocked.
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int length) {
    bump(diag.rx_raw);
    if (!info || !info->src_addr || !data || length < static_cast<int>(sizeof(cart::Header)) ||
        length > static_cast<int>(sizeof(cart::Command))) {
        bump(diag.callback_bad);
        return;
    }
    if (memcmp(info->src_addr, settings::DONGLE_MAC, 6) != 0) {
        bump(diag.wrong_sender);
        return;
    }
    if (receiveLogRadio(data,length)) return;
    RxEvent event = {};
    event.length = length;
    event.received_us = esp_timer_get_time();
    memcpy(event.bytes, data, static_cast<size_t>(length));
    if (xQueueSend(rx_queue, &event, 0) != pdTRUE) {
        bump(diag.rx_drop);
        rx_overflow.store(1);
    }
    if (settings::CONTROL_TASK) xTaskNotifyGive(control_task);
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
void onSent(const esp_now_send_info_t *, esp_now_send_status_t status) {
#else
void onSent(const uint8_t *, esp_now_send_status_t status) {
#endif
    xQueueOverwrite(tx_status_queue, &status);
}

uint32_t dutyForThrottle(float percent) {
    if (!isfinite(percent)) percent = 0;
    percent = cart::clampf(percent, -100, 100);
    const float span = percent >= 0 ? settings::MAX_US - settings::NEUTRAL_US
                                    : settings::NEUTRAL_US - settings::MIN_US;
    const float pulse = settings::NEUTRAL_US + percent * span / 100.0f;
    return static_cast<uint32_t>(lroundf(pulse * ((1UL << settings::PWM_BITS) - 1) /
                                       (1000000.0f / settings::PWM_HZ)));
}

bool writePwm(float throttle, bool force = false) {
    if (!settings::PWM) return true;
    if (!settings::DRIVE) throttle = 0;
    const uint32_t duty = dutyForThrottle(throttle);
    // The hardware repeats the waveform without software refresh. Rewriting
    // the same duty every PI tick can wait for a prior LEDC update to finish.
    if (!force && duty == last_pwm_duty) return true;
    phase(PWM_WRITE);
    bump(diag.pwm_calls);
    const int64_t started = esp_timer_get_time();
    const bool ok = ledcWrite(settings::ESC_PIN, duty);
    const int64_t elapsed = esp_timer_get_time() - started;
    const uint32_t elapsed_us = elapsed <= 0 ? 0 :
        (elapsed >= UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(elapsed));
    diag.pwm_last_us.store(elapsed_us);
    if (elapsed_us > get(diag.pwm_max_us)) diag.pwm_max_us.store(elapsed_us);
    if (elapsed_us >= settings::CONTROL_US) bump(diag.pwm_slow);
    if (!ok) {
        last_pwm_duty = UINT32_MAX; // A failed write must never populate the cache.
        return false;
    }
    last_pwm_duty = duty;
    return true;
}

void publish(const cart::Telemetry &packet) {
    phase(PUBLISH);
    Frame frame = {packet, esp_timer_get_time(),engine.positionTelemetry()};
    xQueueOverwrite(telemetry_queue, &frame);
    diag.session.store(packet.session);
    diag.ack.store(packet.acknowledged_sequence);
    diag.running.store(packet.running);
    diag.magnets.store(packet.track_magnets);
    diag.stop_reason.store(packet.stop_reason);
}

#include "RecorderRuntime.h"

bool receiveLogRadio(const uint8_t *data, int length) {
#if CART_RECORDER_AVAILABLE
    return recorder_runtime::receiveRadio(data,length);
#else
    (void)data; (void)length; return false;
#endif
}

// Control owns production; reportTask is the only runtime UART writer.
int64_t next_log_us = 0;
uint32_t last_log_seq = UINT32_MAX, last_log_markers = UINT32_MAX;
uint8_t last_log_reason = UINT8_MAX;
void queueExperiment(int64_t now, bool periodic) {
    if (!settings::EXPERIMENT_LOG || !log_queue) return;
#if CART_RECORDER_AVAILABLE
    if (!recorder_runtime::recorder.active()) return;
#else
    return;
#endif
    const auto packet = engine.telemetry();
    const bool event = packet.acknowledged_sequence != last_log_seq ||
        packet.stop_reason != last_log_reason || packet.track_magnets != last_log_markers;
    const bool due = periodic && now >= next_log_us;
    if (!event && !due) return;
    #if CART_RECORDER_AVAILABLE
    const unsigned log_interval_us = 1000000u / recorder_runtime::recorder.log_hz;
#else
    const unsigned log_interval_us = settings::LOG_US;
#endif
    if (due) next_log_us += ((now-next_log_us)/log_interval_us+1)*log_interval_us;
    const float p_term = engine.running && (engine.mode == cart::SPEED || engine.mode == cart::MOVE || (engine.mode==cart::APPROACH || engine.mode==cart::PROFILE_RUN)) ?
        engine.pi.kp*(fabsf(engine.target)-engine.speed) : 0;
    const float throttle = engine.throttle();
    const float span = throttle >= 0 ? settings::MAX_US-settings::NEUTRAL_US :
                                      settings::NEUTRAL_US-settings::MIN_US;
    experiment::Sample sample = {now, packet, p_term, engine.pi.integral, get(diag.log_dropped),
        static_cast<uint16_t>(lroundf(settings::NEUTRAL_US+throttle*span/100)),
        static_cast<uint8_t>(event | ((engine.running && engine.mode!=cart::THROTTLE && engine.pi.slew_limited) ? cart::LOG_EVENT_SLEW_LIMITED:0) | ((engine.mode==cart::APPROACH || engine.mode==cart::PROFILE_RUN) ? (engine.mode==cart::PROFILE_RUN && engine.profile_launch ? 7u:static_cast<unsigned>(engine.approach.phase))<<4 : 0)),engine.position.value};
    const auto sensors = readSensors();
    sample.left_wheel_count=sensors.left_wheel.count;
    sample.marker_left_wheel_count=sensors.track_left_wheel_count;
    sample.marker_right_wheel_count=sensors.track_right_wheel_count;
#if CART_RECORDER_AVAILABLE
    recorder_runtime::recorder.sample(sample,recorder_runtime::quiet(now,sensors));
#endif
    if (xQueueSend(log_queue, &sample, 0) != pdTRUE) bump(diag.log_dropped);
    last_log_seq = packet.acknowledged_sequence;
    last_log_reason = packet.stop_reason;
    last_log_markers = packet.track_magnets;
}
void drainExperiment() {
    if (!settings::EXPERIMENT_LOG || !log_queue) return;
    experiment::Sample sample;
    char line[experiment::MAX_LINE];
    for (unsigned n = 0; n < 8 && xQueuePeek(log_queue, &sample, 0) == pdTRUE; ++n) {
        const int length = experiment::format(line, sizeof(line), sample);
        if (length <= 0 || static_cast<size_t>(length) >= sizeof(line)) {
            xQueueReceive(log_queue, &sample, 0);
            bump(diag.log_dropped);
            continue;
        }
        if (Serial.availableForWrite() < length) return;
        xQueueReceive(log_queue, &sample, 0);
        Serial.write(reinterpret_cast<const uint8_t *>(line), static_cast<size_t>(length));
    }
}

bool controlCycle(control::Deadline &deadline) {
#if CART_RECORDER_AVAILABLE
    if (recorder_runtime::maintenance(deadline)) return false;
#endif
    control::Sensors sensors = readSensors();
    int64_t now = esp_timer_get_time();
    phase(CONTROL);
    // Evaluate an existing run's safety limits before allowing a new command.
    engine.service(now, sensors, false, 0);
    if (rx_overflow.exchange(0)) {
        engine.overflow(now);
        xQueueReset(rx_queue); // require a fresh zero after the overflow stop
    }
    phase(RX_QUEUE);
    RxEvent event;
    for (unsigned n = 0; n < settings::QUEUE_LENGTH &&
                         xQueueReceive(rx_queue, &event, 0) == pdTRUE; ++n) {
        control::Input input;
        if (!control::decode(event.bytes, event.length, input)) {
            bump(diag.parse_bad);
            continue;
        }
        now = esp_timer_get_time();
        const uint32_t previous_sequence = engine.history.sequence;
#if CART_RECORDER_AVAILABLE
        engine.motion_blocked = recorder_runtime::recorder.manual &&
            (recorder_runtime::recorder.active() || recorder_runtime::recorder.state==experiment::FROZEN);
        if (input.command && (input.kind==cart::PI_SET || input.kind==cart::PI_STATUS || input.kind>=cart::HOME))
            engine.tuning_blocked=recorder_runtime::recorder.active() || recorder_runtime::recorder.state==experiment::FROZEN;
#endif
        if (!engine.receive(input, event.received_us, now, sensors)) {
            bump(diag.stale_input);
            continue;
        }
        if(input.command && cart::isProfileConfig(input.kind) && engine.profile_sequence==input.sequence && engine.history.sequence==input.sequence){
            const auto &p=engine.profile;
            const cart::ProfileReply reply{cart::makeHeader(cart::PROFILE_REPLY),engine.history.session,input.sequence,p.crc,p.duration,p.count,p.received,static_cast<uint8_t>(p.uploading),engine.profile_result,p.floor,p.launch};
            xQueueOverwrite(profile_reply_queue,&reply);
        }
        if (input.command && (input.kind==cart::PI_SET || input.kind==cart::PI_STATUS) &&
            engine.pi_sequence==input.sequence && engine.history.sequence==input.sequence) {
            const cart::PiReply reply{cart::makeHeader(cart::PI_REPLY),engine.history.session,
                input.sequence,engine.pi.kp,engine.pi.ki,engine.pi_result};
            xQueueOverwrite(pi_reply_queue,&reply);
        }
#if CART_RECORDER_AVAILABLE
        if (input.command && cart::isLogCommand(input.kind) &&
            engine.history.sequence != previous_sequence && engine.history.sequence == input.sequence) {
            recorder_runtime::request({input.kind,input.kind==cart::LOG_ARM ? input.duration_ms : 0},now,sensors);
            if (recorder_runtime::work_state.load()) break;
        }
#else
        (void)previous_sequence;
#endif
        bump(diag.valid);
        if (input.command && input.target != 0 && !settings::DRIVE) bump(diag.refused);
    }
    sensors = readSensors();
    now = esp_timer_get_time();
    float dt = 0;
    const bool periodic = deadline.advance(now, dt);
    if (periodic) bump(diag.pi_count);
    phase(CONTROL);
    engine.service(now, sensors, periodic, dt);
    if (!writePwm(engine.throttle())) {
        diag.pwm_fault.store(1);
        engine.pwmFailure(esp_timer_get_time());
        // Always attempt neutral after a failed write, even if it was cached.
        // Failure stays latched until reset.
        writePwm(0, true);
    }
#if CART_RECORDER_AVAILABLE
    recorder_runtime::observe(esp_timer_get_time(),periodic);
    if (recorder_runtime::work_state.load()) return false;
#endif
    queueExperiment(esp_timer_get_time(), periodic);
    auto packet = engine.telemetry();
#if CART_RECORDER_AVAILABLE
    recorder_runtime::fill(packet);
#else
    packet.log_error = 1; // recorder compiled out
#endif
    publish(packet);
    bump(diag.control_count);
    diag.control_ms.store(millis());
    return periodic;
}

void waitForControlWake(const control::Deadline &deadline) {
    phase(WAIT);
    bool timeout_wake = false;
    for (;;) {
        const int64_t remaining = deadline.remaining(esp_timer_get_time());
        if (remaining <= 0) return;
        if (timeout_wake) bump(diag.early_wakes);
        // RTOS timeouts end on tick boundaries and can be earlier than the
        // microsecond deadline. Re-wait after such a timeout without running
        // another control pass. Real notifications still wake us promptly.
        if (ulTaskNotifyTake(pdTRUE,
                control::ticksForWait(remaining, configTICK_RATE_HZ)) != 0) return;
        timeout_wake = true;
    }
}

void controlTask(void *) {
    control::Deadline deadline(esp_timer_get_time(), settings::CONTROL_US);
    for (;;) {
        const int64_t started = esp_timer_get_time();
        const bool periodic = controlCycle(deadline);
#if CART_RECORDER_AVAILABLE
        if (recorder_runtime::maintenance_pass || recorder_runtime::work_state.load()) {
            vTaskDelay(1); continue; // neutral maintenance is not a motion control pass
        }
#endif
        diag.stack_hwm.store(uxTaskGetStackHighWaterMark(nullptr));
        const int64_t finished = esp_timer_get_time();
        const int64_t elapsed = finished - started;
        const uint32_t elapsed_us = elapsed <= 0 ? 0 :
            (elapsed >= UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(elapsed));
        diag.work_last_us.store(elapsed_us);
        if (elapsed_us > get(diag.work_max_us)) diag.work_max_us.store(elapsed_us);
        const int64_t wait_at = esp_timer_get_time();
        const int64_t remaining = deadline.remaining(wait_at);
        // A short event pass may simply reach the current PI deadline. Run
        // the due update next; that is not a whole-period work overrun.
        // A slow event pass OR a periodic pass crossing its next deadline
        // still blocks for a tick so the lower-priority loop can progress.
        if (wait_at - started >= settings::CONTROL_US || (periodic && remaining <= 0)) {
            // Critical difference from the old sketch: always block on overrun.
            // taskYIELD() alone would not let a lower-priority loop task run.
            bump(diag.overruns);
            phase(OVERRUN_WAIT);
            vTaskDelay(1);
        } else {
            waitForControlWake(deadline);
        }
    }
}

void serviceProbe() {
    if (rx_overflow.exchange(0)) {
        radio_probe = probe::State{};
        xQueueReset(rx_queue);
    }
    RxEvent event;
    for (unsigned n = 0; n < settings::QUEUE_LENGTH &&
                         xQueueReceive(rx_queue, &event, 0) == pdTRUE; ++n) {
        if (esp_timer_get_time() - event.received_us >= cart::LINK_TIMEOUT_US) {
            bump(diag.stale_input);
            continue;
        }
        const probe::Result result = radio_probe.receive(event.bytes, event.length);
        if (result == probe::Result::BAD_PACKET) bump(diag.parse_bad);
        else if (result == probe::Result::MOVEMENT_REFUSED) bump(diag.refused);
        else bump(diag.valid);
    }
    publish(radio_probe.telemetry());
}

void serviceRadio() {
    esp_now_send_status_t status;
    if (xQueueReceive(tx_status_queue, &status, 0) == pdTRUE) {
        tx_in_flight = false;
        diag.tx_busy.store(0);
        if (status == ESP_NOW_SEND_SUCCESS) bump(diag.tx_ok);
        else bump(diag.tx_fail);
    }
    Frame frame;
    if (xQueuePeek(telemetry_queue, &frame, 0) != pdTRUE) return;
    const int64_t age = esp_timer_get_time() - frame.generated_us;
    diag.frame_age_ms.store(age >= 0 ? static_cast<uint32_t>(age / 1000) : UINT32_MAX);
    const uint32_t now = millis();
    if (frame.packet.session == 0 || age < 0 || age >= 200000 || tx_in_flight ||
        now - last_tx_ms < settings::TELEMETRY_MS) return;
    last_tx_ms = now;
    // Configuration replies are request-driven and never sent while driving.
    if(!frame.packet.running && xQueueReceive(profile_reply_queue,&profile_outgoing,0)==pdTRUE){
        if(profile_outgoing.session==frame.packet.session){
            const auto result=esp_now_send(settings::DONGLE_MAC,reinterpret_cast<const uint8_t*>(&profile_outgoing),sizeof(profile_outgoing));
            if(result==ESP_OK){tx_in_flight=true;diag.tx_busy.store(1);bump(diag.tx_queued);}
            else{xQueueOverwrite(profile_reply_queue,&profile_outgoing);bump(diag.tx_error);}
            return;
        }
    }
    if (!frame.packet.running && xQueueReceive(pi_reply_queue,&pi_outgoing,0)==pdTRUE) {
        if (pi_outgoing.session==frame.packet.session) {
            const esp_err_t configured=esp_now_send(settings::DONGLE_MAC,
                reinterpret_cast<const uint8_t*>(&pi_outgoing),sizeof(pi_outgoing));
            if (configured==ESP_OK) { tx_in_flight=true; diag.tx_busy.store(1); bump(diag.tx_queued); }
            else { xQueueOverwrite(pi_reply_queue,&pi_outgoing); bump(diag.tx_error); }
            return;
        }
    }
    if(settings::BENCH && settings::DRIVE && now-last_position_tx_ms>=200) {
        position_outgoing=frame.position;
        const esp_err_t sent=esp_now_send(settings::DONGLE_MAC,reinterpret_cast<const uint8_t*>(&position_outgoing),sizeof(position_outgoing));
        if(sent==ESP_OK) { last_position_tx_ms=now;tx_in_flight=true;diag.tx_busy.store(1);bump(diag.tx_queued); }
        else bump(diag.tx_error);
        return;
    }
    outgoing = frame.packet;
    const esp_err_t result = esp_now_send(settings::DONGLE_MAC,
        reinterpret_cast<const uint8_t *>(&outgoing), sizeof(outgoing));
    if (result == ESP_OK) {
        tx_in_flight = true;
        diag.tx_busy.store(1);
        bump(diag.tx_queued);
    } else {
        bump(diag.tx_error);
    }
}

const char *phaseName(uint32_t p) {
    switch (p) {
        case RX_QUEUE: return "RX_QUEUE";
        case SENSORS: return "SENSORS";
        case CONTROL: return "CONTROL";
        case PWM_WRITE: return "PWM_WRITE";
        case PUBLISH: return "PUBLISH";
        case WAIT: return "WAIT";
        case OVERRUN_WAIT: return "OVERRUN_WAIT";
        case MAINTENANCE: return "MAINTENANCE";
        default: return "BOOT";
    }
}

void reportTask(void *) {
    uint32_t old_loop = 0, old_control = 0, old_pi = 0, old_wheel = 0, old_track = 0;
    uint32_t last_report = millis();
    for (;;) {
#if CART_RECORDER_AVAILABLE
        recorder_runtime::worker();
        recorder_runtime::readSerial();
#endif
        drainExperiment();
        diag.report_stack_hwm.store(uxTaskGetStackHighWaterMark(nullptr));
        if (millis()-last_report >= 1000) {
            last_report = millis();
        const uint32_t now = millis(), loops = get(diag.loop_count), cycles = get(diag.control_count);
        const uint32_t pi = get(diag.pi_count);
        const uint32_t wheel = get(diag.wheel_raw), track = get(diag.track_raw);
        Serial.printf("STAGE %u alive ms=%lu loop_delta=%lu ctrl_delta=%lu pi_delta=%lu loop_age=%lu ctrl_age=%lu phase=%s\n",
            settings::STAGE, static_cast<unsigned long>(now), static_cast<unsigned long>(loops-old_loop),
            static_cast<unsigned long>(cycles-old_control), static_cast<unsigned long>(pi-old_pi),
            static_cast<unsigned long>(now-get(diag.loop_ms)),
            static_cast<unsigned long>(settings::CONTROL_TASK ? now-get(diag.control_ms) : 0),
            phaseName(get(diag.phase)));
        Serial.printf("RADIO RX=%lu valid=%lu bad=%lu wrong=%lu drop=%lu stale=%lu refused=%lu TX_OK=%lu TX_FAIL=%lu tx_err=%lu busy=%lu\n",
            static_cast<unsigned long>(get(diag.rx_raw)), static_cast<unsigned long>(get(diag.valid)),
            static_cast<unsigned long>(get(diag.callback_bad)+get(diag.parse_bad)),
            static_cast<unsigned long>(get(diag.wrong_sender)), static_cast<unsigned long>(get(diag.rx_drop)),
            static_cast<unsigned long>(get(diag.stale_input)), static_cast<unsigned long>(get(diag.refused)),
            static_cast<unsigned long>(get(diag.tx_ok)), static_cast<unsigned long>(get(diag.tx_fail)),
            static_cast<unsigned long>(get(diag.tx_error)), static_cast<unsigned long>(get(diag.tx_busy)));
        Serial.printf("CONTROL session=%lu ack=%lu running=%lu mags=%lu stop=%s frame_age=%lu overruns=%lu pwm_calls=%lu pwm_fault=%lu stack_hwm=%lu pwm_last_us=%lu pwm_max_us=%lu pwm_slow=%lu early_wakes=%lu work_last_us=%lu work_max_us=%lu report_stack_hwm=%lu\n",
            static_cast<unsigned long>(get(diag.session)), static_cast<unsigned long>(get(diag.ack)),
            static_cast<unsigned long>(get(diag.running)), static_cast<unsigned long>(get(diag.magnets)),
            cart::stopReasonName(static_cast<uint8_t>(get(diag.stop_reason))),
            static_cast<unsigned long>(get(diag.frame_age_ms)), static_cast<unsigned long>(get(diag.overruns)),
            static_cast<unsigned long>(get(diag.pwm_calls)), static_cast<unsigned long>(get(diag.pwm_fault)),
            static_cast<unsigned long>(get(diag.stack_hwm)),
            static_cast<unsigned long>(get(diag.pwm_last_us)),
            static_cast<unsigned long>(get(diag.pwm_max_us)),
            static_cast<unsigned long>(get(diag.pwm_slow)),
            static_cast<unsigned long>(get(diag.early_wakes)),
            static_cast<unsigned long>(get(diag.work_last_us)),
            static_cast<unsigned long>(get(diag.work_max_us)),
            static_cast<unsigned long>(get(diag.report_stack_hwm)));
        Serial.printf("SENSORS wheel_irq_delta=%lu wheel_ok=%lu track_irq_delta=%lu track_ok=%lu track_pin=%d track_level=%d\n",
            static_cast<unsigned long>(wheel-old_wheel), static_cast<unsigned long>(get(diag.wheel_accepted)),
            static_cast<unsigned long>(track-old_track), static_cast<unsigned long>(get(diag.track_accepted)),
            settings::TRACK_PIN, settings::TRACK_ISR ? digitalRead(settings::TRACK_PIN) : -1);
        Serial.printf("EXPERIMENT profile=%s protocol=%u log_dropped=%lu\n",
            settings::BENCH ? "BENCH" : "LEGACY", cart::VERSION,
            static_cast<unsigned long>(get(diag.log_dropped)));
#if CART_RECORDER_AVAILABLE
        recorder_runtime::printStatus();
#else
        Serial.println("LOGGER state=0 error=1 enabled_build=0");
#endif
        old_loop = loops; old_control = cycles; old_pi = pi;
        old_wheel = wheel; old_track = track;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void haltSetup(const char *why) {
    if (pwm_ready) ledcWrite(settings::ESC_PIN, dutyForThrottle(0));
    for (;;) {
        Serial.printf("SETUP HALTED: %s\n", why);
        delay(1000);
    }
}
void checkSdk(esp_err_t result, const char *operation) {
    if (result != ESP_OK) {
        Serial.printf("ERROR %s: %s\n", operation, esp_err_to_name(result));
        haltSetup(operation);
    }
}

void setupCart() {
    const size_t tx_size = Serial.setTxBufferSize(2048);
    Serial.begin(115200);
    delay(300);
    Serial.printf("Wheel debounce %lld us; fixed-throttle travel guard %.1f m.\n", static_cast<long long>(cart::WHEEL_DEBOUNCE_US), cart::FIXED_THROTTLE_MAX_DISTANCE_M);
    Serial.printf("Cart onboard-log-33-return2 STAGE=%u; motor power OFF for bench testing.\n", settings::STAGE);
    Serial.printf("Sensor map: right wheel GPIO%d; track GPIO%d INPUT; left wheel GPIO32; feedback GPIO32 in BENCH.\n",
        settings::WHEEL_PIN, settings::TRACK_PIN);
    Serial.printf("Feedback: %s GPIO%d for speed/position/guards; scale %.8f m/pulse.\n",settings::LEFT_FEEDBACK ? "LEFT":"RIGHT",settings::FEEDBACK_WHEEL_PIN,cart::DIST_PER_PULSE_M);
    Serial.printf("Core=%d.%d.%d IDF=%d.%d.%d tick_hz=%u loop_core=%d\n",
        ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH,
        ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH,
        configTICK_RATE_HZ, xPortGetCoreID());
    if (tx_size == 0) haltSetup("UART buffer allocation");
    profile_reply_queue=xQueueCreate(1,sizeof(cart::ProfileReply));
    if(!profile_reply_queue)haltSetup("Profile reply queue allocation");
    pi_reply_queue=xQueueCreate(1,sizeof(cart::PiReply));
    if (!pi_reply_queue) haltSetup("PI reply queue allocation");
    rx_queue = xQueueCreate(settings::QUEUE_LENGTH, sizeof(RxEvent));
    telemetry_queue = xQueueCreate(1, sizeof(Frame));
    tx_status_queue = xQueueCreate(1, sizeof(esp_now_send_status_t));
#if CART_RECORDER_AVAILABLE
    recorder_runtime::requests = xQueueCreate(4,sizeof(recorder_runtime::Request));
    recorder_runtime::replies = xQueueCreate(4,sizeof(recorder_runtime::Reply));
    recorder_runtime::radio_requests = xQueueCreate(2,sizeof(log_radio::Request));
    if (!recorder_runtime::radio_requests) haltSetup("Radio download queue allocation");
    if (!recorder_runtime::requests || !recorder_runtime::replies) haltSetup("Recorder request queue allocation");
#endif
    if (settings::EXPERIMENT_LOG) log_queue = xQueueCreate(settings::LOG_QUEUE_LENGTH, sizeof(experiment::Sample));
    if (!rx_queue || !telemetry_queue || !tx_status_queue ||
        (settings::EXPERIMENT_LOG && !log_queue)) haltSetup("Queue allocation");
    Serial.println("INIT WiFi");
    if (!WiFi.mode(WIFI_STA)) haltSetup("WiFi.mode");
    checkSdk(esp_wifi_set_ps(WIFI_PS_NONE), "disable power saving");
    checkSdk(esp_wifi_set_channel(settings::WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE), "set channel");
    checkSdk(esp_now_init(), "ESP-NOW init");
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, settings::DONGLE_MAC, 6);
    peer.ifidx = WIFI_IF_STA;
    peer.channel = settings::WIFI_CHANNEL;
    peer.encrypt = false;
    Serial.println("INIT fixed dongle peer");
    checkSdk(esp_now_add_peer(&peer), "add dongle peer");
    checkSdk(esp_now_register_send_cb(onSent), "send callback");

    if (settings::PWM) {
        Serial.printf("INIT PWM GPIO%d, %u Hz, neutral %d us\n",
            settings::ESC_PIN, settings::PWM_HZ, settings::NEUTRAL_US);
        if (!ledcAttach(settings::ESC_PIN, settings::PWM_HZ, settings::PWM_BITS)) haltSetup("PWM attach");
        pwm_ready = true;
        if (!writePwm(0, true)) haltSetup("Initial neutral PWM");
        delay(2000);
    }
    if (settings::CONTROL_TASK) {
        Serial.println("INIT control task, core 1, priority 2, period 10 ms");
        if (xTaskCreatePinnedToCore(controlTask, "CartControl", 4096, nullptr, 2,
                                   &control_task, 1) != pdPASS) haltSetup("Control task creation");
    }
    if (settings::WHEEL_ISR) {
        Serial.printf("INIT wheel interrupt GPIO%d (right wheel, existing external pull-up)\n", settings::WHEEL_PIN);
        pinMode(settings::WHEEL_PIN, INPUT);
        attachInterrupt(digitalPinToInterrupt(settings::WHEEL_PIN), wheelISR, FALLING);
        pinMode(settings::LEFT_WHEEL_PIN,INPUT);
        attachInterrupt(digitalPinToInterrupt(settings::LEFT_WHEEL_PIN),leftWheelISR,FALLING);
    }
    if (settings::TRACK_ISR) {
        Serial.printf("INIT track interrupt GPIO%d (track Hall sensor, existing external pull-up)\n", settings::TRACK_PIN);
        pinMode(settings::TRACK_PIN, INPUT);
        attachInterrupt(digitalPinToInterrupt(settings::TRACK_PIN), trackISR, FALLING);
    }
    checkSdk(esp_now_register_recv_cb(onReceive), "receive callback");
    uint8_t ch = 0;
    wifi_second_chan_t secondary;
    checkSdk(esp_wifi_get_channel(&ch, &secondary), "read channel");
    Serial.printf("Cart STA MAC: %s; channel=%u; dongle=88:56:A6:58:49:38\n", WiFi.macAddress().c_str(), ch);
    Serial.printf("READY STAGE=%u DRIVE=%u; runtime limit=%lld ms; track stop=%lu magnets.\n",
        settings::STAGE, settings::DRIVE ? 1u : 0u,
        static_cast<long long>(cart::MAX_RUNTIME_US/1000),
        static_cast<unsigned long>(settings::BENCH ? 0 : cart::TRACK_MAGNET_LIMIT));
    Serial.printf("PROFILE=%s PROTOCOL=%u; PI cap %.1f%%; fixed cap %.1f%% (max %lu ms; fixed travel guard %.1f m); marker auto-stop=%u.\n",
        settings::BENCH ? "BENCH" : "LEGACY", cart::VERSION,
        settings::BENCH ? cart::BENCH_MAX_THROTTLE : 100.0f, cart::BENCH_MAX_FIXED_THROTTLE, static_cast<unsigned long>(cart::FIXED_THROTTLE_MAX_MS), cart::FIXED_THROTTLE_MAX_DISTANCE_M, settings::BENCH ? 0u : 1u);
    Serial.printf("Timed speed: targets up to %.1f m/s, max %lu ms, PI cap %.1f%%, early neutral at %.1f m; default scale %.7f m/pulse.\n",cart::BENCH_MAX_SPEED_MPS,static_cast<unsigned long>(cart::TIMED_SPEED_MAX_MS),cart::TIMED_PI_MAX_THROTTLE,cart::TIMED_SPEED_MAX_DISTANCE_M,cart::DIST_PER_PULSE_M);
    Serial.printf("PI output slew %.1f percentage-points/s; terminal/user/fault neutral immediate.\n",cart::PI_SLEW_PERCENT_PER_S);
    Serial.printf("PI gains: Kp=%.3f Ki=%.3f Kd=0; pi set/status/defaults via dongle while quiet.\n",engine.pi.kp,engine.pi.ki);
    Serial.println("Predictive forward: approach metres speed;0.5..4m,0.5..1m/s,4cm estimated tolerance; no reverse brake. Event high nibble=phase. Existing move remains neutral-cutoff baseline.");
    Serial.println("Position: home [x]; position status/invalidate/direction [-1|0|1]. Move: move metres signed_mps; max4m/1mps/100percent/10s, requires known position; neutral cutoff then coast.");
    Serial.println("Flash both protocol-v3 sketches. Send 0 on dongle after READY. Motor battery OFF.");
    if (settings::EXPERIMENT_LOG)
        Serial.println("CSV_HEADER,event,t_us,seq,mode,target_mps,speed_mps,throttle_pct,pulse_us,wheel_count,total_m,run_m,markers,marker_error_m,flags,log_drops,p_term,integral,meters_per_pulse,calibration_id,raw_position_m,position_m,position_correction_m,move_cutoff_m,position_epoch,position_direction,position_flags,position_reason,cutoff_source,left_wheel_count,marker_left_wheel_count,marker_right_wheel_count");
    Serial.println("Four status lines repeat each second from a separate task on core 0.");
    // Sole runtime Serial writer. Monitor uses only per-counter locks, never
    // the sensor lock; all locks are released before calling Serial.
    if (xTaskCreatePinnedToCore(reportTask, "CartReport", 8192, nullptr, 1,
                               nullptr, 0) != pdPASS) haltSetup("Report task creation");
}

void loopCart() {
    bump(diag.loop_count);
    diag.loop_ms.store(millis());
    if (!settings::CONTROL_TASK) serviceProbe();
    serviceRadio();
    delay(1);
}
} // namespace firmware
