#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <stdarg.h>
#include "CartCommon.h"

// Temporary diagnostics. Callbacks only copy metadata and update counters.
// Serial output is nonblocking and runs once per second from loop().
namespace radio_diag {
enum RxResult : uint8_t {
    NONE, ACCEPTED, BAD_LENGTH, BAD_HEADER, BAD_FIELDS,
    WRONG_SENDER, WRONG_SESSION, QUEUE_FULL
};
struct Snapshot {
    uint32_t queued = 0, queue_errors = 0, tx_ok = 0, tx_fail = 0;
    uint32_t rx = 0, valid = 0;
    uint8_t source[6] = {};
    int length = 0;
    cart::Header header = {};
    uint32_t session = 0;
    RxResult result = NONE;
    esp_err_t queue_error = ESP_OK;
};
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static Snapshot counters;

inline void received(const esp_now_recv_info_t *info, const uint8_t *data,
                     int length, RxResult result) {
    cart::Header header = {};
    uint32_t session = 0;
    if (data && length >= static_cast<int>(sizeof(header)))
        memcpy(&header, data, sizeof(header));
    if (data && length >= static_cast<int>(sizeof(header) + sizeof(session)))
        memcpy(&session, data + sizeof(header), sizeof(session));
    portENTER_CRITICAL(&mux);
    ++counters.rx;
    if (result == ACCEPTED) ++counters.valid;
    memset(counters.source, 0, sizeof(counters.source));
    if (info && info->src_addr) memcpy(counters.source, info->src_addr, 6);
    counters.length = length;
    counters.header = header;
    counters.session = session;
    counters.result = result;
    portEXIT_CRITICAL(&mux);
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
inline void sent(const esp_now_send_info_t *, esp_now_send_status_t status) {
#else
inline void sent(const uint8_t *, esp_now_send_status_t status) {
#endif
    portENTER_CRITICAL(&mux);
    if (status == ESP_NOW_SEND_SUCCESS) ++counters.tx_ok;
    else ++counters.tx_fail;
    portEXIT_CRITICAL(&mux);
}

inline void queued(esp_err_t result) {
    portENTER_CRITICAL(&mux);
    if (result == ESP_OK) ++counters.queued;
    else {
        ++counters.queue_errors;
        counters.queue_error = result;
    }
    portEXIT_CRITICAL(&mux);
}

inline const char *resultName(RxResult result) {
    switch (result) {
        case ACCEPTED: return "ACCEPTED";
        case BAD_LENGTH: return "BAD_LENGTH";
        case BAD_HEADER: return "BAD_HEADER";
        case BAD_FIELDS: return "BAD_FIELDS";
        case WRONG_SENDER: return "WRONG_SENDER";
        case WRONG_SESSION: return "WRONG_SESSION";
        case QUEUE_FULL: return "QUEUE_FULL";
        default: return "NONE";
    }
}

inline void log(const char *format, ...) {
    char line[256];
    va_list args;
    va_start(args, format);
    const int count = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (count > 0 && count < static_cast<int>(sizeof(line)) &&
        Serial.availableForWrite() >= count) {
        Serial.write(reinterpret_cast<const uint8_t *>(line), count);
    }
}

inline bool report(const char *role) {
    static uint32_t last_ms = 0;
    const uint32_t now = millis();
    if (now - last_ms < 1000) return false;
    last_ms = now;
    Snapshot s;
    portENTER_CRITICAL(&mux);
    s = counters;
    portEXIT_CRITICAL(&mux);
    uint8_t primary = 0;
    wifi_second_chan_t secondary = WIFI_SECOND_CHAN_NONE;
    const esp_err_t channel_result = esp_wifi_get_channel(&primary, &secondary);
    log("DBG %s ch=%u ch_err=%d queued=%lu queue_err=%lu TX_OK=%lu TX_FAIL=%lu RX=%lu valid=%lu\n",
        role, primary, static_cast<int>(channel_result),
        static_cast<unsigned long>(s.queued), static_cast<unsigned long>(s.queue_errors),
        static_cast<unsigned long>(s.tx_ok), static_cast<unsigned long>(s.tx_fail),
        static_cast<unsigned long>(s.rx), static_cast<unsigned long>(s.valid));
    if (s.rx != 0) {
        log("DBG %s last=%s from=%02X:%02X:%02X:%02X:%02X:%02X len=%d magic=%04X ver=%u type=%u session=%lu\n",
            role, resultName(s.result), s.source[0], s.source[1], s.source[2],
            s.source[3], s.source[4], s.source[5], s.length, s.header.magic,
            s.header.version, s.header.type, static_cast<unsigned long>(s.session));
    }
    if (s.queue_errors != 0)
        log("DBG %s last_queue_error=%s\n", role, esp_err_to_name(s.queue_error));
    return true;
}
} // namespace radio_diag
