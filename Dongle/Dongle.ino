#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <esp_random.h>
#include <esp_arduino_version.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stdarg.h>
#include "src/CartCommon.h"
#include "src/LogRadio.h"
#include "src/RadioDiagnostics.h"

using namespace cart;
// Retained from the original sketch. Replace with the cart's printed STA MAC.
const uint8_t CONTROLLER_MAC[6] = {0xB4, 0xBF, 0xE9, 0xC1, 0xDD, 0xFC};
constexpr int64_t TRANSMIT_INTERVAL_US = 100000;
constexpr int64_t TELEMETRY_TIMEOUT_US = 1000000;
constexpr int64_t COMMAND_ACK_TIMEOUT_US = 1000000;
constexpr int64_t PRINT_INTERVAL_US = 250000;

struct TelemetryEvent { Telemetry packet; int64_t received_us; };
QueueHandle_t position_queue=nullptr;
PositionTelemetry latest_position={};
int64_t position_received_us=0;
QueueHandle_t profile_reply_queue=nullptr;
QueueHandle_t pi_reply_queue = nullptr;
QueueHandle_t telemetry_queue = nullptr;
QueueHandle_t download_queue = nullptr;
log_radio::Receiver download_receiver;
log_radio::Request download_request = {};
bool downloading=false;
int64_t download_started=0, download_request_us=0, download_progress_us=0;
void cancelDownload() {
    if (!downloading) return;
    auto r=download_request; r.operation=log_radio::CANCEL;
    esp_now_send(CONTROLLER_MAC,reinterpret_cast<const uint8_t*>(&r),sizeof(r));
    downloading=false;
}

uint32_t dongle_session = 0; // Initialized before callback registration; then immutable.

// Only loop() owns the following state.
Telemetry latest = {};
Command pending = {};
LineBuffer input;
uint32_t next_sequence = 0;
bool pending_command = false;
bool transmit_enabled = true;
bool require_zero = true;
bool connected = false;
bool ever_received = false;
bool status_changed = false;
int64_t pending_since_us = 0;
int64_t last_received_us = 0;
int64_t last_transmitted_us = 0;
int64_t last_print_us = 0;
int64_t last_send_error_us = 0;

// UART TX is buffered. There is exactly one runtime writer, and it only writes
// complete lines that fit without waiting. Congested console output is dropped.
void logLine(const char *format, ...) {
    char line[192];
    va_list arguments;
    va_start(arguments, format);
    const int count = vsnprintf(line, sizeof(line), format, arguments);
    va_end(arguments);
    if (count > 0 && count < static_cast<int>(sizeof(line)) && Serial.availableForWrite() >= count) {
        Serial.write(reinterpret_cast<const uint8_t *>(line), static_cast<size_t>(count));
    }
}

void haltDongle(const char *message) {
    Serial.println(message); // Setup failure; no runtime heartbeat to delay.
    for (;;) delay(1000);
}
void checkSdk(esp_err_t result, const char *operation) {
    if (result != ESP_OK) {
        Serial.printf("%s: %s\n", operation, esp_err_to_name(result));
        haltDongle("Dongle initialization failed.");
    }
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int length) {
    if(info && info->src_addr && data && !memcmp(info->src_addr,CONTROLLER_MAC,6) && length==sizeof(PositionTelemetry)) {
        PositionTelemetry pose;memcpy(&pose,data,sizeof(pose));const auto &x=pose.position;
        if(validHeader(pose.header,POSITION) && pose.session==dongle_session && isfinite(x.raw_m) && isfinite(x.x_m) &&
            isfinite(x.correction_m) && isfinite(x.cutoff_m) && x.direction>=-1 && x.direction<=1 && !(x.flags&~15u) &&
            x.reason<=POS_CLEARED && x.cutoff_source<=2 && pose.result<=POS_UNSUPPORTED)
            xQueueOverwrite(position_queue,&pose);
        return;
    }
    if(info && info->src_addr && data && !memcmp(info->src_addr,CONTROLLER_MAC,6) && length==sizeof(ProfileReply)){
        ProfileReply reply;memcpy(&reply,data,sizeof(reply));
        if(validHeader(reply.header,PROFILE_REPLY) && reply.session==dongle_session && reply.sequence && reply.count<=PROFILE_MAX_POINTS && reply.received<=PROFILE_MAX_POINTS && reply.uploading<=1 && reply.result<=PROFILE_UNSUPPORTED && isfinite(reply.duration) && reply.duration>=0 && reply.duration<=cart::APPROACH_MAX_RUNTIME_US/1000000.f && isfinite(reply.speed_floor) && reply.speed_floor>=.1f && reply.speed_floor<=MAX_SPEED_MPS && isfinite(reply.launch_speed) && reply.launch_speed>=reply.speed_floor && reply.launch_speed<=MAX_SPEED_MPS)
            xQueueOverwrite(profile_reply_queue,&reply);
        return;
    }
    if (info && info->src_addr && data && !memcmp(info->src_addr,CONTROLLER_MAC,6) && length==sizeof(PiReply)) {
        PiReply reply; memcpy(&reply,data,sizeof(reply));
        if (validHeader(reply.header,PI_REPLY) && reply.session==dongle_session && reply.sequence &&
            validPiGains(reply.kp,reply.ki) && reply.result<=PI_UNSUPPORTED) xQueueOverwrite(pi_reply_queue,&reply);
        return;
    }
    if (info && info->src_addr && data && !memcmp(info->src_addr,CONTROLLER_MAC,6) && length==sizeof(log_radio::Data)) {
        log_radio::Data packet; memcpy(&packet,data,sizeof(packet));
        if (log_radio::valid(packet,length) && packet.session==dongle_session)
            xQueueSend(download_queue,&packet,0);
        return;
    }
    if (!info || !info->src_addr || !data || length != sizeof(Telemetry)) {
        radio_diag::received(info, data, length, radio_diag::BAD_LENGTH);
        return;
    }
    if (memcmp(info->src_addr, CONTROLLER_MAC, 6) != 0) {
        radio_diag::received(info, data, length, radio_diag::WRONG_SENDER);
        return;
    }
    TelemetryEvent event;
    memcpy(&event.packet, data, sizeof(Telemetry));
    const Telemetry &packet = event.packet;
    if (!validHeader(packet.header, TELEMETRY)) {
        radio_diag::received(info, data, length, radio_diag::BAD_HEADER);
        return;
    }
    if (packet.session != dongle_session) {
        radio_diag::received(info, data, length, radio_diag::WRONG_SESSION);
        return;
    }
    if (!isfinite(packet.speed_magnitude) || packet.speed_magnitude < 0 ||
        !isfinite(packet.target_speed) || fabsf(packet.target_speed) > MAX_SPEED_MPS ||
        packet.running > 1 || packet.stop_reason > STOP_RETURN_OVERSPEED || (packet.mode > CALIBRATE && packet.mode != APPROACH && packet.mode!=PROFILE_RUN) || packet.log_state > 5 || packet.log_error > 9 || packet.log_samples > 1024 ||
        !isfinite(packet.throttle_percent) || fabsf(packet.throttle_percent) > 100 ||
        !isfinite(packet.total_distance_m) || packet.total_distance_m < 0 ||
        !isfinite(packet.run_distance_m) || packet.run_distance_m < 0 ||
        !isfinite(packet.marker_interval_m) || !isfinite(packet.marker_error_m) ||
        !isfinite(packet.meters_per_pulse) || packet.meters_per_pulse <= 0) {
        radio_diag::received(info, data, length, radio_diag::BAD_FIELDS);
        return;
    }
    event.received_us = esp_timer_get_time();
    const BaseType_t result = xQueueOverwrite(telemetry_queue, &event);
    radio_diag::received(info, data, length,
        result == pdTRUE ? radio_diag::ACCEPTED : radio_diag::QUEUE_FULL);
}

void pauseLink(const char *cause) {
    transmit_enabled = false;
    pending_command = false;
    require_zero = true;
    logLine("%s Heartbeats paused; cart times out. Enter 0 to re-arm.\n", cause);
}

void readTelemetry() {
    TelemetryEvent event;
    if (xQueueReceive(telemetry_queue, &event, 0) != pdTRUE) return;
    if (!connected) {
        connected = true;
        logLine(">> CONNECTED TO CART <<\n");
    }
    status_changed = !ever_received || latest.running != event.packet.running ||
                     latest.stop_reason != event.packet.stop_reason ||
                     latest.target_speed != event.packet.target_speed ||
                     latest.track_magnets != event.packet.track_magnets;
    ever_received = true;
    latest = event.packet;

    last_received_us = event.received_us;
    if (latest.flags & FLAG_REARM) {
        require_zero = true;
    }
    if (pending_command && latest.acknowledged_sequence == pending.sequence) {
        // PI commands remain pending until an explicit applied/refused readback.
        if (pending.kind==PI_SET || pending.kind==PI_STATUS || isProfileConfig(pending.kind)) return;
        logLine("Cart acknowledged command #%lu; target %.2f m/s; stop %s.\n",
                static_cast<unsigned long>(pending.sequence), latest.target_speed,
                stopReasonName(latest.stop_reason));
        if (isStop(pending) && latest.running == 0 && latest.target_speed == 0) {
            require_zero = false;
        } else if (latest.stop_reason == STOP_REARM_REQUIRED) {
            require_zero = true;
        }
        pending_command = false;
    }
}

void readPiReply() {
    PiReply reply;
    if (xQueueReceive(pi_reply_queue,&reply,0)!=pdTRUE) return;
    if (!pending_command || reply.sequence!=pending.sequence ||
        (pending.kind!=PI_SET && pending.kind!=PI_STATUS)) return;
    const char *names[]={"STATUS","APPLIED","BUSY","INVALID","UNSUPPORTED"};
    logLine("PI_REPLY sequence=%lu result=%s kp=%.4f ki=%.4f kd=0 persistence=RAM\n",
        static_cast<unsigned long>(reply.sequence),names[reply.result],reply.kp,reply.ki);
    logLine("Cart acknowledged command #%lu; PI result %s.\n",static_cast<unsigned long>(reply.sequence),names[reply.result]);
    if (reply.result==PI_APPLIED) require_zero=true;
    pending_command=false;
}
void readProfileReply(){
    ProfileReply reply;if(xQueueReceive(profile_reply_queue,&reply,0)!=pdTRUE)return;
    if(!pending_command || reply.sequence!=pending.sequence || !isProfileConfig(pending.kind))return;
    const char *names[]={"STATUS","APPLIED","BUSY","INVALID","UNSUPPORTED"};
    logLine("PROFILE_REPLY sequence=%lu result=%s crc=%lu count=%u received=%u uploading=%u duration=%.3f floor=%.3f launch=%.3f\n",
        static_cast<unsigned long>(reply.sequence),names[reply.result],static_cast<unsigned long>(reply.crc),reply.count,reply.received,reply.uploading,reply.duration,reply.speed_floor,reply.launch_speed);
    logLine("Cart acknowledged command #%lu; profile result %s.\n",static_cast<unsigned long>(reply.sequence),names[reply.result]);
    if(reply.result==PROFILE_APPLIED)require_zero=true;
    pending_command=false;
}
void submitCommand(const Command &request) {
    if(isProfileConfig(request.kind) && (!connected || latest.running || latest.speed_magnitude!=0 || pending_command || downloading || (latest.log_state>=1 && latest.log_state<=4))){
        logLine("PROFILE_ERROR BUSY_OR_NOT_QUIET; upload only while stationary and loggerOFF.\n");return;
    }
    if ((request.kind==PI_SET || request.kind==PI_STATUS) &&
        (!connected || latest.running || latest.speed_magnitude!=0 || pending_command || downloading ||
         (latest.log_state>=1 && latest.log_state<=4))) {
        logLine("PI_ERROR BUSY_OR_NOT_QUIET; wait for saved/off, stationary cart.\n"); return;
    }
    if(request.kind==MOVE && request.target_speed < -MOVE_MAX_SPEED_MPS &&
        (!connected || !(latest.flags & FLAG_FAST_REVERSE))) {
        logLine("Invalid command. Fast reverse requires matching cart firmware.\n");return;
    }
    const bool motion = request.kind == SPEED || request.kind == THROTTLE || request.kind == MOVE || request.kind == APPROACH || request.kind==PROFILE_RUN;
    if (require_zero && motion && !isStop(request)) {
        logLine("Enter 0 first to restore the link before requesting movement.\n");
        return;
    }
    if (++next_sequence == 0) ++next_sequence;
    pending = request;
    pending.header = makeHeader(COMMAND);
    pending.session = dongle_session;
    pending.sequence = next_sequence;
    pending_command = true;
    transmit_enabled = true;
    pending_since_us = esp_timer_get_time();
    last_transmitted_us = pending_since_us - TRANSMIT_INTERVAL_US;
    logLine("Queued command #%lu: %s speed=%.2f throttle=%.2f distance=%.3f duration=%lu ms; waiting for cart ACK.\n",
            static_cast<unsigned long>(pending.sequence), kindName(pending.kind), pending.target_speed,
            pending.throttle_percent, pending.distance_m, static_cast<unsigned long>(pending.duration_ms));
}

// Transfer bytes bypass lossy status logLine, are ACKed only after UART write.
// No motion is admitted until download completion/cancellation and fresh zero.
void startDownload(const char *line) {
    if (!strcmp(line,"log cancel")) { cancelDownload(); return; }
    const bool ram=!strcmp(line,"log ram");
    const bool release=!strncmp(line,"log release ",12);
    if (downloading || pending_command || !connected || latest.running || latest.speed_magnitude!=0 ||
        (latest.log_state>=1 && latest.log_state<=3) || (latest.log_state==4 && !ram && !release)) { Serial.println("LOG_ERROR BUSY_OR_NOT_QUIET"); return; }
    uint32_t id=0; uint8_t operation=log_radio::LIST;
    if(ram) operation=log_radio::RAM;
    else if(release) {
        const char *value=line+12;char *end=nullptr;unsigned long parsed=strtoul(value,&end,10);
        if(*value<'1'||*value>'9'||*end||!parsed||parsed>UINT32_MAX) { Serial.println("LOG_ERROR INVALID_CRC");return; }
        id=static_cast<uint32_t>(parsed);operation=log_radio::RELEASE;
    } else if (strcmp(line,"log list")) {
        char *end=nullptr; const char *value=line+9;
        unsigned long parsed=strtoul(value,&end,10);
        if (*value<'1' || *value>'9' || *end || !parsed || parsed>99999999) { Serial.println("LOG_ERROR INVALID_ID"); return; }
        id=parsed; operation=log_radio::DUMP;
    }
    download_request={};download_request.header=log_radio::header(log_radio::REQUEST_TYPE);
    download_request.session=dongle_session;
    do { download_request.transfer=esp_random(); } while (!download_request.transfer);
    download_request.id=id;download_request.operation=operation;
    download_receiver.next=0; xQueueReset(download_queue);
    downloading=true; require_zero=true;
    download_started=download_progress_us=esp_timer_get_time();download_request_us=0;
}
void serviceDownload(int64_t now) {
    if (!downloading) { log_radio::Data ignored; while(xQueueReceive(download_queue,&ignored,0)==pdTRUE) {} return; }
    if (now-download_started>120000000 || now-download_progress_us>5000000) {
        cancelDownload();Serial.println("LOG_ERROR WIRELESS_TIMEOUT");return;
    }
    if (!download_receiver.next && now-download_request_us>=300000) {
        esp_now_send(CONTROLLER_MAC,reinterpret_cast<const uint8_t*>(&download_request),sizeof(download_request));download_request_us=now;
    }
    log_radio::Data packet;
    if (xQueueReceive(download_queue,&packet,0)!=pdTRUE || packet.transfer!=download_request.transfer) return;
    if (download_receiver.accept(packet.offset,packet.length)) {
        Serial.write(packet.bytes,packet.length);
        download_receiver.next+=packet.length;download_progress_us=now;
    } else if (!download_receiver.duplicate(packet.offset,packet.length)) return;
    auto ack=download_request;ack.operation=log_radio::ACK;ack.offset=download_receiver.next;
    esp_now_send(CONTROLLER_MAC,reinterpret_cast<const uint8_t*>(&ack),sizeof(ack));
    // End markers may be split across packets, so leave receiver active to
    // re-ACK a retransmitted last packet. Host sends log cancel after CRC check.
}

void readSerialInput() {
    // Bound work even if the host continuously streams characters.
    for (unsigned budget = 0; budget < 64 && Serial.available() > 0; ++budget) {
        const int value = Serial.read();
        if (value < 0) break;
        const LineResult result = input.push(static_cast<char>(value));
        if (result == LINE_REJECTED) {
            logLine("Invalid/overlong input: the whole line was discarded (max 96 chars).\n");
        } else if (result == LINE_READY && input.bytes[0] != '\0') {
            if (!strcmp(input.bytes,"log ram") || !strncmp(input.bytes,"log release ",12) || !strcmp(input.bytes,"log list") || !strcmp(input.bytes,"log cancel") || !strncmp(input.bytes,"log dump ",9)) {
                startDownload(input.bytes); continue;
            }
            if (downloading) {
                // Zero cancels transfer before passing to the normal command path.
                if (!strcmp(input.bytes,"0") || !strcmp(input.bytes,"log off")) cancelDownload();
                else { Serial.println("LOG_ERROR DOWNLOAD_ACTIVE"); continue; }
            }
            Command request = {};
            if (parseCommand(input.bytes, request)) submitCommand(request);
            else logLine("Invalid command. Use 0; speed [-10..10] [optional duration 0.05..3s]; throttle [-100..100] [seconds; max 2s]; odom reset; calibrate [metres <=5]. log arm [10|50]/off/status/save/hand/finish; home [x]; position status/invalidate/direction [-1|0|1]; move metres signed_mps (max26m/1mps).\n");
        }
    }
}

void serviceRadio(int64_t now) {
    if (!transmit_enabled || now - last_transmitted_us < TRANSMIT_INTERVAL_US) return;
    last_transmitted_us = now;
    esp_err_t result;
    if (pending_command) {
        // Command retries themselves refresh the cart's keepalive timer.
        result = esp_now_send(CONTROLLER_MAC,
            reinterpret_cast<const uint8_t *>(&pending), sizeof(pending));
    } else {
        Heartbeat heartbeat = {makeHeader(HEARTBEAT), dongle_session};
        result = esp_now_send(CONTROLLER_MAC,
            reinterpret_cast<const uint8_t *>(&heartbeat), sizeof(heartbeat));
    }
    radio_diag::queued(result);
    if (result != ESP_OK && now - last_send_error_us >= 1000000) {
        last_send_error_us = now;
        logLine("Queue ESP-NOW packet: %s\n", esp_err_to_name(result));
    }
}

void setup() {
    const size_t tx_buffer_size = Serial.setTxBufferSize(512);
    Serial.begin(115200);
    if (tx_buffer_size == 0) haltDongle("UART TX buffer allocation failed.");
    download_queue=xQueueCreate(4,sizeof(log_radio::Data));
    if (!download_queue) haltDongle("Download queue allocation failed.");
    position_queue=xQueueCreate(1,sizeof(PositionTelemetry));
    if(!position_queue)haltDongle("Position queue allocation failed.");
    profile_reply_queue=xQueueCreate(1,sizeof(ProfileReply));
    if(!profile_reply_queue)haltDongle("Profile reply allocation failed.");
    pi_reply_queue=xQueueCreate(1,sizeof(PiReply));
    if (!pi_reply_queue) haltDongle("PI reply queue allocation failed.");
    telemetry_queue = xQueueCreate(1, sizeof(TelemetryEvent));
    if (!telemetry_queue) haltDongle("Telemetry queue allocation failed.");
    if (!WiFi.mode(WIFI_STA)) haltDongle("Wi-Fi STA initialization failed.");
    checkSdk(esp_wifi_set_ps(WIFI_PS_NONE), "Disable Wi-Fi power saving");
    checkSdk(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE), "Set Wi-Fi channel");
    checkSdk(esp_now_init(), "Initialize ESP-NOW");
    checkSdk(esp_now_register_send_cb(radio_diag::sent), "Register send diagnostics");
    do { dongle_session = esp_random(); } while (dongle_session == 0);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, CONTROLLER_MAC, 6);
    peer.ifidx = WIFI_IF_STA;
    peer.channel = 1;
    peer.encrypt = false;
    const esp_err_t result = esp_now_add_peer(&peer);
    if (result != ESP_OK && result != ESP_ERR_ESPNOW_EXIST) checkSdk(result, "Add cart peer");
    checkSdk(esp_now_register_recv_cb(OnDataRecv), "Register receive callback");
    Serial.printf("Dongle STA MAC: %s\n", WiFi.macAddress().c_str());
    Serial.println("Dongle onboard-log-29-return2: profile upload and fast reverse support.");
    Serial.println("Dongle protocol v3 ready. Enter 0 and wait for ACK before motion. Motor battery OFF for these tests.");
    Serial.println("Commands: speed 0.2; throttle 5 0.25; odom reset; calibrate 3; 0. Wheel speed/distance are magnitudes.");
    Serial.println("Fixed throttle cap 100% (max 2 s; fixed travel guard 13 m); PI cap stays 10%.");
    Serial.println("Timed PI test: speed 1 3; targets up to 3 m/s, max 3 s, output cap 100%, early neutral at 10 m. Untimed PI remains capped at 10%.");
    Serial.println("Stopped-only RAM tuning: pi set Kp Ki; pi status; pi defaults. After APPLIED send 0 before motion. Kd stays 0.");
    Serial.println("Wireless saved logs: log list; log dump ID; log cancel (stopped only).");
    Serial.println("Logging defaults OFF. Commands: log arm; log off; log status; log save.");
    Serial.println("Predictive forward stop: approach metres speed;0.5..4m,0.5..1m/s,known home required. Existing move is neutral-cutoff baseline.");
    Serial.println("Checkpoint 3/4: left wheel feedback/odometry; boot scale7/264, overrides RAM-only. home [x] declares known position while quiet; move metres signed_mps supports up to4m at1m/s,100% cap,10s max; neutral cutoff then coast.");
    Serial.printf("DBG DONGLE destination=%02X:%02X:%02X:%02X:%02X:%02X\n",
        CONTROLLER_MAC[0], CONTROLLER_MAC[1], CONTROLLER_MAC[2],
        CONTROLLER_MAC[3], CONTROLLER_MAC[4], CONTROLLER_MAC[5]);
}

void loop() {
    readTelemetry();
    readPiReply();
    readProfileReply();
    if(xQueueReceive(position_queue,&latest_position,0)==pdTRUE)position_received_us=esp_timer_get_time();
    int64_t now = esp_timer_get_time();
    if (connected && now - last_received_us >= TELEMETRY_TIMEOUT_US) {
        connected = false;
        pauseLink(">> CONNECTION LOST. <<");
    }
    if (pending_command && now - pending_since_us >= COMMAND_ACK_TIMEOUT_US) {
        pauseLink("No command ACK within 1 s.");
    }
    readSerialInput();
    now = esp_timer_get_time();
    serviceRadio(now);
    serviceDownload(now);
    if (!downloading && connected && (status_changed || now - last_print_us >= PRINT_INTERVAL_US)) {
        last_print_us = now;
        status_changed = false;
        logLine("Live -> |v|: %.3f m/s | Target: %.2f | Mags: %lu | %s | Stop: %s\n",
                latest.speed_magnitude, latest.target_speed,
                static_cast<unsigned long>(latest.track_magnets),
                latest.running ? "RUNNING" : "STOPPED", stopReasonName(latest.stop_reason));
        logLine("LOGGER state=%u error=%u samples=%u flags=%lu saved_id=%lu\n",
            latest.log_state,latest.log_error,latest.log_samples,
            static_cast<unsigned long>(latest.log_flags),static_cast<unsigned long>(latest.log_saved_id));
        if(position_received_us && now-position_received_us<1000000) {
            const auto &x=latest_position.position;
            logLine("Position -> raw=%.4f x=%.4f cutoff=%.4f epoch=%lu dir=%d flags=%u reason=%u source=%u request=%lu result=%u seq=%lu\n",
                x.raw_m,x.x_m,x.cutoff_m,static_cast<unsigned long>(x.epoch),x.direction,x.flags,x.reason,x.cutoff_source,
                static_cast<unsigned long>(latest_position.request_sequence),latest_position.result,static_cast<unsigned long>(latest_position.acknowledged_sequence));
        }
        logLine("Odom -> count=%lu total=%.4f m run=%.4f m marker_interval=%.4f m marker_error=%.4f m m_per_pulse=%.7f cal=%lu markers_total=%lu flags=%u mode=%s throttle=%.2f%%\n",
                static_cast<unsigned long>(latest.wheel_count), latest.total_distance_m, latest.run_distance_m,
                latest.marker_interval_m, latest.marker_error_m, latest.meters_per_pulse,
                static_cast<unsigned long>(latest.calibration_id), static_cast<unsigned long>(latest.marker_total), latest.flags, kindName(latest.mode), latest.throttle_percent);
        logLine("Return capability -> max_mps=%.1f\n",
                (latest.flags & FLAG_FAST_REVERSE) ? REVERSE_MOVE_MAX_SPEED_MPS : MOVE_MAX_SPEED_MPS);
    }
    if (!downloading && radio_diag::report("DONGLE")) {
        radio_diag::log("DBG DONGLE tx_enabled=%u pending=%u seq=%lu session=%lu last_ack=%lu\n",
            transmit_enabled ? 1u : 0u, pending_command ? 1u : 0u,
            static_cast<unsigned long>(pending.sequence),
            static_cast<unsigned long>(dongle_session),
            static_cast<unsigned long>(latest.acknowledged_sequence));
    }
    delay(1);
}
