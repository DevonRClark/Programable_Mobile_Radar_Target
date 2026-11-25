#pragma once
// Included inside namespace firmware, after engine/queues/writePwm/publish exist.
#if CART_RECORDER_AVAILABLE
namespace recorder_runtime {
experiment::Recorder recorder;
experiment::Storage storage("/littlefs");
SharedU32 work_state{0}; // 0 idle, 1 granted by neutral control task, 2 completed
SharedU32 maintenance_us{0}, maintenance_count{0};
enum : uint8_t { INIT = 20, LIST, DUMP, ERASE, DISCARD, RAM_DUMP, RAM_RELEASE };
struct Request { uint8_t operation; uint32_t id; };
Request job = {};
QueueHandle_t requests = nullptr, replies = nullptr;
struct Reply { uint8_t operation, error; uint32_t id; };
cart::LineBuffer serial_line;
experiment::FileHeader save_header, recording_header;
bool recording_header_valid=false;
uint32_t ram_release_crc=0; // worker-owned; only a completed immutable snapshot may be released
bool maintenance_pass = false;
QueueHandle_t radio_requests = nullptr;
SharedU32 radio_session{0}, radio_transfer{0}, radio_ack{0}, radio_cancel{0};
log_radio::Request radio_job = {};
uint32_t previous_radio_session=0, previous_radio_transfer=0; // control-owned dedup
bool radio_output = false; // worker-owned; set by control before granting ownership
bool receiveRadio(const uint8_t *data,int length) {
    if (length!=sizeof(log_radio::Request)) return false;
    log_radio::Request r; memcpy(&r,data,sizeof(r));
    if (r.header.type!=log_radio::REQUEST_TYPE) return false;
    if (!log_radio::valid(r)) return true;
    if (r.operation==log_radio::ACK || r.operation==log_radio::CANCEL) {
        if (r.session==radio_session.load() && r.transfer==radio_transfer.load()) {
            if (r.operation==log_radio::ACK) radio_ack.store(r.offset);
            else radio_cancel.store(1);
        }
    } else if (radio_requests) xQueueSend(radio_requests,&r,0);
    return true;
}
bool sendRadioText(const char *text,uint32_t &offset) {
    while (*text) {
        if (radio_cancel.load()) return false;
        log_radio::Data packet = {};
        packet.header=log_radio::header(log_radio::DATA_TYPE);
        packet.session=radio_job.session; packet.transfer=radio_job.transfer; packet.offset=offset;
        packet.length=static_cast<uint16_t>(strlen(text)>log_radio::PAYLOAD ? log_radio::PAYLOAD : strlen(text));
        memcpy(packet.bytes,text,packet.length);
        bool delivered=false;
        for (unsigned retry=0;retry<10 && !radio_cancel.load();++retry) {
            esp_now_send(settings::DONGLE_MAC,reinterpret_cast<const uint8_t*>(&packet),sizeof(packet));
            const int64_t until=esp_timer_get_time()+300000;
            while (esp_timer_get_time()<until && !radio_cancel.load()) {
                if (radio_ack.load()==offset+packet.length) { delivered=true; break; }
                delay(1); // yield Wi-Fi and idle tasks while control is parked at neutral
            }
            if (delivered) break;
        }
        if (!delivered) return false;
        offset+=packet.length; text+=packet.length;
    }
    return true;
}

bool quiet(int64_t now, const control::Sensors &s) {
    return !s.wheel.seen || now-s.wheel.last_us >= cart::WHEEL_STOP_US;
}
void fill(cart::Telemetry &packet) {
    packet.log_state = recorder.state;
    packet.log_error = recorder.error;
    packet.log_samples = recorder.count;
    packet.log_flags = recorder.flags;
    packet.log_saved_id = recorder.saved_id;
}
void printStatus() {
    // Read the published immutable snapshot; never read producer-owned recorder here.
    Frame frame;
    if (xQueuePeek(telemetry_queue,&frame,0) != pdTRUE) return;
    Serial.printf("LOGGER state=%u error=%u samples=%u flags=%lu saved_id=%lu enabled_build=1 maintenance_count=%lu maintenance_us=%lu\n",
        frame.packet.log_state,frame.packet.log_error,frame.packet.log_samples,
        static_cast<unsigned long>(frame.packet.log_flags),static_cast<unsigned long>(frame.packet.log_saved_id),
        static_cast<unsigned long>(maintenance_count.load()),static_cast<unsigned long>(maintenance_us.load()));
}
bool request(Request request, int64_t now, const control::Sensors &s) {
    if (request.operation == cart::LOG_OFF) {
        recorder.off();
        if (log_queue) xQueueReset(log_queue);
        return true;
    }
    if (request.operation == cart::LOG_STATUS) return true;
    if (request.operation == cart::LOG_FINISH) {
        return !engine.running && recorder.finishManual(quiet(now,s));
    }
    if (engine.running || !quiet(now,s) || work_state.load()) { recorder.error = experiment::BUSY; return false; }
    if ((request.operation == cart::LOG_ARM || request.operation == cart::LOG_HAND) && (recorder.active() || recorder.state == experiment::FROZEN)) {
        recorder.error = experiment::BUSY; return false;
    }
    if (request.operation == cart::LOG_SAVE && (recorder.state != experiment::FROZEN || !recorder.count)) {
        recorder.error = experiment::INVALID_REQUEST; return false;
    }
    if (request.operation==RAM_DUMP && (!recorder.count || !recorder.buffer ||
        (recorder.state!=experiment::FROZEN && recorder.state!=experiment::SAVED && !(recorder.state==experiment::OFF && recorder.saved_id)))) {
        recorder.error=experiment::INVALID_REQUEST;return false;
    }
    // Park control BEFORE a worker can touch storage or allocation.
    engine.holdForMaintenance(now);
    engine.maintenance = true;
    if (!writePwm(0,true)) { engine.pwmFailure(now); recorder.error = experiment::IO_ERROR; engine.maintenance = false; return false; }
    if (log_queue) xQueueReset(log_queue);
    job = request;
    save_header = experiment::FileHeader{};
    save_header.wheel_pin = settings::FEEDBACK_WHEEL_PIN;
    save_header.pwm_hz = settings::PWM_HZ;
    save_header.control_hz = 1000000/settings::CONTROL_US;
    save_header.log_hz = recorder.log_hz;
    save_header.output_cap = engine.outputLimit();
    save_header.kp = engine.pi.kp; save_header.ki = engine.pi.ki;
    save_header.overruns = get(diag.overruns); save_header.pwm_faults = get(diag.pwm_fault);
    save_header.work_max_us = get(diag.work_max_us);
    if (save_header.overruns || save_header.pwm_faults) recorder.flags |= experiment::CONTROL_FAULT;
    if ((request.operation==cart::LOG_SAVE || request.operation==RAM_DUMP) && !recording_header_valid) {
        recording_header=save_header;recording_header_valid=true;
    }
    auto packet = engine.telemetry(); fill(packet); publish(packet);
    work_state.store(1); // lock release publishes job/recorder ownership to worker
    return true;
}
bool maintenance(control::Deadline &deadline) {
    maintenance_pass = false;
    const uint32_t state = work_state.load();
    if (!state) return false;
    maintenance_pass = true;
    phase(MAINTENANCE);
    if (state == 2) {
        // Throw away commands received during maintenance; require new zero.
        xQueueReset(rx_queue); rx_overflow.store(0);
        engine.maintenance = false;
        engine.holdForMaintenance(esp_timer_get_time());
        work_state.store(0); // ownership returns to control
        auto packet = engine.telemetry(); fill(packet); publish(packet);
    }
    // Long downloads keep a fresh neutral frame without exposing worker-owned recorder.
    Frame held;
    if (xQueuePeek(telemetry_queue,&held,0)==pdTRUE) publish(held.packet);
    xQueueReset(rx_queue); rx_overflow.store(0);
    const int64_t now = esp_timer_get_time();
    deadline.next_us = now; deadline.previous_us = now-settings::CONTROL_US;
    diag.control_ms.store(millis());
    return true;
}
void observe(int64_t now, bool periodic) {
    log_radio::Request remote;
    if (!work_state.load() && xQueueReceive(radio_requests,&remote,0)==pdTRUE &&
        remote.session==engine.history.session &&
        !(remote.session==previous_radio_session && remote.transfer==previous_radio_transfer)) {
        // Transfer requests never stop an active run or take ownership of its recorder.
        if (!engine.running && quiet(now,readSensors()) && !recorder.active() && (remote.operation==log_radio::RAM || remote.operation==log_radio::RELEASE || recorder.state!=experiment::FROZEN)) {
            previous_radio_session=remote.session; previous_radio_transfer=remote.transfer;
            radio_output=true; radio_job=remote;
            radio_session.store(remote.session); radio_transfer.store(remote.transfer);
            radio_ack.store(0); radio_cancel.store(0);
            if (!request({remote.operation==log_radio::RELEASE ? RAM_RELEASE : remote.operation==log_radio::RAM ? RAM_DUMP : remote.operation==log_radio::DUMP ? DUMP : LIST,remote.id},now,readSensors())) radio_output=false;
        }
    }
    Request local;
    if (xQueueReceive(requests,&local,0) == pdTRUE) {
        const bool accepted = request(local,now,readSensors());
        if (!work_state.load()) {
            const Reply reply{local.operation,accepted ? static_cast<uint8_t>(experiment::OK) : recorder.error,local.id};
            xQueueSend(replies,&reply,0);
        }
    }
    if (work_state.load()) return;
    if (recorder.state == experiment::FROZEN && recorder.auto_save &&
        !engine.running && quiet(now,readSensors())) request({cart::LOG_SAVE,0},now,readSensors());
    (void)periodic;
}
bool mount() {
    if (!storage.mounted) storage.mounted = LittleFS.begin(false);
    return storage.mounted;
}
void worker() {
    if (work_state.load() != 1) return;
    const int64_t began = esp_timer_get_time();
    uint8_t result = experiment::OK;
    uint32_t radio_offset=0; bool radio_good=true;
    auto emit = [&](const char *text) {
        if (radio_output) { if (radio_good) radio_good=sendRadioText(text,radio_offset); }
        else Serial.print(text);
    };
    uint32_t snapshot_crc=0;
    switch(job.operation) {
        case INIT:
            if (!mount()) {
                // Explicit initialization is the only operation allowed to format.
                if (!LittleFS.format() || !mount()) result = experiment::IO_ERROR;
            }
            break;
        case cart::LOG_HAND:
        case cart::LOG_ARM:
            if (!mount()) result = experiment::NOT_MOUNTED;
            else if (!(job.operation == cart::LOG_HAND ? recorder.armManual(engine.odometry.wheel_count) : recorder.arm())) result = recorder.error;
            else { if(job.operation==cart::LOG_ARM) recorder.log_hz=job.id ? job.id : 50; recording_header_valid=false;ram_release_crc=0;}
            break;
        case cart::LOG_SAVE:
            if (!mount()) result = experiment::NOT_MOUNTED;
            else {
                const size_t total = LittleFS.totalBytes(), used = LittleFS.usedBytes();
                result = storage.save(recorder,recording_header,total > used ? total-used : 0);
            }
            break;
        case LIST:
            if (!mount()) result = experiment::NOT_MOUNTED;
            else storage.list(emit);
            break;
        case DUMP:
            if (!mount()) result = experiment::NOT_MOUNTED;
            else if (!storage.dump(job.id,emit)) result = experiment::CORRUPT;
            break;
        case RAM_DUMP:
            // No mount, allocation, flash write, reset or mutation of samples.
            if (!storage.dumpMemory(recorder,recording_header,emit,&snapshot_crc)) result=experiment::INVALID_REQUEST;
            break;
        case RAM_RELEASE:
            if(!job.id || job.id!=ram_release_crc || !recorder.count || recorder.active()) result=experiment::INVALID_REQUEST;
            else {
                recorder.enabled=recorder.auto_save=false;recorder.state=experiment::OFF;
                recorder.count=recorder.start=0;recorder.flags=0;recorder.saved_id=0;
                recording_header_valid=false;ram_release_crc=0;
                char line[64];snprintf(line,sizeof(line),"LOG_RELEASED %lu\n",(unsigned long)job.id);emit(line);
            }
            break;
        case ERASE:
            if (!mount()) result = experiment::NOT_MOUNTED;
            else if (!storage.erase(job.id)) result = experiment::IO_ERROR;
            break;
        case DISCARD:
            recorder.enabled = recorder.auto_save = false;
            recorder.state = experiment::OFF; recorder.count = recorder.start = 0;
            recorder.flags = 0; recorder.saved_id = 0; recording_header_valid=false;ram_release_crc=0;
            break;
        default: result = experiment::INVALID_REQUEST; break;
    }
    if (radio_output) {
        if (result!=experiment::OK) emit("LOG_ERROR WIRELESS_READ_FAILED\n");
        if(job.operation==RAM_DUMP && result==experiment::OK && radio_good)ram_release_crc=snapshot_crc;
        radio_output=false; radio_transfer.store(0);
        if (!radio_good) result=experiment::IO_ERROR;
    }
    recorder.error = result;
    if (result != experiment::OK) recorder.auto_save = false; // explicit save/arm to retry, no flash retry loop
    maintenance_us.store(static_cast<uint32_t>(esp_timer_get_time()-began));
    maintenance_count.increment();
    Serial.printf("LOG_REPLY operation=%u id=%lu error=%u\n",job.operation,static_cast<unsigned long>(job.id),result);
    work_state.store(2); // release worker-owned recorder/header before control resumes
}
void readSerial() {
    Reply reply;
    while (xQueueReceive(replies,&reply,0) == pdTRUE) {
        Serial.printf("LOG_REPLY operation=%u id=%lu error=%u\n",reply.operation,static_cast<unsigned long>(reply.id),reply.error);
    }
    for (unsigned budget = 0; budget < 64 && Serial.available() > 0; ++budget) {
        const auto result = serial_line.push(static_cast<char>(Serial.read()));
        if (result == cart::LINE_PENDING) continue;
        if (result == cart::LINE_REJECTED) { Serial.println("LOG_ERROR INVALID_REQUEST"); continue; }
        const char *line = serial_line.bytes;
        if (!*line) continue;
        Request request = {};
        cart::Command command = {};
        bool valid = false;
        if (cart::parseCommand(line,command) && cart::isLogCommand(command.kind)) {
            request.operation = command.kind; request.id=command.kind==cart::LOG_ARM ? command.duration_ms : 0; valid = true;
        } else if (!strcmp(line,"log init")) { request.operation = INIT; valid = true;
        } else if (!strcmp(line,"log ram")) { request.operation = RAM_DUMP; valid = true;
        } else if (!strcmp(line,"log list")) { request.operation = LIST; valid = true;
        } else if (!strcmp(line,"log discard")) { request.operation = DISCARD; valid = true;
        } else {
            const char *value = nullptr;
            if (!strncmp(line,"log dump ",9)) { request.operation = DUMP; value = line+9; }
            if (!strncmp(line,"log erase ",10)) { request.operation = ERASE; value = line+10; }
            if (value && *value >= '1' && *value <= '9') {
                char *end = nullptr; unsigned long id = strtoul(value,&end,10);
                if (end != value && !*end && id <= 99999999) { request.id = id; valid = true; }
            }
        }
        if (!valid || xQueueSend(requests,&request,0) != pdTRUE) Serial.println("LOG_ERROR INVALID_OR_BUSY");
    }
}
} // namespace recorder_runtime
#endif
