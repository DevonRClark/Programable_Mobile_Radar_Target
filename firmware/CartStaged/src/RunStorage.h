#pragma once
#include <stdio.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include "RunRecorder.h"

namespace experiment {
inline uint32_t crcUpdate(uint32_t crc, const void *bytes, size_t length) {
    const uint8_t *p = static_cast<const uint8_t *>(bytes);
    while (length--) {
        crc ^= *p++;
        for (unsigned n = 0; n < 8; ++n) crc = (crc>>1) ^ (0xedb88320u & (0u-(crc&1)));
    }
    return crc;
}
#pragma pack(push, 1)
struct FileHeader {
    uint32_t magic = 0x524c4f47, version = 3, id = 0;
    uint32_t header_bytes = sizeof(FileHeader), sample_bytes = sizeof(Sample), count = 0;
    uint32_t checksum = 0, flags = 0;
    uint32_t pwm_hz = 400, control_hz = 100, log_hz = 50;
    uint32_t protocol = cart::VERSION, wheel_pin = 32, track_pin = 35;
    float kp = 25, ki = 3, output_cap = cart::BENCH_MAX_THROTTLE;
    uint32_t overruns = 0, pwm_faults = 0, work_max_us = 0;
    char firmware[32] = "onboard-log-33-return2";
};
#pragma pack(pop)
class Storage {
public:
    const char *base;
    bool mounted = false;
    explicit Storage(const char *path) : base(path) {}
    void path(char *out, size_t cap, uint32_t id, const char *suffix = "bin") const {
        snprintf(out, cap, "%s/run-%08lu.%s", base, static_cast<unsigned long>(id), suffix);
    }
    bool exists(uint32_t id, const char *suffix = "bin") const {
        char name[96]; path(name,sizeof(name),id,suffix);
        return access(name,F_OK) == 0;
    }
    uint32_t nextId() const {
        DIR *dir = opendir(base);
        if (!dir) return 0;
        uint32_t maximum = 0;
        while (auto *entry = readdir(dir)) {
            unsigned long id = 0; char suffix[5] = {}, tail = 0;
            if (sscanf(entry->d_name,"run-%8lu.%4s%c",&id,suffix,&tail) == 2 &&
                (!strcmp(suffix,"bin") || !strcmp(suffix,"tmp")) && id > maximum) maximum = id;
        }
        closedir(dir);
        return maximum >= 99999999 ? 0 : maximum+1;
    }
    bool verify(uint32_t id, FileHeader &h, const char *suffix = "bin") const {
        char name[96]; path(name,sizeof(name),id,suffix);
        FILE *file = fopen(name,"rb");
        if (!file) return false;
        bool good = fread(&h,1,sizeof(h),file) == sizeof(h) && h.magic == 0x524c4f47 &&
            (h.version >= 1 && h.version<=3) && h.id == id && h.header_bytes == sizeof(h) &&
            h.sample_bytes == (h.version==1 ? 99u:h.version==2 ? 123u:sizeof(Sample)) && h.count > 0 && h.count <= (h.version<=2 ? 1024u:CAPACITY);
        const uint32_t expected = h.checksum;
        h.checksum = 0;
        uint32_t crc = crcUpdate(0xffffffffu,&h,sizeof(h));
        Sample sample = {};
        for (uint32_t n = 0; good && n < h.count; ++n) {
            good = fread(&sample,1,h.sample_bytes,file) == h.sample_bytes;
            if (good) crc = crcUpdate(crc,&sample,h.sample_bytes);
        }
        good = good && fgetc(file) == EOF && !ferror(file) && (crc^0xffffffffu) == expected;
        fclose(file); h.checksum = expected;
        return good;
    }
    uint8_t save(Recorder &r, FileHeader h, size_t available_bytes) {
        if (!mounted) return NOT_MOUNTED;
        if (r.state != FROZEN || !r.count) return INVALID_REQUEST;
        if (available_bytes < sizeof(h)+r.count*sizeof(Sample)+4096) return STORAGE_FULL;
        h.version=3;h.sample_bytes=sizeof(Sample);
        h.id = nextId();
        if (!h.id) return IO_ERROR;
        h.count = r.count; h.flags = r.flags; h.checksum = 0;
        uint32_t crc = crcUpdate(0xffffffffu,&h,sizeof(h));
        for (size_t n = 0; n < r.count; ++n) crc = crcUpdate(crc,&r.at(n),sizeof(Sample));
        h.checksum = crc^0xffffffffu;
        char temporary[96], final[96]; path(temporary,sizeof(temporary),h.id,"tmp"); path(final,sizeof(final),h.id);
        if (exists(h.id)) return IO_ERROR;
        const int fd = open(temporary,O_WRONLY|O_CREAT|O_EXCL,0600);
        if (fd < 0) return IO_ERROR;
        FILE *file = fdopen(fd,"wb");
        if (!file) { close(fd); return IO_ERROR; }
        bool good = fwrite(&h,1,sizeof(h),file) == sizeof(h);
        for (size_t n = 0; good && n < r.count; ++n) good = fwrite(&r.at(n),1,sizeof(Sample),file) == sizeof(Sample);
        if (fflush(file)) good = false;
        if (fclose(file)) good = false;
        FileHeader checked = {};
        if (!good || !verify(h.id,checked,"tmp")) return IO_ERROR;
        if (exists(h.id) || rename(temporary,final)) return IO_ERROR;
        if (!verify(h.id,checked)) return CORRUPT;
        r.saved_id = h.id; r.state = SAVED; r.enabled = r.auto_save = false;
        return OK;
    }
    template<class Emit> bool dump(uint32_t id, Emit emit) const {
        FileHeader h = {};
        if (!mounted || !verify(id,h)) return false;
        char name[96], line[512]; path(name,sizeof(name),id);
        FILE *file = fopen(name,"rb");
        if (!file || fseek(file,sizeof(h),SEEK_SET)) { if(file) fclose(file); return false; }
        snprintf(line,sizeof(line),"LOG_BEGIN %lu\n",static_cast<unsigned long>(id)); emit(line);
        uint32_t crc = 0xffffffffu;
        snprintf(line,sizeof(line),
            "LOG_META {\"sample_version\":%lu,\"id\":%lu,\"count\":%lu,\"flags\":%lu,\"binary_crc32\":%lu,\"firmware\":\"%.32s\",\"protocol\":%lu,\"pwm_hz\":%lu,\"control_hz\":%lu,\"log_hz\":%lu,\"kp\":%.3f,\"ki\":%.3f,\"output_cap\":%.1f,\"overruns\":%lu,\"pwm_faults\":%lu,\"work_max_us\":%lu}\n",
            static_cast<unsigned long>(h.version),static_cast<unsigned long>(id),static_cast<unsigned long>(h.count),static_cast<unsigned long>(h.flags),
            static_cast<unsigned long>(h.checksum),h.firmware,static_cast<unsigned long>(h.protocol),
            static_cast<unsigned long>(h.pwm_hz),static_cast<unsigned long>(h.control_hz),static_cast<unsigned long>(h.log_hz),
            h.kp,h.ki,h.output_cap,static_cast<unsigned long>(h.overruns),static_cast<unsigned long>(h.pwm_faults),static_cast<unsigned long>(h.work_max_us));
        crc = crcUpdate(crc,line,strlen(line)); emit(line);
        Sample first = {}, last = {};
        if (fread(&first,1,h.sample_bytes,file) != h.sample_bytes ||
            fseek(file,sizeof(h)+(h.count-1)*h.sample_bytes,SEEK_SET) ||
            fread(&last,1,h.sample_bytes,file) != h.sample_bytes || fseek(file,sizeof(h),SEEK_SET)) {
            fclose(file); return false;
        }
        snprintf(line,sizeof(line),"LOG_DETAILS {\"meters_per_pulse\":%.7f,\"calibration_id\":%lu,\"stop_reason\":%u,\"start_us\":%lld,\"end_us\":%lld,\"wheel_pin\":%lu,\"track_pin\":%lu}\n",
            first.packet.meters_per_pulse,static_cast<unsigned long>(first.packet.calibration_id),last.packet.stop_reason,
            static_cast<long long>(first.time_us),static_cast<long long>(last.time_us),
            static_cast<unsigned long>(h.wheel_pin),static_cast<unsigned long>(h.track_pin));
        crc = crcUpdate(crc,line,strlen(line)); emit(line);
        bool good = true;
        for (uint32_t n = 0; n < h.count; ++n) {
            Sample sample = {};
            if (fread(&sample,1,h.sample_bytes,file) != h.sample_bytes) { good = false; break; }
            const int length = format(line,sizeof(line),sample,h.version);
            if (length <= 0 || static_cast<size_t>(length) >= sizeof(line)) { good = false; break; }
            crc = crcUpdate(crc,line,length); emit(line);
        }
        fclose(file);
        if (!good) return false;
        snprintf(line,sizeof(line),"LOG_END %lu %lu %lu\n",static_cast<unsigned long>(id),
            static_cast<unsigned long>(h.count),static_cast<unsigned long>(crc^0xffffffffu)); emit(line);
        return true;
    }
    // Immutable recorder ownership is granted only at neutral/quiet maintenance.
    // id0 is a volatile snapshot, never a filesystem name or persistent recording ID.
    template<class Emit> bool dumpMemory(const Recorder &r, FileHeader h, Emit emit, uint32_t *snapshot_crc=nullptr) const {
        if (!r.buffer || !r.count || (r.state!=FROZEN && r.state!=SAVED && !(r.state==OFF && r.saved_id))) return false;
        h.id=0; h.count=r.count; h.flags=r.flags; h.checksum=0;
        uint32_t binary=crcUpdate(0xffffffffu,&h,sizeof(h));
        for(size_t n=0;n<r.count;++n) binary=crcUpdate(binary,&r.at(n),sizeof(Sample));
        h.checksum=binary^0xffffffffu;
        if(snapshot_crc)*snapshot_crc=h.checksum;
        char line[640]; uint32_t crc=0xffffffffu;
        emit("LOG_BEGIN 0\n");
        snprintf(line,sizeof(line),
            "LOG_META {\"sample_version\":3,\"id\":0,\"count\":%lu,\"flags\":%lu,\"binary_crc32\":%lu,\"firmware\":\"%.32s\",\"protocol\":%lu,\"pwm_hz\":%lu,\"control_hz\":%lu,\"log_hz\":%lu,\"kp\":%.3f,\"ki\":%.3f,\"output_cap\":%.1f,\"overruns\":%lu,\"pwm_faults\":%lu,\"work_max_us\":%lu,\"source\":\"ram\",\"persistent\":false,\"saved_id\":%lu}\n",
            (unsigned long)h.count,(unsigned long)h.flags,(unsigned long)h.checksum,h.firmware,
            (unsigned long)h.protocol,(unsigned long)h.pwm_hz,(unsigned long)h.control_hz,(unsigned long)h.log_hz,
            h.kp,h.ki,h.output_cap,(unsigned long)h.overruns,(unsigned long)h.pwm_faults,(unsigned long)h.work_max_us,(unsigned long)r.saved_id);
        crc=crcUpdate(crc,line,strlen(line));emit(line);
        const auto &first=r.at(0);const auto &last=r.at(r.count-1);
        snprintf(line,sizeof(line),"LOG_DETAILS {\"meters_per_pulse\":%.7f,\"calibration_id\":%lu,\"stop_reason\":%u,\"start_us\":%lld,\"end_us\":%lld,\"wheel_pin\":%lu,\"track_pin\":%lu}\n",
            first.packet.meters_per_pulse,(unsigned long)first.packet.calibration_id,last.packet.stop_reason,
            (long long)first.time_us,(long long)last.time_us,(unsigned long)h.wheel_pin,(unsigned long)h.track_pin);
        crc=crcUpdate(crc,line,strlen(line));emit(line);
        for(size_t n=0;n<r.count;++n) {
            const int length=format(line,sizeof(line),r.at(n),h.version);
            if(length<=0 || static_cast<size_t>(length)>=sizeof(line))return false;
            crc=crcUpdate(crc,line,length);emit(line);
        }
        snprintf(line,sizeof(line),"LOG_END 0 %lu %lu\n",(unsigned long)r.count,(unsigned long)(crc^0xffffffffu));emit(line);
        return true;
    }
    template<class Emit> void list(Emit emit) const {
        DIR *dir = mounted ? opendir(base) : nullptr;
        if (!dir) { emit("LOG_ERROR NOT_MOUNTED\n"); return; }
        while (auto *entry = readdir(dir)) {
            unsigned long id = 0; char suffix[5] = {}, tail = 0;
            if (sscanf(entry->d_name,"run-%8lu.%4s%c",&id,suffix,&tail) != 2 || strcmp(suffix,"bin")) continue;
            FileHeader h = {}; const bool valid = verify(id,h);
            char line[96]; snprintf(line,sizeof(line),"LOG_FILE %lu count=%lu flags=%lu valid=%u\n",id,
                static_cast<unsigned long>(valid ? h.count : 0),static_cast<unsigned long>(valid ? h.flags : 0),valid); emit(line);
        }
        closedir(dir); emit("LOG_LIST_END\n");
    }
    bool erase(uint32_t id) {
        if (!mounted || !id) return false;
        char name[96]; path(name,sizeof(name),id);
        return remove(name) == 0;
    }
};
} // namespace experiment
