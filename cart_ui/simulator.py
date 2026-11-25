"""Deterministic protocol emulator; never accesses hardware. Not a dynamics model."""
import json
import struct
import time
import zlib

class SimulatedSerial:
    simulated = True
    def __init__(self):
        self.buffer = bytearray(); self.commands = []; self.seq = 0; self.closed = False
        self.running_until = 0.; self.next_status = 0.; self.log_state = 0
        self.x = .792; self.epoch = 0; self.position_request = 0; self.position_result = 0
        self.kp, self.ki = 10., 20.; self.points = []; self.crc = 0; self.floor = self.launch = 1.
        self.count = 0; self.uploading = False; self.stop_reason = 'NONE'; self.logging = False
        self.motion_origin=self.x; self.motion_delta=0.; self.motion_started=0.; self.motion_kind='profile'; self.run_target=0.; self.saved_id=0
        self.target = 0.; self.run_sequence = 0; self.suppress_ack = False; self.corrupt_download = False
        self.fast_reverse = True

    def line(self, text): self.buffer.extend((text+'\n').encode())
    def tick(self):
        now = time.monotonic()
        if self.running_until:
            self.x = self.motion_origin + self.motion_delta * min(1., max(0., now-self.motion_started))
        if self.running_until and now >= self.running_until:
            self.running_until = 0.; self.target = 0.; self.stop_reason = 'DISTANCE CUTOFF' if self.motion_kind=='move' else 'AT TARGET' if self.motion_kind=='profile' else 'TEST LIMIT'
            self.log_state = 4 if self.logging else 0
        if now >= self.next_status:
            self.next_status = now + .08; running = bool(self.running_until)
            self.line(f'Live -> |v|: {1.0 if running else 0:.3f} m/s | Target: {self.target:.2f} | Mags: 0 | {"RUNNING" if running else "STOPPED"} | Stop: {self.stop_reason}')
            self.line(f'LOGGER state={2 if running and self.logging else self.log_state} error=0 samples={3 if self.log_state in (4,5) else 0} flags=0 saved_id={self.saved_id}')
            self.line(f'Position -> raw=0 x={self.x:.4f} cutoff=8.792 epoch={self.epoch} dir=1 flags={3 if self.epoch else 0} reason=0 source=0 request={self.position_request} result={self.position_result} seq={self.seq}')
            self.line('Odom -> count=0 total=0 m run=0 m m_per_pulse=0.0265152 cal=1 flags=0 mode=PROFILE throttle=0.00%')
            if self.fast_reverse: self.line('Return capability -> max_mps=2.0')

    @property
    def in_waiting(self): self.tick(); return len(self.buffer)
    def read(self, size=1):
        self.tick(); data = bytes(self.buffer[:size]); del self.buffer[:size]; return data
    def write(self, data):
        for command in data.decode().splitlines():
            if not command: continue
            self.commands.append(command); p = command.split()
            if command == 'log cancel': continue
            if command == 'log ram' or command.startswith('log dump '):
                record_id=0 if command=='log ram' else int(p[-1])
                if self.log_state not in (4,5): self.line('LOG_ERROR NO_RECORDING'); continue
                meta = dict(id=record_id, count=3, sample_version=2, source='ram' if not record_id else 'flash', persistent=bool(record_id),
                            binary_crc32=12345+self.run_sequence, firmware='SIMULATOR', flags=0, stop_reason=16 if self.motion_kind=='move' else 19)
                content = 'LOG_META '+json.dumps(meta)+'\n'
                for i in range(3):
                    position=self.motion_origin+self.motion_delta*i/2
                    fields = [0,i*100000, self.run_sequence,3 if self.motion_kind=='move' else 23,self.run_target,0 if i==2 else abs(self.run_target),20,1600,i,abs(self.motion_delta)*i/2,abs(self.motion_delta)*i/2,0,0,0,0,0,0,.02651515,1]
                    fields += [position,position,0,self.motion_origin+self.motion_delta,self.epoch,-1 if self.motion_delta<0 else 1,3,1,0]
                    content += 'CSV,'+','.join(map(str,fields))+'\n'
                crc = zlib.crc32(content.encode()) + int(self.corrupt_download)
                self.line(f'LOG_BEGIN {record_id}'); self.buffer.extend(content.encode()); self.line(f'LOG_END {record_id} 3 {crc}')
                continue
            if command.startswith('log release '):
                self.log_state = 0; self.line('LOG_RELEASED '+p[-1]); continue
            self.seq += 1; seq = self.seq
            self.line(f'Queued command #{seq}: {command}')
            if command == '0':
                if self.running_until and self.logging: self.log_state=4
                self.running_until=0; self.target=0
            elif p[:2] == ['profile','begin']:
                self.count=int(p[2]); self.floor=float(p[3]); self.launch=float(p[4]); self.points=[]; self.uploading=True
            elif p[:2] == ['profile','point']: self.points.append((float(p[3]),float(p[4])))
            elif p[:2] == ['profile','commit']:
                self.crc=zlib.crc32(b''.join(struct.pack('<ff',*v) for v in self.points)); self.uploading=False
            elif p[0]=='home':
                self.x=float(p[1]); self.epoch+=1; self.position_request=seq; self.position_result=1
            elif command=='position status': self.position_request=seq; self.position_result=0
            elif p[:2]==['pi','set']: self.kp,self.ki=map(float,p[2:])
            elif p[:2]==['log','arm']: self.log_state=1; self.logging=True
            elif command=='log off': self.logging=False; self.log_state=0
            elif p[:2]==['profile','run'] or p[0] in ('speed','throttle','move'):
                self.motion_kind=p[0]; self.motion_origin=self.x; self.motion_started=time.monotonic()
                self.target=self.launch if p[0]=='profile' else float(p[2]) if p[0]=='move' else float(p[1])
                self.run_target=self.target
                self.motion_delta=(self.points[-1][0] if p[0]=='profile' else float(p[1])*(-1 if self.target<0 else 1) if p[0]=='move' else .5*(-1 if self.target<0 else 1))
                self.run_sequence=seq; self.running_until=self.motion_started+1.; self.stop_reason='NONE'
            if p[0]=='profile' and p[1]!='run':
                duration=sum(2*(b[0]-a[0])/(a[1]+b[1]) for a,b in zip(self.points,self.points[1:]) if a[1]+b[1]>0)
                result='STATUS' if p[1]=='status' else 'APPLIED'
                self.line(f'PROFILE_REPLY sequence={seq} result={result} crc={self.crc} count={self.count} received={len(self.points)} uploading={int(self.uploading)} duration={duration:.3f} floor={self.floor:.3f} launch={self.launch:.3f}')
            if p[0]=='pi': self.line(f'PI_REPLY sequence={seq} result={"STATUS" if p[1]=="status" else "APPLIED"} kp={self.kp} ki={self.ki} kd=0 persistence=RAM')
            if not self.suppress_ack: self.line(f'Cart acknowledged command #{seq}; OK.')
            self.next_status=0
        return len(data)
    def close(self): self.closed=True
