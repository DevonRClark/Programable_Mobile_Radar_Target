"""Serial protocol and operations. No Qt, no automatic motion retries."""
from __future__ import annotations
import csv
import io
import json
import math
import os
from pathlib import Path
import re
import time
from datetime import datetime, timezone

from .protocol import (PROFILE_LIMITS, columns_for_metadata, decode_transfer,
                       make_profile, parse_line, parse_pi, plan_return,
                       profile_matches, query_profile, upload_profile, RETURN_LEGACY_MAX_MPS)

HOME = .792
RETURN_SPEED = .75
FRESH_SECONDS = 1.5


def return_neutral_margin(speed):
    return .12 if speed <= RETURN_SPEED else .20

class Cancelled(Exception):
    pass


def load_csv(path):
    with open(path, newline='', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        if reader.fieldnames != ['position_m', 'speed_mps']:
            raise ValueError('CSV needs the header position_m,speed_mps.')
        points = []
        for row in reader:
            if None in row or any(v is None for v in row.values()):
                raise ValueError('Each CSV row must have exactly two values.')
            points.append({k: float(v) for k, v in row.items()})
    return points


def profile_limit_issues(points, limits=PROFILE_LIMITS):
    """Explain geometric limit failures for the firmware or stricter UI limits."""
    if len(points)<2 or any(not math.isfinite(p[k]) or p[k]<0 for p in points for k in ('position_m','speed_mps')):
        return []
    if any(b['position_m']<=a['position_m'] for a,b in zip(points,points[1:])):
        return []
    speed_limit=limits['speed']
    issues=[f"Waypoint {i+1} at {p['position_m']:g} m requests {p['speed_mps']:g} m/s; maximum speed is {speed_limit:g} m/s."
            for i,p in enumerate(points) if p['speed_mps']>speed_limit]
    if issues: return issues
    for a,b in zip(points,points[1:]):
        distance=b['position_m']-a['position_m']
        acceleration=(b['speed_mps']**2-a['speed_mps']**2)/(2*distance)
        kind='acceleration' if acceleration>=0 else 'deceleration'
        limit=limits[kind]
        if abs(acceleration)>limit+1e-5:
            needed=abs(b['speed_mps']**2-a['speed_mps']**2)/(2*limit)
            issues.append(f"{a['position_m']:g}–{b['position_m']:g} m: {a['speed_mps']:g} → {b['speed_mps']:g} m/s needs {abs(acceleration):.4g} m/s² {kind} (limit {limit:g}). Allow at least {needed:.4g} m instead of {distance:g} m, or reduce the speed change.")
    return issues


def save_recording(folder, text, manifest=None, context=None, expected_id=0):
    """Validate before creating final output; caller releases RAM only after return."""
    metadata, rows = decode_transfer(text, expected_id)
    if expected_id == 0 and (metadata.get('source') != 'ram' or metadata.get('persistent') is not False):
        raise ValueError('Expected a frozen RAM recording.')
    # Reject corrupt numeric fields even when the transport CRC happens to match.
    if any(not math.isfinite(float(v)) for row in rows for v in row):
        raise ValueError('Recording contains non-finite samples.')
    context = dict(context or {})
    if context.get('sequence') is not None:
        seq_index = columns_for_metadata(metadata).index('seq')
        if not any(int(row[seq_index]) == context['sequence'] for row in rows):
            context = dict(association_warning='Recording does not contain the attempted run sequence; profile omitted.', attempted_run_context=context)
            manifest = None
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=False)
    outputs = {'recording.metadata.json': json.dumps(metadata, indent=2) + '\n',
               'transfer.log': text, 'context.json': json.dumps(context or {}, indent=2) + '\n'}
    if manifest is not None:
        outputs['profile.json'] = json.dumps(manifest, indent=2) + '\n'
    buf = io.StringIO(newline='')
    writer = csv.writer(buf)
    writer.writerow(columns_for_metadata(metadata)); writer.writerows(rows)
    outputs['recording.csv'] = buf.getvalue()
    for name, content in outputs.items():
        with (folder / name).open('x', encoding='utf-8', newline='') as f:
            f.write(content); f.flush(); os.fsync(f.fileno())
    # Windows cannot open a directory through os.open. Files are synced above on both platforms.
    if os.name == 'posix':
        fd = os.open(folder, os.O_RDONLY)
        try: os.fsync(fd)
        finally: os.close(fd)
    return metadata


class Session:
    def __init__(self, handle, stop_event, emit=lambda message: None, diagnostic=lambda message: None):
        self.handle, self.stop_event, self.emit = handle, stop_event, emit
        self.diagnostic = diagnostic
        self.text = ''; self.partial = ''; self.records = {}; self.summary = {}
        self.manifest = None; self.home_epoch = None; self.run_context = None
        self.connected = False; self.motion_pending = False; self.interruptible = True
        self.raw_file = None
        self.recording_generation = 0
        self.recording_seen = False
        self.download_receipt = None

    def pump(self, seconds=.02):
        until = time.monotonic() + seconds
        while True:
            if self.interruptible and self.stop_event.is_set(): raise Cancelled('Operation interrupted by STOP.')
            data = self.handle.read(max(1, self.handle.in_waiting))
            if data:
                chunk = data.decode(errors='replace'); self.text += chunk; self.partial += chunk
                if self.raw_file: self.raw_file.write(chunk); self.raw_file.flush()
                while '\n' in self.partial:
                    line, self.partial = self.partial.split('\n', 1); line = line.strip()
                    if line.startswith(('Queued command', 'Cart acknowledged', 'LOG_BEGIN ', 'LOG_END ', 'LOG_RELEASED ', '>>', 'No command ACK', 'Enter 0 first')):
                        self.diagnostic(line)
                    parsed = parse_line(line)
                    if parsed:
                        kind, value = parsed
                        if kind == 'logger':
                            previous = self.records.get('logger', (0, {}))[1]
                            if (value['state'] in (1, 2, 3) and previous.get('state') not in (1, 2, 3)) or (value['state'] == 5 and previous.get('state') == 5 and value['saved_id'] != previous.get('saved_id')):
                                self.recording_generation += 1
                                self.download_receipt = None
                            if value['state'] != 0 or value['samples'] > 0: self.recording_seen = True
                        self.records[kind] = (time.monotonic(), value)
                    if 'CONNECTION LOST' in line or 'ESP-ROM:' in line or 'Dongle onboard-' in line:
                        self.manifest = None; self.home_epoch = None
                        self.records.pop('return_capability', None)
                    if line.startswith(('PI_REPLY', 'PROFILE_REPLY', 'LOG_ERROR', 'Invalid command')):
                        self.emit(line)
            if time.monotonic() >= until: break
            time.sleep(.005)

    def latest(self, kind, max_age=FRESH_SECONDS):
        record = self.records.get(kind)
        if record is None or time.monotonic() - record[0] > max_age:
            raise RuntimeError('No recent cart ' + kind + ' status. Check the wireless connection.')
        return record[1]

    def wait(self, predicate, timeout=5):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            try:
                if predicate(): return
            except RuntimeError: pass
            self.pump()
        raise TimeoutError('Cart did not report the expected state in time.')

    def write(self, command):
        if self.interruptible and self.stop_event.is_set(): raise Cancelled()
        self.diagnostic('> '+command)
        data = (command + '\n').encode()
        if self.handle.write(data) != len(data): raise OSError('Incomplete serial write; command outcome is unknown.')
        # Short serial writes only; no unbounded flush() which could block STOP.

    def command(self, value):
        mark = len(self.text); self.write(value)
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            self.pump()
            chunk = self.text[mark:]
            if any(error in chunk for error in ('Invalid command.', 'Invalid/overlong input', 'PI_ERROR', 'PROFILE_ERROR', 'Enter 0 first')):
                raise RuntimeError('Dongle rejected command: ' + chunk[-400:])
            queued = re.search(r'Queued command #(\d+):', chunk)
            if queued and f'Cart acknowledged command #{queued[1]};' in chunk:
                return int(queued[1])
        raise TimeoutError('No matching cart acknowledgement. Motion commands are never retried.')

    def stationary(self, logger_available=False):
        live = self.latest('live'); logger = self.latest('logger')
        if self.motion_pending: raise RuntimeError('A motion command is still awaiting completion. Use STOP if needed.')
        if live['running'] or live['speed'] != 0 or live['target'] != 0:
            raise RuntimeError('Wait for the cart to stop completely.')
        if logger_available and logger['state'] not in (0, 5):
            raise RuntimeError('Download the previous recording first, or wait for recording to finish.')
        return live

    def neutral(self):
        self.command('0')
        self.wait(lambda: not self.latest('live')['running'] and self.latest('live')['target'] == 0)

    def stop(self):
        self.interruptible = False
        try:
            self.write('log cancel')
            self.command('0')
            self.emit('STOP acknowledged: neutral commanded; the cart may still be coasting.')
            self.motion_pending = False
            self.pump(.3)
            # Cancel preparation interrupted after logging was armed, without
            # truncating a recording already collecting its coast-down samples.
            if self.latest('logger')['state'] == 1:
                self.command('log off')
        finally:
            self.interruptible = True
            self.stop_event.clear()

    def position_command(self, command):
        seq = self.command(command)
        self.wait(lambda: self.latest('position')['request'] == seq)
        reply = self.latest('position')
        expected = 0 if command == 'position status' else 1
        if reply['result'] != expected: raise RuntimeError('Position request was refused by the cart.')
        return reply

    def set_home(self):
        self.stationary(True); self.neutral()
        reply = self.position_command(f'home {HOME}')
        if not reply['flags'] & 2 or abs(reply['x'] - HOME) > .001:
            raise RuntimeError('Home position was not verified.')
        self.home_epoch = reply['epoch']; self.neutral()
        self.emit('Home set at the first magnet. Profile distances start here.')

    def upload_profile(self, manifest):
        self.stationary(True); self.neutral(); self.pump(.3)
        self.manifest = None
        upload_profile(self, manifest)
        self.manifest = manifest
        self.emit('Profile uploaded and checksum verified.')

    def read_pi(self):
        self.stationary(True); mark = len(self.text); seq = self.command('pi status')
        reply = parse_pi(self.text[mark:], seq)
        if not reply or reply['result'] != 'STATUS': raise RuntimeError('PI gain readback failed.')
        return reply

    def set_pi(self, kp, ki):
        if not (math.isfinite(kp) and math.isfinite(ki) and 0 <= kp <= 50 and 0 <= ki <= 100 and kp + ki > 0):
            raise ValueError('Use Kp 0–50 and Ki 0–100; not both zero.')
        self.stationary(True)
        mark = len(self.text); seq = self.command(f'pi set {kp:g} {ki:g}')
        reply = parse_pi(self.text[mark:], seq)
        if not reply or reply['result'] != 'APPLIED' or abs(reply['kp']-kp)>.001 or abs(reply['ki']-ki)>.001:
            raise RuntimeError('Gain update was not verified.')
        self.neutral(); return self.read_pi()

    def start(self, manifest, logging_rate=10):
        self.stationary(True)
        if self.manifest != manifest: raise RuntimeError('Upload the current profile before starting.')
        reply = query_profile(self, 'profile status')
        if not profile_matches(reply, manifest):
            self.manifest = None
            raise RuntimeError('Cart profile changed. Upload it again.')
        x = self.position_command('position status')
        if self.home_epoch != x['epoch'] or not x['flags'] & 2 or abs(x['x'] - HOME) > .04:
            self.home_epoch = None
            raise RuntimeError('Align over the first magnet and click Set home before this run.')
        if x['x'] + manifest['points'][-1]['position_m'] > 26:
            raise ValueError('The endpoint is beyond the firmware map. Maximum from this home is 25.208 m.')
        if logging_rate and math.ceil((manifest['planned_duration_s'] + 12) * logging_rate) + 32 > 896:
            raise ValueError('This profile may fill the recording buffer at that rate. Select 10 Hz logging.')
        self._begin(f"profile run {manifest['crc32']} {manifest['launch_and_low_speed_floor_mps']:.9g} {manifest['launch_speed_mps']:.9g}", logging_rate, manifest)
        self.home_epoch = None  # another run requires explicit physical realignment/home

    def return_speed_limit(self):
        try: return self.latest('return_capability')['max_mps']
        except RuntimeError: return RETURN_LEGACY_MAX_MPS

    def return_plan(self, position=None, speed=RETURN_SPEED):
        """Pure eligibility check against fresh state; never issues motion."""
        self.stationary(True)
        logger = self.latest('logger')
        identity = (self.recording_generation, logger['saved_id'], logger['samples'])
        if (logger['state'] == 5 or (logger['state'] == 0 and logger['saved_id'] and logger['samples'])) and self.download_receipt != identity:
            raise RuntimeError('Download the current recording before returning home.')
        if logger['state'] == 0 and self.recording_seen and (self.download_receipt is None or self.download_receipt[0] != self.recording_generation):
            raise RuntimeError('The previous recording has not been verified locally. Download it before returning home.')
        x = position if position is not None else self.latest('position')
        if not x['flags'] & 2:
            raise RuntimeError('Position is invalid. Establish a known physical reference before returning.')
        if x['x'] <= HOME + .22:
            raise ValueError('Already near or behind home. Align manually; no reverse command sent.')
        plan = plan_return(x['x'], HOME, speed, return_neutral_margin(speed))
        if speed > self.return_speed_limit():
            raise ValueError('Return above 1 m/s requires the updated cart and dongle firmware.')
        return plan

    def return_home(self, speed=RETURN_SPEED):
        self.return_plan(speed=speed)
        x = self.position_command('position status')
        plan = self.return_plan(x, speed)
        self.home_epoch = None
        context = dict(operation='return_home', starting_position_map_m=x['x'],
                       goal_map_m=HOME, neutral_margin_m=return_neutral_margin(speed), **plan)
        # Recheck position immediately before transmission, after logger setup.
        def check_position():
            current = self.position_command('position status')
            if not current['flags'] & 2 or current['epoch'] != x['epoch'] or abs(current['x'] - x['x']) > .001:
                raise RuntimeError('Position changed during return preparation. No reverse command sent.')
            if speed > self.return_speed_limit():
                raise RuntimeError('Fast reverse capability was lost during preparation. No reverse command sent.')
        self._begin(f"move {plan['distance_m']:.6f} {plan['target_mps']:g}", 10, context=context, before_motion=check_position)
        self.emit(f'Returning toward home at up to {speed:g} m/s. STOP remains available.')

    def _begin(self, command, rate, manifest=None, context=None, before_motion=None):
        if rate not in (0, 10, 50): raise ValueError('Logging rate must be off, 10 or 50 Hz.')
        if self.latest('odom')['flags'] & 4:
            raise RuntimeError('Cart reports a control fault. Inspect diagnostics before starting.')
        gains = self.read_pi()
        self.neutral(); self.pump(.3)
        if rate:
            self.recording_generation += 1
            self.download_receipt = None
            self.command(f'log arm {rate}')
            self.wait(lambda: self.latest('logger')['state'] == 1 and self.latest('logger')['error'] == 0)
        else:
            self.command('log off')
        self.neutral(); self.pump(.3)
        self.stationary()
        if before_motion is not None: before_motion()
        self.run_context = dict(command=command, profile=manifest, gains=gains, logging_hz=rate,
                                started_utc=datetime.now(timezone.utc).isoformat(), home_map_m=HOME,
                                simulated=bool(getattr(self.handle, 'simulated', False)))
        self.run_context.update(context or {})
        self.motion_pending = True
        self.run_context['sent_monotonic'] = time.monotonic()
        try:
            seq = self.command(command); self.run_context['sequence'] = seq
        except Exception:
            self.home_epoch = None
            # An ACK can be lost after execution; request neutral instead of retrying motion.
            self.stop_event.set()
            raise
        self.emit('Run command acknowledged. The cart executes the profile locally.' if manifest else 'Test command acknowledged.')

    def timed_test(self, mode, value, seconds, rate=50):
        maximum = 100 if mode == 'throttle' else 10
        duration = 2 if mode == 'throttle' else 3
        if mode not in ('throttle', 'speed') or not all(math.isfinite(x) for x in (value, seconds)) or not 0 < abs(value) <= maximum or not .05 <= seconds <= duration:
            raise ValueError('Test exceeds the existing firmware limits.')
        self.stationary(True); self.home_epoch = None
        self._begin(f'{mode} {value:g} {seconds:g}', rate)

    def transfer(self, command, end_pattern, timeout=120):
        mark = len(self.text); self.write(command); deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            chunk = self.text[mark:]
            if 'LOG_ERROR ' in chunk or re.search(r'LOG_REPLY .*error=[1-9]\d*', chunk):
                raise RuntimeError('Recording transfer failed: ' + chunk[-500:])
            if re.search(end_pattern, chunk): return chunk
        raise TimeoutError('Recording transfer timed out. Onboard data has not been released.')

    def download(self, folder):
        self.stationary()
        logger = self.latest('logger')
        if logger['state'] not in (4, 5) and not (logger['state'] == 0 and logger['saved_id'] and logger['samples']):
            raise RuntimeError('No completed recording available yet.')
        identity = (self.recording_generation, logger['saved_id'], logger['samples'])
        record_id = 0 if logger['state'] == 4 else logger['saved_id']
        if logger['state'] == 5 and not record_id: raise RuntimeError('No saved recording ID.')
        self.neutral(); self.pump(2.5)
        self.emit('Downloading recording over ESP-NOW…')
        text = ''
        try:
            text = self.transfer('log ram' if record_id == 0 else f'log dump {record_id}', r'LOG_END \d+ \d+ \d+\r?\n')
            context = dict(self.run_context or {}, serial_session_log=getattr(self.raw_file, 'name', None))
            metadata = save_recording(folder, text, (self.run_context or {}).get('profile'), context, record_id)
            if self.raw_file:
                with Path(folder, 'commands.log').open('x', encoding='utf-8', newline='') as f:
                    with Path(self.raw_file.name).open(encoding='utf-8', newline='') as raw:
                        f.write(raw.read())
                    f.flush(); os.fsync(f.fileno())
        finally:
            self.write('log cancel')
        # A receipt belongs to this recording generation and identity only.
        self.download_receipt = identity
        if record_id == 0:
            crc = metadata['binary_crc32']
            self.pump(.2)
            try:
                result = self.transfer(f'log release {crc}', rf'LOG_RELEASED {crc}\r?\n', 10)
                metadata['ram_released'] = f'LOG_RELEASED {crc}' in result
                Path(folder, 'recording.metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
            except Exception as e:
                self.emit(f'Files saved and verified at {folder}; RAM release not confirmed: {e}')
                return metadata
            finally: self.write('log cancel')
        self.neutral()
        self.emit(f"Recording verified and saved: {folder}. Stop code: {metadata.get('stop_reason', 'unknown')}; recording flags: {metadata.get('flags', 'unknown')}.")
        return metadata
