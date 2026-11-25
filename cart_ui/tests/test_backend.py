import csv
import json
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import pytest
from cart_ui.backend import Session, Cancelled, make_profile, load_csv, HOME
from cart_ui.simulator import SimulatedSerial

POINTS=[dict(position_m=x,speed_mps=v) for x,v in [(0,0),(2,2),(6,2),(8,0)]]

@pytest.fixture
def session():
    s=Session(SimulatedSerial(),threading.Event()); s.pump(); return s

def test_full_workflow(session,tmp_path):
    s=session; m=make_profile(POINTS)
    s.upload_profile(m); assert s.manifest['crc32']==s.handle.crc
    s.set_home(); assert s.latest('position')['x']==HOME
    s.start(m)
    assert len([x for x in s.handle.commands if x.startswith('profile run ')])==1
    s.wait(lambda:s.latest('logger')['state']==4)
    s.motion_pending=False
    meta=s.download(tmp_path/'run')
    assert meta['ram_released'] is True
    assert (tmp_path/'run/recording.csv').exists()
    assert json.loads((tmp_path/'run/profile.json').read_text())==m
    assert json.loads((tmp_path/'run/context.json').read_text())['simulated']
    assert s.handle.log_state==0

def test_home_required_and_edits_invalidate(session):
    s=session; m=make_profile(POINTS); s.upload_profile(m)
    with pytest.raises(RuntimeError,match='Align'): s.start(m)
    s.set_home()
    other=make_profile([dict(p,speed_mps=1 if p['speed_mps'] else 0) for p in POINTS])
    with pytest.raises(RuntimeError,match='Upload'): s.start(other)
    assert not any(x.startswith('profile run') for x in s.handle.commands)

def test_crc_failure_never_releases(session,tmp_path):
    session.handle.log_state=4; session.handle.corrupt_download=True; session.pump(.1)
    with pytest.raises(ValueError,match='checksum'): session.download(tmp_path/'bad')
    assert not (tmp_path/'bad').exists()
    assert not any(c.startswith('log release') for c in session.handle.commands)
    assert session.handle.log_state==4

def test_stop_interrupts_upload_and_does_not_retry_motion(session):
    session.stop_event.set()
    with pytest.raises(Cancelled): session.upload_profile(make_profile(POINTS))
    session.stop()
    assert session.handle.commands==['log cancel','0']
    assert not session.stop_event.is_set()

def test_no_ack_motion_sent_once(session):
    session.handle.suppress_ack=True
    with pytest.raises(TimeoutError): session.command('speed 1 2')
    assert session.handle.commands.count('speed 1 2')==1

def test_stale_status_blocks_commands(session):
    session.records={k:(time.monotonic()-10,v) for k,(_,v) in session.records.items()}
    with pytest.raises(RuntimeError,match='recent'): session.set_home()
    assert not session.handle.commands

def test_pi_applied_and_read_back(session):
    result=session.set_pi(10,20)
    assert result['kp']==10 and result['ki']==20
    assert session.handle.commands==['pi set 10 20','0','pi status']

def test_download_disk_failure_retains_ram(session,tmp_path,monkeypatch):
    import cart_ui.backend as backend
    session.handle.log_state=4; session.pump(.1)
    def fail(*a,**k): raise OSError('disk full')
    monkeypatch.setattr(backend,'save_recording',fail)
    with pytest.raises(OSError): session.download(tmp_path/'bad')
    assert not any(c.startswith('log release') for c in session.handle.commands)

def test_windows_download_flushes_files_and_preserves_csv(session,tmp_path,monkeypatch):
    import cart_ui.backend as backend
    def no_directory_open(*args): pytest.fail('Windows cannot open a directory with os.open.')
    monkeypatch.setattr(backend,'os',SimpleNamespace(name='nt',fsync=os.fsync,open=no_directory_open))
    session.handle.log_state=4; session.pump(.1)
    with (tmp_path/'session.log').open('x',encoding='utf-8',newline='') as raw:
        session.raw_file=raw
        raw.write('Café\n'); raw.flush()
        assert session.download(tmp_path/'Cart Runs'/'run')['ram_released'] is True
    folder=tmp_path/'Cart Runs'/'run'
    csv_bytes=(folder/'recording.csv').read_bytes()
    assert csv_bytes.count(b'\r\n')==4 and b'\r\r\n' not in csv_bytes
    assert (folder/'commands.log').read_text(encoding='utf-8').startswith('Café\n')

def test_profile_readback_verifies_low_speed_settings(session):
    manifest=make_profile(POINTS,floor=.5,launch=.7)
    session.upload_profile(manifest)
    assert session.handle.floor==pytest.approx(.5)
    assert session.handle.launch==pytest.approx(.7)
    session.set_home()
    session.handle.floor=.6
    with pytest.raises(RuntimeError,match='changed'):
        session.start(manifest)
    assert not any(command.startswith('profile run ') for command in session.handle.commands)

def test_invalid_profiles():
    for points in ([dict(position_m=0,speed_mps=0),dict(position_m=1,speed_mps=11),dict(position_m=2,speed_mps=0)],
                   [dict(position_m=0,speed_mps=0),dict(position_m=0,speed_mps=1)],
                   [dict(position_m=0,speed_mps=0),dict(position_m=float('nan'),speed_mps=1)]):
        with pytest.raises(ValueError): make_profile(points)

def test_csv_contract(tmp_path):
    p=tmp_path/'p.csv'; p.write_text('position_m,speed_mps\n0,0\n2,2\n8,0\n')
    assert len(load_csv(p))==3
    p.write_text('distance,speed\n0,0\n')
    with pytest.raises(ValueError,match='header'): load_csv(p)

def test_frozen_recording_blocks_overwrite(session):
    session.handle.log_state=4; session.pump(.1)
    with pytest.raises(RuntimeError,match='Download'): session.upload_profile(make_profile(POINTS))
    with pytest.raises(RuntimeError,match='Download'): session.set_pi(10,20)
    assert session.handle.commands==[]

def test_duplicate_test_rejected_while_pending(session):
    session.timed_test('throttle',10,.5)
    with pytest.raises(RuntimeError): session.timed_test('throttle',10,.5)
    assert session.handle.commands.count('throttle 10 0.5')==1

def test_interrupted_transfer_keeps_buffer(session,tmp_path):
    session.handle.log_state=4; session.pump(.1)
    original=session.handle.write
    def write(data):
        result=original(data)
        if data==b'log ram\n': session.stop_event.set()
        return result
    session.handle.write=write
    with pytest.raises(Cancelled): session.download(tmp_path/'interrupted')
    assert not (tmp_path/'interrupted').exists()
    assert not any(c.startswith('log release') for c in session.handle.commands)
    session.stop()
    assert session.handle.log_state==4

def test_fragmented_serial_and_unrelated_ack(session):
    original=session.handle.read
    session.handle.read=lambda n:original(min(n,31))
    session.handle.line('Cart acknowledged command #999; stale acknowledgement')
    assert session.command('0')==1

def test_reboot_profile_mismatch_blocks_start(session):
    m=make_profile(POINTS); session.upload_profile(m); session.set_home()
    session.handle.crc=0
    with pytest.raises(RuntimeError,match='changed'): session.start(m)
    assert session.manifest is None
    assert not any(c.startswith('profile run') for c in session.handle.commands)

def test_logging_budget_requires_lower_rate(session):
    m=make_profile(POINTS); session.upload_profile(m); session.set_home()
    with pytest.raises(ValueError,match='buffer'): session.start(m,50)
    assert not any(c.startswith('profile run') for c in session.handle.commands)

def test_stop_cancels_armed_preparation(session):
    session.command('log arm 10'); session.pump(.1)
    session.stop()
    assert 'log off' in session.handle.commands
    session.pump(.1)
    assert session.latest('logger')['state'] != 1
