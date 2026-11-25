import csv
import json
import threading
import time
import pytest
from cart_ui.backend import Session, HOME, make_profile, Cancelled
from cart_ui.simulator import SimulatedSerial
from cart_ui.tests.test_backend import POINTS

@pytest.fixture
def session():
    s=Session(SimulatedSerial(),threading.Event()); s.handle.epoch=1; s.handle.x=8.792; s.pump(); return s

def finish(s):
    s.wait(lambda:not s.latest('live')['running'] and s.latest('live')['speed']==0)
    s.motion_pending=False

def test_reverse_position_context_and_recording(session,tmp_path):
    s=session; s.return_home()
    assert s.handle.commands.count('move 7.880000 -0.75')==1
    assert s.home_epoch is None and s.run_context['profile'] is None
    assert s.run_context['operation']=='return_home' and s.run_context['neutral_margin_m']==.12
    assert 'log arm 10' in s.handle.commands
    assert not any(c.startswith('pi set') for c in s.handle.commands)
    finish(s)
    assert s.latest('position')['x']==pytest.approx(.912)
    assert s.latest('live')['stop']=='DISTANCE CUTOFF'
    s.download(tmp_path/'return')
    with (tmp_path/'return/recording.csv').open() as f: rows=list(csv.DictReader(f))
    assert float(rows[0]['position_m'])>float(rows[-1]['position_m'])
    assert not (tmp_path/'return/profile.json').exists()
    assert json.loads((tmp_path/'return/context.json').read_text())['operation']=='return_home'
    with pytest.raises(ValueError,match='near'): s.return_home()

def test_fast_return_sends_selected_speed_and_records_larger_margin(session,tmp_path):
    session.return_home(2)
    assert session.handle.commands.count('move 7.800000 -2')==1
    finish(session)
    session.download(tmp_path/'fast')
    context=json.loads((tmp_path/'fast/context.json').read_text())
    assert context['target_mps']==-2
    assert context['neutral_margin_m']==.20
    assert context['cutoff_m']==pytest.approx(.992)

def test_fast_return_requires_fresh_paired_firmware_capability(session):
    session.handle.fast_reverse=False
    session.records['return_capability']=(time.monotonic()-5,{'max_mps':2})
    with pytest.raises(ValueError,match='updated cart and dongle'):
        session.return_home(2)
    assert not any(command.startswith(('move ','log arm ')) for command in session.handle.commands)
    assert session.return_plan(speed=1)['target_mps']==-1

@pytest.mark.parametrize('speed',[0.49,2.01,float('nan')])
def test_invalid_return_speed_sends_nothing(session,speed):
    with pytest.raises(ValueError):
        session.return_home(speed)
    assert not session.handle.commands

@pytest.mark.parametrize('position',[.792,.912,0,26.1])
def test_outside_return_bounds(session,position):
    session.handle.x=position; session.pump(.1)
    with pytest.raises(ValueError): session.return_home()
    assert not any(c.startswith('move ') for c in session.handle.commands)

def test_invalid_stale_position(session):
    session.handle.epoch=0; session.pump(.1)
    with pytest.raises(RuntimeError,match='invalid'): session.return_home()
    session.records['position']=(time.monotonic()-10,dict(flags=3,x=8.792))
    with pytest.raises(RuntimeError,match='recent'): session.return_home()
    assert not any(c.startswith('move ') for c in session.handle.commands)

def test_saved_receipt_is_for_current_recording(session,tmp_path):
    s=session; s.handle.log_state=5; s.handle.saved_id=1; s.pump(.1)
    with pytest.raises(RuntimeError,match='Download'): s.return_home()
    s.download(tmp_path/'one'); assert s.return_plan()['distance_m']==pytest.approx(7.88)
    s.handle.saved_id=2; s.pump(.1)
    with pytest.raises(RuntimeError,match='Download'): s.return_home()
    assert not any(c.startswith('move ') for c in s.handle.commands)

def test_bad_download_blocks_return(session,tmp_path):
    s=session; s.handle.log_state=5; s.handle.saved_id=1; s.handle.corrupt_download=True; s.pump(.1)
    with pytest.raises(ValueError): s.download(tmp_path/'bad')
    with pytest.raises(RuntimeError,match='Download'): s.return_home()
    assert s.download_receipt is None

def test_missing_ack_no_second_return(session):
    original=session.handle.write
    def write(data):
        if data.startswith(b'move '): session.handle.suppress_ack=True
        return original(data)
    session.handle.write=write
    with pytest.raises(TimeoutError): session.return_home()
    assert session.stop_event.is_set()
    assert len([c for c in session.handle.commands if c.startswith('move ')])==1

def test_stop_during_return_preparation(session):
    original=session.handle.write
    def write(data):
        result=original(data)
        if data==b'log arm 10\n': session.stop_event.set()
        return result
    session.handle.write=write
    with pytest.raises(Cancelled): session.return_home()
    session.stop()
    assert not any(c.startswith('move ') for c in session.handle.commands)

def test_stop_and_duplicate_return(session):
    session.return_home()
    with pytest.raises(RuntimeError): session.return_home()
    session.stop()
    assert not session.handle.running_until
    assert len([c for c in session.handle.commands if c.startswith('move ')])==1

def test_position_changed_during_preparation(session):
    original=session.handle.write
    def write(data):
        if data==b'log arm 10\n': session.handle.x+=.1
        return original(data)
    session.handle.write=write
    with pytest.raises(RuntimeError,match='changed'): session.return_home()
    assert not any(c.startswith('move ') for c in session.handle.commands)

def test_unlogged_forward_run_can_return(session):
    s=session; s.handle.x=HOME; s.pump(.1)
    m=make_profile(POINTS); s.upload_profile(m); s.set_home(); s.start(m,0); finish(s)
    s.return_home(); assert s.run_context['operation']=='return_home'
