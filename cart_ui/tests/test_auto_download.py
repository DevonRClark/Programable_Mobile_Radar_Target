import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import threading
import pytest
from cart_ui.worker import Worker
from cart_ui.backend import Session
from cart_ui.simulator import SimulatedSerial


def ready_worker(tmp_path):
    worker=Worker(tmp_path)
    s=Session(SimulatedSerial(),threading.Event())
    s.handle.log_state=4; s.handle.epoch=1; s.pump()
    s.run_context=dict(sequence=1,logging_hz=10,operation='return_home')
    worker.session=s; worker.auto_download_sequence=1
    return worker,s


def test_failure_not_retried_automatically(tmp_path,monkeypatch):
    w,s=ready_worker(tmp_path)
    attempts=[]
    def fail(folder): attempts.append(folder); raise OSError('disk full')
    monkeypatch.setattr(s,'download',fail)
    w.maybe_auto_download(); w.maybe_auto_download()
    assert len(attempts)==1
    assert s.handle.log_state==4 and w.session is s
    assert not w.busy and w.auto_download_sequence is None
    assert not any(c.startswith('log release') for c in s.handle.commands)


def test_no_transfer_while_moving_or_recording(tmp_path,monkeypatch):
    w,s=ready_worker(tmp_path); attempts=[]
    monkeypatch.setattr(s,'download',lambda folder:attempts.append(folder))
    w.monitor_motion=True; w.maybe_auto_download(); assert not attempts
    w.monitor_motion=False; s.handle.log_state=3; s.pump(.1)
    w.maybe_auto_download(); assert not attempts
    assert w.auto_download_sequence==1


def test_stop_cancels_queued_auto_transfer(tmp_path,monkeypatch):
    w,s=ready_worker(tmp_path); attempts=[]
    monkeypatch.setattr(s,'download',lambda folder:attempts.append(folder))
    w.stop_now(); w.maybe_auto_download()
    assert not attempts and w.auto_download_sequence is None


def test_unlogged_run_has_no_auto_transfer(tmp_path):
    w,s=ready_worker(tmp_path)
    s.run_context['logging_hz']=0
    w.watch_run(s)
    assert w.auto_download_sequence is None


def test_stale_run_context_cannot_download_newer_buffer(tmp_path,monkeypatch):
    w,s=ready_worker(tmp_path); attempts=[]
    monkeypatch.setattr(s,'download',lambda folder:attempts.append(folder))
    s.run_context['sequence']=2
    w.maybe_auto_download()
    assert not attempts and w.auto_download_sequence is None
