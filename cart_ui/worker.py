"""One serial owner; all long operations stay off the Qt GUI thread."""
import json
import os
from pathlib import Path
import queue
import threading
import time
from datetime import datetime, timezone
from PySide6.QtCore import QThread, Signal
from .backend import Session, Cancelled, RETURN_SPEED
from .simulator import SimulatedSerial

class Worker(QThread):
    status = Signal(str)
    diagnostic = Signal(str)
    snapshot = Signal(dict)
    outcome = Signal(str, object)
    def __init__(self, output_root):
        super().__init__(); self.output_root=Path(output_root)
        self.jobs=queue.Queue(); self.stop_event=threading.Event(); self.quit_event=threading.Event()
        self.session=None; self.busy=False; self.reserved=False; self.lock=threading.Lock()
        self.seen_running=False; self.last_link=None; self.port=''; self.monitor_motion=False
        self.auto_download_sequence=None; self.waiting_for_recording=False

    def submit(self, action, payload=None):
        with self.lock:
            if self.reserved or self.busy: return False
            self.reserved=True
        self.jobs.put((action,payload)); return True

    def stop_now(self):
        self.auto_download_sequence=None
        self.stop_event.set()
        while True:
            try: self.jobs.get_nowait()
            except queue.Empty: break
        with self.lock: self.reserved=False

    def disconnect(self):
        if self.session:
            self.session.handle.close()
            if self.session.raw_file: self.session.raw_file.close()
        self.session=None; self.monitor_motion=False; self.seen_running=False
        self.auto_download_sequence=None; self.waiting_for_recording=False

    def connect_port(self, port):
        self.disconnect(); self.port=port
        if port=='Demo — no hardware': handle=SimulatedSerial()
        else:
            import serial
            handle=serial.Serial(port=None,baudrate=115200,timeout=.02,write_timeout=.1,
                                 exclusive=True if os.name=='posix' else None)
            handle.port=port; handle.dtr=True; handle.rts=False; handle.open()
        self.session=Session(handle,self.stop_event,self.status.emit,self.diagnostic.emit)
        dest=self.output_root/('demo' if getattr(handle,'simulated',False) else 'hardware')/'sessions'
        dest.mkdir(parents=True,exist_ok=True)
        stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        self.session.raw_file=(dest/f'{stamp}.log').open('x',encoding='utf-8',newline='')
        self.status.emit('Dongle port opened; waiting for the cart…')
        try:
            self.session.wait(lambda: bool(self.session.latest('live')),8)
            self.session.neutral()
            self.session.connected=True
            self.status.emit('Demo connected — simulated data only.' if getattr(handle,'simulated',False) else 'Dongle and cart connected. Neutral acknowledged.')
        except Exception:
            self.disconnect(); raise

    def operate(self, action, payload):
        if action=='connect': self.connect_port(payload); return
        if action=='disconnect':
            warning=None
            try:
                if self.session: self.session.stop()
            except Exception as e: warning=str(e)
            finally: self.disconnect()
            self.status.emit('Disconnected. STOP was not confirmed: '+warning if warning else 'Disconnected; neutral was acknowledged.')
            return
        s=self.session
        if not s: raise RuntimeError('Connect to a dongle first.')
        if action=='home': s.set_home()
        elif action=='upload': s.upload_profile(payload)
        elif action=='start':
            s.start(payload['manifest'],payload['rate']); self.watch_run(s)
        elif action=='return_home':
            s.return_home(RETURN_SPEED if payload is None else payload); self.watch_run(s)
        elif action=='pi_read': self.outcome.emit('pi',s.read_pi())
        elif action=='pi_set': self.outcome.emit('pi',s.set_pi(*payload))
        elif action=='test':
            s.timed_test(*payload); self.watch_run(s)
        elif action=='log_off':
            s.stationary()
            if s.latest('logger')['state'] not in (0,1,5): raise RuntimeError('Download the completed recording first, or wait for coast recording to finish.')
            s.command('log off'); self.status.emit('Logging disabled. Any retained recording can still be downloaded.')
        elif action=='download':
            name=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
            folder=Path(payload)/('demo' if getattr(s.handle,'simulated',False) else 'hardware')/name
            result=s.download(folder); self.outcome.emit('download',dict(folder=str(folder),metadata=result)); return result
        else: raise ValueError('Unknown operation')

    def publish(self):
        s=self.session; state=dict(port_open=bool(s),cart_online=False,busy=self.busy or self.reserved,uploaded=None,home=False,stationary=False,logger_state=-1,logger_error=0,simulated=False,can_return=False,return_hint='Connect to the cart first.')
        if s:
            state['simulated']=bool(getattr(s.handle,'simulated',False))
            state['return_speed_max']=s.return_speed_limit()
            try:
                live=s.latest('live'); logger=s.latest('logger'); x=s.latest('position')
                state.update(cart_online=True,stationary=not live['running'] and live['speed']==0 and live['target']==0,
                             logger_state=logger['state'],logger_error=logger['error'],downloadable=logger['state'] in (4,5) or (logger['state']==0 and bool(logger['saved_id']) and bool(logger['samples'])),uploaded=s.manifest,
                             home=s.home_epoch is not None and x['epoch']==s.home_epoch and bool(x['flags']&2) and abs(x['x']-.792)<=.04)
                if self.monitor_motion:
                    if live['running']: self.seen_running=True
                    elif self.seen_running and state['stationary']:
                        self.monitor_motion=False; s.motion_pending=False
                        if (s.run_context or {}).get('operation') == 'return_home':
                            self.status.emit('Return finished—check alignment before setting home. Recording will be saved automatically.' if live['stop']=='DISTANCE CUTOFF' and x['flags']&2 else 'Return stopped: '+live['stop']+'. Check position; no automatic retry.')
                        else:
                            self.status.emit('Run stopped: '+live['stop']+'. '+('Recording ready; automatic download is queued.' if logger['state'] in (4,5) and self.auto_download_sequence is not None else 'Recording ready. Click Download recording to save it.' if logger['state'] in (4,5) else 'Waiting for recording to finish (no transfer yet).' if (s.run_context or {}).get('logging_hz') else 'Logging was disabled.'))
                if self.monitor_motion and not self.seen_running and time.monotonic()-(s.run_context or {}).get('sent_monotonic',time.monotonic()) > 2:
                    self.monitor_motion=False; self.stop_event.set()
                    self.status.emit('Run did not report movement: '+live['stop']+'. Requesting neutral; no automatic retry.')
                if s.home_epoch is not None and x['epoch']!=s.home_epoch: s.home_epoch=None
            except RuntimeError: pass
            # Download suppresses telemetry; only interpret stale data when idle.
            if not self.busy and not state['cart_online'] and self.last_link is True:
                s.manifest=None; s.home_epoch=None
                self.status.emit('Cart status is stale. Stop is not confirmed; reconnect/check the link.')
                if self.monitor_motion: self.stop_event.set(); self.monitor_motion=False
            if not self.busy: self.last_link=state['cart_online']
        if s and s.motion_pending: state['stationary']=False
        if s and state['cart_online'] and not state['busy']:
            try:
                s.return_plan(); state['can_return']=True
            except (RuntimeError,ValueError) as e: state['return_hint']=str(e)
        self.snapshot.emit(state)

    def watch_run(self, session):
        self.monitor_motion=True; self.seen_running=False
        context=session.run_context or {}
        self.auto_download_sequence=context.get('sequence') if context.get('logging_hz') else None

    def maybe_auto_download(self):
        """One transfer attempt per run, after motion and recording have finished."""
        s=self.session
        if not s or self.auto_download_sequence is None or self.monitor_motion or self.stop_event.is_set(): return
        if self.busy or self.reserved or not self.jobs.empty(): return
        if (s.run_context or {}).get('sequence')!=self.auto_download_sequence:
            self.auto_download_sequence=None; return
        try:
            s.stationary()
            logger=s.latest('logger')
        except RuntimeError: return
        if logger['state'] not in (4,5): return
        with self.lock:
            if self.reserved or self.stop_event.is_set(): return
            self.busy=True
        # Clear before attempting: failures keep the onboard copy and require an
        # explicit retry, rather than repeatedly hammering storage/radio.
        self.auto_download_sequence=None
        self.publish()
        try:
            result=self.operate('download',self.output_root)
            if result.get('source')=='ram' and not result.get('ram_released'):
                self.status.emit('Recording saved, but onboard buffer release is not confirmed. Click Download recording to retry before another run.')
            elif (s.run_context or {}).get('operation')=='return_home':
                self.status.emit('Return recording saved automatically. Check alignment, then Set home for the next run.')
        except Cancelled:
            self.status.emit('Automatic download cancelled. Recording retained; click Download recording to retry.')
        except Exception as e:
            self.status.emit(f'Automatic download failed: {e}. Recording retained; click Download recording to retry.')
            self.outcome.emit('error',str(e))
        finally:
            self.busy=False; self.publish()

    def run(self):
        while not self.quit_event.is_set():
            try:
                if self.stop_event.is_set():
                    self.busy=True
                    try:
                        if self.session: self.session.stop()
                        else: self.status.emit('No connection: STOP could not be sent.')
                    finally:
                        self.stop_event.clear(); self.busy=False
                try: action,payload=self.jobs.get_nowait()
                except queue.Empty:
                    if self.session: self.session.pump(.02)
                    else: time.sleep(.04)
                else:
                    self.busy=True; self.publish()
                    try:
                        self.operate(action,payload)
                        self.outcome.emit('complete',action)
                    except Exception:
                        if action in ('start','test','return_home'): self.stop_event.set()
                        raise
                    finally:
                        with self.lock: self.busy=False; self.reserved=False
                self.publish()
                self.maybe_auto_download()
            except Cancelled:
                self.status.emit('Operation cancelled; sending STOP…')
            except Exception as e:
                self.status.emit(str(e)); self.outcome.emit('error',str(e))
                # Physical removal/serial failure: stop cannot be acknowledged.
                if self.session:
                    try:
                        import serial
                        if isinstance(e,(serial.SerialException,OSError)):
                            self.disconnect()
                    except ImportError: pass
                self.publish(); time.sleep(.05)
        self.disconnect()
