import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import time
import json
from types import SimpleNamespace
import pytest
from PySide6.QtWidgets import QApplication, QFileDialog, QMessageBox, QPushButton
from cart_ui.app import Window

@pytest.fixture(scope='module')
def app(): return QApplication.instance() or QApplication([])

def until(app,predicate,timeout=10):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        app.processEvents()
        if predicate(): return
        time.sleep(.01)
    raise AssertionError('GUI condition timed out')

def test_basic_buttons_demo_end_to_end(app,tmp_path,monkeypatch):
    from PySide6.QtWidgets import QFileDialog
    def no_folder_prompt(*args,**kwargs): raise AssertionError('Normal runs must not ask for a recording folder.')
    monkeypatch.setattr(QFileDialog,'getExistingDirectory',no_folder_prompt)
    w=Window(tmp_path,demo=True); w.show()
    try:
        assert not w.start.isEnabled()
        w.connect_button.click()
        until(app,lambda:w.state.get('cart_online') and not w.state.get('busy'))
        assert w.upload.isEnabled() and not w.start.isEnabled()
        w.upload.click()
        until(app,lambda:w.state.get('uploaded') is not None and not w.state.get('busy'))
        w.home.click()
        until(app,lambda:w.start.isEnabled())
        w.start.click()
        assert not w.start.isEnabled()  # explicit realignment/home required for repeat
        until(app,lambda:bool(list(tmp_path.glob('demo/*/recording.csv'))) and not w.state.get('busy'),12)
        assert w.upload.isEnabled()
        assert list(tmp_path.glob('demo/*/commands.log'))
        assert not list(tmp_path.glob('hardware/*'))
        until(app,lambda:w.return_button.isEnabled())
        w.return_button.click()
        until(app,lambda:w.worker.monitor_motion)
        assert not w.return_button.isEnabled()
        until(app,lambda:len(list(tmp_path.glob('demo/*/recording.csv')))==2 and not w.state.get('busy'),12)
        assert not w.start.isEnabled()
        assert w.worker.session.handle.x==pytest.approx(.912)
        assert w.worker.session.handle.commands.count('move 7.880000 -0.75')==1
        assert w.home.isEnabled() and not w.download.isEnabled()
        assert not w.worker.session.handle.log_state
        contexts=[json.loads(p.read_text()) for p in tmp_path.glob('demo/*/context.json')]
        assert sum(c.get('operation')=='return_home' for c in contexts)==1
        assert len(list(tmp_path.glob('demo/*/profile.json')))==1
        w.worker.session.handle.x=.792  # simulate the user's final physical alignment
        w.home.click(); until(app,lambda:w.start.isEnabled())
        w.table.item(1,1).setText('1.5')
        assert not w.start.isEnabled()
    finally:
        w.close(); until(app,lambda:not w.worker.isRunning(),8)

def test_stop_preempts_active_operation(app,tmp_path):
    w=Window(tmp_path,demo=True); w.show()
    try:
        w.connect_button.click(); until(app,lambda:w.state.get('cart_online') and not w.state.get('busy'))
        handle=w.worker.session.handle
        handle.suppress_ack=True
        w.submit('pi_read')
        until(app,lambda:'pi status' in handle.commands)
        begin=time.monotonic(); w.stop()
        until(app,lambda:'log cancel' in handle.commands,1)
        assert time.monotonic()-begin<1
        handle.suppress_ack=False
        until(app,lambda:not w.worker.busy,6)
        assert not any(c.startswith(('profile run','speed ','throttle ')) for c in handle.commands)
    finally:
        w.close(); until(app,lambda:not w.worker.isRunning(),10)

def test_invalid_edit_and_small_window(app,tmp_path):
    w=Window(tmp_path)
    try:
        w.resize(1100,760); w.show(); app.processEvents()
        w.table.item(1,0).setText('bad')
        assert w.current is None and not w.upload.isEnabled()
        assert 'attention' in w.status_label.text()
        w.table.item(1,0).setText('2')
        assert w.current is not None
    finally:
        w.close(); until(app,lambda:not w.worker.isRunning())

def test_return_speed_selection_requires_paired_firmware(app,tmp_path,monkeypatch):
    w=Window(tmp_path)
    commands=[]
    monkeypatch.setattr(w,'submit',lambda action,payload:commands.append((action,payload)))
    try:
        speed=w.advanced.return_speed
        assert speed.value()==.75
        speed.setValue(2)
        status=dict(cart_online=True,stationary=True,can_return=True)
        w.update_state(status)
        assert not w.return_button.isEnabled()
        assert 'updated cart and dongle' in w.return_button.toolTip()
        w.update_state(dict(status,return_speed_max=2))
        assert w.return_button.isEnabled()
        w.return_button.click()
        assert commands==[('return_home',2)]
        w.update_state(dict(status,busy=True))
        assert speed.value()==2 and not w.return_button.isEnabled()
    finally:
        w.close(); until(app,lambda:not w.worker.isRunning())

def test_windows_port_listing_refreshes_when_dongle_appears(app,tmp_path,monkeypatch):
    import cart_ui.app as ui
    from serial.tools import list_ports
    from serial.tools.list_ports_common import ListPortInfo
    monkeypatch.setattr(ui,'sys',SimpleNamespace(platform='win32'))
    ports=[]
    monkeypatch.setattr(list_ports,'comports',lambda:ports)
    w=Window(tmp_path)
    commands=[]
    monkeypatch.setattr(w,'submit',lambda action,payload:commands.append((action,payload)))
    try:
        assert w.ports.currentText()==ui.NO_SERIAL_PORTS
        w.connect_button.click()
        assert not commands and 'COM port' in w.status_label.text()
        ports.extend([ListPortInfo('COM10'),ListPortInfo('COM3')])
        w.scan_ports()
        assert [w.ports.itemText(i) for i in range(w.ports.count())]==[
            'COM3','COM10','Demo — no hardware']
        w.ports.setCurrentText('COM10')
        w.connect_button.click()
        assert commands==[('connect','COM10')]
    finally:
        w.close(); until(app,lambda:not w.worker.isRunning())

def test_view_tracking_opens_latest_download_and_can_browse(app,tmp_path,monkeypatch):
    latest=tmp_path/'latest'/'recording.csv'
    latest.parent.mkdir()
    latest.write_text('t_us,target_mps,speed_mps,throttle_pct,position_m\n'
                      '900000,0,0,0,0.8\n1000000,1,0,12,0.8\n'
                      '1100000,1,0.6,16,0.9\n1200000,0,0.5,0,1\n')
    latest.with_name('profile.json').write_text(json.dumps(
        {'points':[{'position_m':0,'speed_mps':0},{'position_m':0.5,'speed_mps':1},
                   {'position_m':1,'speed_mps':0}]}))
    older=tmp_path/'older.csv'
    older.write_text('t_us,target_mps,speed_mps\n0,0,0\n100000,0,0.3\n')
    def no_picker(*args,**kwargs): pytest.fail('Latest download should open directly.')
    monkeypatch.setattr(QFileDialog,'getOpenFileName',no_picker)
    w=Window(tmp_path)
    try:
        w.worker.outcome.emit('download',{'folder':str(latest.parent)})
        w.tracking_button.click()
        view=w.tracking_view
        assert view.isVisible() and view.current_path==latest
        assert view.toolbar.isVisible()
        axes=view.canvas.figure.axes
        assert len(axes)==4
        assert list(axes[0].lines[1].get_ydata())==[0,0,0.6,0.5]
        assert any(line.get_label()=='UI waypoint profile' for ax in axes for line in ax.lines)

        monkeypatch.setattr(QFileDialog,'getOpenFileName',lambda *args:(str(older),''))
        open_button=next(b for b in view.findChildren(QPushButton) if b.text()=='Open recording…')
        open_button.click()
        assert view.current_path==older and len(view.canvas.figure.axes)==2
    finally:
        w.close(); until(app,lambda:not w.worker.isRunning())

def test_view_tracking_browses_without_download_and_reports_invalid_csv(app,tmp_path,monkeypatch):
    invalid=tmp_path/'waypoints.csv'
    invalid.write_text('position_m,speed_mps\n0,0\n')
    monkeypatch.setattr(QFileDialog,'getOpenFileName',lambda *args:(str(invalid),''))
    w=Window(tmp_path)
    try:
        w.tracking_button.click()
        assert w.tracking_view.isVisible()
        assert w.tracking_view.current_path is None
        assert 'Missing CSV columns' in w.tracking_view.notice.text()
    finally:
        w.close(); until(app,lambda:not w.worker.isRunning())

def test_low_speed_warning_preserves_profile_admission(app,tmp_path):
    w=Window(tmp_path)
    try:
        assert w.low_speed_warning.isHidden()  # zero endpoints are normal
        w.table.item(1,1).setText('1')
        assert w.low_speed_warning.isHidden()
        w.table.item(1,1).setText('0.999')
        assert not w.low_speed_warning.isHidden()
        assert w.current is not None

        w.table.item(1,1).setText('2')
        assert w.low_speed_warning.isHidden()
        w.advanced.floor.setValue(0.5)
        assert not w.low_speed_warning.isHidden()  # ramps can now command under 1 m/s
        w.advanced.launch.setValue(0.7)
        w.fill([dict(position_m=x,speed_mps=v) for x,v in [(0,0),(1,0.8),(2,0)]])
        assert w.current['launch_and_low_speed_floor_mps']==pytest.approx(0.5)
        assert w.current['launch_speed_mps']==pytest.approx(0.7)
        w.update_state(dict(cart_online=True,stationary=True,logger_state=0,
                            uploaded=w.current,home=True))
        assert w.upload.isEnabled() and w.start.isEnabled()
    finally:
        w.state={}
        w.close(); until(app,lambda:not w.worker.isRunning())

def test_timed_test_warns_for_signed_low_speeds(app,tmp_path,monkeypatch):
    w=Window(tmp_path)
    messages,commands=[],[]
    def confirm(parent,title,message):
        messages.append(message)
        return QMessageBox.Yes
    monkeypatch.setattr(QMessageBox,'question',confirm)
    monkeypatch.setattr(w,'submit',lambda action,payload:commands.append((action,payload)))
    try:
        for mode,value,warning in [(1,0.5,True),(1,-0.5,True),(1,1,False),
                                   (1,0,False),(0,0.5,False)]:
            w.advanced.mode.setCurrentIndex(mode)
            w.advanced.value.setValue(value)
            w.advanced.run_test()
            assert ('may track poorly' in messages[-1])==warning
            assert commands[-1][1][1]==value
    finally:
        w.close(); until(app,lambda:not w.worker.isRunning())

def test_stricter_profile_limits_recheck_uploaded_waypoints(app,tmp_path):
    w=Window(tmp_path)
    try:
        manifest=w.current
        w.update_state(dict(cart_online=True,stationary=True,logger_state=0,
                            uploaded=manifest,home=True))
        assert w.start.isEnabled()

        w.advanced.max_acceleration.setValue(.8)
        w.advanced.max_deceleration.setValue(.8)
        assert w.current is None
        assert not w.upload.isEnabled() and not w.start.isEnabled()
        assert '2.5 m' in w.status_label.text()
        assert 'deceleration' in w.status_label.toolTip()
        assert w.plot.points==w.points()

        w.advanced.max_acceleration.setValue(1)
        w.advanced.max_deceleration.setValue(1)
        assert w.current==manifest and w.start.isEnabled()

        w.advanced.max_deceleration.setValue(.8)
        assert 'deceleration' in w.status_label.text()
        w.table.item(2,0).setText('5.5')
        assert w.current is not None and w.upload.isEnabled()
        assert not w.start.isEnabled()
    finally:
        w.state={}
        w.close(); until(app,lambda:not w.worker.isRunning())

def test_infeasible_waypoint_still_updates_plot(app,tmp_path):
    w=Window(tmp_path)
    try:
        w.show(); app.processEvents()
        original=w.plot.grab().toImage()
        w.add_point()
        assert len(w.plot.points)==5
        w.table.item(3,1).setText('5')
        app.processEvents()
        assert w.plot.points[3]=={'position_m':7.,'speed_mps':5.}
        assert w.plot.axis_limits()[1]>=5
        assert w.plot.preview_speed(7)==pytest.approx(5)
        assert w.plot.preview_speed(6.5)==pytest.approx(((2**2+5**2)/2)**.5)
        assert w.plot.grab().toImage()!=original
        assert w.current is None and not w.upload.isEnabled() and not w.start.isEnabled()
        assert 'upload blocked' in w.status_label.text()
        w.table.selectRow(3); w.remove_point()
        assert len(w.plot.points)==4 and w.current is not None
        assert w.plot.axis_limits()[1]==2.5
    finally:
        w.close(); until(app,lambda:not w.worker.isRunning())

@pytest.mark.parametrize('row,col,value',[(1,1,''),(1,1,'bad'),(1,1,'nan'),(1,1,'inf'),(1,0,'0'),(1,0,'7'),(1,1,'-1')])
def test_unplottable_cells_clear_stale_curve(app,tmp_path,row,col,value):
    w=Window(tmp_path)
    try:
        old=w.table.item(row,col).text()
        w.table.item(row,col).setText(value)
        assert w.plot.points is None and 'Preview unavailable' in w.plot.preview_error
        assert w.current is None and not w.upload.isEnabled() and not w.start.isEnabled()
        w.plot.grab()  # exercise the empty/error painter
        w.table.item(row,col).setText(old)
        assert w.plot.preview_error is None and w.current is not None
    finally:
        w.close(); until(app,lambda:not w.worker.isRunning())

def test_out_of_firmware_bounds_remains_visible(app,tmp_path):
    w=Window(tmp_path)
    try:
        w.table.item(3,0).setText('40')
        assert w.plot.points[-1]['position_m']==40
        assert w.plot.axis_limits()[0]==40
        assert w.current is None
        w.table.item(1,1).setText('12')
        assert w.plot.preview_speed(2)==12 and w.current is None
        w.plot.grab()
    finally:
        w.close(); until(app,lambda:not w.worker.isRunning())

@pytest.mark.parametrize('count',[6,16,32])
@pytest.mark.parametrize('size',[(1100,760),(1536,1024)])
def test_waypoint_scroll_reaches_last_row_without_overlap(app,tmp_path,count,size):
    from PySide6.QtCore import QPoint, Qt
    w=Window(tmp_path)
    try:
        w.resize(*size); w.show(); app.processEvents()
        minimum=w.table.minimumHeight()
        w.fill([dict(position_m=i*.5,speed_mps=0 if i in (0,count-1) else 1) for i in range(count)])
        app.processEvents()
        assert w.table.minimumHeight()==minimum
        bar=w.table.verticalScrollBar()
        assert w.table.verticalScrollBarPolicy()==Qt.ScrollBarAlwaysOn
        assert bar.isVisible()
        if count>=16: assert bar.maximum()>0
        w.table.scrollToBottom(); app.processEvents()
        last=w.table.visualItemRect(w.table.item(count-1,1))
        assert w.table.viewport().rect().contains(last.center())
        table_bottom=w.table.mapTo(w,QPoint(0,w.table.height())).y()
        button_top=w.add.mapTo(w,QPoint(0,0)).y()
        assert table_bottom<=button_top
        assert w.add.mapTo(w,QPoint(0,w.add.height())).y()<w.height()
        w.table.item(count-1,1).setText('2')
        assert w.plot.points[-1]['speed_mps']==2
    finally:
        w.close(); until(app,lambda:not w.worker.isRunning())

def test_added_waypoint_selected_and_scrolled_into_view(app,tmp_path):
    w=Window(tmp_path)
    try:
        w.resize(1100,760); w.show()
        w.fill([dict(position_m=i*.5,speed_mps=0 if i in (0,15) else 1) for i in range(16)])
        app.processEvents(); w.table.scrollToTop(); w.add_point(); app.processEvents()
        assert w.table.currentRow()==15
        rect=w.table.visualItemRect(w.table.item(15,1))
        assert w.table.viewport().rect().contains(rect.center())
        w.remove_point(); assert len(w.plot.points)==16
    finally:
        w.close(); until(app,lambda:not w.worker.isRunning())

def test_segment_limit_message_and_readonly_display(app,tmp_path):
    w=Window(tmp_path)
    try:
        w.fill([dict(position_m=x,speed_mps=v) for x,v in [(0,0),(2,2),(6,5),(7,2),(7.5,5),(8,0)]])
        assert '6–7 m' in w.status_label.text()
        assert '10.5 m/s² deceleration' in w.status_label.text()
        assert '2.625 m' in w.status_label.text()
        assert '21 m/s² acceleration' in w.status_label.toolTip()
        assert 'at least 3 m' in w.status_label.toolTip()
        assert '3.5 m/s²' in w.advanced.limits_label.text()
        assert w.current is None
    finally:
        w.close(); until(app,lambda:not w.worker.isRunning())

def test_logs_selectable_and_scroll_position_preserved(app,tmp_path):
    from PySide6.QtGui import QTextCursor
    from PySide6.QtCore import Qt
    w=Window(tmp_path)
    try:
        w.logs_button.click(); app.processEvents()
        log=w.log_viewer.view
        assert log.isReadOnly()
        for i in range(100): w.record_log('message '+str(i))
        app.processEvents(); bar=log.verticalScrollBar(); assert bar.maximum()>0
        bar.setValue(0)
        cursor=log.textCursor(); cursor.setPosition(0); cursor.setPosition(8,QTextCursor.KeepAnchor); log.setTextCursor(cursor)
        selected=log.textCursor().selectedText()
        w.message('new event while reading older messages'); app.processEvents()
        assert log.textCursor().selectedText()==selected and bar.value()==0
        assert 'new event' in log.toPlainText()
        assert w.status_label.textInteractionFlags() & Qt.TextSelectableByMouse
        log.moveCursor(QTextCursor.End); bar.setValue(bar.maximum())
        w.record_log('follow newest'); app.processEvents()
        assert bar.value()==bar.maximum()
    finally:
        w.log_viewer.close(); w.close(); until(app,lambda:not w.worker.isRunning())
