"""Run with python -m cart_ui.app."""
from __future__ import annotations
import argparse
from datetime import datetime
import math
from pathlib import Path
import sys
from PySide6.QtCore import Qt, QRectF, QPointF, QSize, QStandardPaths, QTimer
from PySide6.QtGui import QColor, QFont, QFontDatabase, QPainter, QPainterPath, QPen, QIcon, QPixmap, QShortcut, QKeySequence, QTextCursor
from PySide6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QFrame, QTableWidget, QTableWidgetItem, QHeaderView, QFileDialog, QDialog,
    QFormLayout, QDoubleSpinBox, QPlainTextEdit, QMessageBox, QSizePolicy, QAbstractItemView, QScrollArea)
from .backend import (make_profile, load_csv, profile_limit_issues, PROFILE_LIMITS,
                      RETURN_SPEED, return_neutral_margin)
from .protocol import RETURN_LEGACY_MAX_MPS, RETURN_MAX_MPS, RETURN_MIN_MPS
from .worker import Worker
from .theme import apply_theme, ui_font

BLUE='#0875ff'; INK='#070d2b'
DEFAULT_POINTS=[{'position_m':x,'speed_mps':v} for x,v in [(0,0),(2,2),(6,2),(8,0)]]
LOW_SPEED_WARNING='Low-speed warning. The cart may track poorly or move unevenly below 1 m/s.'
NO_SERIAL_PORTS='No USB serial ports found'


def is_low_speed(speed):
    return 0 < abs(speed) < 1


def icon(kind, color=BLUE, size=36):
    pix=QPixmap(size,size); pix.fill(Qt.transparent)
    p=QPainter(pix); p.setRenderHint(QPainter.Antialiasing)
    p.scale(size/36,size/36); p.setPen(QPen(QColor(color),2.5,Qt.SolidLine,Qt.RoundCap,Qt.RoundJoin))
    if kind in ('upload','download'):
        p.drawPolyline([QPointF(4,23),QPointF(4,31),QPointF(32,31),QPointF(32,23)])
        a,b=(26,5) if kind=='upload' else (5,26)
        p.drawLine(18,a,18,b)
        c=b+7 if kind=='upload' else b-7
        p.drawPolyline([QPointF(11,c),QPointF(18,b),QPointF(25,c)])
    elif kind=='home':
        p.drawPolyline([QPointF(3,17),QPointF(18,3),QPointF(33,17)])
        p.drawPolyline([QPointF(8,15),QPointF(8,32),QPointF(15,32),QPointF(15,22),QPointF(22,22),QPointF(22,32),QPointF(28,32),QPointF(28,15)])
    elif kind=='play':
        p.setBrush(QColor(color)); p.drawPolygon([QPointF(9,5),QPointF(30,18),QPointF(9,31)])
    elif kind=='stop': p.setBrush(QColor(color)); p.drawRoundedRect(QRectF(5,5,26,26),3,3)
    elif kind=='plus': p.drawLine(18,6,18,30); p.drawLine(6,18,30,18)
    elif kind=='trash':
        p.drawLine(7,9,29,9); p.drawLine(13,5,23,5); p.drawRect(10,10,16,21)
    elif kind=='file':
        p.drawPolyline([QPointF(23,3),QPointF(7,3),QPointF(7,32),QPointF(29,32),QPointF(29,10),QPointF(23,3),QPointF(23,11),QPointF(29,11)])
    else:
        p.setBrush(QColor(color)); p.drawEllipse(QPointF(18,18),2.4,2.4); p.setBrush(Qt.NoBrush)
        for radius in (7,12,17):
            r=QRectF(18-radius,18-radius,2*radius,2*radius)
            p.drawArc(r,-50*16,100*16); p.drawArc(r,130*16,100*16)
    p.end(); return QIcon(pix)


class ProfilePlot(QWidget):
    def __init__(self):
        super().__init__(); self.points=DEFAULT_POINTS; self.preview_error=None; self.setMinimumSize(400,320)
    def set_points(self, points):
        # Plot geometry is independent of the firmware's admission limits.
        if len(points)<2: raise ValueError('Enter at least two waypoints.')
        if any(not math.isfinite(p[k]) or p[k]<0 for p in points for k in ('position_m','speed_mps')):
            raise ValueError('Positions and speeds must be finite, non-negative numbers.')
        if any(b['position_m']<=a['position_m'] for a,b in zip(points,points[1:])):
            raise ValueError('Waypoint positions must increase from row to row.')
        self.points=points; self.preview_error=None; self.update()
    def clear_preview(self, reason):
        self.points=None; self.preview_error='Preview unavailable: '+reason; self.update()
    def axis_limits(self):
        xmax=max(1,max(v['position_m'] for v in self.points))
        peak=max(v['speed_mps'] for v in self.points)
        ymax=max(.5,peak*1.25 if math.isfinite(peak*1.25) else peak)
        if ymax<=6:
            step=.5 if ymax<=3 else 1
            ymax=math.ceil(ymax/step)*step
        return xmax,ymax
    def preview_speed(self, x):
        if x<=self.points[0]['position_m']: return self.points[0]['speed_mps']
        for a,b in zip(self.points,self.points[1:]):
            if x<=b['position_m']:
                t=(x-a['position_m'])/(b['position_m']-a['position_m'])
                return math.hypot(math.sqrt(1-t)*a['speed_mps'],math.sqrt(t)*b['speed_mps'])
        return self.points[-1]['speed_mps']
    def paintEvent(self,event):
        p=QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        w,h=self.width(),self.height(); area=QRectF(85,35,max(100,w-120),max(100,h-115))
        if self.preview_error or not self.points:
            p.setPen(QColor('#536080')); p.setFont(ui_font(14))
            p.drawText(self.rect().adjusted(30,30,-30,-30),Qt.AlignCenter|Qt.TextWordWrap,self.preview_error or 'Preview unavailable.')
            p.end(); return
        points=self.points
        xmax,ymax=self.axis_limits()
        ycount=round(ymax/(.5 if ymax<=3 else 1)) if ymax<=6 else 5
        xticks=sorted(set([0.,xmax]+[float(x) for x in range(2 if xmax<=10 else 5,math.ceil(xmax),2 if xmax<=10 else 5)])) if xmax<=30 else [xmax*(i/5) for i in range(6)]
        def pos(x,y): return QPointF(area.left()+x/xmax*area.width(),area.bottom()-y/ymax*area.height())
        p.setFont(ui_font(14))
        for x in xticks:
            pt=pos(x,0); p.setPen(QPen(QColor('#dce4ed'),1)); p.drawLine(QPointF(pt.x(),area.top()),pt)
            p.setPen(QColor(INK)); p.drawText(QRectF(pt.x()-30,area.bottom()+12,60,30),Qt.AlignCenter,f'{x:g}')
        for i in range(ycount+1):
            y=ymax*(i/ycount); pt=pos(0,y); p.setPen(QPen(QColor('#dce4ed'),1)); p.drawLine(pt,pos(xmax,y))
            p.setPen(QColor(INK)); p.drawText(QRectF(5,pt.y()-15,65,30),Qt.AlignRight|Qt.AlignVCenter,(f'{y:.1f}' if ymax<=6 else f'{y:g}') if y else '0')
        p.setPen(QPen(QColor('#252d4b'),1.5)); p.drawLine(area.bottomLeft(),area.topLeft()); p.drawLine(area.bottomLeft(),area.bottomRight())
        p.setFont(ui_font(15)); p.drawText(QRectF(area.left(),h-40,area.width(),35),Qt.AlignCenter,'Position (m)')
        p.save(); p.translate(25,area.center().y()); p.rotate(-90); p.drawText(QRectF(-100,-20,200,40),Qt.AlignCenter,'Speed (m/s)'); p.restore()
        path=QPainterPath()
        try:
            for i in range(401):
                x=xmax*(i/400); y=self.preview_speed(x)
                if i==0: path.moveTo(pos(x,y))
                else: path.lineTo(pos(x,y))
            p.setPen(QPen(QColor(BLUE),3.5)); p.drawPath(path)
            p.setBrush(QColor('white'))
            for point in points: p.drawEllipse(pos(point['position_m'],point['speed_mps']),7,7)
        except (ValueError,ZeroDivisionError): pass
        p.end()


def append_log(view, text):
    """Append without stealing a selection or a reader's scroll position."""
    bar=view.verticalScrollBar(); old=bar.value()
    follow=old>=bar.maximum()-2 and not view.textCursor().hasSelection()
    cursor=view.textCursor()
    view.appendPlainText(text)
    if follow: bar.setValue(bar.maximum())
    else:
        view.setTextCursor(cursor); bar.setValue(old)


class LogDialog(QDialog):
    def __init__(self, main):
        super().__init__(main); self.main=main; self.setWindowTitle('Cart logs'); self.resize(900,540)
        layout=QVBoxLayout(self)
        layout.addWidget(QLabel('Timestamped status and command acknowledgements. Select text to copy; scroll to review earlier messages.'))
        self.view=QPlainTextEdit(); self.view.setReadOnly(True); self.view.setMaximumBlockCount(5000)
        self.view.setFont(QFontDatabase.systemFont(QFontDatabase.FixedFont)); layout.addWidget(self.view,1)
        self.folder_label=QLabel(); self.folder_label.setWordWrap(True); self.folder_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.folder_label); self.refresh_folder()
        row=QHBoxLayout(); layout.addLayout(row)
        copy=QPushButton('Copy all'); copy.clicked.connect(lambda:QApplication.clipboard().setText(self.view.toPlainText())); row.addWidget(copy)
        save=QPushButton('Save logs…'); save.clicked.connect(self.save); row.addWidget(save)
        folder=QPushButton('Recording folder…'); folder.clicked.connect(self.choose_folder); row.addWidget(folder)
    def refresh_folder(self):
        self.folder_label.setText('Automatic recording folder: '+str(self.main.worker.output_root))
    def choose_folder(self):
        path=QFileDialog.getExistingDirectory(self,'Automatic recording folder',str(self.main.worker.output_root))
        if path:
            self.main.worker.output_root=Path(path); self.refresh_folder()
            self.main.message('Future recordings will be saved automatically under '+path)
    def save(self):
        path,_=QFileDialog.getSaveFileName(self,'Save logs','cart-logs.txt','Text files (*.txt)')
        if path:
            try: Path(path).write_text(self.view.toPlainText()+'\n',encoding='utf-8')
            except OSError as e: self.main.message('Could not save logs: '+str(e))


class Advanced(QDialog):
    def __init__(self, main):
        super().__init__(main); self.main=main; self.setWindowTitle('Advanced controls')
        available=main.screen().availableGeometry().size()
        self.resize(QSize(620,700).boundedTo(available-QSize(24,56)))
        dialog_layout=QVBoxLayout(self)
        scroll=QScrollArea()
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidgetResizable(True)
        content=QWidget()
        layout=QVBoxLayout(content)
        layout.setContentsMargins(0,0,8,0)
        scroll.setWidget(content)
        dialog_layout.addWidget(scroll)
        layout.addWidget(QLabel('Settings apply to the current cart session.'))
        form=QFormLayout(); layout.addLayout(form)
        self.kp=self.spin(0,50,10); self.ki=self.spin(0,100,20)
        form.addRow('Kp',self.kp); form.addRow('Ki',self.ki)
        row=QHBoxLayout(); self.read=QPushButton('Read gains'); self.apply=QPushButton('Apply gains'); row.addWidget(self.read); row.addWidget(self.apply); layout.addLayout(row)
        self.read.clicked.connect(lambda:main.submit('pi_read'))
        self.apply.clicked.connect(lambda:main.submit('pi_set',(self.kp.value(),self.ki.value())))
        self.floor=self.spin(.1,10,1); self.launch=self.spin(.1,10,1)
        self.floor.setSingleStep(.1); self.launch.setSingleStep(.1)
        form.addRow('Minimum moving speed (m/s)',self.floor); form.addRow('Launch speed (m/s)',self.launch)
        self.floor.valueChanged.connect(main.edited); self.launch.valueChanged.connect(main.edited)
        self.max_acceleration=self.spin(.1,PROFILE_LIMITS['acceleration'],PROFILE_LIMITS['acceleration'])
        self.max_deceleration=self.spin(.1,PROFILE_LIMITS['deceleration'],PROFILE_LIMITS['deceleration'])
        for field in (self.max_acceleration,self.max_deceleration):
            field.setDecimals(2)
            field.setSingleStep(.1)
            field.setKeyboardTracking(False)
            field.valueChanged.connect(main.edited)
        form.addRow('Profile max acceleration (m/s²)',self.max_acceleration)
        form.addRow('Profile max deceleration (m/s²)',self.max_deceleration)
        self.return_speed=self.spin(RETURN_MIN_MPS,RETURN_MAX_MPS,RETURN_SPEED)
        self.return_speed.setDecimals(2); self.return_speed.setSingleStep(.05)
        form.addRow('Return cruise speed (m/s)',self.return_speed)
        self.return_speed.valueChanged.connect(main.update_return_button)
        self.rate=QComboBox(); self.rate.addItems(['10 Hz','50 Hz','Off — no recording'])
        form.addRow('Logging for the next run',self.rate)
        note=QLabel('Minimum moving speed can be set as low as 0.1 m/s. Keep launch speed at or above the minimum and at or below the highest waypoint speed.\n50 Hz can fill the 896-sample buffer on longer runs. Use 10 Hz for profiles.'); note.setWordWrap(True); layout.addWidget(note)
        self.limits_label=QLabel(f"Firmware ceilings are {PROFILE_LIMITS['speed']:g} m/s, {PROFILE_LIMITS['acceleration']:g} m/s² acceleration, and {PROFILE_LIMITS['deceleration']:g} m/s² deceleration. Lower limits apply to the waypoint profile.")
        self.limits_label.setWordWrap(True); layout.addWidget(self.limits_label)
        self.mode=QComboBox(); self.mode.addItems(['Fixed throttle (%)','Timed speed (m/s)'])
        self.value=self.spin(-100,100,10); self.seconds=self.spin(.05,2,.5)
        form.addRow('Test mode',self.mode); form.addRow('Test value',self.value); form.addRow('Duration (s)',self.seconds)
        self.mode.currentIndexChanged.connect(self.test_mode)
        self.test=QPushButton('Run timed test'); self.test.clicked.connect(self.run_test); layout.addWidget(self.test)
        self.stop=QPushButton('STOP'); self.stop.setObjectName('stop'); self.stop.clicked.connect(main.stop); dialog_layout.addWidget(self.stop)
        self.off=QPushButton('Disable logging now'); self.off.clicked.connect(lambda:main.submit('log_off')); layout.addWidget(self.off)
        layout.addWidget(QLabel('Diagnostics'))
        self.log=QPlainTextEdit(); self.log.setFont(QFontDatabase.systemFont(QFontDatabase.FixedFont)); self.log.setReadOnly(True); self.log.setMaximumBlockCount(500); layout.addWidget(self.log)
    def spin(self,lo,hi,val):
        w=QDoubleSpinBox(); w.setRange(lo,hi); w.setDecimals(3); w.setValue(val); return w
    def test_mode(self,index):
        self.value.setRange(-10 if index else -100,10 if index else 100); self.seconds.setMaximum(3 if index else 2)
    def rate_value(self): return [10,50,0][self.rate.currentIndex()]
    def profile_limits(self):
        return dict(PROFILE_LIMITS, acceleration=self.max_acceleration.value(),
                    deceleration=self.max_deceleration.value())
    def run_test(self):
        mode='speed' if self.mode.currentIndex() else 'throttle'
        value,seconds=self.value.value(),self.seconds.value()
        message=f'Command {value:g} {"m/s" if mode=="speed" else "% throttle"} for {seconds:g} s?\nThe cart will move. Confirm the route is clear.'
        if mode=='speed' and is_low_speed(value):
            message+='\n\n'+LOW_SPEED_WARNING
        if QMessageBox.question(self,'Start timed test',message)==QMessageBox.Yes:
            self.main.submit('test',(mode,value,seconds,self.rate_value()))
    def update_state(self,state):
        ready=state.get('cart_online') and state.get('stationary') and not state.get('busy')
        available=ready and state.get('logger_state') in (0,5)
        for w in (self.apply,self.test): w.setEnabled(bool(available))
        self.off.setEnabled(bool(ready and state.get('logger_state') in (0,1,5)))
        self.read.setEnabled(bool(available))
        for w in (self.floor,self.launch,self.rate,self.return_speed,self.max_acceleration,self.max_deceleration):
            w.setEnabled(not state.get('busy') and (not state.get('port_open') or bool(ready)))
        limit=state.get('return_speed_max',RETURN_LEGACY_MAX_MPS)
        hint='Speeds above 1 m/s require updated firmware on both boards. Fast returns slow near home.'
        if state.get('cart_online'):
            hint+=f' Available now up to {limit:g} m/s.'
        self.return_speed.setToolTip(hint)
        self.stop.setEnabled(state.get('port_open',False))


class Window(QWidget):
    def __init__(self, output, demo=False):
        apply_theme(QApplication.instance())
        super().__init__(); self.setObjectName('mainWindow'); self.setWindowTitle('Mobile Radar Target'); self.resize(1440,960); self.setMinimumSize(960,570)
        self.current=None; self.state={}; self.closing=False; self.allow_close=False
        self.latest_recording=None; self.tracking_view=None
        self.worker=Worker(output); self.worker.status.connect(self.message); self.worker.diagnostic.connect(self.record_log); self.worker.snapshot.connect(self.update_state); self.worker.outcome.connect(self.outcome)
        root=QVBoxLayout(self); self.root_layout=root; root.setContentsMargins(32,30,32,25); root.setSpacing(24)
        top=QHBoxLayout(); mark=QLabel(); self.radar_label=mark; mark.setPixmap(icon('radar',size=90).pixmap(90,90)); top.addWidget(mark); top.addSpacing(12)
        title=QLabel('Mobile Radar Target'); title.setObjectName('title'); self.title_label=title; top.addWidget(title); top.addStretch()
        self.ports=QComboBox(); self.ports.setMinimumWidth(235); self.scan_ports(); top.addWidget(self.ports)
        self.connect_button=QPushButton('Connect'); self.connect_button.clicked.connect(self.toggle_connection); top.addWidget(self.connect_button); root.addLayout(top)
        card=QFrame(); card.setObjectName('card'); c=QVBoxLayout(card); self.card_layout=c; c.setContentsMargins(32,26,24,22); c.setSpacing(25)
        heading=QHBoxLayout(); name=QLabel('Velocity profile'); name.setObjectName('heading'); self.heading_label=name; heading.addWidget(name); heading.addStretch()
        self.load=self.button('Load CSV','file',self.load_file); heading.addWidget(self.load); c.addLayout(heading)
        body=QHBoxLayout(); body.setSpacing(30); self.plot=ProfilePlot(); body.addWidget(self.plot,5)
        right=QVBoxLayout(); self.right_layout=right; right.setSpacing(20)
        label=QLabel('Waypoints'); label.setStyleSheet('font-size: 26px; font-weight: 700'); right.addWidget(label)
        self.table=QTableWidget(0,2); self.table.setHorizontalHeaderLabels(['Position (m)','Speed (m/s)']); self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch); self.table.verticalHeader().hide()
        self.table.setSelectionBehavior(QTableWidget.SelectRows); self.table.setMinimumWidth(400); self.table.setMinimumHeight(160); self.table.setMaximumHeight(380); self.table.verticalHeader().setDefaultSectionSize(52); self.table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn); self.table.setVerticalScrollMode(QAbstractItemView.ScrollPerPixel); self.table.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding)
        self.table.itemChanged.connect(self.edited); right.addWidget(self.table,1)
        buttons=QHBoxLayout(); self.add=self.button('Add point','plus',self.add_point); self.remove=self.button('Remove point','trash',self.remove_point); buttons.addWidget(self.add); buttons.addWidget(self.remove); right.addLayout(buttons)
        body.addLayout(right,3); c.addLayout(body,1)
        self.low_speed_warning=QLabel(LOW_SPEED_WARNING); self.low_speed_warning.setWordWrap(True)
        self.low_speed_warning.setStyleSheet('color: #744300; background: #fff4df; padding: 6px 10px; border-radius: 6px;')
        c.addWidget(self.low_speed_warning); root.addWidget(card,1)
        actions=QHBoxLayout(); actions.setSpacing(16)
        self.upload=self.button('Upload profile','upload',lambda:self.submit('upload',self.current)); self.home=self.button('Set home','home',lambda:self.submit('home'))
        self.start=self.button('Start run','play',self.start_run,white=True); self.start.setObjectName('start')
        self.stop_button=self.button('STOP','stop',self.stop,white=True); self.stop_button.setObjectName('stop')
        for w in (self.upload,self.home,self.start,self.stop_button): actions.addWidget(w)
        line=QFrame(); line.setFrameShape(QFrame.VLine); line.setStyleSheet('color:#dbe3eb'); actions.addWidget(line)
        self.download=self.button('Download recording','download',self.download_recording); actions.addWidget(self.download); root.addLayout(actions)
        bottom=QHBoxLayout(); self.hint=QLabel('Align the cart at home before starting.'); self.hint.setObjectName('muted'); bottom.addWidget(self.hint); bottom.addStretch()
        self.return_button=self.button('Return home','home',self.start_return)
        self.return_button.setStyleSheet('font-size: 16px; padding: 6px 12px;'); self.return_button.setMinimumHeight(36)
        bottom.addWidget(self.return_button)
        self.tracking_button=QPushButton('View tracking'); self.tracking_button.setObjectName('advanced')
        self.tracking_button.setToolTip('View the latest download or open an older recording.')
        self.tracking_button.clicked.connect(self.show_tracking); bottom.addWidget(self.tracking_button)
        self.logs_button=QPushButton('Logs'); self.logs_button.setObjectName('advanced'); bottom.addWidget(self.logs_button)
        self.more=QPushButton('Advanced'); self.more.setObjectName('advanced'); bottom.addWidget(self.more); root.addLayout(bottom)
        self.status_label=QLabel('Not connected. Select the USB dongle port to begin.'); self.status_label.setObjectName('muted'); self.status_label.setTextInteractionFlags(Qt.TextSelectableByMouse|Qt.TextSelectableByKeyboard); self.status_label.setWordWrap(True); self.status_label.setMinimumHeight(42); root.addWidget(self.status_label)
        self.log_viewer=LogDialog(self); self.logs_button.clicked.connect(self.log_viewer.show)
        self.advanced=Advanced(self); self.more.clicked.connect(self.show_advanced)
        self.fill(DEFAULT_POINTS)
        QShortcut(QKeySequence('Escape'),self,activated=self.stop,context=Qt.ApplicationShortcut)
        self.worker.start()
        self.refresh_timer=QTimer(self); self.refresh_timer.timeout.connect(self.scan_ports); self.refresh_timer.start(2500)
        if demo: self.ports.setCurrentText('Demo — no hardware')
    def show_advanced(self):
        self.advanced.show()
        if self.advanced.read.isEnabled(): self.submit('pi_read')
    def resizeEvent(self,event):
        super().resizeEvent(event)
        if not hasattr(self,'download'): return
        compact=self.width()<1350 or self.height()<850
        short=self.height()<700
        self.root_layout.setSpacing(8 if short else 12 if compact else 24)
        root_margins=(24,10,24,10) if short else (24,16,24,16) if compact else (32,30,32,25)
        self.root_layout.setContentsMargins(*root_margins)
        self.card_layout.setSpacing(8 if short else 12 if compact else 25)
        card_margins=(24,10,24,10) if short else (24,16,24,16) if compact else (32,26,24,22)
        self.card_layout.setContentsMargins(*card_margins)
        self.right_layout.setSpacing(8 if compact else 20)
        self.radar_label.setPixmap(icon('radar',size=72 if compact else 90).pixmap(72 if compact else 90,72 if compact else 90))
        self.plot.setMinimumSize(350 if compact else 400,180 if short else 200 if compact else 320)
        self.table.verticalHeader().setDefaultSectionSize(32 if compact else 52)
        self.table.setStyleSheet('font-size: 16px;' if compact else 'font-size: 21px;')
        self.table.horizontalHeader().setStyleSheet('QHeaderView::section {font-size:16px; padding:6px;}' if compact else 'QHeaderView::section {font-size:20px; padding:12px;}')
        self.table.setMinimumHeight(120 if compact else 160)
        self.table.setMaximumHeight(260 if compact else 380)
        for b in (self.upload,self.home,self.start,self.stop_button,self.download,self.add,self.remove,self.load,self.connect_button):
            b.setStyleSheet('font-size: %dpx; padding: %dpx %dpx;' % ((16,10,10) if compact else (21,13,17)))
            b.setIconSize(QSize(24,24) if compact else QSize(30,30))
            b.setMinimumHeight(44 if compact else 60)
        self.title_label.setStyleSheet('font-size: %dpx; font-weight: 750;' % (30 if compact else 38))
        self.heading_label.setStyleSheet('font-size: %dpx; font-weight: 750;' % (30 if compact else 37))
    def button(self,text,kind,callback,white=False):
        b=QPushButton(text); b.setIcon(icon(kind,'white' if white else BLUE)); b.setIconSize(QSize(30,30)); b.setMinimumHeight(60); b.clicked.connect(callback); return b
    def scan_ports(self):
        if self.state.get('port_open'): return
        from serial.tools import list_ports
        current=self.ports.currentText() if hasattr(self,'ports') else ''
        ports=list_ports.comports()
        if sys.platform=='win32':
            names=[port.device for port in sorted(ports)] or [NO_SERIAL_PORTS]
        else:
            names=[port.device for port in ports if not port.device.startswith('/dev/ttyS')]
            names.sort(key=lambda name:(not name.startswith('/dev/ttyACM'),name))
            if not names: names=['/dev/ttyACM0']
        names+=['Demo — no hardware']
        if [self.ports.itemText(i) for i in range(self.ports.count())]!=names:
            self.ports.clear(); self.ports.addItems(names)
            if current in names: self.ports.setCurrentText(current)
    def fill(self,points):
        self.table.blockSignals(True); self.table.setRowCount(len(points))
        for row,point in enumerate(points):
            for col,key in enumerate(('position_m','speed_mps')):
                item=QTableWidgetItem(f'{point[key]:g}'); item.setTextAlignment(Qt.AlignCenter); self.table.setItem(row,col,item)
        self.table.blockSignals(False); self.edited()
    def points(self):
        return [{key:float(self.table.item(row,col).text()) for col,key in enumerate(('position_m','speed_mps'))} for row in range(self.table.rowCount())]
    def edited(self,*args):
        self.current=None
        self.status_label.setToolTip('')
        floor,launch=self.advanced.floor.value(),self.advanced.launch.value()
        low_speed=is_low_speed(floor) or is_low_speed(launch)
        try:
            points=self.points()
            self.plot.set_points(points)
        except (ValueError,TypeError,AttributeError,OverflowError) as e:
            self.plot.clear_preview(str(e))
            self.message('Profile needs attention. '+self.plot.preview_error)
        else:
            low_speed=low_speed or any(is_low_speed(point['speed_mps']) for point in points)
            limits=self.advanced.profile_limits()
            issues=[]
            try:
                candidate=make_profile(points,floor,launch)
                if candidate['points'][-1]['position_m']>25.208: raise ValueError('Maximum endpoint from the first-magnet home is 25.208 m.')
                issues=profile_limit_issues(candidate['points'],limits)
                if issues: raise ValueError('Infeasible input: profile exceeds the selected limits.')
                self.current=candidate
                self.message('Profile ready to upload. Preview uses the firmware’s speed-squared interpolation.')
            except (ValueError,TypeError,AttributeError,OverflowError) as e:
                reason=str(e)
                if reason.startswith('Infeasible input:'):
                    issues=issues or profile_limit_issues(points,limits)
                    reason=issues[0] if issues else 'The profile exceeds the configured limits. Reduce speed or increase the ramp distance.'
                    if len(issues)>1: reason+=f' ({len(issues)-1} more; hover here for details.)'
                    self.status_label.setToolTip('\n\n'.join(issues))
                self.message('Preview updated; upload blocked. '+reason)
        self.low_speed_warning.setVisible(low_speed)
        self.update_state(self.state)
    def add_point(self):
        try:
            p=self.points(); x=p[-1]['position_m']; v=max(a['speed_mps'] for a in p)
            if len(p)>=32: raise ValueError('The firmware accepts at most 32 waypoints.')
            row=len(p)-1
            p.insert(row,dict(position_m=(p[-2]['position_m']+x)/2,speed_mps=v)); self.fill(p)
            self.table.selectRow(row); self.table.setCurrentCell(row,1)
            self.table.scrollToItem(self.table.item(row,1),QAbstractItemView.PositionAtCenter)
        except Exception as e: self.message(str(e))
    def remove_point(self):
        row=self.table.currentRow()
        if row<0: self.message('Select the waypoint to remove.'); return
        if self.table.rowCount()<=2: self.message('A profile needs at least two waypoints.'); return
        self.table.removeRow(row); self.edited()
    def load_file(self):
        path,_=QFileDialog.getOpenFileName(self,'Load velocity profile','','CSV files (*.csv)')
        if path:
            try: self.fill(load_csv(path))
            except Exception as e: self.message(str(e))
    def message(self,text):
        if self.state.get('simulated'): text='DEMO · '+text
        self.status_label.setText(text)
        self.record_log(text)
    def record_log(self,text):
        entry=datetime.now().astimezone().strftime('%H:%M:%S')+'  '+text
        if hasattr(self,'log_viewer'): append_log(self.log_viewer.view,entry)
        if hasattr(self,'advanced'): append_log(self.advanced.log,entry)
    def submit(self,action,payload=None):
        if not self.worker.submit(action,payload): self.message('An operation is already in progress. STOP remains available.'); return
        self.state['busy']=True; self.update_state(self.state); self.message(action.replace('_',' ').capitalize()+'…')
    def toggle_connection(self):
        port=self.ports.currentText()
        if not self.state.get('port_open') and port==NO_SERIAL_PORTS:
            self.message('Plug in the USB dongle and select its COM port, or choose Demo.')
            return
        self.submit('disconnect' if self.state.get('port_open') else 'connect',port)
    def start_run(self):
        if self.current: self.submit('start',dict(manifest=self.current,rate=self.advanced.rate_value()))
    def start_return(self):
        self.submit('return_home',self.advanced.return_speed.value())
    def update_return_button(self,*args):
        state=self.state
        ready=state.get('cart_online') and state.get('stationary') and not state.get('busy')
        speed=self.advanced.return_speed.value()
        supported=speed <= state.get('return_speed_max',RETURN_LEGACY_MAX_MPS)
        self.return_button.setEnabled(bool(state.get('can_return') and ready and supported))
        hint=state.get('return_hint','Wait for cart status.')
        if state.get('can_return') and ready:
            if supported:
                hint=f'Reverse at up to {speed:g} m/s, with a {return_neutral_margin(speed)*100:g} cm neutral margin.'
                if is_low_speed(speed): hint+='\n'+LOW_SPEED_WARNING
            else:
                hint='Select 1 m/s or less, or install the updated cart and dongle firmware.'
        self.return_button.setToolTip(hint)
    def stop(self):
        self.worker.stop_now(); self.message('STOP requested. Waiting for acknowledgement…')
    def download_recording(self):
        self.submit('download',self.worker.output_root)
    def show_tracking(self):
        if self.tracking_view is None:
            try:
                from .tracking import TrackingDialog
            except ImportError as e:
                self.message(f'Could not open the tracking viewer. Install cart_ui/requirements.txt. {e}')
                return
            self.tracking_view=TrackingDialog(self,self.worker.output_root)
        view=self.tracking_view
        view.output_root=self.worker.output_root
        view.show(); view.raise_(); view.activateWindow()
        if self.latest_recording is not None and self.latest_recording.is_file():
            if self.latest_recording!=view.current_path:
                view.load_recording(self.latest_recording)
        elif view.current_path is None:
            view.choose_recording()
    def outcome(self,kind,value):
        if kind=='pi':
            self.advanced.kp.setValue(value['kp']); self.advanced.ki.setValue(value['ki']); self.message(f"Cart gains: Kp {value['kp']:g}, Ki {value['ki']:g} (RAM settings).")
        if kind=='download':
            self.latest_recording=Path(value['folder'])/'recording.csv'
            self.message('Recording saved. Click View tracking to open it. '+value['folder'])
        if kind=='complete' and value=='disconnect' and self.closing:
            self.worker.quit_event.set(); self.worker.wait(1500); self.allow_close=True; self.close()
    def update_state(self,state):
        self.state=state; busy=state.get('busy',False); online=state.get('cart_online',False); stationary=state.get('stationary',False)
        ready=online and stationary and not busy; available=ready and state.get('logger_state') in (0,5)
        edit=not busy and (not state.get('port_open') or ready)
        for w in (self.load,self.table,self.add,self.remove): w.setEnabled(edit)
        self.upload.setEnabled(bool(available and self.current)); self.home.setEnabled(bool(available))
        self.start.setEnabled(bool(available and self.current and state.get('uploaded')==self.current and state.get('home')))
        self.download.setEnabled(bool(ready and state.get('downloadable')))
        self.stop_button.setEnabled(bool(state.get('port_open')))
        self.connect_button.setText('Disconnect' if state.get('port_open') else 'Connect'); self.connect_button.setEnabled(not busy)
        self.ports.setEnabled(not state.get('port_open') and not busy)
        self.advanced.update_state(state)
        self.update_return_button()
        self.hint.setText('DEMO — no hardware connected.' if state.get('simulated') else 'Align the cart at home before starting.')
    def closeEvent(self,event):
        if self.allow_close: event.accept(); return
        if not self.state.get('port_open') and not self.state.get('busy'):
            self.worker.quit_event.set(); self.worker.wait(1500); event.accept(); return
        event.ignore()
        if self.closing: return
        self.closing=True; self.stop()
        def finish():
            if self.worker.busy or self.worker.reserved or self.worker.stop_event.is_set(): QTimer.singleShot(100,finish)
            elif self.worker.session: self.submit('disconnect')
            else:
                self.worker.quit_event.set(); self.worker.wait(1500); self.allow_close=True; self.close()
        QTimer.singleShot(100,finish)


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--demo',action='store_true'); parser.add_argument('--screenshot',type=Path); parser.add_argument('--output',type=Path)
    args=parser.parse_args(); app=QApplication(sys.argv[:1])
    documents=QStandardPaths.writableLocation(QStandardPaths.DocumentsLocation)
    output=args.output or (Path(documents) if documents else Path.home()/'Documents')/'MobileRadarTarget'
    window=Window(output,args.demo)
    available=window.screen().availableGeometry().size()
    window.resize(window.size().boundedTo(available-QSize(24,56)))
    window.show()
    if args.screenshot:
        def capture():
            window.resize(1536,1024); app.processEvents(); window.grab().save(str(args.screenshot)); window.close()
        QTimer.singleShot(500,capture)
    return app.exec()
if __name__=='__main__': sys.exit(main())
