import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import pytest
from PySide6.QtWidgets import QApplication, QLabel
from PySide6.QtGui import QPalette, QColor
from cart_ui.app import Window
from cart_ui.theme import TEXT, BACKGROUND

@pytest.mark.parametrize('dark',[True,False])
def test_theme_overrides_desktop_palette(tmp_path,dark):
    app=QApplication.instance() or QApplication([])
    palette=QPalette()
    for group in (QPalette.Active,QPalette.Inactive,QPalette.Disabled):
        for role in (QPalette.Window,QPalette.Base,QPalette.Button):
            palette.setColor(group,role,QColor('#222833' if dark else '#ffffff'))
        for role in (QPalette.WindowText,QPalette.Text,QPalette.ButtonText):
            palette.setColor(group,role,QColor('#ffffff' if dark else '#000000'))
    app.setStyleSheet(''); app.setPalette(palette)
    w=Window(tmp_path)
    try:
        w.resize(1100,760); w.show(); w.advanced.show(); app.processEvents()
        assert w.palette().color(QPalette.Window).name()==BACKGROUND
        assert w.advanced.palette().color(QPalette.Window).name()==BACKGROUND
        assert app.palette().color(QPalette.Inactive,QPalette.Text).name()==TEXT
        for box in (w.advanced.kp,w.advanced.seconds):
            box.setEnabled(True); box.setFocus(); box.selectAll(); app.processEvents()
            assert box.palette().color(QPalette.Text).name()==TEXT
            assert box.palette().color(QPalette.Base).name()=='#ffffff'
            box.setEnabled(False); app.processEvents()
            assert box.palette().color(QPalette.Disabled,QPalette.Text).name()=='#61728a'
        w.advanced.mode.showPopup(); app.processEvents()
        assert w.advanced.mode.view().palette().color(QPalette.Text).name()==TEXT
        assert w.advanced.mode.view().palette().color(QPalette.Base).name()=='#ffffff'
        w.advanced.mode.hidePopup()
        assert w.advanced.log.palette().color(QPalette.Text).name()==TEXT
        assert w.advanced.log.palette().color(QPalette.Base).name()=='#ffffff'
        label=w.advanced.findChildren(QLabel)[0]
        assert not label.autoFillBackground()
        w.table.selectRow(1); app.processEvents()
        assert w.table.palette().color(QPalette.HighlightedText).name()==TEXT
        # Exercise painting for both top-level windows under the overridden theme.
        assert not w.grab().isNull() and not w.advanced.grab().isNull()
    finally:
        w.advanced.close(); w.close(); w.worker.wait(1500); app.processEvents()
