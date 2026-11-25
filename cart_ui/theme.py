"""Application-owned light theme, independent of the operating-system palette."""
from pathlib import Path
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QFont, QFontDatabase, QPalette

BACKGROUND = '#f7fbfd'
TEXT = '#070d2b'
MUTED = '#536080'
ACCENT = '#0066e5'

STYLESHEET = '''
QWidget { color: #070d2b; font-size: 17px; }
QWidget#mainWindow, QDialog { background-color: #f7fbfd; }
QLabel { background: transparent; border: none; }
QLabel:disabled { color: #61728a; }
QFrame#card { background: white; border: 1px solid #dbe3eb; border-radius: 14px; }
QLabel#title { font-size: 38px; font-weight: 750; }
QLabel#heading { font-size: 37px; font-weight: 750; }
QLabel#muted { color: #536080; font-size: 18px; }
QPushButton { color: #0066e5; background: white; border: 1.5px solid #0066e5; border-radius: 7px; padding: 13px 17px; font-size: 21px; font-weight: 600; }
QPushButton:hover { background: #eaf3ff; }
QPushButton:pressed { background: #dbe9ff; }
QPushButton:focus { border: 2px solid #004baa; }
QPushButton:disabled { color: #61728a; border-color: #cfdae6; background: #edf2f7; }
QPushButton#start { color: white; background: #0066e5; }
QPushButton#start:hover { background: #0054bd; }
QPushButton#start:disabled { color: #506888; background: #dbe9ff; border-color: #b9cee8; }
QPushButton#stop { color: white; background: #d90835; border-color: #d90835; }
QPushButton#stop:hover { background: #b9062d; }
QPushButton#stop:disabled { color: #8a4353; background: #fce2e8; border-color: #e8bac5; }
QPushButton#advanced { border: none; font-size: 15px; padding: 4px; background: transparent; }
QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox {
    color: #070d2b; background: white; border: 1px solid #bdc9dd;
    border-radius: 6px; padding: 6px; selection-background-color: #dbe9ff; selection-color: #070d2b;
}
QLineEdit:focus, QDoubleSpinBox:focus, QSpinBox:focus, QComboBox:focus { border: 1px solid #0066e5; }
QLineEdit:disabled, QDoubleSpinBox:disabled, QSpinBox:disabled, QComboBox:disabled {
    color: #61728a; background: #edf2f7; border-color: #cfdae6;
}
QDoubleSpinBox QLineEdit, QSpinBox QLineEdit { background: transparent; border: none; padding: 0; }
QDoubleSpinBox, QSpinBox { padding-right: 28px; }
QDoubleSpinBox::up-button, QSpinBox::up-button { subcontrol-origin: border; subcontrol-position: top right; width: 24px; background: #eef3f8; border-left: 1px solid #bdc9dd; border-bottom: 1px solid #bdc9dd; border-top-right-radius: 6px; }
QDoubleSpinBox::down-button, QSpinBox::down-button { subcontrol-origin: border; subcontrol-position: bottom right; width: 24px; background: #eef3f8; border-left: 1px solid #bdc9dd; border-bottom-right-radius: 6px; }
QDoubleSpinBox::up-arrow, QSpinBox::up-arrow { image: url("@ASSETS@/chevron-up.svg"); width: 12px; height: 8px; }
QDoubleSpinBox::down-arrow, QSpinBox::down-arrow { image: url("@ASSETS@/chevron-down.svg"); width: 12px; height: 8px; }
QComboBox { padding: 12px; padding-right: 30px; font-size: 20px; }
QComboBox::drop-down { subcontrol-origin: border; subcontrol-position: top right; width: 26px; border: none; background: transparent; }
QComboBox::down-arrow { image: url("@ASSETS@/chevron-down.svg"); width: 12px; height: 8px; }
QComboBox QAbstractItemView {
    color: #070d2b; background: white; border: 1px solid #bdc9dd;
    selection-background-color: #dbe9ff; selection-color: #070d2b; outline: none;
}
QTableWidget { color: #070d2b; background: white; alternate-background-color: #f2f5f8; border: 1px solid #dbe3eb; border-radius: 5px; gridline-color: #dbe3eb; font-size: 21px; selection-background-color: #dbe9ff; selection-color: #070d2b; }
QTableWidget:disabled { color: #61728a; background: #f2f5f8; }
QTableWidget::item:selected { background: #dbe9ff; color: #070d2b; }
QHeaderView, QTableCornerButton::section { background: #f2f5f8; }
QHeaderView::section { color: #070d2b; background: #f2f5f8; border: none; border-bottom: 1px solid #dbe3eb; padding: 12px; font-size: 20px; }
QPlainTextEdit { color: #070d2b; background: white; border: 1px solid #bdc9dd; border-radius: 5px; padding: 6px; font-size: 12px; selection-background-color: #dbe9ff; selection-color: #070d2b; }
QScrollBar:vertical { background: #eef3f8; width: 16px; margin: 0; border-radius: 7px; }
QScrollBar::handle:vertical { background: #7d9cc4; min-height: 30px; border-radius: 7px; }
QScrollBar::handle:vertical:hover { background: #0066e5; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: none; }
QDialog QLabel { font-size: 15px; }
QDialog QPushButton { font-size: 17px; padding: 8px 12px; }
QDialog QComboBox { font-size: 16px; padding: 6px; padding-right: 30px; }
QDialog QDoubleSpinBox { font-size: 16px; padding: 5px; padding-right: 28px; }
QToolTip { color: #070d2b; background: #ffffff; border: 1px solid #bdc9dd; padding: 6px; }
'''


def apply_theme(app):
    app.setStyle('Fusion')
    palette = QPalette()
    roles = {
        'Window': BACKGROUND, 'WindowText': TEXT, 'Base': '#ffffff',
        'AlternateBase': '#f2f5f8', 'Text': TEXT, 'Button': '#ffffff',
        'ButtonText': TEXT, 'Highlight': '#dbe9ff', 'HighlightedText': TEXT,
        'ToolTipBase': '#ffffff', 'ToolTipText': TEXT, 'PlaceholderText': MUTED,
        'Light': '#ffffff', 'Midlight': '#eef3f8', 'Mid': '#bdc9dd',
        'Dark': '#7d9cc4', 'Shadow': '#536080', 'Link': ACCENT, 'LinkVisited': '#514fa2',
    }
    for group in (QPalette.Active, QPalette.Inactive, QPalette.Disabled):
        for role, colour in roles.items():
            palette.setColor(group, getattr(QPalette, role), QColor(colour))
    for role in (QPalette.WindowText, QPalette.Text, QPalette.ButtonText, QPalette.PlaceholderText):
        palette.setColor(QPalette.Disabled, role, QColor('#61728a'))
    for role in (QPalette.Base, QPalette.Button):
        palette.setColor(QPalette.Disabled, role, QColor('#edf2f7'))
    if hasattr(QPalette, 'Accent'):
        palette.setColor(QPalette.Accent, QColor(ACCENT))
    app.setPalette(palette)
    available = set(QFontDatabase.families())
    family = next((f for f in ('Segoe UI', 'Noto Sans', 'DejaVu Sans', 'Arial') if f in available), app.font().family())
    app.setFont(QFont(family, 11))
    app.setStyleSheet(STYLESHEET.replace("@ASSETS@", (Path(__file__).parent / "assets").resolve().as_posix()))
    # Keep application dialogs consistent too; native window borders remain OS-owned.
    app.setAttribute(Qt.AA_DontUseNativeDialogs, True)


def ui_font(point_size):
    from PySide6.QtWidgets import QApplication
    font = QFont(QApplication.font())
    font.setPointSizeF(point_size)
    return font
