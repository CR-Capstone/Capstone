"""Shared dark stylesheet and trace colours, matching the simulators."""

MOTOR_COLORS = ("#76d5f7", "#ef5da8")
GRID = "#1d2a3a"
BACKGROUND = "#101923"
TEXT = "#d6e0ec"
WARN = "#f7b76d"
DANGER = "#ef5d5d"
OK = "#6de29a"

STYLE = """
QWidget { background:#111925; color:#d6e0ec; font:13px 'Inter','Helvetica Neue'; }
QMainWindow { background:#0b1018; }
QToolBar { spacing:8px; padding:8px; border-bottom:1px solid #26364a; }
QPushButton { background:#24354a; border:1px solid #38516d; border-radius:4px; padding:7px 12px; font-weight:600; }
QPushButton:hover { background:#2d4661; }
QPushButton:checked { background:#9c3f66; border-color:#ef5da8; }
QPushButton:disabled { color:#65758a; background:#18222f; }
QPushButton#arm { background:#2a4a33; border-color:#3f7d55; }
QPushButton#arm:checked { background:#b43b3b; border-color:#ef5d5d; color:#ffecec; }
QPushButton#stop { background:#7a2020; border-color:#ef5d5d; color:#ffdede; font-weight:800; }
QPushButton#stop:hover { background:#9c2a2a; }
QComboBox,QDoubleSpinBox,QSpinBox,QLineEdit { background:#0c131d; border:1px solid #2d4057; border-radius:3px; padding:5px; color:#b9e7f7; }
QPlainTextEdit { background:#0a1017; border:1px solid #26364a; color:#9fd2e8; font:12px 'SF Mono','Menlo',monospace; }
QGroupBox { border:1px solid #26364a; border-radius:4px; margin-top:14px; padding:10px 8px 8px 8px; }
QGroupBox::title { subcontrol-origin:margin; left:10px; color:#76d5f7; font-weight:700; letter-spacing:1px; }
QTabWidget::pane { border:1px solid #26364a; }
QTabBar::tab { padding:9px 16px; background:#172333; }
QTabBar::tab:selected { background:#29435e; color:#76d5f7; }
QLabel#section { color:#76d5f7; font-weight:700; letter-spacing:1px; }
QLabel#mono { font:12px 'SF Mono','Menlo',monospace; color:#9fd2e8; }
QLabel#banner { padding:8px; border-radius:4px; font-weight:700; }
QSlider::groove:horizontal { height:4px; background:#2d4057; }
QSlider::handle:horizontal { background:#76d5f7; width:14px; margin:-6px 0; border-radius:7px; }
QSlider::groove:vertical { width:4px; background:#2d4057; }
QSlider::handle:vertical { background:#76d5f7; height:14px; margin:0 -6px; border-radius:7px; }
QStatusBar { border-top:1px solid #26364a; }
QToolTip { background:#172333; color:#d6e0ec; border:1px solid #38516d; }
"""
