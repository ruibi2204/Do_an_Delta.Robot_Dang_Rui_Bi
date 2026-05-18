# ============================================================
#  main.py — Chương trình chính Robot Delta
# ============================================================
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import logging
from PyQt5.QtWidgets import QApplication
from gui.control_panel import ControlPanel

# ------ Cấu hình logging ------
logging.basicConfig(
    level   = logging.DEBUG,
    format  = "[%(levelname)s] %(name)s: %(message)s",
)

def main():
    app    = QApplication(sys.argv)
    window = ControlPanel()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
