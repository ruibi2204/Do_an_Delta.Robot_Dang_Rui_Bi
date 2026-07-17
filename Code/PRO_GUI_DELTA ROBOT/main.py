# ============================================================================
# FILE: main.py
# ============================================================================
"""
main.py - File chạy chính cho chương trình điều khiển robot Delta
Tích hợp tất cả các module: giao diện, điều khiển, camera, quỹ đạo
"""

import sys
import os
import traceback

# Thêm đường dẫn hiện tại vào sys.path để import các module
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PyQt5.QtWidgets import QApplication, QMessageBox, QSplashScreen
from PyQt5.QtGui import QFont, QPixmap
from PyQt5.QtCore import Qt, QTimer

# Import các module cần thiết
from main_gui import MainWindow
from Math_Control.gear_ratio import GEAR_RATIO

# Kiểm tra các thư viện cần thiết
try:
    import serial

    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

try:
    import cv2

    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    import numpy as np

    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False


class Application:
    """Lớp quản lý ứng dụng chính"""

    def __init__(self):
        self.app = None
        self.splash = None
        self.main_window = None

    def check_dependencies(self) -> bool:
        """
        Kiểm tra các thư viện phụ thuộc

        Returns:
            bool: True nếu tất cả đều có, False nếu thiếu
        """
        missing = []

        if not HAS_SERIAL:
            missing.append("pyserial")
        if not HAS_CV2:
            missing.append("opencv-python")
        if not HAS_NUMPY:
            missing.append("numpy")

        if missing:
            msg = "Thiếu các thư viện sau:\n\n"
            for lib in missing:
                msg += f"  - {lib}\n"
            msg += f"\nCài đặt bằng lệnh:\n"
            msg += f"pip install {' '.join(missing)}"

            QMessageBox.critical(None, "Thiếu thư viện", msg)
            return False

        return True

    def show_splash(self):
        """Hiển thị màn hình chào"""
        splash_pixmap = QPixmap()
        # Nếu có file splash.png thì hiển thị, không thì tạo màn hình chào đơn giản
        splash_path = os.path.join(os.path.dirname(__file__), "splash.png")
        if os.path.exists(splash_path):
            splash_pixmap.load(splash_path)
            splash_pixmap = splash_pixmap.scaled(600, 400, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        else:
            # Tạo splash đơn giản nếu không có file ảnh
            from PyQt5.QtGui import QPainter, QColor
            splash_pixmap = QPixmap(600, 400)
            splash_pixmap.fill(QColor(30, 30, 60))
            painter = QPainter(splash_pixmap)
            painter.setPen(Qt.white)
            painter.setFont(QFont("Arial", 24, QFont.Bold))
            painter.drawText(splash_pixmap.rect(), Qt.AlignCenter, "ROBOT DELTA\nĐang khởi động...")
            painter.end()

        self.splash = QSplashScreen(splash_pixmap, Qt.WindowStaysOnTopHint)
        self.splash.show()
        self.splash.showMessage(
            f"Đang khởi động...\nGear Ratio: u = {GEAR_RATIO}\n",
            Qt.AlignBottom | Qt.AlignCenter,
            Qt.white
        )
        self.app.processEvents()

    def hide_splash(self):
        """Ẩn màn hình chào"""
        if self.splash:
            self.splash.finish(self.main_window)
            self.splash = None

    def run(self):
        """Chạy ứng dụng"""
        # Tạo QApplication
        self.app = QApplication(sys.argv)
        self.app.setFont(QFont("Segoe UI", 9))
        self.app.setApplicationName("Delta Robot Controller")
        self.app.setApplicationVersion("1.0.0")
        self.app.setOrganizationName("Mechatronics")

        # Hiển thị splash
        self.show_splash()

        # Kiểm tra dependencies
        if not self.check_dependencies():
            self.splash.close()
            sys.exit(1)

        # Cập nhật splash
        if self.splash:
            self.splash.showMessage(
                "Đang khởi tạo giao diện...\n",
                Qt.AlignBottom | Qt.AlignCenter,
                Qt.white
            )
            self.app.processEvents()

        # Tạo cửa sổ chính
        try:
            self.main_window = MainWindow()

            # Cập nhật splash
            if self.splash:
                self.splash.showMessage(
                    "Đã sẵn sàng!\n",
                    Qt.AlignBottom | Qt.AlignCenter,
                    Qt.green
                )
                self.app.processEvents()

            # Hiển thị cửa sổ chính
            self.main_window.show()

            # Ẩn splash sau 500ms
            QTimer.singleShot(500, self.hide_splash)

        except Exception as e:
            # Đóng splash và hiển thị lỗi
            if self.splash:
                self.splash.close()

            QMessageBox.critical(
                None,
                "Lỗi khởi tạo",
                f"Không thể khởi tạo ứng dụng:\n\n{str(e)}\n\n{traceback.format_exc()}"
            )
            sys.exit(1)

        # Chạy vòng lặp sự kiện
        try:
            sys.exit(self.app.exec_())
        except Exception as e:
            print(f"Lỗi khi chạy ứng dụng: {e}")
            traceback.print_exc()
            sys.exit(2)


def create_desktop_entry():
    """
    Tạo file .desktop cho Linux hoặc shortcut cho Windows
    (Chỉ chạy khi được gọi)
    """
    import platform

    if platform.system() == "Windows":
        # Tạo shortcut Windows
        try:
            import winshell
            from win32com.client import Dispatch

            desktop = winshell.desktop()
            path = os.path.join(desktop, "Delta Robot Controller.lnk")
            target = sys.executable
            wdir = os.path.dirname(os.path.abspath(__file__))
            icon = os.path.join(wdir, "icon.ico")

            shell = Dispatch('WScript.Shell')
            shortcut = shell.CreateShortCut(path)
            shortcut.Targetpath = target
            shortcut.Arguments = f'"{os.path.join(wdir, "main.py")}"'
            shortcut.WorkingDirectory = wdir
            if os.path.exists(icon):
                shortcut.IconLocation = icon
            shortcut.save()
            print(f"Đã tạo shortcut tại: {path}")
        except Exception as e:
            print(f"Không thể tạo shortcut: {e}")

    elif platform.system() == "Linux":
        # Tạo .desktop file
        desktop = os.path.expanduser("~/Desktop")
        path = os.path.join(desktop, "Delta_Robot_Controller.desktop")

        content = f"""[Desktop Entry]
Version=1.0
Type=Application
Name=Delta Robot Controller
Comment=Điều khiển robot Delta
Exec=python3 "{os.path.abspath(__file__)}"
Icon={os.path.join(os.path.dirname(os.path.abspath(__file__)), "icon.png")}
Terminal=false
Categories=Engineering;Robotics;
"""
        try:
            with open(path, 'w') as f:
                f.write(content)
            os.chmod(path, 0o755)
            print(f"Đã tạo desktop entry tại: {path}")
        except Exception as e:
            print(f"Không thể tạo desktop entry: {e}")


def print_banner():
    """In banner khi khởi động từ terminal"""
    banner = """
    ╔═══════════════════════════════════════════════════════════════╗
    ║                                                               ║
    ║     ██████╗ ███████╗██╗  ████████╗ █████╗                     ║
    ║     ██╔══██╗██╔════╝██║  ╚══██╔══╝██╔══██╗                    ║
    ║     ██║  ██║█████╗  ██║     ██║   ███████║                    ║
    ║     ██║  ██║██╔══╝  ██║     ██║   ██╔══██║                    ║
    ║     ██████╔╝███████╗███████╗██║   ██║  ██║                    ║
    ║     ╚═════╝ ╚══════╝╚══════╝╚═╝   ╚═╝  ╚═╝                    ║
    ║                                                               ║
    ║              ROBOT DELTA - ĐIỀU KHIỂN & VISION                ║
    ║                                                               ║
    ║       Gear Ratio: u = {:<6}                                   ║
    ║       Version: 1.0.0                                          ║
    ║                                                               ║
    ╚═══════════════════════════════════════════════════════════════╝
    """.format(GEAR_RATIO)
    print(banner)

def main():
    """Hàm main chính"""
    # In banner nếu chạy từ terminal
    if sys.stdout.isatty():
        print_banner()

    # Kiểm tra và tạo shortcut nếu cần
    if len(sys.argv) > 1 and sys.argv[1] == "--create-shortcut":
        create_desktop_entry()
        return

    # Chạy ứng dụng
    app = Application()
    app.run()


if __name__ == "__main__":
    main()