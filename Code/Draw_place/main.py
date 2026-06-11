# ============================================================
#  main.py — Entry Point Robot Delta Trajectory Controller
# ============================================================
"""
Chạy chương trình:
    python main.py

Yêu cầu:
    pip install numpy pyserial     (pyserial tùy chọn nếu không dùng UART thật)

Cấu trúc dự án:
    delta_robot/
    ├── main.py
    ├── kinematics/
    │   ├── __init__.py
    │   └── inverse_kinematics.py   ← Động học ngược Delta
    ├── trajectory/
    │   ├── __init__.py
    │   └── generator.py            ← Sinh quỹ đạo (tròn/vuông/tam giác)
    ├── communication/
    │   ├── __init__.py
    │   └── uart_comm.py            ← Gửi lệnh UART xuống Arduino
    ├── PID_control/
    │   ├── __init__.py
    │   └── PID_control.py          ← Bộ điều khiển PID 3 khớp
    └── gui/
        ├── __init__.py
        └── controller_gui.py       ← Giao diện Tkinter tích hợp PID
"""

import sys
import os
import tkinter as tk

# Đảm bảo Python tìm thấy các module con khi chạy từ thư mục khác
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def check_dependencies() -> bool:
    """Kiểm tra thư viện cần thiết trước khi chạy."""
    missing = []

    try:
        import numpy  # noqa: F401
    except ImportError:
        missing.append("numpy")

    if missing:
        print("=" * 50)
        print("THIẾU THƯ VIỆN! Hãy cài bằng lệnh:")
        print(f"    pip install {' '.join(missing)}")
        print("=" * 50)
        return False

    # pyserial là tùy chọn — thiếu thì tự động dùng DRY-RUN
    try:
        import serial  # noqa: F401
    except ImportError:
        print("[WARN] pyserial chưa cài — UART sẽ chạy ở chế độ DRY-RUN")
        print("       Cài bằng: pip install pyserial")

    return True


def main():
    if not check_dependencies():
        sys.exit(1)

    # Import sau khi đã kiểm tra dependency
    from gui.controller_gui import DeltaRobotGUI

    root = tk.Tk()

    # Style ttk chung (scrollbar, combobox, progressbar)
    style = ttk.Style(root) if _has_ttk() else None
    if style:
        try:
            style.theme_use("clam")
            style.configure("TProgressbar",
                            troughcolor="#21262d",
                            background="#58a6ff",
                            darkcolor="#58a6ff",
                            lightcolor="#58a6ff",
                            bordercolor="#30363d")
            style.configure("TCombobox",
                            fieldbackground="#161b22",
                            background="#21262d",
                            foreground="#e6edf3",
                            selectbackground="#21262d",
                            selectforeground="#e6edf3")
        except Exception:
            pass

    app = DeltaRobotGUI(root)

    # Xử lý thoát an toàn
    def on_close():
        app._stop()
        if app._uart and app._uart.is_connected:
            app._uart.disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    print("=" * 50)
    print("  DELTA ROBOT TRAJECTORY CONTROLLER")
    print("  Nhấn Ctrl+C tại terminal để thoát")
    print("=" * 50)

    root.mainloop()


def _has_ttk() -> bool:
    try:
        from tkinter import ttk  # noqa: F401
        return True
    except ImportError:
        return False


if __name__ == "__main__":
    # Import ttk ở đây để dùng trong main()
    try:
        from tkinter import ttk
    except ImportError:
        ttk = None

    main()
