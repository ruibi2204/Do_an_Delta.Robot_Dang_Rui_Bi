import sys
import os
import tkinter as tk

# Thêm thư mục hiện tại vào sys.path để import các module
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def check_dependencies() -> bool:
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

    try:
        import serial  # noqa: F401
    except ImportError:
        print("[WARN] pyserial chưa cài — UART sẽ chạy ở chế độ DRY-RUN")
        print("       Cài bằng: pip install pyserial")

    # Kiểm tra thư viện camera (không bắt buộc)
    try:
        import cv2        # noqa: F401
        import PIL        # noqa: F401
        print("[INFO] Camera: opencv-python + pillow sẵn sàng ✓")
    except ImportError:
        print("[WARN] opencv-python / pillow chưa cài — tab Camera sẽ bị vô hiệu")
        print("       Cài bằng: pip install opencv-python pillow")

    return True


def _apply_ttk_style(root: tk.Tk) -> None:
    """Áp dụng theme TTK — giữ nguyên từ main gốc."""
    if not _has_ttk():
        return
    from tkinter import ttk
    style = ttk.Style(root)
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


def main():
    if not check_dependencies():
        sys.exit(1)

    # Import GUI đã sửa để vẽ đường thẳng (file gui_control.py)
    from gui_control import DeltaRobotGUI

    root = tk.Tk()
    _apply_ttk_style(root)

    app = DeltaRobotGUI(root)

    def on_close():
        # Dừng camera trước (nếu đang chạy)
        if hasattr(app, "_camera_running") and app._camera_running:
            app._stop_camera()

        # Dừng trajectory loop
        app._stop()

        # Ngắt UART
        if app._uart and app._uart.is_connected:
            app._uart.disconnect()

        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    print("=" * 50)
    print("  DELTA ROBOT — LINE DRAW CONTROLLER")
    print("  Vẽ đường thẳng giữa hai điểm đã chọn.")
    print("  Tab [QUỸ ĐẠO] — hiển thị quỹ đạo mô phỏng")
    print("  Tab [CAMERA]  — quan sát robot vẽ thực tế")
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
    try:
        from tkinter import ttk  # noqa: F401
    except ImportError:
        pass
    main()