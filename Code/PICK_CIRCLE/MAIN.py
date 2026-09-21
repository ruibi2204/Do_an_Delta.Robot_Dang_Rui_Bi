# main.py
import sys
# Thêm vào đầu main.py, ngay dưới các import
import os

def resource_path(relative_path: str) -> str:
    """Trả về đường dẫn tuyệt đối tới file, hoạt động đúng cả khi chạy .py
    lẫn khi đã đóng gói bằng PyInstaller (--onefile)."""
    base_path = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_path, relative_path)

from PySide6.QtCore import Qt
from PySide6.QtGui import QDoubleValidator, QPixmap, QPainter, QFont
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QFrame, QLineEdit, QGroupBox, QDialog,
)

from windows.manual_window import ManualWindow
from windows.camera_window import CameraWindow
from windows.csv_window import CsvWindow
from windows.dynamic_window import DynamicWindow
from windows.dynamic_run4dof_window import DynamicRun4DofWindow   # nút mới
from windows.connection_window import ConnectionWindow
from windows.repeat_test_window import RepeatTestWindow           # nút đánh giá độ lặp lại
from windows.draw_window import DrawWindow                        # nút vẽ hình
from windows.board_calib_window import BoardCalibWindow            # nút hiệu chỉnh camera bằng bàn cờ
from shared_state import AppContext, apply_light_palette, STYLE_SHEET, APP_FONT_FAMILY


# =====================================================================
# GIAO DIỆN MÀN HÌNH CHÍNH (LAUNCHER) - PHIÊN BẢN GỌN, 4 NÚT CHÍNH
# ---------------------------------------------------------------------
# Toàn bộ các nút điều hướng trước đây nằm rải rác thành 3 cột (menu bên
# trái, khung tham số động học ở giữa, khung đánh giá robot bên phải) nay
# được GOM LẠI thành đúng 4 nút bấm chính, bố trí cân đối, canh giữa màn
# hình, theo phong cách "bảng điều khiển" chuyên nghiệp:
#
#   1) "MENU CHỨC NĂNG"     -> mở hộp thoại chứa 5 nút điều hướng cũ
#                               (Manual, Camera..., Static..., v.v.)
#   2) "THAM SỐ ĐỘNG HỌC"   -> mở hộp thoại nhập 4 tham số cơ khí (R, r, a, b)
#   3) "ĐÁNH GIÁ CƠ KHÍ"    -> mở hộp thoại chứa 3 nút đánh giá robot
#                               (Repeat, Draw, Camera Calibration)
#   4) "KẾT NỐI"             -> giữ nguyên hành vi cũ, mở ConnectionWindow
#
# Khung "THAM SỐ TỐC ĐỘ" đã được GỠ BỎ hoàn toàn khỏi giao diện theo yêu
# cầu (giá trị mặc định vẫn được AppContext khởi tạo ở shared_state.py để
# không phá vỡ các phần khác của chương trình đọc ctx.speed_params).
# =====================================================================


# Đường dẫn ảnh dùng làm HÌNH NỀN MỜ (watermark) chìm phía sau toàn bộ
# giao diện màn hình chính. Đổi lại đường dẫn này cho khớp với ảnh thực tế
# của bạn (logo trường, sơ đồ động học robot Delta...).
KIN_PANEL_IMAGE_PATH = "delta.png"

# Độ mờ (0.0 = trong suốt hoàn toàn, 1.0 = đậm như ảnh gốc). Giá trị nhỏ để
# ảnh chỉ "chìm" nhẹ phía sau, không che chữ/nút phía trên.
WATERMARK_OPACITY = 0.08

# Ảnh LOGO (ví dụ logo Trường ĐHKTCN Cần Thơ) - đặt cố định ở GÓC TRÊN BÊN
# PHẢI màn hình chính, hiển thị RÕ (không mờ, khác với ảnh watermark ở
# trên). Đổi tên file này cho khớp với ảnh logo thực tế của bạn.
LOGO_IMAGE_PATH = "CT.png"
LOGO_MAX_HEIGHT = 130   # chiều cao tối đa của logo (px), giữ nguyên tỉ lệ
LOGO_MARGIN = 18        # khoảng cách từ logo tới mép trên/phải cửa sổ (px)

# ---------------------------------------------------------------------
# 3 NÚT NHỎ xếp NGANG trên 1 hàng (Tham số động học / Đánh giá cơ khí /
# Menu chức năng) - nhỏ hơn để vừa bày theo chiều ngang.
# ---------------------------------------------------------------------
TOP_BTN_WIDTH  = 280
TOP_BTN_HEIGHT = 64
TOP_BTN_SPACING = 24

# Giữ tên biến cũ để tương thích với phần còn lại của file.
MAIN_BTN_WIDTH  = TOP_BTN_WIDTH
MAIN_BTN_HEIGHT = TOP_BTN_HEIGHT
MAIN_BTN_SPACING = TOP_BTN_SPACING
COLUMN_BTN_WIDTH  = TOP_BTN_WIDTH
COLUMN_BTN_HEIGHT = TOP_BTN_HEIGHT
COLUMN_BTN_SPACING = TOP_BTN_SPACING

MAIN_BTN_STYLE = """
QPushButton#mainMenuBtn {
    background-color: #0078d4;
    color: #ffffff;
    border: 2px solid #005fa8;
    border-radius: 14px;
    padding: 8px 14px;
    font-weight: 700;
    font-size: 12pt;
}
QPushButton#mainMenuBtn:hover {
    background-color: #005fa8;
}
QPushButton#mainMenuBtn:pressed {
    background-color: #00477e;
}
"""

# Biến dự phòng (không còn dùng nút kết nối độc lập nữa)
CONNECT_BTN_WIDTH  = TOP_BTN_WIDTH
CONNECT_BTN_HEIGHT = TOP_BTN_HEIGHT

CONNECT_BTN_STYLE = MAIN_BTN_STYLE.replace("mainMenuBtn", "connectMenuBtn")

# Style dùng cho các nút BÊN TRONG các hộp thoại con (Menu chức năng, Đánh
# giá cơ khí) - cùng tông xanh dương với 4 nút chính, chỉ nhỏ hơn.
SUB_BTN_WIDTH = 460
SUB_BTN_HEIGHT = 80
SUB_BTN_SPACING = 12

SUB_BTN_STYLE = """
QPushButton#subMenuBtn {
    background-color: #0078d4;
    color: #ffffff;
    border: 2px solid #005fa8;
    border-radius: 14px;
    padding: 8px 12px;
    font-weight: 700;
    font-size: 13pt;
    text-align: center;
}
QPushButton#subMenuBtn:hover {
    background-color: #005fa8;
}
QPushButton#subMenuBtn:pressed {
    background-color: #00477e;
}
"""

# Nút "Đóng" dùng chung cho các hộp thoại con.
DIALOG_CLOSE_BTN_STYLE = """
QPushButton#dialogCloseBtn {
    background-color: #e0e0e0;
    color: #1a1a1a;
    border: 2px solid #b0b0b0;
    border-radius: 12px;
    padding: 8px 20px;
    font-weight: 700;
    font-size: 12pt;
}
QPushButton#dialogCloseBtn:hover {
    background-color: #d0d0d0;
    border-color: #757575;
}
"""

# Kích thước riêng cho nút "Lưu tham số" trong hộp thoại THAM SỐ ĐỘNG HỌC.
KIN_APPLY_BTN_WIDTH = 260
KIN_APPLY_BTN_HEIGHT = 52

# Danh sách 5 mục điều hướng trước đây nằm ở cột trái, nay đưa vào hộp
# thoại "MENU CHỨC NĂNG". Tên nút đã dịch sang tiếng Việt, VIẾT HOA toàn bộ
# để đồng bộ với 4 nút chính. "Bù trừ tọa độ" là thuật ngữ kỹ thuật chuyên
# nghiệp hơn cho "offset tọa độ".
NAV_MENU_ITEMS = [
    ("ĐIỀU KHIỂN THỦ CÔNG", "manual", ManualWindow),
    ("CAMERA && BÙ TRỪ TỌA ĐỘ", "camera", CameraWindow),
    ("GẮP THẢ TĨNH", "csv", CsvWindow),
    ("GẮP THẢ TĨNH 4 BẬC TỰ DO", "dynamic", DynamicWindow),
    ("GẮP THẢ ĐỘNG", "dynamic_run4dof", DynamicRun4DofWindow),
]

# Danh sách 3 mục đánh giá cơ khí, đưa vào hộp thoại "ĐÁNH GIÁ CƠ KHÍ".
# Tên nút đã dịch sang tiếng Việt, VIẾT HOA toàn bộ.
ROBOT_EVAL_ITEMS = [
    ("ĐÁNH GIÁ ĐỘ LẶP LẠI", "repeat_test", RepeatTestWindow),
    ("VẼ HÌNH", "draw", DrawWindow),
    ("HIỆU CHỈNH CAMERA", "board_calib", BoardCalibWindow),
]


class WatermarkCentralWidget(QWidget):
    """QWidget trung tâm tự vẽ một ảnh mờ (watermark) làm nền, nằm CHÌM phía
    sau toàn bộ các khung/nút của giao diện chính. Vì việc vẽ nền diễn ra
    trong paintEvent của chính widget cha, các widget con (thêm qua layout)
    luôn được Qt vẽ chồng lên sau đó - nên không cần thay đổi gì ở phần bố
    cục hiện có (menu, khung tham số...), ảnh sẽ tự nằm phía dưới cùng."""

    def __init__(self, image_path: str, opacity: float = 0.08, scale: float = 0.6,
                 logo_path: str = None, logo_max_height: int = 92, logo_margin: int = 18,
                 parent=None):
        super().__init__(parent)
        self._pixmap = QPixmap(image_path)
        self._opacity = opacity
        self._scale = scale  # 1.0 = phủ kín cửa sổ, nhỏ hơn 1.0 = ảnh nhỏ lại, canh giữa

        # ---- Logo góc trên bên phải: hiển thị RÕ NÉT (không áp dụng độ mờ
        # watermark), luôn nằm TRÊN CÙNG (đè lên mọi widget con khác) và
        # được canh lại vị trí mỗi khi cửa sổ đổi kích thước (resizeEvent).
        self._logo_margin = logo_margin
        self._logo_label = None
        if logo_path:
            logo_pixmap = QPixmap(logo_path)
            if not logo_pixmap.isNull():
                scaled_logo = logo_pixmap.scaledToHeight(
                    logo_max_height, Qt.SmoothTransformation
                )
                self._logo_label = QLabel(self)
                self._logo_label.setPixmap(scaled_logo)
                self._logo_label.setFixedSize(scaled_logo.size())
                self._logo_label.raise_()

    def paintEvent(self, event):
        if not self._pixmap.isNull() and self.width() > 0 and self.height() > 0:
            painter = QPainter(self)
            painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
            painter.setOpacity(self._opacity)
            # target_size theo _scale để chỉnh ảnh to/nhỏ theo ý muốn, vẫn
            # giữ tỉ lệ gốc và canh giữa cửa sổ.
            target_size = self.size() * self._scale
            scaled = self._pixmap.scaled(
                target_size, Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            x = (self.width() - scaled.width()) // 2
            y = (self.height() - scaled.height()) // 2
            painter.drawPixmap(x, y, scaled)
            painter.end()
        super().paintEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._logo_label is not None:
            x = self.width() - self._logo_label.width() - self._logo_margin
            y = self._logo_margin
            self._logo_label.move(max(0, x), y)
            self._logo_label.raise_()


def _make_sub_menu_button(text: str) -> QPushButton:
    """Tạo 1 nút điều hướng dùng bên trong các hộp thoại con (Menu chức
    năng / Đánh giá cơ khí) - kích thước + màu sắc đồng bộ."""
    btn = QPushButton(text)
    btn.setObjectName("subMenuBtn")
    btn.setStyleSheet(SUB_BTN_STYLE)
    btn.setFixedWidth(SUB_BTN_WIDTH)
    btn.setFixedHeight(SUB_BTN_HEIGHT)
    return btn


def _make_dialog_close_button(dialog: QDialog) -> QPushButton:
    btn = QPushButton("ĐÓNG")
    btn.setObjectName("dialogCloseBtn")
    btn.setStyleSheet(DIALOG_CLOSE_BTN_STYLE)
    btn.setFixedSize(140, 46)
    btn.clicked.connect(dialog.reject)
    return btn


class LauncherWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROBOT DELTA 4 BẬC TỰ DO - ĐHKTCN CẦN THƠ")

        self.ctx = AppContext()
        self._windows = {}

        # Tham chiếu widget nhập tham số động học - được tạo bên trong hộp
        # thoại "THAM SỐ ĐỘNG HỌC", cần giữ lại để đọc/ghi giá trị.
        self._kin_edits = {}
        self._kin_indicators = {}
        self._kin_dialog = None

        self._nav_dialog = None
        self._eval_dialog = None

        self._conn_win_embed = None  # sẽ được tạo trong _build_ui
        self._build_ui()

    # -----------------------------------------------------------------
    # XÂY DỰNG GIAO DIỆN CHÍNH: header + 4 nút bấm, canh giữa màn hình
    # -----------------------------------------------------------------
    def _build_ui(self):
        # Central widget vẫn là WatermarkCentralWidget để ảnh delta.png nằm
        # chìm làm nền mờ phía sau toàn bộ giao diện.
        central = WatermarkCentralWidget(
            resource_path(KIN_PANEL_IMAGE_PATH), opacity=WATERMARK_OPACITY, scale=1,
            logo_path=resource_path(LOGO_IMAGE_PATH),
            logo_max_height=LOGO_MAX_HEIGHT, logo_margin=LOGO_MARGIN,
        )
        self.setCentralWidget(central)

        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(60, 40, 60, 40)
        main_layout.setSpacing(28)

        # ---------- Header: tên trường + niên khóa + sinh viên + đề tài,
        #            canh giữa trên toàn bề ngang màn hình ----------
        header = self._build_header()
        main_layout.addLayout(header)

        # ---------- Thân trang: 3 nút nhỏ trên đầu + ConnectionWindow nhúng
        #            chiếm toàn bộ phần còn lại của màn hình ----------
        control_panel = self._build_control_panel()
        main_layout.addLayout(control_panel, 1)


    def _build_header(self):
        """Khối tiêu đề trên cùng: canh giữa theo TOÀN BỀ NGANG màn hình.
        4 dòng theo đúng yêu cầu:
          1) Tên trường
          2) Niên khóa
          3) Sinh viên thực hiện
          4) Tên đề tài
        """
        header = QVBoxLayout()
        header.setSpacing(4)
        header.setContentsMargins(0, 0, 0, 0)

        title1 = QLabel("TRƯỜNG ĐẠI HỌC KỸ THUẬT - CÔNG NGHỆ CẦN THƠ")
        title1.setObjectName("menuTitle1")
        title1.setAlignment(Qt.AlignCenter)
        title1.setWordWrap(True)
        header.addWidget(title1)

        title2 = QLabel("NIÊN KHÓA 2022")
        title2.setObjectName("menuTitle2")
        title2.setAlignment(Qt.AlignCenter)
        header.addWidget(title2)

        title3 = QLabel("SINH VIÊN THỰC HIỆN: ĐẶNG RUI BI")
        title3.setObjectName("menuTitle3")
        title3.setAlignment(Qt.AlignCenter)
        header.addWidget(title3)

        title4 = QLabel(
            "ĐỀ TÀI: THIẾT KẾ CHẾ TẠO ROBOT DELTA ỨNG DỤNG HỌC TẬP VÀ NGHIÊN CỨU"
        )
        title4.setObjectName("menuTitle4")
        title4.setAlignment(Qt.AlignCenter)
        title4.setWordWrap(True)
        header.addWidget(title4)

        header.addSpacing(10)
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet("color:#c0c0c0;")
        header.addWidget(line)

        return header

    # -----------------------------------------------------------------
    # BẢNG ĐIỀU KHIỂN: 4 nút xếp thành MỘT HÀNG DỌC, canh giữa màn hình.
    # Thứ tự từ trên xuống: KẾT NỐI -> THAM SỐ ĐỘNG HỌC -> ĐÁNH GIÁ CƠ KHÍ
    # -> MENU CHỨC NĂNG. Cả 4 nút cùng kích thước để cột thẳng hàng, nhìn
    # cân đối, chuyên nghiệp.
    # -----------------------------------------------------------------
    def _build_control_panel(self):
        """Bảng điều khiển:
          - Hàng trên: 3 nút nhỏ xếp NGANG đều (Tham số ĐH / Đánh giá CK / Menu)
          - Bên dưới: khung ConnectionWindow nhúng trực tiếp (embed)
        """
        outer = QVBoxLayout()
        outer.setSpacing(14)

        # ---- Hàng 3 nút nhỏ trải đều trái → phải ----
        btn_row = QHBoxLayout()
        btn_row.setSpacing(TOP_BTN_SPACING)
        btn_row.addStretch(1)

        btn_kin = self._make_main_button("THAM SỐ ĐỘNG HỌC")
        btn_kin.clicked.connect(self._open_kinematics_dialog)
        btn_row.addWidget(btn_kin)

        btn_eval = self._make_main_button("ĐÁNH GIÁ CƠ KHÍ")
        btn_eval.clicked.connect(self._open_robot_eval_menu)
        btn_row.addWidget(btn_eval)

        btn_menu = self._make_main_button("MENU CHỨC NĂNG")
        btn_menu.clicked.connect(self._open_nav_menu)
        btn_row.addWidget(btn_menu)

        btn_row.addStretch(1)
        outer.addLayout(btn_row)

        # ---- Nhúng ConnectionWindow trực tiếp (không có khung bên ngoài) ----
        self._conn_win_embed = ConnectionWindow(self.ctx, self)
        self._conn_win_embed.setWindowFlags(Qt.Widget)
        self._conn_win_embed.btn_back.hide()
        outer.addWidget(self._conn_win_embed, 1)

        return outer

    def _make_main_button(self, text: str) -> QPushButton:
        btn = QPushButton(text)
        btn.setObjectName("mainMenuBtn")
        btn.setStyleSheet(MAIN_BTN_STYLE)
        btn.setFixedWidth(MAIN_BTN_WIDTH)
        btn.setFixedHeight(MAIN_BTN_HEIGHT)
        return btn

    # -----------------------------------------------------------------
    # (1) HỘP THOẠI "MENU CHỨC NĂNG" - gom 5 nút điều hướng cũ
    # -----------------------------------------------------------------
    def _open_nav_menu(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("Menu chức năng")
        layout = QVBoxLayout(dialog)
        layout.setContentsMargins(28, 24, 28, 24)
        layout.setSpacing(SUB_BTN_SPACING)

        title = QLabel("CHỌN CHỨC NĂNG")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 15pt; font-weight: 800; color: #0078d4;")
        layout.addWidget(title)
        layout.addSpacing(6)

        for text, key, cls in NAV_MENU_ITEMS:
            btn = _make_sub_menu_button(text)
            btn.clicked.connect(lambda checked=False, k=key, c=cls: self._nav_and_close(dialog, k, c))
            layout.addWidget(btn)

        layout.addSpacing(8)
        close_row = QHBoxLayout()
        close_row.addStretch(1)
        close_row.addWidget(_make_dialog_close_button(dialog))
        close_row.addStretch(1)
        layout.addLayout(close_row)

        self._nav_dialog = dialog
        dialog.exec()

    # -----------------------------------------------------------------
    # (3) HỘP THOẠI "ĐÁNH GIÁ CƠ KHÍ" - gom 3 nút Repeat/Draw/Camera Calib
    # -----------------------------------------------------------------
    def _open_robot_eval_menu(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("Đánh giá cơ khí")
        layout = QVBoxLayout(dialog)
        layout.setContentsMargins(28, 24, 28, 24)
        layout.setSpacing(SUB_BTN_SPACING)

        title = QLabel("ĐÁNH GIÁ CƠ KHÍ ROBOT")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 15pt; font-weight: 800; color: #0078d4;")
        layout.addWidget(title)
        layout.addSpacing(6)

        for text, key, cls in ROBOT_EVAL_ITEMS:
            btn = _make_sub_menu_button(text)
            btn.clicked.connect(lambda checked=False, k=key, c=cls: self._nav_and_close(dialog, k, c))
            layout.addWidget(btn)

        layout.addSpacing(8)
        close_row = QHBoxLayout()
        close_row.addStretch(1)
        close_row.addWidget(_make_dialog_close_button(dialog))
        close_row.addStretch(1)
        layout.addLayout(close_row)

        self._eval_dialog = dialog
        dialog.exec()

    def _nav_and_close(self, dialog: QDialog, key: str, window_cls):
        dialog.accept()
        self._open_tab(key, window_cls)

    # -----------------------------------------------------------------
    # (2) HỘP THOẠI "THAM SỐ ĐỘNG HỌC" - 4 tham số cơ khí R, r, a, b
    # -----------------------------------------------------------------
    def _open_kinematics_dialog(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("Tham số động học")
        layout = QVBoxLayout(dialog)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(16)

        group = self._build_kinematics_group()
        layout.addWidget(group)

        close_row = QHBoxLayout()
        close_row.addStretch(1)
        close_row.addWidget(_make_dialog_close_button(dialog))
        close_row.addStretch(1)
        layout.addLayout(close_row)

        self._kin_dialog = dialog
        dialog.exec()

    def _build_kinematics_group(self):
        """Panel THAM SỐ ĐỘNG HỌC - nhập 4 tham số động học nghịch: R, r, a, b —
        có đèn báo hợp lệ (xanh/xám). Được đặt bên trong hộp thoại riêng,
        mở ra khi nhấn nút "THAM SỐ ĐỘNG HỌC" ở màn hình chính."""
        group = QGroupBox("Kinematics")
        group.setObjectName("kinGroup")
        group.setAlignment(Qt.AlignCenter)  # canh giữa chữ tiêu đề khung
        group.setMinimumWidth(480)
        group.setStyleSheet("QGroupBox#kinGroup { font-size: 16pt; font-weight: 700; }")
        grid = QGridLayout(group)
        grid.setContentsMargins(28, 36, 28, 28)
        grid.setHorizontalSpacing(18)
        grid.setVerticalSpacing(22)

        defaults = getattr(self.ctx, "kin_params", None) or {
            "R": 122.0, "r": 40.0, "a": 130.0, "b": 298.0,
        }

        self._kin_edits = {}
        self._kin_indicators = {}
        validator = QDoubleValidator(-100000.0, 100000.0, 4)
        validator.setNotation(QDoubleValidator.StandardNotation)

        labels = [
            ("R", "Fixed Base (R, mm)"),
            ("r", "End Effector (r, mm)"),
            ("a", "Active Arm (a, mm)"),
            ("b", "Passive Arm (b, mm)"),
        ]

        for row, (key, desc) in enumerate(labels):
            lbl = QLabel(desc)
            lbl.setStyleSheet("font-size: 14pt;")

            edit = QLineEdit(str(defaults[key]))
            edit.setValidator(validator)
            edit.setObjectName("kinEdit")
            edit.setMinimumHeight(48)
            edit.setStyleSheet("QLineEdit#kinEdit { font-size: 14pt; padding: 6px 10px; }")
            edit.textChanged.connect(lambda text, k=key: self._validate_kin_field(k))
            edit.editingFinished.connect(self._save_kin_params)
            self._kin_edits[key] = edit

            dot = QLabel()
            dot.setFixedSize(15, 15)
            dot.setObjectName("kinDot")
            self._kin_indicators[key] = dot

            field_row = QHBoxLayout()
            field_row.setSpacing(10)
            field_row.addWidget(edit)
            field_row.addWidget(dot)

            grid.addWidget(lbl, row, 0)
            grid.addLayout(field_row, row, 1)

        apply_btn = QPushButton("LƯU")
        apply_btn.setObjectName("kinApplyBtn")
        apply_btn.setFixedWidth(KIN_APPLY_BTN_WIDTH)
        apply_btn.setFixedHeight(KIN_APPLY_BTN_HEIGHT)
        apply_btn.setStyleSheet(
            "QPushButton#kinApplyBtn {"
            "  background-color: #0078d4; color: #ffffff;"
            "  border: 2px solid #005fa8; border-radius: 16px;"
            "  padding: 10px; font-weight: 700; font-size: 14pt;"
            "}"
            "QPushButton#kinApplyBtn:hover { background-color: #005fa8; }"
            "QPushButton#kinApplyBtn:pressed { background-color: #00477e; }"
        )
        apply_btn.clicked.connect(self._save_kin_params)
        btn_row = QHBoxLayout()
        btn_row.addStretch()
        btn_row.addWidget(apply_btn)
        btn_row.addStretch()
        grid.addLayout(btn_row, len(labels), 0, 1, 2)
        grid.setRowMinimumHeight(len(labels) + 1, 16)

        self.ctx.kin_params = defaults

        # Đánh giá đèn ngay khi khởi tạo, với giá trị mặc định
        for key in self._kin_edits:
            self._validate_kin_field(key)

        return group

    def _is_valid_kin_value(self, key: str, text: str) -> bool:
        """Quy tắc hợp lệ: là số thực và > 0 (kích thước cơ khí không thể âm hoặc bằng 0)."""
        try:
            value = float(text)
        except ValueError:
            return False
        return value > 0

    def _validate_kin_field(self, key: str) -> bool:
        """Cập nhật đèn xanh/xám cho 1 ô, trả về True nếu hợp lệ."""
        edit = self._kin_edits[key]
        dot = self._kin_indicators[key]
        valid = self._is_valid_kin_value(key, edit.text())

        # Hợp lệ: xanh lá cây (#2ecc71). Không hợp lệ: xám trung tính
        # (#9e9e9e) thay cho đỏ, để đồng bộ với việc loại bỏ màu đỏ khỏi
        # toàn bộ giao diện.
        color = "#2ecc71" if valid else "#9e9e9e"
        dot.setStyleSheet(
            f"background-color: {color}; border-radius: 13px; border: 1px solid #888;"
        )
        return valid

    def _save_kin_params(self):
        """Đọc 4 ô nhập, chỉ lưu nếu TẤT CẢ đều hợp lệ (đèn xanh)."""
        all_valid = True
        params = {}
        for key, edit in self._kin_edits.items():
            valid = self._validate_kin_field(key)
            all_valid = all_valid and valid
            if valid:
                params[key] = float(edit.text())

        if not all_valid:
            return  # còn ô báo không hợp lệ -> không lưu

        self.ctx.kin_params = params

    # -----------------------------------------------------------------
    # ĐIỀU HƯỚNG SANG CÁC CỬA SỔ TAB (không đổi so với bản gốc)
    # -----------------------------------------------------------------
    def _open_tab(self, key, window_cls):
        win = self._windows.get(key)
        if win is None:
            win = window_cls(self.ctx, self)
            self._windows[key] = win
        self.hide()
        win.showNormal()  # reset trạng thái trước khi maximize lại
        win.showMaximized()
        win.show()
        if win.layout():
            win.layout().activate()  # ép layout tính lại
        win.updateGeometry()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_F11:
            if self.isFullScreen():
                self.showMaximized()
            else:
                self.showFullScreen()
        elif event.key() == Qt.Key_Escape and self.isFullScreen():
            self.showMaximized()
        super().keyPressEvent(event)

    def closeEvent(self, event):
        for win in self._windows.values():
            win._force_close = True
            win.close()
        self.ctx.shutdown()
        event.accept()


def main():
    app = QApplication(sys.argv)
    apply_light_palette(app)
    # Font chữ mặc định cho TOÀN BỘ ứng dụng: Times New Roman (áp dụng ở cả
    # cấp QApplication.setFont lẫn trong STYLE_SHEET, để những widget tạo
    # ra sau này hoặc không có stylesheet riêng vẫn dùng đúng font này).
    app.setFont(QFont("Times New Roman", 11))
    app.setStyleSheet(STYLE_SHEET)
    win = LauncherWindow()
    win.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()