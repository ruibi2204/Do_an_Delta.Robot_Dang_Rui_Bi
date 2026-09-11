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
from PySide6.QtGui import QDoubleValidator, QPixmap, QPainter
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QFrame, QLineEdit, QGroupBox,
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
from shared_state import AppContext, apply_light_palette, STYLE_SHEET


# Đường dẫn ảnh dùng làm HÌNH NỀN MỜ (watermark) chìm phía sau toàn bộ
# giao diện màn hình chính. Đổi lại đường dẫn này cho khớp với ảnh thực tế
# của bạn (logo trường, sơ đồ động học robot Delta...).
KIN_PANEL_IMAGE_PATH = "delta.png"

# Độ mờ (0.0 = trong suốt hoàn toàn, 1.0 = đậm như ảnh gốc). Giá trị nhỏ để
# ảnh chỉ "chìm" nhẹ phía sau, không che chữ/nút phía trên.
WATERMARK_OPACITY = 0.08

# Kích thước cố định của các nút menu bên trái (canh lề trái), có khoảng
# cách rõ ràng giữa các nút (không dính liền nhau).
MENU_BTN_WIDTH = 480
MENU_BTN_HEIGHT = 90
MENU_BTN_SPACING = 10  # khoảng trống giữa 2 nút liên tiếp

# Nút menu (menuBtnSmall) - ĐÃ ĐỔI từ xanh dương sang XÁM theo yêu cầu.
MENU_BTN_STYLE = """
QPushButton#menuBtnSmall {
    background-color: #757575;
    color: #ffffff;
    border: 2px solid #5a6268;
    border-radius: 12px;
    padding: 8px 12px;
    font-weight: 700;
    font-size: 13pt;
}
QPushButton#menuBtnSmall:hover {
    background-color: #616161;
}
QPushButton#menuBtnSmall:pressed {
    background-color: #4a4a4a;
}
"""

# Kích thước riêng cho nút "Lưu tham số" trong khung THAM SỐ TỐC ĐỘ (cột phải).
APPLY_BTN_WIDTH = 240
APPLY_BTN_HEIGHT = 41

# Kích thước riêng cho nút "Lưu tham số" trong khung THAM SỐ ĐỘNG HỌC (cột
# giữa) - TO HƠN nút bên khung tốc độ vì khung này giờ chiếm nhiều không
# gian hơn (đã bỏ ảnh minh họa nhỏ bên dưới, ảnh giờ dùng làm nền mờ).
KIN_APPLY_BTN_WIDTH = 320
KIN_APPLY_BTN_HEIGHT = 56

# Kích thước riêng cho nút "KẾT NỐI" - đặt canh giữa, ngay dưới khung THAM
# SỐ ĐỘNG HỌC (cột giữa), KHÔNG còn nằm trong danh sách nút menu bên trái.
CONNECT_BTN_WIDTH = 320
CONNECT_BTN_HEIGHT = 64

# Danh sách các tham số tốc độ / thời gian di chuyển của robot, đồng bộ với
# các hằng số cùng tên trong move_delta_4dof.py (DeltaMotionPlanner).
# key: (nhãn hiển thị, giá trị mặc định)
SPEED_PARAM_DEFS = [
    ("TIME_MOVE_FAST", "Time Fast (s)", 0.6),
    ("TIME_MOVE_DOWN", "Time Down (s)", 0.15),
    ("TIME_MOVE_ACTION", "Time up (s)", 0.15),
    ("TIME_DELAY_GRIPPER", "Gripper delay (s)", 0.25),
    ("TIME_DELAY_DOF4", "Rolate 4 DOF (s)", 0.15),
    ("CONTROL_HZ", "FREQUENCY (Hz)", 100.0),
]

# Style dùng chung cho các nút nhỏ trong khung "ĐÁNH GIÁ ROBOT"
# (Độ lặp lại / Cơ khí chính xác / Hiệu chỉnh camera bàn cờ...) - cùng
# objectName "repeatBtn" để tất cả nút trong khung này đồng bộ kích thước
# và màu sắc, không phải khai báo lặp lại từng nút.
# ĐÃ ĐỔI từ xanh dương sang XÁM theo yêu cầu.
ROBOT_EVAL_BTN_STYLE = (
    "QPushButton#repeatBtn {"
    "  background-color: #757575; color: #ffffff;"
    "  border: 2px solid #5a6268; border-radius: 12px;"
    "  padding: 6px; font-weight: 700; font-size: 13pt;"
    "}"
    "QPushButton#repeatBtn:hover { background-color: #616161; }"
    "QPushButton#repeatBtn:pressed { background-color: #4a4a4a; }"
)

# Style riêng cho nút "KẾT NỐI" đứng độc lập (canh giữa dưới khung THAM SỐ
# ĐỘNG HỌC) - CÙNG tông xám với các nút khác, chỉ khác kích thước/objectName
# để không lẫn với các nút menu bên trái.
CONNECT_BTN_STYLE = (
    "QPushButton#connectMenuBtn {"
    "  background-color: #757575; color: #ffffff;"
    "  border: 2px solid #5a6268; border-radius: 14px;"
    "  padding: 10px; font-weight: 700; font-size: 14pt;"
    "}"
    "QPushButton#connectMenuBtn:hover { background-color: #616161; }"
    "QPushButton#connectMenuBtn:pressed { background-color: #4a4a4a; }"
)


class WatermarkCentralWidget(QWidget):
    """QWidget trung tâm tự vẽ một ảnh mờ (watermark) làm nền, nằm CHÌM phía
    sau toàn bộ các khung/nút của giao diện chính. Vì việc vẽ nền diễn ra
    trong paintEvent của chính widget cha, các widget con (thêm qua layout)
    luôn được Qt vẽ chồng lên sau đó - nên không cần thay đổi gì ở phần bố
    cục hiện có (menu, khung tham số...), ảnh sẽ tự nằm phía dưới cùng."""

    def __init__(self, image_path: str, opacity: float = 0.08, scale: float = 0.6, parent=None):
        super().__init__(parent)
        self._pixmap = QPixmap(image_path)
        self._opacity = opacity
        self._scale = scale  # 1.0 = phủ kín cửa sổ, nhỏ hơn 1.0 = ảnh nhỏ lại, canh giữa

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

class LauncherWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROBOT DELTA 4 BẬC TỰ DO - ĐHKTCN CẦN THƠ")

        self.ctx = AppContext()
        self._windows = {}

        self._build_ui()

    def _build_ui(self):
        # Central widget giờ là WatermarkCentralWidget để ảnh delta.png nằm
        # chìm làm nền mờ phía sau toàn bộ giao diện (thay vì chỉ là 1 ảnh
        # nhỏ nằm dưới khung THAM SỐ ĐỘNG HỌC như trước).
        central = WatermarkCentralWidget(
            resource_path(KIN_PANEL_IMAGE_PATH), opacity=WATERMARK_OPACITY, scale=1
        )
        self.setCentralWidget(central)

        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(60, 40, 60, 40)
        main_layout.setSpacing(28)

        # ---------- Header: tên trường + tên đề tài + tác giả, canh giữa
        #            trên toàn bề ngang màn hình, nằm trên cùng ----------
        header = self._build_header()
        main_layout.addLayout(header)

        # ---------- Thân trang: 3 cột ----------
        body = QHBoxLayout()
        body.setSpacing(36)

        left_col = self._build_menu_panel()
        middle_col = self._build_kinematics_panel()

        right_col = QVBoxLayout()
        right_col.setAlignment(Qt.AlignTop)
        robot_eval_group = self._build_robot_eval_panel()
        right_col.addWidget(robot_eval_group)

        # Khung tham số tốc độ, đặt ngay bên dưới khung "ĐÁNH GIÁ ROBOT"
        # trong cùng cột phải. Không đụng tới các cột/khung khác.
        right_col.addSpacing(16)
        speed_group = self._build_speed_params_panel()
        right_col.addWidget(speed_group)

        right_col.addStretch()

        body.addLayout(left_col, 3)
        body.addLayout(middle_col, 3)
        body.addLayout(right_col, 2)

        main_layout.addLayout(body, 1)

    def _build_header(self):
        """Khối tiêu đề trên cùng: canh giữa theo TOÀN BỀ NGANG màn hình,
        không thuộc riêng cột nào."""
        header = QVBoxLayout()
        header.setSpacing(4)
        header.setContentsMargins(0, 0, 0, 0)

        title1 = QLabel("CONTROL INTERFACE")
        title1.setObjectName("menuTitle1")
        title1.setAlignment(Qt.AlignCenter)
        title1.setWordWrap(True)
        header.addWidget(title1)

        title2 = QLabel("ROBOT DELTA 4 BẬC TỰ DO")
        title2.setObjectName("menuTitle2")
        title2.setAlignment(Qt.AlignCenter)
        header.addWidget(title2)

        title3 = QLabel("ĐẶNG RUI BI")
        title3.setObjectName("menuTitle3")
        title3.setAlignment(Qt.AlignCenter)
        header.addWidget(title3)

        header.addSpacing(10)
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet("color:#c0c0c0;")
        header.addWidget(line)

        return header

    def _build_menu_panel(self):
        """Cột trái: chỉ còn các nút điều hướng, canh lề trái, kích thước
        nhỏ gọn, có khoảng cách rõ ràng giữa các nút (không dính liền).

        LƯU Ý: nút "KẾT NỐI" đã được TÁCH RA khỏi danh sách này - giờ nằm
        canh giữa màn hình, ngay dưới khung "THAM SỐ ĐỘNG HỌC" (cột giữa),
        xem _build_connection_button() / _build_kinematics_panel()."""
        left_col = QVBoxLayout()
        left_col.setContentsMargins(0, 0, 0, 0)
        left_col.setSpacing(MENU_BTN_SPACING)
        left_col.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        buttons = [
            ("THỦ CÔNG", "manual", ManualWindow),
            ("CAMERA && OFFSET TỌA ĐỘ", "camera", CameraWindow),
            ("BÀI TOÁN TĨNH", "csv", CsvWindow),
            ("BÀI TOÁN BẬC 4 TĨNH", "dynamic", DynamicWindow),
            ("BÀI TOÁN ĐỘNG", "dynamic_run4dof", DynamicRun4DofWindow),  # <-- nút mới
        ]
        for text, key, cls in buttons:
            btn = QPushButton(text)
            # objectName riêng ("menuBtnSmall") + stylesheet riêng, KHÔNG dùng
            # chung "menuBtn" (min-height 80px trong STYLE_SHEET toàn cục) để
            # kích thước nhỏ thật sự được áp dụng, không bị đè lại.
            btn.setObjectName("menuBtnSmall")
            btn.setStyleSheet(MENU_BTN_STYLE)
            btn.setFixedWidth(MENU_BTN_WIDTH)
            btn.setFixedHeight(MENU_BTN_HEIGHT)
            btn.clicked.connect(lambda checked=False, k=key, c=cls: self._open_tab(k, c))
            left_col.addWidget(btn, alignment=Qt.AlignLeft)

        left_col.addStretch()
        return left_col

    def _build_connection_button(self):
        """Nút "KẾT NỐI" đứng ĐỘC LẬP, canh giữa màn hình, đặt ngay dưới
        khung "THAM SỐ ĐỘNG HỌC" (cột giữa) - KHÔNG còn nằm trong danh sách
        nút menu bên trái nữa."""
        btn = QPushButton("KẾT NỐI")
        btn.setObjectName("connectMenuBtn")
        btn.setStyleSheet(CONNECT_BTN_STYLE)
        btn.setFixedWidth(CONNECT_BTN_WIDTH)
        btn.setFixedHeight(CONNECT_BTN_HEIGHT)
        btn.clicked.connect(lambda checked=False: self._open_tab("connection", ConnectionWindow))
        return btn

    def _add_robot_eval_button(self, layout, text, key, window_cls):
        """Tạo 1 nút trong khung 'ĐÁNH GIÁ ROBOT', dùng CHUNG style/kích
        thước với các nút đã có (Độ lặp lại, Cơ khí chính xác...) - cùng
        objectName 'repeatBtn' + ROBOT_EVAL_BTN_STYLE + setFixedHeight(48)."""
        btn = QPushButton(text)
        btn.setObjectName("repeatBtn")
        btn.setFixedHeight(48)
        btn.setStyleSheet(ROBOT_EVAL_BTN_STYLE)
        btn.clicked.connect(lambda checked=False: self._open_tab(key, window_cls))
        layout.addWidget(btn)
        return btn

    def _build_robot_eval_panel(self):
        """Khung tiêu đề 'ĐÁNH GIÁ ROBOT' - cùng phong cách với khung tham số
        động học nghịch. Chứa các nút truy cập giao diện đánh giá độ lặp
        lại, vẽ hình và hiệu chỉnh camera bằng bàn cờ."""
        group = QGroupBox("ĐÁNH GIÁ ROBOT")
        group.setObjectName("kinGroup")
        layout = QVBoxLayout(group)
        layout.setSpacing(10)

        self._add_robot_eval_button(layout, "Độ lặp lại", "repeat_test", RepeatTestWindow)
        self._add_robot_eval_button(layout, "Cơ khí chính xác", "draw", DrawWindow)
        # Nút mới: mở giao diện HIỆU CHỈNH CAMERA BẰNG BÀN CỜ - cùng kích
        # thước/màu với 2 nút phía trên (không phải khai báo style riêng).
        self._add_robot_eval_button(layout, "Hiệu chỉnh Camera", "board_calib", BoardCalibWindow)

        return group

    def _build_speed_params_panel(self):
        """Khung 'THAM SỐ TỐC ĐỘ' - nhập các tham số thời gian/tốc độ dùng
        trong DeltaMotionPlanner (move_delta_4dof.py): TIME_MOVE_FAST,
        TIME_MOVE_DOWN, TIME_MOVE_ACTION, TIME_DELAY_GRIPPER,
        TIME_DELAY_DOF4, CONTROL_HZ. Cùng phong cách với khung THAM SỐ ĐỘNG
        HỌC (có đèn báo hợp lệ xanh/đỏ), đặt ngay dưới khung ĐÁNH GIÁ ROBOT
        ở cột phải, không ảnh hưởng tới bố cục các khung khác.
        """
        group = QGroupBox("THAM SỐ TỐC ĐỘ")
        group.setObjectName("kinGroup")
        grid = QGridLayout(group)
        grid.setSpacing(1)

        defaults = getattr(self.ctx, "speed_params", None) or {
            key: default for key, _desc, default in SPEED_PARAM_DEFS
        }

        self._speed_edits = {}
        self._speed_indicators = {}
        validator = QDoubleValidator(0.0001, 100000.0, 4)
        validator.setNotation(QDoubleValidator.StandardNotation)

        for row, (key, desc, _default) in enumerate(SPEED_PARAM_DEFS):
            lbl = QLabel(desc)

            edit = QLineEdit(str(defaults.get(key, _default)))
            edit.setValidator(validator)
            edit.setObjectName("kinEdit")
            edit.textChanged.connect(lambda text, k=key: self._validate_speed_field(k))
            edit.editingFinished.connect(self._save_speed_params)
            self._speed_edits[key] = edit

            dot = QLabel()
            dot.setFixedSize(10, 10)
            dot.setObjectName("kinDot")
            self._speed_indicators[key] = dot

            field_row = QHBoxLayout()
            field_row.setSpacing(4)
            field_row.addWidget(edit)
            field_row.addWidget(dot)

            grid.addWidget(lbl, row, 0)
            grid.addLayout(field_row, row, 1)

        apply_btn = QPushButton("Lưu tham số")
        apply_btn.setObjectName("kinApplyBtn")
        # Cùng kích thước với nút "Lưu tham số" của khung động học, để đồng
        # bộ phong cách giữa hai khung tham số.
        apply_btn.setFixedWidth(APPLY_BTN_WIDTH)
        apply_btn.setFixedHeight(APPLY_BTN_HEIGHT)
        apply_btn.setStyleSheet(
            "QPushButton#kinApplyBtn {"
            "  background-color: #f5f5f5; color: #1a1a1a;"
            "  border: 2px solid #b0b0b0; border-radius: 14px;"
            "  padding: 10px; font-weight: 700; font-size: 13pt;"
            "}"
            "QPushButton#kinApplyBtn:hover { background-color: #e6e6e6; border-color: #909090; }"
            "QPushButton#kinApplyBtn:pressed { background-color: #d8d8d8; }"
        )
        apply_btn.clicked.connect(self._save_speed_params)
        btn_row = QHBoxLayout()
        btn_row.addStretch()
        btn_row.addWidget(apply_btn)
        btn_row.addStretch()
        grid.addLayout(btn_row, len(SPEED_PARAM_DEFS), 0, 1, 2)

        self.ctx.speed_params = dict(defaults)

        # Đánh giá đèn ngay khi khởi tạo, với giá trị mặc định
        for key in self._speed_edits:
            self._validate_speed_field(key)

        return group

    def _build_kinematics_panel(self):
        """Panel ĐÁNH GIÁ CƠ KHÍ - nhập 4 tham số động học nghịch: R, r, a, b —
        có đèn báo hợp lệ (xanh/đỏ). Được canh vào giữa màn hình (cột giữa).

        Kích thước đã được TĂNG LÊN đáng kể (khung rộng hơn, chữ to hơn, ô
        nhập cao hơn, khoảng cách giữa các hàng giãn ra) để khung này lấp
        đầy khoảng trống còn lại trong cột giữa - trước đây khoảng trống đó
        được lấp bằng 1 ảnh minh họa nhỏ bên dưới, nay ảnh đã chuyển thành
        nền mờ (watermark) cho toàn bộ cửa sổ nên không còn ảnh riêng ở đây
        nữa.

        Ngay dưới khung này giờ có thêm nút "KẾT NỐI" - canh giữa theo
        chiều ngang (Qt.AlignHCenter), độc lập với danh sách nút menu bên
        trái (xem _build_connection_button()).
        """
        col = QVBoxLayout()
        col.setAlignment(Qt.AlignTop | Qt.AlignHCenter)

        group = QGroupBox("THAM SỐ ĐỘNG HỌC")
        group.setObjectName("kinGroup")
        group.setAlignment(Qt.AlignCenter)  # canh giữa chữ tiêu đề khung
        # Khung rộng hơn hẳn trước (650 -> 760) và có chiều cao tối thiểu để
        # thực sự lấp khoảng trống trong cột giữa.
        group.setMinimumWidth(500)
        group.setMaximumWidth(600)
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
            ("R", "Bán kính đế (R, mm)"),
            ("r", "Bán kính bàn (r, mm)"),
            ("a", "Chiều dài cánh trên (a, mm)"),
            ("b", "Chiều dài cánh dưới (b, mm)"),
        ]

        for row, (key, desc) in enumerate(labels):
            lbl = QLabel(desc)
            # Nhãn chữ to hơn hẳn (13pt -> 15pt) để tương xứng với khung
            # đã được phóng lớn.
            lbl.setStyleSheet("font-size: 15pt;")

            edit = QLineEdit(str(defaults[key]))
            edit.setValidator(validator)
            edit.setObjectName("kinEdit")
            # Ô nhập cao hơn (52px) và chữ to hơn (15pt) để dễ đọc, dễ bấm.
            edit.setMinimumHeight(52)
            edit.setStyleSheet("QLineEdit#kinEdit { font-size: 15pt; padding: 6px 10px; }")
            edit.textChanged.connect(lambda text, k=key: self._validate_kin_field(k))
            edit.editingFinished.connect(self._save_kin_params)
            self._kin_edits[key] = edit

            dot = QLabel()
            # Đèn báo cũng to hơn tương ứng (20px -> 26px).
            dot.setFixedSize(15, 15)
            dot.setObjectName("kinDot")
            self._kin_indicators[key] = dot

            field_row = QHBoxLayout()
            field_row.setSpacing(10)
            field_row.addWidget(edit)
            field_row.addWidget(dot)

            grid.addWidget(lbl, row, 0)
            grid.addLayout(field_row, row, 1)

        apply_btn = QPushButton("Lưu tham số")
        apply_btn.setObjectName("kinApplyBtn")
        # Nút màu xám trắng (khác với các nút hành động màu xám đậm) để phân
        # biệt đây là thao tác lưu cấu hình, không phải điều khiển robot.
        # Kích thước TO HƠN nút cùng loại ở khung THAM SỐ TỐC ĐỘ, vì khung
        # này giờ chiếm nhiều diện tích hơn.
        apply_btn.setFixedWidth(KIN_APPLY_BTN_WIDTH)
        apply_btn.setFixedHeight(KIN_APPLY_BTN_HEIGHT)
        apply_btn.setStyleSheet(
            "QPushButton#kinApplyBtn {"
            "  background-color: #f5f5f5; color: #1a1a1a;"
            "  border: 2px solid #b0b0b0; border-radius: 16px;"
            "  padding: 10px; font-weight: 700; font-size: 15pt;"
            "}"
            "QPushButton#kinApplyBtn:hover { background-color: #e6e6e6; border-color: #909090; }"
            "QPushButton#kinApplyBtn:pressed { background-color: #d8d8d8; }"
        )
        apply_btn.clicked.connect(self._save_kin_params)
        # Đặt nút nằm giữa 2 cột của grid, canh giữa theo chiều ngang (không
        # kéo giãn hết bề rộng khung nữa vì đã có kích thước cố định).
        btn_row = QHBoxLayout()
        btn_row.addStretch()
        btn_row.addWidget(apply_btn)
        btn_row.addStretch()
        grid.addLayout(btn_row, len(labels), 0, 1, 2)
        grid.setRowMinimumHeight(len(labels) + 1, 20)

        col.addWidget(group, alignment=Qt.AlignHCenter)

        # ---- Nút "KẾT NỐI" - canh giữa, ngay dưới khung THAM SỐ ĐỘNG HỌC ----
        col.addSpacing(24)
        connect_btn = self._build_connection_button()
        col.addWidget(connect_btn, alignment=Qt.AlignHCenter)

        # Khoảng trống bên dưới để trống có chủ đích (không còn ảnh minh họa
        # nhỏ ở đây nữa - ảnh delta.png đã chuyển thành nền mờ của toàn bộ
        # cửa sổ, xem WatermarkCentralWidget).
        col.addStretch()

        self.ctx.kin_params = defaults

        # Đánh giá đèn ngay khi khởi tạo, với giá trị mặc định
        for key in self._kin_edits:
            self._validate_kin_field(key)

        return col

    def _is_valid_kin_value(self, key: str, text: str) -> bool:
        """Quy tắc hợp lệ: là số thực và > 0 (kích thước cơ khí không thể âm hoặc bằng 0)."""
        try:
            value = float(text)
        except ValueError:
            return False
        return value > 0

    def _validate_kin_field(self, key: str) -> bool:
        """Cập nhật đèn xanh/đỏ cho 1 ô, trả về True nếu hợp lệ."""
        edit = self._kin_edits[key]
        dot = self._kin_indicators[key]
        valid = self._is_valid_kin_value(key, edit.text())

        color = "#2ecc71" if valid else "#e74c3c"  # xanh lá / đỏ
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
            return  # còn ô đỏ -> không lưu, đèn đã tự báo cho người dùng biết

        self.ctx.kin_params = params

    def _is_valid_speed_value(self, text: str) -> bool:
        """Tham số tốc độ/thời gian phải là số thực và > 0."""
        try:
            value = float(text)
        except ValueError:
            return False
        return value > 0

    def _validate_speed_field(self, key: str) -> bool:
        """Cập nhật đèn xanh/đỏ cho 1 ô tham số tốc độ, trả về True nếu hợp lệ."""
        edit = self._speed_edits[key]
        dot = self._speed_indicators[key]
        valid = self._is_valid_speed_value(edit.text())

        color = "#2ecc71" if valid else "#e74c3c"  # xanh lá / đỏ
        dot.setStyleSheet(
            f"background-color: {color}; border-radius: 7px; border: 1px solid #888;"
        )
        return valid

    def _save_speed_params(self):
        """Đọc các ô tham số tốc độ, chỉ lưu nếu TẤT CẢ đều hợp lệ (đèn xanh).
        Giá trị được lưu vào self.ctx.speed_params để các cửa sổ/luồng điều
        khiển (ví dụ DeltaMotionPlanner) có thể đọc lại khi cần."""
        all_valid = True
        params = {}
        for key, edit in self._speed_edits.items():
            valid = self._validate_speed_field(key)
            all_valid = all_valid and valid
            if valid:
                params[key] = float(edit.text())

        if not all_valid:
            return  # còn ô đỏ -> không lưu

        self.ctx.speed_params = params

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
    app.setStyleSheet(STYLE_SHEET)
    win = LauncherWindow()
    win.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()