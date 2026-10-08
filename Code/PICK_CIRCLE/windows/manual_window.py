import math
from collections import deque

from PySide6.QtCore import Qt, QPointF, QRectF, QTimer
from PySide6.QtGui import QPainter, QPainterPath, QPen, QColor, QFont, QPolygonF
from PySide6.QtWidgets import (
    QHBoxLayout, QVBoxLayout, QGridLayout, QPushButton, QLabel,
    QGroupBox, QDoubleSpinBox, QSlider, QMessageBox,
)

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from shared_state import BaseTabWindow, FnWorker, inverse_kinematics, home_and_wait, save_config
from kinematics.manual_kinematic import LinearMotionWorker, Dof4RotateWorker

# Giới hạn tốc độ điều chỉnh được qua thanh trượt (mm/s)
SPEED_MIN = 5
SPEED_MAX = 150
SPEED_DEFAULT = 40

# Số điểm tối đa giữ lại trên đồ thị real-time (trục hoành "Điểm trên quỹ đạo").
# Quỹ đạo dài hơn số này sẽ tự cuộn (chỉ giữ lại các điểm gần nhất).
TRAJ_MAX_POINTS = 400

# ---------------------------------------------------------------------
# TỶ SỐ TRUYỀN (bộ truyền đai): góc_motor = góc_canh_tay_L1 * TỶ_SỐ.
# Đây là mô hình TUYẾN TÍNH đơn giản để có ngay đồ thị "qua tỷ số truyền" -
# nếu bộ truyền thật không tuyến tính, chỉnh giá trị này trên giao diện
# (hoặc thay hàm _theta_to_motor bên dưới) cho khớp với robot thật.
# ---------------------------------------------------------------------
GEAR_RATIO_DEFAULT = 3.3

ARM_COLORS = ("#e63946", "#00ff7f", "#3f8efc")   # đỏ / xanh lá / xanh dương - giống hình mẫu

# ---------------------------------------------------------------------
# BẬC TỰ DO 4 (bàn xoay / step, điều khiển qua UART 2: lệnh "STEP:<độ>")
# ---------------------------------------------------------------------
DOF4_STEP_DEG = 10.0        # mỗi lần nhấn quay CỐ ĐỊNH 10 độ
# Quy ước chiều: nhấn mũi tên CÙNG chiều kim đồng hồ -> gửi STEP:+10.
# Nếu robot thật quay NGƯỢC lại, đổi giá trị này thành -1.0 (không cần sửa
# chỗ nào khác; góc θ4 hiển thị vẫn tăng khi quay cùng chiều kim đồng hồ).
DOF4_CW_SIGN = 1.0
DOF4_COOLDOWN_MS = 500      # khóa 2 nút quay ngắn sau mỗi lần nhấn, chờ step quay xong


# =====================================================================
# NÚT TAM GIÁC ĐỀU (equilateral) - bo góc mềm
# ---------------------------------------------------------------------
# - Tam giác đều thật: 3 cạnh bằng nhau, mỗi góc 60°.
# - Bo góc bằng QPainterPath: mỗi đỉnh được "cắt" một đoạn `cut` rồi nối
#   bằng 1 đường cong bậc 2 (quadTo) → bo mềm, không nhọn.
# - Tự vẽ đủ 4 trạng thái: normal / hover / pressed / disabled.
# - Chữ (X / Y / Z) nằm giữa tâm tam giác.
# =====================================================================
class TriangleButton(QPushButton):
    def __init__(self, direction, label="", parent=None):
        super().__init__(parent)
        self.direction = direction  # "up" / "down" / "left" / "right"
        self.label = label
        self._hover = False
        self._pressed = False
        self.setCursor(Qt.PointingHandCursor)
        self.setObjectName("triBtn")
        # Ghi đè QSS toàn cục: widget tự vẽ hết, không để QSS vẽ nền chữ nhật.
        self.setStyleSheet(
            "QPushButton#triBtn { background: transparent; border: none; padding: 0; }"
        )

    # -- 3 đỉnh tam giác đều tâm (cx, cy), cạnh `side`, xoay theo direction --
    def _triangle_vertices(self, cx, cy, side):
        h = side * (3 ** 0.5) / 2.0        # chiều cao tam giác đều
        if self.direction == "up":
            return [
                QPointF(cx,                 cy - 2 * h / 3.0),   # đỉnh trên
                QPointF(cx + side / 2.0,    cy +     h / 3.0),   # đáy phải
                QPointF(cx - side / 2.0,    cy +     h / 3.0),   # đáy trái
            ]
        if self.direction == "down":
            return [
                QPointF(cx,                 cy + 2 * h / 3.0),   # đỉnh dưới
                QPointF(cx - side / 2.0,    cy -     h / 3.0),   # đáy trái
                QPointF(cx + side / 2.0,    cy -     h / 3.0),   # đáy phải
            ]
        if self.direction == "left":
            return [
                QPointF(cx - 2 * h / 3.0,   cy),                 # đỉnh trái
                QPointF(cx +     h / 3.0,   cy - side / 2.0),    # đáy trên
                QPointF(cx +     h / 3.0,   cy + side / 2.0),    # đáy dưới
            ]
        # right
        return [
            QPointF(cx + 2 * h / 3.0,       cy),                 # đỉnh phải
            QPointF(cx -     h / 3.0,       cy + side / 2.0),    # đáy dưới
            QPointF(cx -     h / 3.0,       cy - side / 2.0),    # đáy trên
        ]

    # -- Path tam giác đều có 3 đỉnh bo mềm --
    @staticmethod
    def _rounded_path(vertices, cut):
        path = QPainterPath()
        n = len(vertices)
        for i in range(n):
            prev = vertices[i - 1]
            cur = vertices[i]
            nxt = vertices[(i + 1) % n]

            dx1, dy1 = prev.x() - cur.x(), prev.y() - cur.y()
            dx2, dy2 = nxt.x() - cur.x(), nxt.y() - cur.y()
            len1 = (dx1 * dx1 + dy1 * dy1) ** 0.5
            len2 = (dx2 * dx2 + dy2 * dy2) ** 0.5
            ux1, uy1 = dx1 / len1, dy1 / len1
            ux2, uy2 = dx2 / len2, dy2 / len2

            c1 = min(cut, len1 * 0.5)
            c2 = min(cut, len2 * 0.5)
            p_in = QPointF(cur.x() + ux1 * c1, cur.y() + uy1 * c1)
            p_out = QPointF(cur.x() + ux2 * c2, cur.y() + uy2 * c2)

            if i == 0:
                path.moveTo(p_in)
            else:
                path.lineTo(p_in)
            # quadTo: đường cong bậc 2 với điểm điều khiển = đỉnh gốc
            path.quadTo(cur, p_out)
        path.closeSubpath()
        return path

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)

        if not self.isEnabled():
            fill, border, text_color = QColor("#d0d0d0"), QColor("#b0b0b0"), QColor("#888888")
        elif self._pressed:
            fill, border, text_color = QColor("#4da6ff"), QColor("#2a7dd6"), QColor("#ffffff")
        elif self._hover:
            fill, border, text_color = QColor("#cde0ff"), QColor("#4da6ff"), QColor("#1a1a1a")
        else:
            fill, border, text_color = QColor("#e8e8e8"), QColor("#b0b0b0"), QColor("#1a1a1a")

        cx = self.width() / 2.0
        cy = self.height() / 2.0
        # Cạnh tam giác = cạnh nhỏ hơn của ô trừ padding, để tam giác
        # luôn nằm gọn trong ô vuông (không bị cắt / không méo).
        side = min(self.width(), self.height()) - 14
        vertices = self._triangle_vertices(cx, cy, side)
        # cut ~ 0.17 * cạnh → bo góc có bán kính hiệu dụng ~8-9 px,
        # đủ mềm mắt nhưng không thành "giọt nước".
        path = self._rounded_path(vertices, cut=side * 0.17)

        p.setPen(QPen(border, 2))
        p.setBrush(fill)
        p.drawPath(path)

        if self.label:
            p.setPen(text_color)
            p.setFont(QFont("Times New Roman", 16, QFont.Bold))
            rect = QRectF(cx - 40, cy - 20, 80, 40)
            p.drawText(rect, Qt.AlignCenter, self.label)

    def enterEvent(self, event):
        self._hover = True
        self.update()

    def leaveEvent(self, event):
        self._hover = False
        self.update()

    def mousePressEvent(self, event):
        self._pressed = True
        self.update()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        self._pressed = False
        self.update()
        super().mouseReleaseEvent(event)


# =====================================================================
# NÚT MŨI TÊN HÌNH VÒNG CUNG (xoay bậc tự do 4)
# ---------------------------------------------------------------------
# - clockwise=True : mũi tên cung tròn quay CÙNG chiều kim đồng hồ.
# - clockwise=False: mũi tên cung tròn quay NGƯỢC chiều kim đồng hồ.
# - Cung tròn 280° (hở 80° ở phía trên), đầu mũi tên là tam giác đặc ở
#   cuối cung, hướng theo chiều quay. Chữ (vd "10°") nằm giữa vòng.
# - Tự vẽ 4 trạng thái giống TriangleButton: normal / hover / pressed / disabled.
# =====================================================================
class ArcArrowButton(QPushButton):
    # (Không còn dùng GAP_HALF_DEG - giờ vẽ cố định đúng nửa vòng tròn 180°,
    # xem paintEvent bên dưới.)

    def __init__(self, clockwise, label="", parent=None):
        super().__init__(parent)
        self.clockwise = clockwise
        self.label = label
        self._hover = False
        self._pressed = False
        self.setCursor(Qt.PointingHandCursor)
        self.setObjectName("arcBtn")
        self.setStyleSheet(
            "QPushButton#arcBtn { background: transparent; border: none; padding: 0; }"
        )

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)

        if not self.isEnabled():
            fill, border, text_color = QColor("#d0d0d0"), QColor("#b0b0b0"), QColor("#888888")
        elif self._pressed:
            fill, border, text_color = QColor("#4da6ff"), QColor("#2a7dd6"), QColor("#ffffff")
        elif self._hover:
            fill, border, text_color = QColor("#cde0ff"), QColor("#4da6ff"), QColor("#e8f1ff")
        else:
            fill, border, text_color = QColor("#e8e8e8"), QColor("#b0b0b0"), QColor("#e8f1ff")

        cx = self.width() / 2.0
        cy = self.height() / 2.0
        r = min(self.width(), self.height()) / 2.0 - 18
        rect = QRectF(cx - r, cy - r, 2 * r, 2 * r)

        # FIX: trước đây chỉ vẽ 1 khe hở nhỏ (~80°) trên đỉnh, gần như khép
        # kín cả vòng tròn -> quá rối, khó thấy hướng. Giờ chỉ vẽ ĐÚNG NỬA
        # VÒNG TRÒN (180°): bắt đầu từ đỉnh (12 giờ), CW quét sang phải
        # xuống đáy, CCW quét sang trái xuống đáy. Đầu mũi tên đặt ở điểm
        # cuối (đáy, 6 giờ).
        start = 90.0
        sweep = -180.0 if self.clockwise else 180.0

        path = QPainterPath()
        path.arcMoveTo(rect, start)
        path.arcTo(rect, start, sweep)
        end_point = path.currentPosition()

        # FIX: bản trước tự suy ra hướng tiếp tuyến bằng công thức lượng giác
        # tự viết, dựa trên một quy ước góc của Qt mà thực tế không khớp với
        # cách QPainterPath thực sự vẽ (khe hở/đầu mũi tên bị lệch chỗ, có
        # khi coi như "biến mất"). Giờ lấy TRỰC TIẾP tọa độ Qt đã tính sẵn:
        # dựng thêm 1 đường cung gần giống hệt nhưng NGẮN HƠN vài độ, rồi lấy
        # hướng từ điểm đó tới điểm cuối thật -> luôn đúng 100% với những gì
        # Qt vẽ ra, không phụ thuộc quy ước góc tự giả định.
        eps_deg = 6.0
        pre_sweep = sweep - math.copysign(eps_deg, sweep)
        path_pre = QPainterPath()
        path_pre.arcMoveTo(rect, start)
        path_pre.arcTo(rect, start, pre_sweep)
        pre_point = path_pre.currentPosition()

        ex, ey = end_point.x(), end_point.y()
        tx, ty = ex - pre_point.x(), ey - pre_point.y()
        tlen = math.hypot(tx, ty)
        if tlen < 1e-6:
            tx, ty = 1.0, 0.0
        else:
            tx, ty = tx / tlen, ty / tlen
        nx, ny = -ty, tx  # pháp tuyến (vuông góc với hướng di chuyển)

        length, width = 22.0, 24.0
        tip = QPointF(ex + tx * length * 0.6, ey + ty * length * 0.6)
        b1 = QPointF(ex - tx * length * 0.4 + nx * width / 2.0,
                     ey - ty * length * 0.4 + ny * width / 2.0)
        b2 = QPointF(ex - tx * length * 0.4 - nx * width / 2.0,
                     ey - ty * length * 0.4 - ny * width / 2.0)
        head = QPolygonF([tip, b1, b2])

        # Thân cung: vẽ viền (nét to, màu viền) rồi phủ nét nhỏ hơn (màu nền).
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(border, 12, Qt.SolidLine, Qt.RoundCap))
        p.drawPath(path)
        p.setPen(QPen(fill, 8, Qt.SolidLine, Qt.RoundCap))
        p.drawPath(path)

        # FIX: đầu mũi tên trước đây dùng cùng màu (fill/border) với thân cung
        # nên gần như "chìm" vào cung, khó nhận ra chiều quay. Tô riêng 1 màu
        # nổi bật (accent xanh) để đầu mũi tên luôn rõ hướng ở mọi trạng thái.
        if not self.isEnabled():
            head_fill = QColor("#9aa7b8")
        else:
            head_fill = QColor("#ffb703")  # cam sáng - tương phản mạnh với nền xanh/xám
        # FIX (NGUYÊN NHÂN GỐC RỄ của toàn bộ crash "mở tab Thủ công là đơ tự
        # thoát"): chữ ký PySide6 là QPen(brush, width, style, cap, join) -
        # tham số thứ 4 là CAP chứ không phải JOIN. Dòng cũ QPen(color, 2,
        # Qt.SolidLine, Qt.RoundJoin) truyền nhầm kiểu PenJoinStyle vào đúng
        # vị trí PenCapStyle -> TypeError ngay trong paintEvent -> vì đây là
        # 1 hàm override C++ (virtual), exception này khiến Qt crash cứng cả
        # tiến trình (không phải lỗi Python bình thường có thể bắt được) mỗi
        # khi 2 nút mũi tên xoay θ4 được vẽ - tức là ngay khi mở tab. Lỗi này
        # vốn đã có sẵn trong code gốc (dùng "border" thay vì màu cam), chỉ
        # là chưa ai để ý vì exception không hiện traceback rõ ràng trên UI.
        p.setPen(QPen(QColor("#5a3d00"), 2, Qt.SolidLine, Qt.SquareCap, Qt.RoundJoin))
        p.setBrush(head_fill)
        p.drawPolygon(head)

        if self.label:
            p.setPen(text_color)
            p.setFont(QFont("Times New Roman", 13, QFont.Bold))
            p.drawText(QRectF(cx - 40, cy - 16, 80, 32), Qt.AlignCenter, self.label)

    def enterEvent(self, event):
        self._hover = True
        self.update()

    def leaveEvent(self, event):
        self._hover = False
        self.update()

    def mousePressEvent(self, event):
        self._pressed = True
        self.update()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        self._pressed = False
        self.update()
        super().mouseReleaseEvent(event)


class ManualWindow(BaseTabWindow):
    def __init__(self, ctx, launcher):
        super().__init__(ctx, launcher, "THỦ CÔNG")
        if not hasattr(self.ctx, "jog_speed"):
            self.ctx.jog_speed = SPEED_DEFAULT
        # Góc θ4 (bậc tự do 4) tích lũy theo các lần nhấn; firmware không báo
        # ngược góc thật nên coi lúc mở app là 0° (có nút ĐẶT 0 để căn lại).
        if not hasattr(self.ctx, "theta4_deg"):
            self.ctx.theta4_deg = 0.0

        # FIX: _build_ui() bên dưới đọc trực tiếp các thuộc tính này ngay khi
        # khởi tạo widget (setValue của spinbox, _home_str(), _update_pos_display()).
        # Nếu ctx chưa kịp có các thuộc tính này (tuỳ thứ tự khởi tạo tab), Python
        # sẽ ném AttributeError ngay trong __init__ -> exception không ai bắt ->
        # ứng dụng Qt thoát ngay không rõ lý do (đúng triệu chứng "mở tab Thủ công
        # là đơ rồi tự thoát"). Thêm guard mặc định để tab này luôn mở được, dù
        # các giá trị thật sẽ được ghi đè sau khi nơi khác trong app gán cho ctx.
        if not hasattr(self.ctx, "jog_step_xy"):
            self.ctx.jog_step_xy = 5.0
        if not hasattr(self.ctx, "jog_step_z"):
            self.ctx.jog_step_z = 5.0
        if not hasattr(self.ctx, "current_pos"):
            self.ctx.current_pos = [0.0, 0.0, 0.0]
        if not hasattr(self.ctx, "home_pos"):
            self.ctx.home_pos = [0.0, 0.0, 0.0]
        if not hasattr(self.ctx, "busy"):
            self.ctx.busy = False
        if not hasattr(self.ctx, "cfg"):
            self.ctx.cfg = {}

        self._motion_worker = None
        self._dof4_worker = None

        # Bộ đệm dữ liệu cho đồ thị real-time: 3 cánh tay chủ động (L1) và
        # 3 giá trị motor tương ứng (đã quy đổi qua tỷ số truyền).
        self._buf_theta = [deque(maxlen=TRAJ_MAX_POINTS) for _ in range(3)]
        self._buf_motor = [deque(maxlen=TRAJ_MAX_POINTS) for _ in range(3)]

        self._build_ui()
        self._update_pos_display()
        self._update_theta4_display()
        self._reset_chart()

    def showEvent(self, event):
        super().showEvent(event)
        self._update_pos_display()
        self._update_theta4_display()
        self._set_manual_busy(self.ctx.busy)

    # ------------------------------------------------------------------
    def _build_ui(self):
        root = QHBoxLayout(self.content_widget)
        root.setSpacing(14)

        # ================= CỘT TRÁI: ĐIỀU KHIỂN =================
        controls_col = QVBoxLayout()
        controls_col.setSpacing(10)

        # ---- 0) GÓC θ4 (UART 2) + TỌA ĐỘ HIỆN TẠI: đặt NGANG HÀNG cạnh nhau
        # thay vì xếp trên-dưới, vừa tiết kiệm diện tích dọc vừa tránh bị co
        # hẹp/cắt chữ khi mỗi khung phải tự chiếm hết 1 hàng riêng.
        t4_box = QGroupBox("GÓC θ4")
        t4_box.setObjectName("compactBox")
        t4_box.setMaximumWidth(300)
        t4_layout = QVBoxLayout()
        t4_layout.setContentsMargins(8, 6, 8, 6)
        t4_layout.setSpacing(4)

        t4_row = QHBoxLayout()
        t4_row.setSpacing(8)
        self.lbl_theta4 = QLabel("θ4: 0.00°")
        self.lbl_theta4.setObjectName("posDisplay")
        self.lbl_theta4.setAlignment(Qt.AlignCenter)
        self.lbl_theta4.setStyleSheet(
            "font-size:13pt; font-weight:700; padding:6px; min-height:0px;"
        )
        t4_row.addWidget(self.lbl_theta4, 1)

        self.btn_theta4_zero = QPushButton("ĐẶT 0")
        self.btn_theta4_zero.setObjectName("homeBtn")
        self.btn_theta4_zero.setFixedHeight(34)
        self.btn_theta4_zero.setToolTip(
            "Coi vị trí hiện tại là 0° (chỉ đổi số hiển thị, không quay)."
        )
        self.btn_theta4_zero.setStyleSheet(
            "QPushButton#homeBtn { min-height:34px; max-height:34px; "
            "font-size:11pt; font-weight:700; padding:2px 10px; }"
        )
        self.btn_theta4_zero.clicked.connect(self.on_reset_theta4)
        t4_row.addWidget(self.btn_theta4_zero, 0)
        t4_layout.addLayout(t4_row)

        self.lbl_uart2_state = QLabel("")
        self.lbl_uart2_state.setAlignment(Qt.AlignCenter)
        t4_layout.addWidget(self.lbl_uart2_state)

        t4_box.setLayout(t4_layout)

        # ---- 1) TỌA ĐỘ HIỆN TẠI ----
        pos_box = QGroupBox("TỌA ĐỘ HIỆN TẠI (mm)")
        pos_box.setObjectName("compactBox")
        pos_box.setMaximumWidth(300)
        pos_layout = QVBoxLayout()
        pos_layout.setContentsMargins(8, 6, 8, 6)
        pos_layout.setSpacing(4)

        self.lbl_pos = QLabel("X:0.00  Y:0.00  Z:0.00")
        self.lbl_pos.setObjectName("posDisplay")
        self.lbl_pos.setAlignment(Qt.AlignCenter)
        self.lbl_pos.setStyleSheet(
            "font-size:13pt; font-weight:700; padding:6px; min-height:0px;"
        )
        pos_layout.addWidget(self.lbl_pos)

        self.lbl_angles = QLabel("θ1:--°  θ2:--°  θ3:--°")
        self.lbl_angles.setAlignment(Qt.AlignCenter)
        self.lbl_angles.setStyleSheet("color:#cfe3ff; font-size:11pt;")
        pos_layout.addWidget(self.lbl_angles)

        pos_box.setLayout(pos_layout)

        # 2 khung đặt cạnh nhau trong 1 hàng ngang, canh giữa cột trái.
        # Cả hai khung đều dùng stretch=1 để chia đều chiều ngang.
        pos_box.setMinimumWidth(300)
        info_row = QHBoxLayout()
        info_row.setSpacing(10)
        info_row.addStretch(1)
        info_row.addWidget(t4_box, 1)
        info_row.addWidget(pos_box, 1)
        info_row.addStretch(1)
        controls_col.addLayout(info_row)

        # ---- 2) KHUNG ĐIỀU KHIỂN X / Y / Z (tam giác đều, bo góc) ----
        xyz_box = QGroupBox("ĐIỀU KHIỂN X / Y / Z / θ4")
        xyz_box.setObjectName("compactBox")
        xyz_layout = QHBoxLayout()
        xyz_layout.setSpacing(18)
        xyz_layout.setContentsMargins(14, 8, 14, 8)

        # Ô vuông cho mọi tam giác → tất cả cùng kích thước, không méo.
        CELL = 100

        # -- pad XY: 3×3 ô vuông, mỗi ô chứa 1 tam giác đều (hoặc nhãn XY) --
        xy_grid = QGridLayout()
        xy_grid.setSpacing(6)
        xy_grid.setContentsMargins(0, 0, 0, 0)

        self.btn_y_plus  = TriangleButton("up",    "X+"); self.btn_y_plus.setFixedSize(CELL, CELL)
        self.btn_y_minus = TriangleButton("down",  "X-"); self.btn_y_minus.setFixedSize(CELL, CELL)
        self.btn_x_minus = TriangleButton("left",  "Y-"); self.btn_x_minus.setFixedSize(CELL, CELL)
        self.btn_x_plus  = TriangleButton("right", "Y+"); self.btn_x_plus.setFixedSize(CELL, CELL)

        self.btn_center_info = QLabel("XY")
        self.btn_center_info.setAlignment(Qt.AlignCenter)
        self.btn_center_info.setFixedSize(CELL, CELL)
        self.btn_center_info.setStyleSheet(
            "font-size:14pt; font-weight:800; color:#9fb6d6;"
        )

        xy_grid.addWidget(self.btn_y_plus,      0, 1)
        xy_grid.addWidget(self.btn_x_minus,     1, 0)
        xy_grid.addWidget(self.btn_center_info, 1, 1)
        xy_grid.addWidget(self.btn_x_plus,      1, 2)
        xy_grid.addWidget(self.btn_y_minus,     2, 1)

        self.btn_y_plus.clicked.connect(lambda: self.jog(dx=self.ctx.jog_step_xy))
        self.btn_y_minus.clicked.connect(lambda: self.jog(dx=-self.ctx.jog_step_xy))
        self.btn_x_plus.clicked.connect(lambda: self.jog(dy=self.ctx.jog_step_xy))
        self.btn_x_minus.clicked.connect(lambda: self.jog(dy=-self.ctx.jog_step_xy))

        # Bọc XY pad trong 1 VBox có stretch dọc để canh giữa theo chiều cao.
        xy_wrap = QVBoxLayout()
        xy_wrap.addStretch(1)
        xy_wrap.addLayout(xy_grid)
        xy_wrap.addStretch(1)
        xyz_layout.addLayout(xy_wrap)

        # -- đường phân cách dọc giữa XY và Z --
        sep = QLabel("")
        sep.setFixedWidth(2)
        sep.setStyleSheet("background-color:#2a4a75;")
        xyz_layout.addWidget(sep)

        # -- cột Z: 2 tam giác đều cùng cỡ XY, căn giữa dọc --
        z_wrap = QVBoxLayout()
        z_wrap.addStretch(1)

        self.btn_z_plus  = TriangleButton("up",   "Z-"); self.btn_z_plus.setFixedSize(CELL, CELL)
        self.btn_z_minus = TriangleButton("down", "Z+"); self.btn_z_minus.setFixedSize(CELL, CELL)

        z_wrap.addWidget(self.btn_z_plus,  0, Qt.AlignHCenter)
        z_wrap.addSpacing(6)
        z_wrap.addWidget(self.btn_z_minus, 0, Qt.AlignHCenter)

        z_wrap.addStretch(1)

        self.btn_z_plus.clicked.connect(lambda: self.jog(dz=-self.ctx.jog_step_z))
        self.btn_z_minus.clicked.connect(lambda: self.jog(dz=self.ctx.jog_step_z))

        # -- 2 mũi tên vòng cung 2 bên cột Z: xoay bậc tự do 4 (θ4) --
        #    TRÁI: ngược chiều kim đồng hồ | PHẢI: cùng chiều kim đồng hồ.
        step_txt = f"{DOF4_STEP_DEG:g}°"
        self.btn_dof4_ccw = ArcArrowButton(clockwise=False, label=step_txt)
        self.btn_dof4_ccw.setFixedSize(CELL, CELL)
        self.btn_dof4_ccw.setToolTip(f"Quay bậc tự do 4: {step_txt} ngược chiều kim đồng hồ")
        self.btn_dof4_ccw.clicked.connect(lambda: self.rotate_dof4(clockwise=False))

        self.btn_dof4_cw = ArcArrowButton(clockwise=True, label=step_txt)
        self.btn_dof4_cw.setFixedSize(CELL, CELL)
        self.btn_dof4_cw.setToolTip(f"Quay bậc tự do 4: {step_txt} cùng chiều kim đồng hồ")
        self.btn_dof4_cw.clicked.connect(lambda: self.rotate_dof4(clockwise=True))

        def _centered(widget):
            col = QVBoxLayout()
            col.addStretch(1)
            col.addWidget(widget, 0, Qt.AlignHCenter)
            col.addStretch(1)
            return col

        z_row = QHBoxLayout()
        z_row.setSpacing(4)
        z_row.addLayout(_centered(self.btn_dof4_ccw))
        z_row.addLayout(z_wrap)
        z_row.addLayout(_centered(self.btn_dof4_cw))
        xyz_layout.addLayout(z_row)

        xyz_box.setLayout(xyz_layout)
        controls_col.addWidget(xyz_box)

        # ---- 3) HÀNG: BƯỚC DI CHUYỂN + TỐC ĐỘ (XY và Z cùng 1 hàng) ----
        step_speed_row = QHBoxLayout()
        step_speed_row.setSpacing(10)

        step_box = QGroupBox("BƯỚC (mm/step)")
        step_box.setObjectName("compactBox")
        step_box.setMaximumWidth(320)
        step_layout = QHBoxLayout()
        step_layout.setSpacing(8)
        step_layout.setContentsMargins(8, 4, 8, 4)

        step_layout.addWidget(QLabel("XY:"))
        self.spin_step_xy = QDoubleSpinBox()
        self.spin_step_xy.setRange(0.1, 50.0)
        self.spin_step_xy.setValue(self.ctx.jog_step_xy)
        self.spin_step_xy.setSingleStep(0.5)
        self.spin_step_xy.setMinimumWidth(80)
        self.spin_step_xy.valueChanged.connect(self._on_step_xy_changed)
        step_layout.addWidget(self.spin_step_xy)

        step_layout.addSpacing(6)
        step_layout.addWidget(QLabel("Z:"))
        self.spin_step_z = QDoubleSpinBox()
        self.spin_step_z.setRange(0.1, 50.0)
        self.spin_step_z.setValue(self.ctx.jog_step_z)
        self.spin_step_z.setSingleStep(0.5)
        self.spin_step_z.setMinimumWidth(80)
        self.spin_step_z.valueChanged.connect(self._on_step_z_changed)
        step_layout.addWidget(self.spin_step_z)

        step_box.setLayout(step_layout)
        step_speed_row.addWidget(step_box, 0)

        speed_box = QGroupBox("TỐC ĐỘ (mm/s)")
        speed_box.setObjectName("compactBox")
        speed_layout = QVBoxLayout()
        self.slider_speed = QSlider(Qt.Horizontal)
        self.slider_speed.setObjectName("speedSlider")
        self.slider_speed.setMinimum(SPEED_MIN)
        self.slider_speed.setMaximum(SPEED_MAX)
        self.slider_speed.setValue(int(self.ctx.jog_speed))
        self.slider_speed.setTickPosition(QSlider.TicksBelow)
        self.slider_speed.setTickInterval(10)
        self.slider_speed.valueChanged.connect(self._on_speed_changed)

        self.lbl_speed_value = QLabel(f"{int(self.ctx.jog_speed)} mm/s")
        self.lbl_speed_value.setAlignment(Qt.AlignCenter)
        self.lbl_speed_value.setStyleSheet("font-size:11pt; font-weight:600; color:#4da6ff;")

        speed_layout.addWidget(self.slider_speed)
        speed_layout.addWidget(self.lbl_speed_value)
        speed_box.setLayout(speed_layout)
        step_speed_row.addWidget(speed_box)

        controls_col.addLayout(step_speed_row)

        # ---- 4) HOME (3 nút xếp NGANG, chia đều, chiều cao gọn) ----
        home_box = QGroupBox("HOME")
        home_box.setObjectName("compactBox")
        home_layout = QVBoxLayout()
        home_layout.setSpacing(4)
        home_layout.setContentsMargins(8, 2, 8, 2)

        HOME_BTN_H = 34

        home_btn_row = QHBoxLayout()
        home_btn_row.setSpacing(8)

        def mk_home_btn(text, obj_name):
            b = QPushButton(text)
            b.setObjectName(obj_name)
            b.setFixedHeight(HOME_BTN_H)
            b.setStyleSheet(
                f"QPushButton#{obj_name} {{ "
                f"min-height: {HOME_BTN_H}px; max-height: {HOME_BTN_H}px; "
                f"font-size: 12pt; font-weight: 700; padding: 2px 6px; }}"
            )
            return b

        self.btn_home_wait = mk_home_btn("HOME", "homeBtn")
        self.btn_home_wait.clicked.connect(self.on_home_wait)
        home_btn_row.addWidget(self.btn_home_wait, 1)

        self.btn_goto_saved_home = mk_home_btn("ĐẾN HOME ĐÃ LƯU", "homeBtn")
        self.btn_goto_saved_home.clicked.connect(self.on_goto_saved_home)
        home_btn_row.addWidget(self.btn_goto_saved_home, 1)

        self.btn_save_home = mk_home_btn("LƯU HOME AN TOÀN", "saveBtn")
        self.btn_save_home.clicked.connect(self.on_save_home)
        home_btn_row.addWidget(self.btn_save_home, 1)

        home_layout.addLayout(home_btn_row)

        self.lbl_saved_home = QLabel(self._home_str())
        self.lbl_saved_home.setStyleSheet("color:#4cd98a; font-size:10.5pt;")
        self.lbl_saved_home.setWordWrap(True)
        home_layout.addWidget(self.lbl_saved_home)

        home_box.setLayout(home_layout)
        controls_col.addWidget(home_box)

        self.lbl_manual_status = QLabel("SẴN SÀNG.")
        self.lbl_manual_status.setWordWrap(True)
        self.lbl_manual_status.setStyleSheet("color:#cfe3ff; font-size:10.5pt;")
        controls_col.addWidget(self.lbl_manual_status)
        controls_col.addStretch()

        # ================= CỘT PHẢI: ĐỒ THỊ REAL-TIME (luôn hiển thị) =================
        chart_col = QVBoxLayout()

        chart_box = QGroupBox("ĐỒ THỊ GÓC THEO QUỸ ĐẠO")
        chart_box.setObjectName("compactBox")
        chart_layout = QVBoxLayout()

        self.fig = Figure(figsize=(4.8, 6.2))
        self.fig.patch.set_facecolor("#0f2540")
        self.fig.subplots_adjust(hspace=0.45, left=0.12, right=0.97, top=0.93, bottom=0.09)
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setStyleSheet("background-color:#0f2540;")

        self.ax_link = self.fig.add_subplot(2, 1, 1)
        self.ax_motor = self.fig.add_subplot(2, 1, 2)
        self._style_axis(self.ax_link, "Góc 3 cánh tay chủ động (Chưa qua tỷ số truyền)")
        self._style_axis(self.ax_motor, "Góc 3 động cơ (Đã qua tỷ số truyền)")
        self.ax_motor.set_xlabel("t (s)", color="#cfe3ff")

        self._lines_link = [
            self.ax_link.plot([], [], color=c, lw=2, label=f"Cánh tay {i + 1}")[0]
            for i, c in enumerate(ARM_COLORS)
        ]
        leg1 = self.ax_link.legend(loc="upper right", fontsize=8, facecolor="#0f2540", edgecolor="#2a4a75")
        for t in leg1.get_texts():
            t.set_color("#e8f1ff")

        self._lines_motor = [
            self.ax_motor.plot([], [], color=c, lw=2, ls="--", label=f"Motor {i + 1}")[0]
            for i, c in enumerate(ARM_COLORS)
        ]
        leg2 = self.ax_motor.legend(loc="upper right", fontsize=8, facecolor="#0f2540", edgecolor="#2a4a75")
        for t in leg2.get_texts():
            t.set_color("#e8f1ff")

        chart_layout.addWidget(self.canvas, 1)

        # -- Tham số tỷ số truyền, chỉnh trực tiếp -> áp dụng ngay cho đồ thị dưới --
        gear_row = QHBoxLayout()
        gear_row.setSpacing(8)
        gear_row.addWidget(QLabel("u:"))
        self.spin_gear_ratio = QDoubleSpinBox()
        self.spin_gear_ratio.setRange(0.1, 50.0)
        self.spin_gear_ratio.setSingleStep(0.1)
        self.spin_gear_ratio.setDecimals(2)
        self.spin_gear_ratio.setValue(float(self.ctx.cfg.get("manual_gear_ratio", GEAR_RATIO_DEFAULT)))
        self.spin_gear_ratio.valueChanged.connect(self._on_gear_params_changed)
        gear_row.addWidget(self.spin_gear_ratio)
        gear_row.addStretch()
        chart_layout.addLayout(gear_row)

        chart_box.setLayout(chart_layout)
        chart_col.addWidget(chart_box)

        root.addLayout(controls_col, 5)
        root.addLayout(chart_col, 5)

    # ---------------- Đồ thị real-time ----------------
    def _style_axis(self, ax, title):
        ax.set_facecolor("#0f2540")
        ax.set_title(title, color="#e8f1ff", fontsize=9.5, fontweight="bold")
        ax.set_ylabel("Góc (độ)", color="#cfe3ff")
        ax.tick_params(colors="#9fb6d6", labelsize=8)
        for spine in ax.spines.values():
            spine.set_color("#2a4a75")
        ax.grid(True, color="#2a4a75", alpha=0.4)

    def _reset_chart(self):
        for buf in self._buf_theta:
            buf.clear()
        for buf in self._buf_motor:
            buf.clear()
        self._redraw_chart()

    def _redraw_chart(self):
        x = list(range(len(self._buf_theta[0])))
        for i in range(3):
            self._lines_link[i].set_data(x, list(self._buf_theta[i]))
            self._lines_motor[i].set_data(x, list(self._buf_motor[i]))
        for ax in (self.ax_link, self.ax_motor):
            ax.relim()
            ax.autoscale_view()
        self.canvas.draw_idle()

    def _theta_to_motor(self, theta_deg):
        return theta_deg * self.spin_gear_ratio.value()

    def _append_traj_point(self, pos):
        """Tính góc 3 cánh tay chủ động (động học nghịch) tại điểm quỹ đạo
        `pos` (X,Y,Z) rồi đẩy vào bộ đệm đồ thị real-time. Gọi mỗi khi robot
        vừa đi tới 1 điểm nội suy trên quỹ đạo thẳng (LinearMotionWorker)."""
        try:
            t1, t2, t3 = inverse_kinematics(*pos)
        except Exception:
            return
        for i, t in enumerate((t1, t2, t3)):
            self._buf_theta[i].append(t)
            self._buf_motor[i].append(self._theta_to_motor(t))
        self._redraw_chart()

    def _on_gear_params_changed(self, _value):
        # Tính lại toàn bộ đường Motor hiện có theo tỷ số/offset mới -
        # không cần chạy lại chuyển động mới thấy kết quả ngay.
        for i in range(3):
            self._buf_motor[i].clear()
            self._buf_motor[i].extend(self._theta_to_motor(t) for t in self._buf_theta[i])
        self._redraw_chart()
        self.ctx.cfg["manual_gear_ratio"] = self.spin_gear_ratio.value()
        save_config(self.ctx.cfg)

    def _home_str(self):
        x, y, z = self.ctx.home_pos
        return f"Home đã lưu: X={x:.2f}  Y={y:.2f}  Z={z:.2f}"

    def _on_step_xy_changed(self, v):
        self.ctx.jog_step_xy = v

    def _on_step_z_changed(self, v):
        self.ctx.jog_step_z = v

    def _on_speed_changed(self, v):
        self.ctx.jog_speed = float(v)
        self.lbl_speed_value.setText(f"{v} mm/s")

    def _update_pos_display(self):
        x, y, z = self.ctx.current_pos
        self.lbl_pos.setText(f"X:{x:.2f}  Y:{y:.2f}  Z:{z:.2f}")
        try:
            t1, t2, t3 = inverse_kinematics(x, y, z)
            self.lbl_angles.setText(f"θ1:{t1:.2f}°  θ2:{t2:.2f}°  θ3:{t3:.2f}°")
        except Exception:
            self.lbl_angles.setText("θ1:--°  θ2:--°  θ3:--°  (ngoài vùng làm việc)")

    def _set_manual_busy(self, busy, msg=""):
        self.ctx.busy = busy
        for b in (self.btn_x_plus, self.btn_x_minus, self.btn_y_plus, self.btn_y_minus,
                  self.btn_z_plus, self.btn_z_minus, self.btn_home_wait,
                  self.btn_goto_saved_home, self.btn_save_home,
                  self.btn_dof4_cw, self.btn_dof4_ccw):
            b.setEnabled(not busy)
        if msg:
            self.lbl_manual_status.setText(msg)

    # ------------------------------------------------------------------
    # BẬC TỰ DO 4 (θ4) - xoay qua UART 2, mỗi lần nhấn cố định DOF4_STEP_DEG
    # ------------------------------------------------------------------
    @staticmethod
    def _wrap_180(angle_deg):
        return ((angle_deg + 180.0) % 360.0) - 180.0

    def _update_theta4_display(self):
        self.lbl_theta4.setText(f"θ4: {self._wrap_180(self.ctx.theta4_deg):+.2f}°")
        uart2 = getattr(self.ctx, "pneu_uart", None)
        connected = bool(uart2 is not None and uart2.is_connected)
        if connected:
            self.lbl_uart2_state.setText("UART 2: ĐÃ KẾT NỐI")
            self.lbl_uart2_state.setStyleSheet("color:#4cd98a; font-size:10.5pt; font-weight:700;")
        else:
            self.lbl_uart2_state.setText("UART 2: CHƯA KẾT NỐI")
            self.lbl_uart2_state.setStyleSheet("color:#9aa7b8; font-size:10.5pt; font-weight:700;")

    def _set_dof4_buttons_enabled(self, enabled):
        self.btn_dof4_cw.setEnabled(enabled)
        self.btn_dof4_ccw.setEnabled(enabled)

    def rotate_dof4(self, clockwise):
        """Quay bậc tự do 4 đúng DOF4_STEP_DEG độ, gửi "STEP:<độ>" qua UART 2.

        FIX: trước đây gọi thẳng uart2.step_rotate(...) đồng bộ ngay trong
        hàm xử lý click (chạy trên GUI thread) -> nếu lệnh bị chặn (chờ ACK,
        cổng serial bận...) cả giao diện sẽ đơ. Giờ chuyển sang chạy qua
        Dof4RotateWorker (QThread riêng), giống hệt cách LinearMotionWorker
        đã xử lý cho jog X/Y/Z - không còn gọi UART trực tiếp trên GUI thread.
        """
        if self.ctx.busy:
            return
        if self._dof4_worker is not None and self._dof4_worker.isRunning():
            return

        uart2 = getattr(self.ctx, "pneu_uart", None)
        if uart2 is None or not uart2.is_connected:
            self._update_theta4_display()
            self.lbl_manual_status.setText(
                "⚠ Chưa kết nối UART 2"
            )
            return

        delta = DOF4_STEP_DEG if clockwise else -DOF4_STEP_DEG
        direction_txt = "CÙNG" if clockwise else "NGƯỢC"

        # Khóa 2 nút quay NGAY khi bấm (trước khi lệnh gửi xong) để tránh bấm
        # dồn trong lúc worker nền đang chạy.
        self._set_dof4_buttons_enabled(False)
        self.lbl_manual_status.setText(
            f"↻ Đang gửi lệnh quay {DOF4_STEP_DEG:g}° {direction_txt} chiều kim đồng hồ..."
        )

        worker = Dof4RotateWorker(uart2.step_rotate, delta * DOF4_CW_SIGN)
        worker.finished_ok.connect(lambda ok: self._on_dof4_rotate_done(ok, delta, direction_txt))
        worker.error.connect(self._on_dof4_rotate_error)
        self._dof4_worker = worker
        # FIX: _track_worker() (trong shared_state.py) đã tự gọi worker.start()
        # bên trong nó rồi -> KHÔNG được gọi start() thêm lần nữa ở đây, nếu
        # không 2 luồng thật sẽ cùng chạy run() song song, tranh chấp UART.
        self._track_worker(worker)

    def _on_dof4_rotate_done(self, ok, delta, direction_txt):
        self._dof4_worker = None
        if ok:
            self.ctx.theta4_deg += delta
            self._update_theta4_display()
            self.lbl_manual_status.setText(
                f"↻ Đã quay {DOF4_STEP_DEG:g}° {direction_txt} chiều kim đồng hồ."
            )
        else:
            self.lbl_manual_status.setText("⚠ Gửi lệnh quay bậc 4 thất bại (kiểm tra UART 2).")

        # Khóa 2 nút quay thêm 1 khoảng ngắn để step quay xong rồi mới nhận lệnh tiếp.
        QTimer.singleShot(
            DOF4_COOLDOWN_MS,
            lambda: self._set_dof4_buttons_enabled(not self.ctx.busy),
        )

    def _on_dof4_rotate_error(self, err):
        self._dof4_worker = None
        self._set_dof4_buttons_enabled(not self.ctx.busy)
        self.lbl_manual_status.setText(f"⚠ Lỗi khi quay bậc 4: {err}")

    def on_reset_theta4(self):
        self.ctx.theta4_deg = 0.0
        self._update_theta4_display()
        self.lbl_manual_status.setText("θ4 đã được đặt lại về 0°.")

    # ------------------------------------------------------------------
    # Chuyển động thẳng (straight-line, không rung) dùng LinearMotionWorker
    # ------------------------------------------------------------------
    def _run_linear_motion(self, target_pos, busy_msg, on_finished):
        """Khởi chạy 1 quỹ đạo thẳng từ vị trí hiện tại tới target_pos, dùng
        tốc độ đang đặt trên thanh trượt. `on_finished(ok)` được gọi khi xong.
        Đồ thị real-time được reset và bắt đầu vẽ lại từ điểm xuất phát."""
        if self.ctx.busy:
            return
        if self._motion_worker is not None and self._motion_worker.isRunning():
            return

        self._set_manual_busy(True, busy_msg)

        # Reset đồ thị và vẽ điểm xuất phát (t=0) trước khi robot bắt đầu di
        # chuyển, để đường vẽ luôn bắt đầu đúng từ vị trí hiện tại.
        self._reset_chart()
        self._append_traj_point(list(self.ctx.current_pos))

        worker = LinearMotionWorker(
            send_fn=self.ctx.planner.send_position,
            start_pos=list(self.ctx.current_pos),
            end_pos=list(target_pos),
            speed_mm_s=self.ctx.jog_speed,
        )
        worker.point_reached.connect(self._on_traj_point)
        worker.finished_ok.connect(lambda ok: self._on_traj_finished(ok, target_pos, on_finished))
        worker.error.connect(self._on_traj_error)
        self._motion_worker = worker
        # FIX: _track_worker() đã tự gọi worker.start() bên trong -> KHÔNG gọi
        # start() thêm ở đây nữa. Gọi 2 lần khiến 2 luồng cùng chạy run() song
        # song, cùng tranh chấp UART -> đây là nguyên nhân chính gây "đơ" khi
        # jog/về Home (bug đã tồn tại từ code gốc, không phải do lượt sửa trước).
        self._track_worker(worker)

    def _on_traj_point(self, pt):
        # Cập nhật hiển thị vị trí/góc + đồ thị real-time theo thời gian thực
        # khi robot đang di chuyển trên quỹ đạo thẳng.
        self.ctx.current_pos = list(pt)
        self._update_pos_display()
        self._append_traj_point(pt)

    def _on_traj_finished(self, ok, target_pos, on_finished):
        self._motion_worker = None
        self._set_manual_busy(False)
        if ok:
            self.ctx.current_pos = list(target_pos)
            self._update_pos_display()
        on_finished(ok)

    def _on_traj_error(self, err):
        self._motion_worker = None
        self._set_manual_busy(False)
        self.lbl_manual_status.setText(f"⚠ Lỗi: {err}")

    def jog(self, dx=0.0, dy=0.0, dz=0.0):
        if self.ctx.busy:
            return
        new_pos = [self.ctx.current_pos[0] + dx, self.ctx.current_pos[1] + dy, self.ctx.current_pos[2] + dz]

        try:
            inverse_kinematics(*new_pos)
        except Exception as e:
            self.lbl_manual_status.setText(f"⚠ Ngoài vùng làm việc: {e}")
            return

        def _finished(ok):
            if ok:
                self.lbl_manual_status.setText("Sẵn sàng.")
            else:
                self.lbl_manual_status.setText("⚠ Gửi lệnh thất bại (kiểm tra kết nối UART Robot).")

        self._run_linear_motion(new_pos, "Đang di chuyển (đường thẳng)...", _finished)

    def on_home_wait(self):
        if self.ctx.busy:
            return
        if not self.ctx.robot_uart.is_connected:
            self.lbl_manual_status.setText(
                "⚠ Chưa kết nối Robot (UART 1). Hãy kết nối ở tab KẾT NỐI trước."
            )
            return

        self._set_manual_busy(
            True, "Đang HOME (chạm 3 công tắc hành trình)... chờ STM32 phản hồi READY"
        )

        worker = FnWorker(home_and_wait, self.ctx.robot_uart, timeout=20.0)
        worker.done_ok.connect(self._on_home_wait_done)
        worker.done_err.connect(self._on_jog_error)
        # FIX (ĐÍNH CHÍNH lượt sửa trước): _track_worker() (shared_state.py)
        # đã tự gọi worker.start() bên trong nó rồi. Lượt trước tôi từng thêm
        # 1 dòng worker.start() ở đây do đoán nhầm _track_worker không start
        # giùm -> khiến start() bị gọi 2 LẦN -> 2 luồng cùng chạy home_and_wait
        # song song, cùng gửi lệnh HOME / đọc phản hồi UART chồng chéo nhau
        # -> đây mới là nguyên nhân thật sự gây "đơ" khi bấm HOME. Đã gỡ bỏ.
        self._track_worker(worker)

    def _on_jog_error(self, err):
        self._set_manual_busy(False)
        self.lbl_manual_status.setText(f"⚠ Lỗi: {err}")

    def _on_home_wait_done(self, ready):
        self._set_manual_busy(False)
        if ready:
            msg = (
                "✔ Đã HOME xong (chạm công tắc hành trình, encoder = 0).\n"
                "Bấm 'ĐẾN HOME ĐÃ LƯU' để về vị trí an toàn."
            )
            self.lbl_manual_status.setText(msg)
            QMessageBox.information(self, "HOME", msg)
        else:
            msg = "⚠ Không nhận được READY từ STM32 (kiểm tra dây UART1 / công tắc hành trình)."
            self.lbl_manual_status.setText(msg)
            QMessageBox.warning(self, "HOME", msg)

    def on_goto_saved_home(self):
        if self.ctx.busy:
            return

        def _finished(ok):
            if ok:
                self.lbl_manual_status.setText("✔ Đã đến Home an toàn.")
                QMessageBox.information(self, "HOME", "✔ Đã đến Home an toàn.")
            else:
                self.lbl_manual_status.setText("⚠ Gửi lệnh thất bại (kiểm tra kết nối UART Robot).")

        self._run_linear_motion(
            list(self.ctx.home_pos),
            "Đang di chuyển tới Home đã lưu (đường thẳng)...",
            _finished,
        )

    def on_save_home(self):
        self.ctx.home_pos = list(self.ctx.current_pos)
        # Cập nhật HOME cho MỌI planner để không planner nào còn giữ HOME cũ
        # cho tới khi khởi động lại app.
        for name in ("planner", "planner_dof4", "planner_run4dof", "planner_repeat", "planner_3dof"):
            planner = getattr(self.ctx, name, None)
            if planner is not None:
                planner.HOME = tuple(self.ctx.home_pos)
        self.ctx.cfg["home_position"] = self.ctx.home_pos
        save_config(self.ctx.cfg)
        self.lbl_saved_home.setText(self._home_str())
        self.lbl_manual_status.setText("💾 Đã lưu vị trí hiện tại làm Home an toàn.")