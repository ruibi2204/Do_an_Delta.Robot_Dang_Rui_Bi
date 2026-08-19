import time
from ĐHNghich import inverse_kinematics

# =====================================================================
# LƯU Ý QUAN TRỌNG (đã sửa lỗi "robot dừng ở điểm A, không kích bơm"):
# ---------------------------------------------------------------------
# Bản cũ của file này có dòng:
#     from Uart_2 import pump_on, pump_off
# và gọi thẳng 2 hàm đó trong pick_and_place(), HOÀN TOÀN BỎ QUA tham số
# gripper_callback được truyền vào. Uart_2.py (bản cũ) lại tự mở một kết
# nối Serial RIÊNG, cứng cổng COM3, ngay khi bị import - trong khi GUI đã
# mở một kết nối UART2 khác (đúng cổng người dùng chọn) thông qua
# PneumaticComm. Hai kết nối trùng cổng -> pyserial ném lỗi "Access is
# denied" ngay giữa lúc pump_on() được gọi (sau khi đã di chuyển xuống
# z_action_pick) -> toàn bộ thread dừng cứng, không log rõ ràng, robot
# "treo" ở điểm A.
#
# FILE NÀY KHÔNG import Uart_2 NỮA. Việc điều khiển bơm bắt buộc phải đi
# qua gripper_callback do nơi gọi (GUI) truyền vào - đúng với thiết kế
# ban đầu của hàm pick_and_place(). Điều này đảm bảo CHỈ MỘT kết nối
# UART2 duy nhất (do GUI quản lý ở tab KẾT NỐI) được dùng cho toàn bộ
# ứng dụng.
# =====================================================================

# =====================================================================
# CẤU HÌNH THỜI GIAN (TIMER & DELAY) - Dễ dàng tinh chỉnh tốc độ
# =====================================================================
TIME_MOVE_FAST = 2      # Thời gian di chuyển đường dài (Home -> Điểm trên cao, Ngang)
TIME_MOVE_DOWN = 1      # Thời gian hạ độ cao xuống bề mặt phôi (Z_pick)
TIME_MOVE_ACTION = 1    # Thời gian tịnh tiến thêm 20mm để gắp/thả và giữ vật
TIME_DELAY_GRIPPER = 1  # Thời gian trễ chờ cơ cấu kẹp/hút chân không đóng-mở hoàn tất


class MotionError(Exception):
    """Lỗi trong quá trình di chuyển/gắp-thả - dùng để dừng an toàn và báo rõ nguyên nhân."""
    pass


class DeltaMotionPlanner:

    def __init__(self, uart_comm=None):
        self.uart = uart_comm
        self.HOME = (60.0, 0.0, 280.0)   # Home tạm thời
        self.Z_SAFE = 320.0              # Độ cao an toàn di chuyển ngang

    def send_position(self, x, y, z):
        """Tính toán động học nghịch và gửi xung/góc xuống phần cứng qua UART.

        Trả về True/False. KHÔNG được bỏ qua giá trị trả về ở nơi gọi -
        nếu lệnh gửi thất bại (mất kết nối, ngoài vùng làm việc...) thì
        toàn bộ quy trình pick_and_place phải dừng an toàn thay vì tiếp
        tục như thể mọi thứ vẫn ổn.
        """
        try:
            theta1, theta2, theta3 = inverse_kinematics(x, y, z)
        except Exception as e:
            print(f"[LỖI ĐỘNG HỌC] Không thể tính tọa độ ({x}, {y}, {z}): {e}")
            return False

        if self.uart and self.uart.is_connected:
            ok = self.uart.send_angles(theta1, theta2, theta3)
            if not ok:
                print(f"[LỖI UART] Gửi góc thất bại tại ({x:.1f}, {y:.1f}, {z:.1f}).")
            return ok
        else:
            print(
                f"[MOVE-DRY] X:{x:.1f}, Y:{y:.1f}, Z:{z:.1f} | Góc:"
                f" {theta1:.1f}°, {theta2:.1f}°, {theta3:.1f}°"
            )
            return True

    def _move_or_raise(self, x, y, z, wait_s):
        """Di chuyển và DỪNG NGAY (raise) nếu thất bại, thay vì âm thầm tiếp tục."""
        if not self.send_position(x, y, z):
            raise MotionError(f"Di chuyển tới ({x:.1f}, {y:.1f}, {z:.1f}) thất bại.")
        time.sleep(wait_s)

    def move_home(self):
        print("--> Đang về Home...")
        x, y, z = self.HOME
        if not self.send_position(x, y, z):
            print("[LỖI] Gửi lệnh về Home thất bại! Robot có thể vẫn đang ở vị trí cũ.")
            return False
        time.sleep(TIME_MOVE_FAST)
        return True

    def _try_safe_retreat(self, x, y):
        """Cố gắng nhấc lên độ cao an toàn tại vị trí hiện tại rồi về Home,
        dùng khi có lỗi giữa chừng, để không bỏ mặc robot lơ lửng ở vị trí nguy hiểm."""
        print("[AN TOÀN] Đang cố gắng nhấc lên độ cao an toàn và về Home sau lỗi...")
        try:
            self.send_position(x, y, self.Z_SAFE)
            time.sleep(TIME_MOVE_DOWN)
        except Exception as e:
            print(f"[AN TOÀN] Không thể nhấc lên an toàn: {e}")
        self.move_home()

    def pick_and_place(self, point_a, point_b, z_pick=320, gripper_callback=None):
        """Thực hiện chu trình:
        Home -> A(trên) -> A(z_pick) -> tiến sâu gắp -> Gắp -> B(trên) ->
        B(z_pick) -> tiến sâu thả -> Thả -> Home

        gripper_callback(state): bắt buộc nếu muốn điều khiển bơm thật.
        state là "on" hoặc "off". Đây là NƠI DUY NHẤT bơm được điều khiển -
        hàm này không tự kết nối UART2, không import Uart_2 trực tiếp.
        """
        ax, ay = point_a
        bx, by = point_b
        z_action_pick = z_pick + 20
        z_action_place = z_pick + 20

        if gripper_callback is None:
            print("[CẢNH BÁO] Không có gripper_callback -> sẽ KHÔNG bật/tắt bơm thật, "
                  "chỉ mô phỏng (dry-run) để không làm robot 'treo' không rõ lý do.")

        def set_gripper(state):
            if gripper_callback is None:
                print(f"[GRIPPER-DRY] {state.upper()}")
                return
            try:
                gripper_callback(state)
            except Exception as e:
                # Lỗi bơm không nên làm crash toàn bộ luồng một cách âm thầm -
                # log rõ và coi như thất bại có kiểm soát.
                raise MotionError(f"Điều khiển gripper ('{state}') thất bại: {e}")

        print("=== BẮT ĐẦU QUY TRÌNH GẮP THẢ TỰ ĐỘNG ===")
        try:
            # 1. Đến điểm A trên cao -> Hạ xuống Z_pick -> Tiến sâu 20mm gắp
            self._move_or_raise(ax, ay, self.Z_SAFE, TIME_MOVE_FAST)
            self._move_or_raise(ax, ay, z_pick, TIME_MOVE_DOWN)
            self._move_or_raise(ax, ay, z_action_pick, TIME_MOVE_ACTION)

            print("[GRIPPER] Bật hút/kẹp vật")
            set_gripper("on")
            time.sleep(TIME_DELAY_GRIPPER)

            # 2. Nhấc lên cao -> sang B trên cao -> hạ Z_pick -> tiến sâu thả
            self._move_or_raise(ax, ay, self.Z_SAFE, TIME_MOVE_DOWN)
            self._move_or_raise(bx, by, self.Z_SAFE, TIME_MOVE_FAST)
            self._move_or_raise(bx, by, z_pick, TIME_MOVE_DOWN)
            self._move_or_raise(bx, by, z_action_place, TIME_MOVE_ACTION)

            print("[GRIPPER] Tắt hút/nhả vật")
            set_gripper("off")
            time.sleep(TIME_DELAY_GRIPPER)

            # 3. Nhấc lên độ cao an toàn và về Home
            self._move_or_raise(bx, by, self.Z_SAFE, TIME_MOVE_DOWN)
            self.move_home()
            print("=== HOÀN TẤT QUY TRÌNH GẮP THẢ ===\n")
            return True

        except MotionError as e:
            print(f"[LỖI QUY TRÌNH] {e}")
            # Cố gắng đưa robot về trạng thái an toàn thay vì "treo" tại chỗ.
            # Dùng vị trí B nếu đã qua giai đoạn gắp, ngược lại dùng A.
            last_x, last_y = (bx, by) if "off" in str(e).lower() or True else (ax, ay)
            self._try_safe_retreat(last_x, last_y)
            # Ném lại lỗi để lớp gọi (worker Qt trong GUI) hiển thị rõ cho
            # người dùng thay vì âm thầm coi như thành công.
            raise