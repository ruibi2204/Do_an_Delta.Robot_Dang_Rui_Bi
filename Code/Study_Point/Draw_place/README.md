# Delta Robot Trajectory Controller

Phần mềm điều khiển quỹ đạo Robot Delta — Đồ án tốt nghiệp.

## Cấu trúc dự án

```
delta_robot/
├── main.py                          ← Chạy file này
├── kinematics/
│   ├── __init__.py
│   └── inverse_kinematics.py        ← Động học nghịch (Weierstrass)
├── trajectory/
│   ├── __init__.py
│   └── generator.py                 ← Sinh quỹ đạo tròn / vuông / tam giác
├── communication/
│   ├── __init__.py
│   └── uart_comm.py                 ← Giao tiếp Serial với stepper driver
└── gui/
    ├── __init__.py
    └── controller_gui.py            ← Giao diện Tkinter full-screen
```

## Thông số robot

| Tham số | Giá trị | Mô tả |
|---------|---------|-------|
| R       | 100 mm  | Bán kính mâm tĩnh |
| a       | 130 mm  | Chiều dài cánh tay trên |
| b       | 298 mm  | Chiều dài cánh tay dưới |
| r       | 35.3 mm | Bán kính mâm động |
| u       | 3.8     | Tỉ số truyền đai stepper |

## Cài đặt

```bash
pip install numpy pyserial
```

> `pyserial` là tùy chọn — nếu chưa cài, chương trình tự động dùng chế độ **DRY-RUN** (mô phỏng, không cần phần cứng).

## Chạy chương trình

```bash
cd delta_robot
python main.py
```

## Hướng dẫn sử dụng

1. **Chọn hình** (Tròn / Vuông / Tam giác) ở panel trái
2. **Nhập tham số** (bán kính, cạnh, tâm XY, độ cao Z)
3. **Kết nối Serial**: chọn cổng COM (hoặc để DRY-RUN để test)
4. Nhấn **▶ BẮT ĐẦU VẼ** — quan sát quỹ đạo trên canvas và góc khớp bên dưới
5. Nhấn **⌂ HOME** để đưa robot về vị trí gốc

## Định dạng UART

Mỗi bước gửi một dòng ASCII:

```
T1:<stepper1> T2:<stepper2> T3:<stepper3>\n
```

Trong đó `stepper_i = theta_i × 3.8` (tỉ số truyền đai).

**Ví dụ:**
```
T1:42.56 T2:-31.08 T3:38.95
HOME
```

## Mở rộng

- Thêm hình vẽ mới: tạo hàm trong `trajectory/generator.py` và thêm tab trong `gui/controller_gui.py`
- Thay đổi giao thức UART: chỉnh hàm `send_angles()` trong `communication/uart_comm.py`
- Thay đổi thông số robot: chỉnh hằng số trong `kinematics/inverse_kinematics.py`
