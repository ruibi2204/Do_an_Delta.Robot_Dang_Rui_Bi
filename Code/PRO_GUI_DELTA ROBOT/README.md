# Robot Delta - Bo Dieu Khien (tach module)

## Cai dat
```bash
pip install PyQt5 pyserial opencv-python matplotlib numpy
```

## Chay GUI
```bash
python main_gui.py
```

## Cau truc file

| File | Chuc nang |
|---|---|
| `main_gui.py` | Giao dien chinh (5 tab), ghep toan bo cac module ben duoi |
| `kinematics.py` | Dong hoc nguoc: (X,Y,Z) -> goc khop (theta1,theta2,theta3) |
| `gear_ratio.py` | Ty so truyen dong co/khop, u = 3 |
| `uart_comm.py` | Giao tiep Serial, lenh dang `T1:.. T2:.. T3:.. F:..\n` |
| `camera_module.py` | Doc camera realtime (thread) + chup 1 frame don le |
| `vision_coords.py` | Phat hien vat theo mau (HSV) + hieu chinh camera -> toa do robot |
| `circle_trajectory.py` | Sinh danh sach diem cho quy dao hinh tron |

## Luong xu ly khi robot di chuyen toi 1 diem

```
(X, Y, Z)
   │  kinematics.inverse_kinematics()
   ▼
(theta1, theta2, theta3)   <- goc KHOP
   │  gear_ratio.joints_to_motors()   [u = 3]
   ▼
(m1, m2, m3)               <- goc TRUC DONG CO
   │  uart_comm.send_motor_angles()
   ▼
"T1:m1 T2:m2 T3:m3 F:feed\n"   -> gui qua Serial xuong robot
```

## Sua doi khi robot khac thong so

- **Kich thuoc co khi khac** (R, a, b, r): sua truc tiep trong `kinematics.py`.
- **Ty so truyen khac 3**: sua `GEAR_RATIO` trong `gear_ratio.py`.
- **Dinh dang lenh UART khac**: sua 3 ham `build_move_command()`,
  `build_home_command()`, `build_estop_command()` trong `uart_comm.py`.
- **Mau vat can nhan dien khac (khong phai mau do)**: chinh cac gia tri
  HSV trong tab "Camera && Nhan toa do" cua GUI (khong can sua code).

## Hieu chinh camera (calibration) de nhan toa do

1. Bat camera trong tab **Camera && Nhan toa do**.
2. Dat vat mau (mau da chon o phan HSV) tai 1 vi tri **da biet truoc toa do
   robot thuc te** (X, Y).
3. Bam **"Phat hien vat"** de lay toa do pixel.
4. Nhap dung X, Y thuc te vao o tuong ung, bam **"Them cap diem"**.
5. Lap lai buoc 2-4 tai it nhat **3 vi tri khac nhau** (khong thang hang).
6. Bam **"Tinh hieu chinh"**.
7. Tu do, moi lan "Phat hien vat" + "Di chuyen robot toi vat" se tu dong
   doi pixel -> toa do robot va di chuyen toi do.
