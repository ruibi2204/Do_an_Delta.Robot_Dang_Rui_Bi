# ROBOT DELTA – BỘ ĐIỀU KHIỂN PID SIMULINK

## Cấu trúc file

| File | Chức năng |
|------|-----------|
| `delta_robot_pid.m` | Script chính: mô phỏng PID + tự tạo model Simulink |
| `build_delta_simulink.m` | Hàm riêng: chỉ tạo file `.slx` (dùng API Simulink) |

---

## Cách sử dụng

### Cách 1 – Chạy nhanh (không cần Simulink license)
```matlab
run('delta_robot_pid.m')
```
- Mô phỏng bằng vòng lặp MATLAB (RK4)
- Vẽ 9 đồ thị: góc, sai số, mô-men
- Tính RMSE từng khớp
- Tự động gọi `create_simulink_model()` để sinh file `.slx`

### Cách 2 – Chỉ tạo Simulink model
```matlab
build_delta_simulink()
```
Yêu cầu: **MATLAB R2019b+** có Simulink.

---

## Sơ đồ khối Simulink (mỗi khớp)

```
Ref_Ji ──┬──► [Sum] ──► [PID Controller] ──► [Plant 1/(Js+b)] ──► [Integrator] ──► θi
         │      ▲─────────────────────────────────────────────────────────────────────┘
         │    (phản hồi âm)
         └── [Scope / To Workspace]
```

**3 vòng điều khiển độc lập** – một vòng cho mỗi khớp (θ₁, θ₂, θ₃).

---

## Thông số mặc định

### Cơ học robot Delta
| Ký hiệu | Giá trị | Đơn vị |
|---------|---------|--------|
| L1 (cánh tay trên) | 0.30 | m |
| L2 (cánh tay dưới) | 0.60 | m |
| r_base | 0.20 | m |
| r_platform | 0.05 | m |
| m (tải) | 0.50 | kg |
| J (quán tính khớp) | 0.01 | kg.m² |
| b (cản) | 0.05 | N.m.s/rad |

### Tham số PID (3 khớp giống nhau)
| Tham số | Giá trị |
|---------|---------|
| Kp | 120 |
| Ki | 15 |
| Kd | 8 |
| τ_max | ±30 N.m |
| Anti-windup | Clamping |

### Quỹ đạo tham chiếu (hình sin lệch pha 120°)
```
θ1_ref(t) = 0.5·sin(π·t)
θ2_ref(t) = 0.5·sin(π·t + 2π/3)
θ3_ref(t) = 0.5·sin(π·t + 4π/3)
```

---

## Điều chỉnh PID

Thay đổi giá trị trong phần **Section 2** của `delta_robot_pid.m`:
```matlab
pid.Kp1 = 120;   pid.Ki1 = 15;   pid.Kd1 = 8;
```

**Hướng dẫn chỉnh thô:**
- Tăng `Kp` → đáp ứng nhanh hơn, dễ vọt lố
- Tăng `Ki` → giảm sai số tĩnh, dễ dao động
- Tăng `Kd` → giảm vọt lố, tăng ổn định

---

## Yêu cầu
- MATLAB R2019b trở lên
- Simulink (để chạy `build_delta_simulink.m`)
- Control System Toolbox (tùy chọn)
