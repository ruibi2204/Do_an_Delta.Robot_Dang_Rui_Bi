# Bộ chương trình Calib Camera (bàn cờ 9x7)

Yêu cầu: `pip install opencv-python numpy`

## 3 file:
1. **capture_images.py** — chụp ảnh bàn cờ từ webcam
2. **calibrate_camera.py** — tính toán calib từ các ảnh đã chụp
3. **undistort_demo.py** — dùng kết quả calib để khử méo ảnh/video

## Các bước sử dụng

### Bước 1: Chụp ảnh bàn cờ (10–20 ảnh, nhiều góc độ/khoảng cách khác nhau)
```bash
python capture_images.py --board_w 9 --board_h 7 --out_dir images --camera 0
```
Đưa bàn cờ vào khung hình, khi thấy các chấm màu xanh nối lên bàn cờ (nghĩa là
đã nhận diện được) thì nhấn **SPACE** để lưu ảnh. Chụp ở nhiều góc nghiêng,
khoảng cách, và vị trí khác nhau trong khung hình (góc trên/dưới/trái/phải/giữa)
để calib chính xác. Nhấn **q** để thoát.

### Bước 2: Chạy calib
```bash
python calibrate_camera.py --images_dir images --board_w 9 --board_h 7 --square_size 25 --output calib_result
```
- `--square_size`: chiều dài cạnh 1 ô vuông trên bàn cờ, đo bằng thước (đơn vị
  mm hoặc cm tùy bạn chọn, chỉ cần dùng nhất quán).
- Script tự động thử cả 9x7 và 8x6 để tìm đúng số **góc trong** (giao điểm
  giữa các ô đen/trắng) — vì OpenCV cần số góc trong, không phải số ô vuông.
  Nếu bàn cờ của bạn có 9x7 ô vuông thì số góc trong thực tế là 8x6, script
  sẽ tự phát hiện và báo cho bạn.
- Kết quả in ra: ma trận camera (fx, fy, cx, cy), hệ số méo ống kính
  (k1, k2, p1, p2, k3), và sai số reprojection (càng thấp càng tốt, dưới 0.5
  pixel là tốt).
- Kết quả được lưu vào `calib_result.npz` (dùng lại bằng Python/numpy) và
  `calib_result.yaml` (dùng lại bằng OpenCV ở C++/Python khác).

### Bước 3 (tuỳ chọn): Kiểm tra bằng cách khử méo ảnh
```bash
# Với 1 ảnh:
python undistort_demo.py --calib calib_result.npz --image test.jpg --out result.jpg

# Trực tiếp từ webcam (xem cạnh nhau: gốc | đã khử méo):
python undistort_demo.py --calib calib_result.npz --camera 0
```

## Lưu ý quan trọng
- **9x7 là số ô vuông hay số góc trong?** Nếu không chắc, cứ để mặc định
  `--board_w 9 --board_h 7`, script tự thử cả hai và báo kết quả đúng.
- Chụp càng nhiều ảnh (>=15) ở nhiều góc độ, khoảng cách, vị trí trong khung
  hình càng cho kết quả calib chính xác.
- In bàn cờ ra giấy phẳng, dán lên bề mặt cứng (bìa cứng, bảng) để tránh cong
  vênh làm sai lệch kết quả.
