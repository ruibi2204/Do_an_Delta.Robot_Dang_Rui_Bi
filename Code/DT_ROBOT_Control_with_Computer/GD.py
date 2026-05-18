import tkinter as tk
from tkinter import messagebox


class RobotDeltaGUI:
    def __init__(self, root, on_send_callback=None):
        """
        :param root: Cửa sổ chính của Tkinter
        :param on_send_callback: Hàm con xử lý khi bấm nút "GỬI TỌA ĐỘ"
                                 (Sẽ liên kết với Main để truyền sang UART)
        """
        self.root = root
        self.root.title("BẢNG ĐIỀU KHIỂN ROBOT DELTA")

        # Lưu hàm callback để truyền dữ liệu đi
        self.on_send_callback = on_send_callback

        # ---------------------------------------------------------
        # 1. CẤU HÌNH KÍCH THƯỚC GIAO DIỆN (To khoảng nửa màn hình)
        # ---------------------------------------------------------
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()

        # Thiết lập giao diện chiếm 50% chiều rộng và 60% chiều cao màn hình
        window_width = int(screen_width * 0.45)
        window_height = int(screen_height * 0.60)

        # Căn giữa cửa sổ trên màn hình
        position_top = int(screen_height / 2 - window_height / 2)
        position_right = int(screen_width / 2 - window_width / 2)

        self.root.geometry(f"{window_width}x{window_height}+{position_right}+{position_top}")
        self.root.configure(bg="#2C3E50")  # Màu nền tối hiện đại, dịu mắt

        # Cấu hình font chữ to, rõ ràng
        font_title = ("Helvetica", 20, "bold")
        font_label = ("Helvetica", 16, "bold")
        font_entry = ("Helvetica", 18)
        font_button = ("Helvetica", 16, "bold")

        # ---------------------------------------------------------
        # 2. THIẾT KẾ TIÊU ĐỀ
        # ---------------------------------------------------------
        title_label = tk.Label(root, text="HỆ THỐNG ĐIỀU KHIỂN ROBOT DELTA",
                               font=font_title, fg="#1ABC9C", bg="#2C3E50", pady=20)
        title_label.pack()

        # Frame chứa các ô nhập tọa độ để căn chỉnh ngay ngắn
        input_frame = tk.Frame(root, bg="#2C3E50", pady=10)
        input_frame.pack(expand=True)

        # ---------------------------------------------------------
        # 3. CÁC Ô NHẬP TỌA ĐỘ X, Y, Z (Phóng to)
        # ---------------------------------------------------------
        # Trục X
        tk.Label(input_frame, text="Tọa độ X (mm):", font=font_label, fg="#ECF0F1", bg="#2C3E50", width=15,
                 anchor="e").grid(row=0, column=0, padx=10, pady=15)
        self.entry_x = tk.Entry(input_frame, font=font_entry, width=10, justify="center")
        self.entry_x.insert(0, "0.0")  # Giá trị mặc định
        self.entry_x.grid(row=0, column=1, padx=10, pady=15)

        # Trục Y
        tk.Label(input_frame, text="Tọa độ Y (mm):", font=font_label, fg="#ECF0F1", bg="#2C3E50", width=15,
                 anchor="e").grid(row=1, column=0, padx=10, pady=15)
        self.entry_y = tk.Entry(input_frame, font=font_entry, width=10, justify="center")
        self.entry_y.insert(0, "0.0")
        self.entry_y.grid(row=1, column=1, padx=10, pady=15)

        # Trục Z (Lưu ý Robot Delta hoạt động ở miền âm phía dưới)
        tk.Label(input_frame, text="Tọa độ Z (mm):", font=font_label, fg="#ECF0F1", bg="#2C3E50", width=15,
                 anchor="e").grid(row=2, column=0, padx=10, pady=15)
        self.entry_z = tk.Entry(input_frame, font=font_entry, width=10, justify="center")
        self.entry_z.insert(0, "-230.0")
        self.entry_z.grid(row=2, column=1, padx=10, pady=15)

        # ---------------------------------------------------------
        # 4. NÚT NHẤN ĐIỀU KHIỂN (To và màu sắc phân biệt rõ ràng)
        # ---------------------------------------------------------
        button_frame = tk.Frame(root, bg="#2C3E50", pady=20)
        button_frame.pack(fill="x", side="bottom")

        # Nút Gửi tọa độ (Màu xanh lá)
        self.btn_send = tk.Button(button_frame, text="GỬI TỌA ĐỘ", font=font_button,
                                  bg="#2ECC71", fg="white", width=15, height=2,
                                  activebackground="#27AE60", command=self.handle_send)
        self.btn_send.pack(side="left", expand=True, padx=20, pady=10)

        # Nút Xóa nhanh dữ liệu (Màu cam)
        self.btn_clear = tk.Button(button_frame, text="XÓA NHẬP LIỆU", font=font_button,
                                   bg="#E67E22", fg="white", width=15, height=2,
                                   activebackground="#D35400", command=self.handle_clear)
        self.btn_clear.pack(side="right", expand=True, padx=20, pady=10)

    # ---------------------------------------------------------
    # 5. CÁC HÀM XỬ LÝ SỰ KIỆN NÚT BẤM
    # ---------------------------------------------------------
    def handle_send(self):
        """Xử lý kiểm tra dữ liệu nhập vào và truyền đi"""
        try:
            x = float(self.entry_x.get())
            y = float(self.entry_y.get())
            z = float(self.entry_z.get())

            # Nếu có hàm callback kết nối với mạch, tiến hành gọi hàm đó
            if self.on_send_callback:
                self.on_send_callback(x, y, z)
            else:
                # Nếu chạy độc lập file GD.py, nó sẽ hiện thông báo để bạn test giao diện
                messagebox.showinfo("Thông báo Test", f"Đã xác nhận tọa độ mục tiêu:\nX = {x}\nY = {y}\nZ = {z}")

        except ValueError:
            messagebox.showerror("Lỗi nhập liệu", "Vui lòng chỉ nhập số thực (Ví dụ: 10.5 hoặc -200)!")

    def handle_clear(self):
        """Xóa nhanh dữ liệu cũ để nhập lại"""
        self.entry_x.delete(0, tk.END)
        self.entry_y.delete(0, tk.END)
        self.entry_z.delete(0, tk.END)
        self.entry_x.insert(0, "0.0")
        self.entry_y.insert(0, "0.0")
        self.entry_z.insert(0, "0.0")


# Đoạn code để chạy test độc lập giao diện khi bạn ấn Run file GD.py
if __name__ == "__main__":
    root = tk.Tk()
    app = RobotDeltaGUI(root)
    root.mainloop()