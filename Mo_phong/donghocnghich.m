%==========================================================================
% MÔ PHỎNG ĐỘNG HỌC NGHỊCH ROBOT DELTA
%==========================================================================
clear; clc; close all;

%% 1. Thông số hình học (mm)
L1 = 130;   % Cánh tay chủ động
L2 = 298;   % Cánh tay bị động
R = 100;    % Bán kính đế cố định
r = 40;     % Bán kính đế di động

wb = R - r; % Khoảng cách hiệu dụng

%% 2. Tạo quỹ đạo chuyển động cho Bàn kẹp (End-effector)
% Cho robot chạy một đường tròn nằm ngang ở độ sâu Z = -250mm
t = linspace(0, 2*pi, 200); % Biến thời gian/chu kỳ (200 điểm)
R_trajectory = 80;          % Bán kính đường tròn quỹ đạo (mm)

X_path = R_trajectory * cos(t);
Y_path = R_trajectory * sin(t);
Z_path = -250 * ones(size(t)); % Độ sâu cố định -250mm

%% 3. Tính toán Động học nghịch cho từng điểm trên quỹ đạo
Theta1 = zeros(size(t));
Theta2 = zeros(size(t));
Theta3 = zeros(size(t));

for i = 1:length(t)
    x0 = X_path(i);
    y0 = Y_path(i);
    z0 = Z_path(i);
    
    % Tính toán góc cho 3 nhánh sử dụng hàm phụ trợ phía dưới
    Theta1(i) = delta_calcAngle(x0, y0, z0, L1, L2, wb, 0);   % Nhánh 1 (0 độ)
    Theta2(i) = delta_calcAngle(x0, y0, z0, L1, L2, wb, 120); % Nhánh 2 (120 độ)
    Theta3(i) = delta_calcAngle(x0, y0, z0, L1, L2, wb, 240); % Nhánh 3 (240 độ)
end

%% 4. Vẽ đồ thị 2 chiều thể hiện sự thay đổi của 3 góc Theta
figure('Color', [1 1 1]);
hold on; grid on;

plot(rad2deg(Theta1), 'r-', 'LineWidth', 2.5);
plot(rad2deg(Theta2), 'g-', 'LineWidth', 2.5);
plot(rad2deg(Theta3), 'b-', 'LineWidth', 2.5);

% Trang trí đồ thị đơn giản
title('Su thay doi cua 3 goc khop (Dong hoc nghich)');
xlabel('Diem tren quy dao (Thoi gian)');
ylabel('Goc quay Theta (Do)');
legend('Theta 1 (Truc 1)', 'Theta 2 (Truc 2)', 'Theta 3 (Truc 3)', 'Location', 'Best');

%% ========================================================================
% HÀM PHỤ TRỢ: TÍNH GÓC THETA CHO MỘT NHÁNH (Hình học giải tích)
% ========================================================================
function theta = delta_calcAngle(x0, y0, z0, L1, L2, wb, phi_deg)
    % Xoay hệ tọa độ của nhánh về trục X phẳng để giải toán 2D
    phi = deg2rad(phi_deg);
    x = x0*cos(phi) + y0*sin(phi);
    y = -x0*sin(phi) + y0*cos(phi);
    z = z0;
    
    % Tịnh tiến hệ tọa độ về tâm khớp quay của L1 trên đế
    x = x - wb;
    
    % Phương trình giao điểm đường tròn (L1) và mặt cầu (L2) dạng: A*cos(th) + B*sin(th) = C
    A = 2*x*L1;
    B = 2*z*L1;
    C = x^2 + y^2 + z^2 + L1^2 - L2^2;
    
    % Giải phương trình lượng giác bằng vạn năng
    delta = A^2 + B^2 - C^2;
    if delta >= 0
        % Chọn nghiệm phù hợp cấu hình robot (khớp khuỷu lồi ra ngoài)
        t1 = (-B - sqrt(delta)) / (A - C);
        theta = 2 * atan(t1);
    else
        theta = NaN; % Không tới được điểm này
    end
end