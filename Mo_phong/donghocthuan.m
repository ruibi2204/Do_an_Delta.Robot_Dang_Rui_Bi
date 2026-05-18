%==========================================================================
% CẬP NHẬT: MÔ PHỎNG VÙNG HOẠT ĐỘNG ROBOT DELTA CHUẨN ĐỐI XỨNG
%==========================================================================
clear; clc; close all;

%% 1. Thông số hình học (mm)
L1 = 130;   % Cánh tay chủ động
L2 = 298;   % Cánh tay bị động
R = 100;    % Bán kính đế cố định
r = 40;     % Bán kính đế di động

% Giới hạn góc quay thực tế khớp chủ động (độ)
theta_min = -30; 
theta_max = 90;  
step = 8 % Bước quét (giảm xuống để mịn hơn)

%% 2. Tạo lưới quét góc cho 3 trục
[T1, T2, T3] = meshgrid(theta_min:step:theta_max, ...
                        theta_min:step:theta_max, ...
                        theta_min:step:theta_max);
T1 = deg2rad(T1(:)); 
T2 = deg2rad(T2(:)); 
T3 = deg2rad(T3(:));

X = []; Y = []; Z = [];

% Khoảng cách hiệu dụng từ tâm robot đến các trục khớp khuỷu
e = r; 
w = R;
wb = w - e; 

%% 3. Vòng lặp tính toán Động học thuận chuẩn
for i = 1:length(T1)
    % Tọa độ khớp khuỷu (khớp nối giữa L1 và L2) của 3 nhánh
    % Nhánh 1 (Hướng 0 độ hoặc 90 độ tùy quy ước, ở đây dùng chuẩn 3 trục đối xứng)
    t1 = T1(i); t2 = T2(i); t3 = T3(i);
    
    % Nhánh 1: góc 0 rad (Dọc theo trục X)
    P1 = [wb + L1*cos(t1); 0; -L1*sin(t1)];
    
    % Nhánh 2: xoay 120 độ
    P2 = [ (wb + L1*cos(t2))*cos(2*pi/3); (wb + L1*cos(t2))*sin(2*pi/3); -L1*sin(t2) ];
    
    % Nhánh 3: xoay 240 độ
    P3 = [ (wb + L1*cos(t3))*cos(4*pi/3); (wb + L1*cos(t3))*sin(4*pi/3); -L1*sin(t3) ];
    
    % Giải giao điểm 3 mặt cầu tâm P1, P2, P3 bán kính L2
    x1 = P1(1); y1 = P1(2); z1 = P1(3);
    x2 = P2(1); y2 = P2(2); z2 = P2(3);
    x3 = P3(1); y3 = P3(2); z3 = P3(3);
    
    % Hệ phương trình tuyến tính trung gian
    a1 = 2*(x2 - x1); b1 = 2*(y2 - y1); c1 = 2*(z2 - z1); d1 = (x2^2+y2^2+z2^2) - (x1^2+y1^2+z1^2);
    a2 = 2*(x3 - x1); b2 = 2*(y3 - y1); c2 = 2*(z3 - z1); d2 = (x3^2+y3^2+z3^2) - (x1^2+y1^2+z1^2);
    
    % Khử x, y theo z
    den = a1*b2 - a2*b1;
    if abs(den) < 1e-6; continue; end
    
    X0 = (b2*d1 - b1*d2)/den;  Xz = (b1*c2 - b2*c1)/den;
    Y0 = (a1*d2 - a2*d1)/den;  Yz = (a2*c1 - a1*c2)/den;
    
    % Phương trình bậc 2 theo Z: A*z^2 + B*z + C = 0
    A_eq = Xz^2 + Yz^2 + 1;
    B_eq = 2*(Xz*(X0 - x1) + Yz*(Y0 - y1) - z1);
    C_eq = (X0 - x1)^2 + (Y0 - y1)^2 + z1^2 - L2^2;
    
    delta_eq = B_eq^2 - 4*A_eq*C_eq;
    if delta_eq >= 0
        % Chọn nghiệm thấp hơn (robot hoạt động bên dưới đế)
        z_sol = (-B_eq - sqrt(delta_eq)) / (2*A_eq);
        x_sol = X0 + Xz*z_sol;
        y_sol = Y0 + Yz*z_sol;
        
        % Kiểm tra điều kiện hình học thực tế tránh nghiệm ảo
        X = [X; x_sol]; Y = [Y; y_sol]; Z = [Z; z_sol];
    end
end

%% 4. Vẽ đồ thị đồ họa 3D
figure('Color', [1 1 1]);
hold on; grid on; axis equal;

% Vẽ đám mây điểm vùng làm việc
plot3(X, Y, Z, '.', 'Color', [0 0.4470 0.7410], 'MarkerSize', 4);

% Vẽ đường tròn đế cố định (Base) tại Z = 0
alpha = linspace(0, 2*pi, 100);
plot3(R*cos(alpha), R*sin(alpha), zeros(size(alpha)), 'r-', 'LineWidth', 3);

% Vẽ đường tròn đế di động (Platform) tại vị trí thấp nhất để minh họa giới hạn dưới
plot3(r*cos(alpha), r*sin(alpha), min(Z)*ones(size(alpha)), 'g--', 'LineWidth', 2);

% Trang trí đồ thị
title('Vung hoat dong Robot Delta (Chuan doi xung)');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
legend('Vung lam viec (TCP)', 'De co dinh (Base)', 'Gioi han duoi (Platform)', 'Location', 'Best');
view(3);