%==========================================================================
% MÔ PHỎNG ĐỘNG HỌC NGHỊCH ROBOT DELTA - GÓC QUAY ĐỘNG CƠ QUA BỘ TRUYỀN ĐAI
%==========================================================================
clear; clc; close all;

%% 1. Thông số hình học và bộ truyền
L1 = 130;   % Cánh tay chủ động (mm)
L2 = 298;   % Cánh tay bị động (mm)
R = 100;    % Bán kính đế cố định (mm)
r = 40;     % Bán kính đế di động (mm)
wb = R - r; % Khoảng cách hiệu dụng

u = 3.8;    % Tỉ số truyền của bộ truyền đai (u = 3.8)

%% 2. Tạo quỹ đạo chuyển động cho Bàn kẹp (End-effector)
% Cho robot chạy một đường tròn nằm ngang ở độ sâu Z = -250mm
t = linspace(0, 2*pi, 200); 
R_trajectory = 80;          % Bán kính đường tròn quỹ đạo (mm)

X_path = R_trajectory * cos(t);
Y_path = R_trajectory * sin(t);
Z_path = -250 * ones(size(t));

%% 3. Tính toán Động học nghịch
Theta1_arm = zeros(size(t));
Theta2_arm = zeros(size(t));
Theta3_arm = zeros(size(t));

for i = 1:length(t)
    x0 = X_path(i); y0 = Y_path(i); z0 = Z_path(i);
    
    % Góc của cánh tay L1 (đơn vị: radian)
    Theta1_arm(i) = delta_calcAngle(x0, y0, z0, L1, L2, wb, 0);
    Theta2_arm(i) = delta_calcAngle(x0, y0, z0, L1, L2, wb, 120);
    Theta3_arm(i) = delta_calcAngle(x0, y0, z0, L1, L2, wb, 240);
end

% Đổi sang độ cho cánh tay
Th1_arm_deg = rad2deg(Theta1_arm);
Th2_arm_deg = rad2deg(Theta2_arm);
Th3_arm_deg = rad2deg(Theta3_arm);

%% 4. Tính góc quay của ĐỘNG CƠ (Sau khi qua bộ truyền đai u = 3.8)
Th1_motor_deg = Th1_arm_deg * u;
Th2_motor_deg = Th2_arm_deg * u;
Th3_motor_deg = Th3_arm_deg * u;

%% 5. Vẽ đồ thị 2 chiều so sánh
figure('Color', [1 1 1]);

% --- ĐỒ THỊ 1: GÓC CỦA CÁNH TAY L1 ---
subplot(2,1,1);
hold on; grid on;
plot(Th1_arm_deg, 'r-', 'LineWidth', 2);
plot(Th2_arm_deg, 'g-', 'LineWidth', 2);
plot(Th3_arm_deg, 'b-', 'LineWidth', 2);
title('Theta cua 3 canh tay chu dong L1');
ylabel('Goc (Do)');
legend('Canh tay 1', 'Canh tay 2', 'Canh tay 3', 'Location', 'Best');

% --- ĐỒ THỊ 2: GÓC CỦA ĐỘNG CƠ (SAU BỘ TRUYỀN ĐAI) ---
subplot(2,1,2);
hold on; grid on;
plot(Th1_motor_deg, 'r--', 'LineWidth', 2.5);
plot(Th2_motor_deg, 'g--', 'LineWidth', 2.5);
plot(Th3_motor_deg, 'b--', 'LineWidth', 2.5);
title(['Theta DONG CO (Qua bo truyen dai u = ' num2cell(u) ')']);
xlabel('Diem tren quy dao (Thoi gian)');
ylabel('Goc (Do)');
legend('Motor 1', 'Motor 2', 'Motor 3', 'Location', 'Best');

%% ========================================================================
% HÀM PHỤ TRỢ: TÍNH GÓC THETA CHUẨN
% ========================================================================
function theta = delta_calcAngle(x0, y0, z0, L1, L2, wb, phi_deg)
    phi = deg2rad(phi_deg);
    x = x0*cos(phi) + y0*sin(phi) - wb;
    y = -x0*sin(phi) + y0*cos(phi);
    z = z0;
    
    A = 2*x*L1;
    B = 2*z*L1;
    C = x^2 + y^2 + z^2 + L1^2 - L2^2;
    
    delta = A^2 + B^2 - C^2;
    if delta >= 0
        t1 = (-B - sqrt(delta)) / (A - C);
        theta = 2 * atan(t1);
    else
        theta = NaN;
    end
end