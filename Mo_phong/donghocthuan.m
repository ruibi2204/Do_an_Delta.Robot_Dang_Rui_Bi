%==========================================================================
% CẬP NHẬT: MÔ PHỎNG VÙNG HOẠT ĐỘNG ROBOT DELTA CHUẨN ĐỐI XỨNG
% Quy ước tọa độ: gốc O tại tâm đế cố định, trục Z dương hướng TỪ TRÊN XUỐNG
% (vùng làm việc nằm bên dưới đế => Z > 0)
%==========================================================================
clear; clc; close all;

%% 1. Thông số hình học (mm)
L1 = 130;   % Cánh tay chủ động
L2 = 298;   % Cánh tay bị động
R = 120;    % Bán kính đế cố định
r = 40;     % Bán kính đế di động

% Giới hạn góc quay thực tế khớp chủ động (độ)
% (theta = 0: cánh tay nằm ngang, theta > 0: cánh tay hạ xuống phía dưới đế)
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
    t1 = T1(i); t2 = T2(i); t3 = T3(i);

    % Tọa độ khớp khuỷu (khớp nối giữa L1 và L2) của 3 nhánh
    % Trục Z dương hướng xuống => cánh tay hạ xuống thì z = +L1*sin(theta)
    % Nhánh 1: góc 0 rad (dọc theo trục X)
    P1 = [wb + L1*cos(t1); 0; L1*sin(t1)];
    % Nhánh 2: xoay 120 độ
    P2 = [(wb + L1*cos(t2))*cos(2*pi/3); (wb + L1*cos(t2))*sin(2*pi/3); L1*sin(t2)];
    % Nhánh 3: xoay 240 độ
    P3 = [(wb + L1*cos(t3))*cos(4*pi/3); (wb + L1*cos(t3))*sin(4*pi/3); L1*sin(t3)];

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
        % Z dương hướng xuống => chọn nghiệm LỚN hơn (robot hoạt động bên dưới đế)
        z_sol = (-B_eq + sqrt(delta_eq)) / (2*A_eq);
        x_sol = X0 + Xz*z_sol;
        y_sol = Y0 + Yz*z_sol;

        X = [X; x_sol]; Y = [Y; y_sol]; Z = [Z; z_sol];
    end
end

%% 4. Xác định 6 điểm giới hạn của vùng làm việc (2 điểm / truc X, Y, Z)
[~, idx_xmin] = min(X);
[~, idx_xmax] = max(X);
[~, idx_ymin] = min(Y);
[~, idx_ymax] = max(Y);
[~, idx_zmin] = min(Z);
[~, idx_zmax] = max(Z);

P_xmin = [X(idx_xmin), Y(idx_xmin), Z(idx_xmin)];
P_xmax = [X(idx_xmax), Y(idx_xmax), Z(idx_xmax)];
P_ymin = [X(idx_ymin), Y(idx_ymin), Z(idx_ymin)];
P_ymax = [X(idx_ymax), Y(idx_ymax), Z(idx_ymax)];
P_zmin = [X(idx_zmin), Y(idx_zmin), Z(idx_zmin)];
P_zmax = [X(idx_zmax), Y(idx_zmax), Z(idx_zmax)];

limit_pts   = [P_xmin; P_xmax; P_ymin; P_ymax; P_zmin; P_zmax];
limit_names = {'Xmin', 'Xmax', 'Ymin', 'Ymax', 'Zmin (cao nhat)', 'Zmax (thap nhat)'};

% In toa do 6 diem gioi han ra Command Window
fprintf('\n===== TOA DO 6 DIEM GIOI HAN VUNG LAM VIEC =====\n');
for k = 1:6
    fprintf('%-18s: X = %8.2f mm | Y = %8.2f mm | Z = %8.2f mm\n', ...
        limit_names{k}, limit_pts(k,1), limit_pts(k,2), limit_pts(k,3));
end
fprintf('=================================================\n\n');

%% 5. Vẽ đồ thị đồ họa 3D
figure('Color', [1 1 1]);
hold on; grid on; axis equal;

% Vẽ đám mây điểm vùng làm việc
plot3(X, Y, Z, '.', 'Color', [0 0.4470 0.7410], 'MarkerSize', 4);

% Vẽ đường tròn đế cố định (Base) tại Z = 0
alpha = linspace(0, 2*pi, 100);
plot3(R*cos(alpha), R*sin(alpha), zeros(size(alpha)), 'r-', 'LineWidth', 3);

% Vẽ đường tròn đế di động (Platform) tại vị trí thấp nhất (Z lớn nhất)
% để minh họa giới hạn dưới
plot3(r*cos(alpha), r*sin(alpha), max(Z)*ones(size(alpha)), 'g--', 'LineWidth', 2);

% Đánh dấu và ghi chú tọa độ 6 điểm giới hạn (chu to)
plot3(limit_pts(:,1), limit_pts(:,2), limit_pts(:,3), ...
    'ko', 'MarkerSize', 10, 'MarkerFaceColor', [1 0.6 0], 'LineWidth', 1.5);
for k = 1:6
    txt = sprintf('  %s\n  (%.0f, %.0f, %.0f)', limit_names{k}, ...
        limit_pts(k,1), limit_pts(k,2), limit_pts(k,3));
    text(limit_pts(k,1), limit_pts(k,2), limit_pts(k,3), txt, ...
        'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
end

% Trang trí đồ thị
title('Vung hoat dong Robot Delta', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('X (mm)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('Y (mm)', 'FontSize', 10, 'FontWeight', 'bold');
zlabel('Z (mm) - duong huong xuong', 'FontSize', 10, 'FontWeight', 'bold');
legend('Vung lam viec (TCP)', 'De co dinh (Base)', 'Gioi han duoi (Platform)', ...
    '6 diem gioi han', 'Location', 'Best', 'FontSize', 10);
set(gca, 'FontSize', 10);

% Trục Z dương chiều từ trên xuống dưới (Z = 0 ở đế phía trên)
set(gca, 'ZDir', 'reverse');
view(3);