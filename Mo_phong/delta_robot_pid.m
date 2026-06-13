%% ============================================================
%  DELTA ROBOT - BỘ ĐIỀU KHIỂN PID (SIMULINK)
%  Mô phỏng robot Delta 3 bậc tự do với PID độc lập mỗi khớp
%  Tác giả: Auto-generated
%  Ngày:    2024
%% ============================================================

clear; clc; close all;

%% ===== 1. THÔNG SỐ CƠ HỌC ROBOT DELTA =====
params.L1  = 0.13;       % Chiều dài cánh tay trên [m]
params.L2  = 0.298;       % Chiều dài cánh tay dưới [m]
params.r_b = 0.1;       % Bán kính đế tĩnh [m]
params.r_p = 0.0353;      % Bán kính nền di động [m]
params.m   = 0.5;       % Khối lượng tải [kg]
params.g   = 9.81;      % Gia tốc trọng trường [m/s²]
params.J   = 0.01;      % Moment quán tính khớp [kg.m²]
params.b   = 0.05;      % Hệ số cản khớp [N.m.s/rad]

%% ===== 2. THAM SỐ PID CHO 3 KHỚP (θ1, θ2, θ3) =====
% --- Khớp 1 ---
pid.Kp1 = 120;   pid.Ki1 = 15;   pid.Kd1 = 8;
% --- Khớp 2 ---
pid.Kp2 = 120;   pid.Ki2 = 15;   pid.Kd2 = 8;
% --- Khớp 3 ---
pid.Kp3 = 120;   pid.Ki3 = 15;   pid.Kd3 = 8;

% Giới hạn tích phân (Anti-windup)
pid.imax =  5;
pid.imin = -5;

% Giới hạn mô-men động cơ [N.m]
pid.tau_max =  30;
pid.tau_min = -30;

%% ===== 3. QUỸ ĐẠO MỤC TIÊU (Không gian khớp) =====
% Góc mục tiêu cho 3 khớp [rad]
% Ví dụ: chuyển động hình sin (pick & place đơn giản)
traj.theta1_ref = @(t)  0.5*sin(pi*t);
traj.theta2_ref = @(t)  0.5*sin(pi*t + 2*pi/3);
traj.theta3_ref = @(t)  0.5*sin(pi*t + 4*pi/3);

%% ===== 4. THÔNG SỐ MÔ PHỎNG =====
sim_time  = 5;      % Thời gian mô phỏng [s]
dt        = 0.001;  % Bước lấy mẫu [s]
t         = 0:dt:sim_time;
N         = length(t);

%% ===== 5. KHỞI TẠO BIẾN LƯU TRỮ =====
theta     = zeros(3, N);   % Góc thực tế [rad]
dtheta    = zeros(3, N);   % Vận tốc góc [rad/s]
theta_ref = zeros(3, N);   % Góc mục tiêu [rad]
tau       = zeros(3, N);   % Mô-men điều khiển [N.m]
error     = zeros(3, N);   % Sai số
e_int     = zeros(3, 1);   % Tích phân sai số
e_prev    = zeros(3, 1);   % Sai số bước trước

% Điều kiện ban đầu
theta(:,1)  = [0.1; 0.1; 0.1];   % Góc ban đầu [rad]
dtheta(:,1) = [0;   0;   0];     % Vận tốc ban đầu

%% ===== 6. VÒNG LẶP MÔ PHỎNG RUNGE-KUTTA 4 =====
fprintf('Đang mô phỏng robot Delta với PID...\n');

Kp = [pid.Kp1; pid.Kp2; pid.Kp3];
Ki = [pid.Ki1; pid.Ki2; pid.Ki3];
Kd = [pid.Kd1; pid.Kd2; pid.Kd3];

for k = 1:N-1
    % Quỹ đạo tham chiếu
    theta_ref(1,k) = traj.theta1_ref(t(k));
    theta_ref(2,k) = traj.theta2_ref(t(k));
    theta_ref(3,k) = traj.theta3_ref(t(k));

    % Tính sai số
    e = theta_ref(:,k) - theta(:,k);
    error(:,k) = e;

    % Tích phân (Anti-windup clamp)
    e_int = e_int + e * dt;
    e_int = max(pid.imin, min(pid.imax, e_int));

    % Đạo hàm
    e_dot = (e - e_prev) / dt;
    e_prev = e;

    % Luật điều khiển PID
    u = Kp.*e + Ki.*e_int + Kd.*e_dot;

    % Bão hòa mô-men
    u = max(pid.tau_min, min(pid.tau_max, u));
    tau(:,k) = u;

    % Động lực học robot Delta (mô hình đơn giản hóa cho mỗi khớp)
    % J*ddtheta + b*dtheta = tau - tau_gravity
    tau_grav = params.m * params.g * params.L1/2 .* cos(theta(:,k));
    ddtheta  = (u - params.b * dtheta(:,k) - tau_grav) / params.J;

    % Tích phân RK4
    [th_next, dth_next] = rk4_step(theta(:,k), dtheta(:,k), u, ...
                                   params, dt);
    theta(:,k+1)  = th_next;
    dtheta(:,k+1) = dth_next;
end

% Bước cuối
theta_ref(1,N) = traj.theta1_ref(t(N));
theta_ref(2,N) = traj.theta2_ref(t(N));
theta_ref(3,N) = traj.theta3_ref(t(N));
error(:,N) = theta_ref(:,N) - theta(:,N);

fprintf('Mô phỏng hoàn thành!\n');

%% ===== 7. VẼ KẾT QUẢ =====
figure('Name','Delta Robot PID - Góc khớp','NumberTitle','off',...
       'Position',[50 50 1200 800]);

joint_names = {'\theta_1','\theta_2','\theta_3'};
colors_ref  = {'b','r','g'};
colors_act  = {'c','m','y'};

for i = 1:3
    subplot(3,3,i);
    plot(t, theta_ref(i,:), '--', 'Color', colors_ref{i}, 'LineWidth', 1.5);
    hold on;
    plot(t, theta(:,:), 'Color', colors_act{i}, 'LineWidth', 1.2);
    xlabel('Thời gian [s]'); ylabel('Góc [rad]');
    title(['Góc khớp ' joint_names{i}]);
    legend('Tham chiếu','Thực tế','Location','best');
    grid on;
end

for i = 1:3
    subplot(3,3,3+i);
    plot(t, error(i,:), 'Color', colors_ref{i}, 'LineWidth', 1.2);
    xlabel('Thời gian [s]'); ylabel('Sai số [rad]');
    title(['Sai số khớp ' joint_names{i}]);
    grid on;
    yline(0,'k--','LineWidth',0.8);
end

for i = 1:3
    subplot(3,3,6+i);
    plot(t, tau(i,:), 'Color', colors_act{i}, 'LineWidth', 1.2);
    xlabel('Thời gian [s]'); ylabel('Mô-men [N.m]');
    title(['Mô-men điều khiển ' joint_names{i}]);
    grid on;
end

sgtitle('ROBOT DELTA – Điều khiển PID 3 Khớp', 'FontSize', 14, 'FontWeight','bold');

%% ===== 8. TÍNH RMSE =====
fprintf('\n===== CHỈ SỐ HIỆU SUẤT =====\n');
for i = 1:3
    rmse = sqrt(mean(error(i,:).^2));
    fprintf('Khớp %d – RMSE: %.5f rad (%.4f°)\n', i, rmse, rad2deg(rmse));
end

%% ===== 9. TẠO MÔ HÌNH SIMULINK =====
fprintf('\nĐang tạo mô hình Simulink...\n');
create_simulink_model(params, pid);

%% ===================================================================
%  HÀM PHỤ TRỢ
%% ===================================================================

function [th_next, dth_next] = rk4_step(th, dth, u, p, dt)
%RK4_STEP  Tích phân Runge-Kutta bậc 4 cho động lực học khớp Delta

    f = @(th_in, dth_in) delta_dynamics(th_in, dth_in, u, p);

    [k1_th, k1_dth] = f(th,               dth);
    [k2_th, k2_dth] = f(th + dt/2*k1_th,  dth + dt/2*k1_dth);
    [k3_th, k3_dth] = f(th + dt/2*k2_th,  dth + dt/2*k2_dth);
    [k4_th, k4_dth] = f(th + dt*k3_th,    dth + dt*k3_dth);

    th_next  = th  + dt/6*(k1_th  + 2*k2_th  + 2*k3_th  + k4_th);
    dth_next = dth + dt/6*(k1_dth + 2*k2_dth + 2*k3_dth + k4_dth);
end

function [dth, ddth] = delta_dynamics(th, dth, tau, p)
%DELTA_DYNAMICS  Mô hình động lực học đơn giản hóa robot Delta
%   J*ddtheta + b*dtheta + tau_grav = tau

    tau_grav = p.m * p.g * (p.L1/2) .* cos(th);
    ddth = (tau - p.b * dth - tau_grav) / p.J;
end

function create_simulink_model(params, pid)
%CREATE_SIMULINK_MODEL  Tạo mô hình Simulink cho robot Delta PID

    mdl = 'delta_robot_pid_simulink';

    % Xóa model cũ nếu có
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    if exist([mdl '.slx'],'file'), delete([mdl '.slx']); end

    % Tạo model mới
    new_system(mdl);
    open_system(mdl);

    % ---- Cài đặt Solver ----
    set_param(mdl, 'Solver','ode45', 'StopTime','5', ...
              'SolverType','Variable-step', 'MaxStep','0.001');

    %% ---- KHỐI CLOCK ----
    add_block('simulink/Sources/Clock',           [mdl '/Clock'], ...
              'Position',[30 230 60 260]);

    %% ---- KHỐI SIGNAL BUILDER (Quỹ đạo tham chiếu) ----
    % Dùng Sine Wave cho 3 tham chiếu
    offsets = [0, 2*pi/3, 4*pi/3];
    ref_names = {'Ref_J1','Ref_J2','Ref_J3'};

    for i = 1:3
        blk = [mdl '/' ref_names{i}];
        add_block('simulink/Sources/Sine Wave', blk, ...
                  'Amplitude','0.5', ...
                  'Frequency','pi', ...
                  'Phase',    num2str(offsets(i)), ...
                  'SampleTime','0', ...
                  'Position', [30 30+(i-1)*120, 90 60+(i-1)*120]);
    end

    %% ---- SUBSYSTEM PID CHO MỖI KHỚP ----
    pid_names  = {'PID_J1','PID_J2','PID_J3'};
    Kps = [pid.Kp1, pid.Kp2, pid.Kp3];
    Kis = [pid.Ki1, pid.Ki2, pid.Ki3];
    Kds = [pid.Kd1, pid.Kd2, pid.Kd3];

    for i = 1:3
        blk = [mdl '/' pid_names{i}];
        add_block('simulink/Continuous/PID Controller', blk, ...
                  'P',   num2str(Kps(i)), ...
                  'I',   num2str(Kis(i)), ...
                  'D',   num2str(Kds(i)), ...
                  'N',   '100', ...
                  'LimitOutput','on', ...
                  'UpperSaturationLimit', '30', ...
                  'LowerSaturationLimit','-30', ...
                  'AntiWindupMode','clamping', ...
                  'Position',[160 20+(i-1)*120, 280 70+(i-1)*120]);
    end

    %% ---- SUBSYSTEM ĐỘNG LỰC HỌC KHỚP ----
    joint_names = {'Joint1','Joint2','Joint3'};

    for i = 1:3
        blk = [mdl '/' joint_names{i}];
        add_block('simulink/User-Defined Functions/MATLAB Function', blk, ...
                  'Position',[340 20+(i-1)*120, 460 70+(i-1)*120]);
        % Ghi code vào MATLAB Function block
        sf = sfroot();
        m  = sf.find('Name', mdl, '-isa','Simulink.BlockDiagram');
        if ~isempty(m)
            fc = m.find('Name', joint_names{i}, '-isa','Stateflow.EMChart');
            if ~isempty(fc)
                fc.Script = sprintf([
                    'function [theta_out, dtheta_out] = fcn(tau, theta_in, dtheta_in)\n'...
                    '%%#codegen\n'...
                    'J = %.4f; b = %.4f; m = %.4f; g = 9.81; L1 = %.4f;\n'...
                    'tau_grav = m*g*(L1/2)*cos(theta_in);\n'...
                    'ddtheta = (tau - b*dtheta_in - tau_grav)/J;\n'...
                    'dtheta_out = dtheta_in + ddtheta*1e-3;\n'...
                    'theta_out  = theta_in  + dtheta_out*1e-3;\n'
                    ], params.J, params.b, params.m, params.L1);
            end
        end
    end

    %% ---- KHỐI SCOPE ----
    add_block('simulink/Sinks/Scope',      [mdl '/Scope_Angles'], ...
              'NumInputPorts','3', 'Position',[550 100 610 200]);
    add_block('simulink/Sinks/Scope',      [mdl '/Scope_Torques'], ...
              'NumInputPorts','3', 'Position',[550 220 610 320]);
    add_block('simulink/Sinks/To Workspace',[mdl '/ToWS_theta'], ...
              'VariableName','theta_sim', ...
              'SaveFormat','Array', ...
              'Position',[550 50 650 80]);

    %% ---- KẾT NỐI DÂY ----
    for i = 1:3
        % Ref -> PID (cổng +)
        add_line(mdl, [ref_names{i} '/1'], [pid_names{i} '/1'], ...
                 'autorouting','on');
        % PID -> Joint
        add_line(mdl, [pid_names{i} '/1'], [joint_names{i} '/1'], ...
                 'autorouting','on');
        % Joint output -> Scope_Angles
        add_line(mdl, [joint_names{i} '/1'], ['Scope_Angles/' num2str(i)], ...
                 'autorouting','on');
        % Torque -> Scope_Torques
        add_line(mdl, [pid_names{i} '/1'], ['Scope_Torques/' num2str(i)], ...
                 'autorouting','on');
    end

    % Lưu model
    save_system(mdl);
    fprintf('Mô hình Simulink đã được tạo: %s.slx\n', mdl);
    fprintf('Mở Simulink và nhấn Run để mô phỏng!\n');

end
