% =========================================================================
% SCRIPT TỰ ĐỘNG TẠO FILE SIMULINK MÔ PHỎNG PID (MATLAB 2019)
% =========================================================================
clear; clc; close all;

modelName = 'Delta_Robot_PID';

% Kiểm tra và đóng nếu file đang mở
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

% Tạo hệ thống mới
new_system(modelName);
open_system(modelName);

% 1. Khối Step (Tín hiệu đặt - Vị trí mong muốn)
add_block('simulink/Sources/Step', [modelName '/Setpoint']);
set_param([modelName '/Setpoint'], 'Position', [50, 100, 80, 130]);
set_param([modelName '/Setpoint'], 'Time', '0', 'Before', '0', 'After', '1'); % Bước nhảy 1 đơn vị

% 2. Khối Sum (Tính sai số)
add_block('simulink/Math Operations/Sum', [modelName '/Sum']);
set_param([modelName '/Sum'], 'Position', [150, 105, 170, 125]);
set_param([modelName '/Sum'], 'Inputs', '+-');

% 3. Khối PID Controller
add_block('simulink/Continuous/PID Controller', [modelName '/PID_Controller']);
set_param([modelName '/PID_Controller'], 'Position', [220, 97, 260, 133]);
set_param([modelName '/PID_Controller'], 'P', 'Kp', 'I', 'Ki', 'D', 'Kd');

% 4. Khối Hàm Truyền (Plant - Động cơ/Robot)
add_block('simulink/Continuous/Transfer Fcn', [modelName '/Robot_Plant']);
set_param([modelName '/Robot_Plant'], 'Position', [330, 97, 410, 133]);
% Giả sử hàm truyền bậc 2 cơ bản: 1 / (s^2 + 10s + 20). Thay đổi thông số này theo mô hình động cơ thực tế.
set_param([modelName '/Robot_Plant'], 'Numerator', '[1]', 'Denominator', '[1 10 20]');

% 5. To Workspace (Lưu tín hiệu Đặt)
add_block('simulink/Sinks/To Workspace', [modelName '/To_y_ref']);
set_param([modelName '/To_y_ref'], 'Position', [150, 20, 210, 50]);
set_param([modelName '/To_y_ref'], 'VariableName', 'y_ref', 'SaveFormat', 'Timeseries');

% 6. To Workspace (Lưu sai số)
add_block('simulink/Sinks/To Workspace', [modelName '/To_error']);
set_param([modelName '/To_error'], 'Position', [220, 170, 280, 200]);
set_param([modelName '/To_error'], 'VariableName', 'error_signal', 'SaveFormat', 'Timeseries');

% 7. To Workspace (Lưu tín hiệu Đáp ứng)
add_block('simulink/Sinks/To Workspace', [modelName '/To_y_out']);
set_param([modelName '/To_y_out'], 'Position', [480, 100, 540, 130]);
set_param([modelName '/To_y_out'], 'VariableName', 'y_out', 'SaveFormat', 'Timeseries');

% --- Nối dây (Routing) ---
% Nối Setpoint -> Sum
add_line(modelName, 'Setpoint/1', 'Sum/1', 'autorouting', 'on');
% Nối Setpoint -> To_y_ref
add_line(modelName, 'Setpoint/1', 'To_y_ref/1', 'autorouting', 'on');

% Nối Sum -> PID
add_line(modelName, 'Sum/1', 'PID_Controller/1', 'autorouting', 'on');
% Nối Sum -> To_error
add_line(modelName, 'Sum/1', 'To_error/1', 'autorouting', 'on');

% Nối PID -> Plant
add_line(modelName, 'PID_Controller/1', 'Robot_Plant/1', 'autorouting', 'on');

% Nối Plant -> To_y_out
add_line(modelName, 'Robot_Plant/1', 'To_y_out/1', 'autorouting', 'on');
% Nối Plant -> Sum (Hồi tiếp âm)
add_line(modelName, 'Robot_Plant/1', 'Sum/2', 'autorouting', 'on');

% --- Cài đặt tham số mô phỏng ---
set_param(modelName, 'StopTime', '10'); % Chạy trong 10 giây

% Lưu model dưới định dạng MATLAB 2019a
save_system(modelName, [modelName '.slx'], 'SaveAsVersion', 'R2019a');

disp('===========================================================');
disp(['ĐÃ TẠO THÀNH CÔNG FILE SIMULINK: ' modelName '.slx']);
disp('Bạn có thể mở file này để thay đổi hàm truyền hệ thống (Robot_Plant) cho phù hợp với đồ án của mình.');
disp('Bây giờ bạn có thể chạy file PID_Tuning_Simulink.m để mô phỏng và dò thông số!');
disp('===========================================================');
