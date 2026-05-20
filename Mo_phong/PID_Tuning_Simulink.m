% =========================================================================
% CHƯƠNG TRÌNH TÌM VÀ TỐI ƯU THÔNG SỐ PID CHO MÔ PHỎNG SIMULINK
% Chạy tốt trên MATLAB 2019
% =========================================================================
clear; clc; close all;

%% 1. Khai báo các thông số cơ khí của Robot (Dùng chung với model động học)
L1 = 130;   
L2 = 298;   
R = 100;    
r = 40;     
wb = R - r; 
u = 3.8;    

% =========================================================================
% LƯU Ý QUAN TRỌNG:
% Hãy thay đổi 'Ten_File_Simulink_Cua_Ban' thành tên file .slx của bạn 
% (không bao gồm đuôi .slx).
% File .slx phải được lưu cùng thư mục với script này.
%
% Trong file Simulink:
% 1. Các khối PID Controller (hoặc Gain) phải nhận biến là Kp, Ki, Kd
% 2. Phải có khối 'To Workspace' xuất ra biến 'error_signal' (lưu dạng Array hoặc Timeseries)
%    chứa giá trị sai số e(t) = Tín hiệu đặt - Tín hiệu thực tế.
% 3. Khuyến nghị: Xuất thêm khối 'To Workspace' với tên 'y_ref' (Tín hiệu đặt) 
%    và 'y_out' (Tín hiệu đáp ứng) để chương trình tự động vẽ đồ thị.
% =========================================================================
simulink_model = 'Ten_File_Simulink_Cua_Ban'; 

%% 2. Giao diện điều khiển
disp('=====================================================');
disp('CHƯƠNG TRÌNH CHỌN VÀ TỐI ƯU THÔNG SỐ PID');
disp('1. Nhập Kp, Ki, Kd thủ công để chạy thử mô phỏng');
disp('2. Dùng thuật toán tự động dò Kp, Ki, Kd tối ưu (ITAE)');
disp('=====================================================');
choice = input('Nhập lựa chọn của bạn (1 hoặc 2): ');

if choice == 1
    %% CHẾ ĐỘ 1: CHẠY THỬ NGHIỆM THỦ CÔNG
    Kp = input('Nhập Kp: ');
    Ki = input('Nhập Ki: ');
    Kd = input('Nhập Kd: ');
    
    % Đưa biến vào Workspace để Simulink nhận
    assignin('base', 'Kp', Kp);
    assignin('base', 'Ki', Ki);
    assignin('base', 'Kd', Kd);
    
    disp('Đang tiến hành chạy mô phỏng Simulink...');
    try
        out = sim(simulink_model, 'ReturnWorkspaceOutputs', 'on');
        disp('Mô phỏng hoàn tất! Đang tạo đồ thị trực quan...');
        plot_results(out);
    catch ME
        disp('LỖI: Không thể chạy file Simulink. Hãy chắc chắn đúng tên file và file đang ở thư mục hiện tại.');
        disp(ME.message);
    end

elseif choice == 2
    %% CHẾ ĐỘ 2: TỐI ƯU HÓA TỰ ĐỘNG Kp, Ki, Kd BẰNG FMINSEARCH
    disp('Bắt đầu tự động dò tìm PID. Sẽ mất một chút thời gian mô phỏng...');
    
    % Giá trị ban đầu dự đoán [Kp, Ki, Kd]
    PID_Init = [1.0, 0.1, 0.01]; 
    
    % Thiết lập giới hạn chạy của fminsearch
    options = optimset('Display','iter', 'MaxIter', 50, 'TolFun', 1e-3);
    
    % Chạy tối ưu hóa bằng hàm mục tiêu
    try
        PID_opt = fminsearch(@(PID) cost_function(PID, simulink_model), PID_Init, options);
        
        fprintf('\n====================================\n');
        fprintf('ĐÃ TÌM THẤY THÔNG SỐ PID TỐI ƯU:\n');
        fprintf('Kp = %.4f\n', PID_opt(1));
        fprintf('Ki = %.4f\n', PID_opt(2));
        fprintf('Kd = %.4f\n', PID_opt(3));
        fprintf('====================================\n');
        
        % Chạy lại mô phỏng lần cuối với thông số tốt nhất
        assignin('base', 'Kp', PID_opt(1));
        assignin('base', 'Ki', PID_opt(2));
        assignin('base', 'Kd', PID_opt(3));
        out = sim(simulink_model, 'ReturnWorkspaceOutputs', 'on');
        disp('Đã lưu Kp, Ki, Kd tối ưu vào Workspace và chạy lại mô phỏng.');
        plot_results(out);
    catch ME
        disp('LỖI trong quá trình tối ưu hóa. Hãy kiểm tra lại file Simulink.');
        disp(ME.message);
    end
else
    disp('Lựa chọn không hợp lệ. Chương trình kết thúc.');
end

%% =========================================================================
% HÀM TÍNH TOÁN CHI PHÍ (COST FUNCTION) - SỬ DỤNG CHUẨN ITAE
% =========================================================================
function J = cost_function(PID, model)
    % Ép các hệ số luôn dương
    Kp = abs(PID(1)); 
    Ki = abs(PID(2));
    Kd = abs(PID(3));
    
    assignin('base', 'Kp', Kp);
    assignin('base', 'Ki', Ki);
    assignin('base', 'Kd', Kd);
    
    try
        % Chạy mô phỏng, bắt kết quả trả về
        out = sim(model, 'ReturnWorkspaceOutputs', 'on');
        
        % Tìm biến error_signal từ output
        if isfield(out, 'error_signal')
            % Nếu xuất ra dưới dạng TimeSeries
            if isa(out.error_signal, 'timeseries')
                e = out.error_signal.Data;
                t = out.error_signal.Time;
            else
                % Nếu xuất ra dạng mảng
                e = out.error_signal(:, 2); % Giả sử cột 2 là Data
                t = out.error_signal(:, 1); % Cột 1 là Time
            end
            
            % Tính chuẩn ITAE (Integral of Time multiplied by Absolute Error)
            J = trapz(t, t .* abs(e(:)));
        else
            % Phạt nếu không tìm thấy tín hiệu sai số
            J = 1e6; 
        end
    catch
        % Phạt nếu hệ thống mất ổn định / không chạy được
        J = 1e6;
    end
end

%% =========================================================================
% HÀM VẼ ĐỒ THỊ TRỰC QUAN
% =========================================================================
function plot_results(out)
    figure('Name', 'Ket qua mo phong PID', 'Color', [1 1 1], 'NumberTitle', 'off');
    
    has_ref_out = isfield(out, 'y_ref') && isfield(out, 'y_out');
    has_err = isfield(out, 'error_signal');
    
    if has_ref_out && has_err
        subplot(2,1,1);
    end
    
    % Vẽ Tín hiệu đặt và Tín hiệu thực tế
    if has_ref_out
        if ~has_err; subplot(1,1,1); end
        hold on; grid on;
        
        if isa(out.y_ref, 'timeseries')
            plot(out.y_ref.Time, out.y_ref.Data, 'b--', 'LineWidth', 1.5);
            plot(out.y_out.Time, out.y_out.Data, 'r-', 'LineWidth', 1.5);
        else
            plot(out.y_ref(:,1), out.y_ref(:,2), 'b--', 'LineWidth', 1.5);
            plot(out.y_out(:,1), out.y_out(:,2), 'r-', 'LineWidth', 1.5);
        end
        title('Tín hiệu Đặt (Setpoint) và Đáp ứng (Feedback)', 'FontSize', 12);
        xlabel('Thời gian (s)'); ylabel('Biên độ');
        legend('Tín hiệu đặt (y_{ref})', 'Tín hiệu thực tế (y_{out})', 'Location', 'best');
    end
    
    % Vẽ Tín hiệu sai số
    if has_err
        if has_ref_out; subplot(2,1,2); end
        hold on; grid on;
        
        if isa(out.error_signal, 'timeseries')
            plot(out.error_signal.Time, out.error_signal.Data, 'k-', 'LineWidth', 1.2);
        else
            plot(out.error_signal(:,1), out.error_signal(:,2), 'k-', 'LineWidth', 1.2);
        end
        title('Sai số e(t)', 'FontSize', 12);
        xlabel('Thời gian (s)'); ylabel('Biên độ sai số');
    end
    
    if ~has_ref_out && ~has_err
        disp('LƯU Ý: Không tìm thấy biến y_ref, y_out, error_signal trong Workspace để vẽ đồ thị.');
    end
end
