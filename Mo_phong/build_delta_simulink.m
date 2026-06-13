%% ============================================================
%  TẠO MÔ HÌNH SIMULINK - ROBOT DELTA PID
%  Chạy file này để tạo file .slx tự động
%% ============================================================

function build_delta_simulink()
% BUILD_DELTA_SIMULINK  Tạo toàn bộ mô hình Simulink robot Delta PID
% Gọi: build_delta_simulink()

    mdl = 'delta_robot_pid';

    %% Tham số
    J   = 0.01;   b   = 0.05;
    m   = 0.5;    g   = 9.81;  L1 = 0.3;

    Kp1=120; Ki1=15; Kd1=8;
    Kp2=120; Ki2=15; Kd2=8;
    Kp3=120; Ki3=15; Kd3=8;

    %% Xóa & tạo mới
    if bdIsLoaded(mdl), close_system(mdl,0); end
    new_system(mdl);
    open_system(mdl);
    set_param(mdl,'Solver','ode45','StopTime','5',...
              'SolverType','Variable-step','MaxStep','0.001');

    % ── Lưới vị trí (cột, hàng) ──────────────────────────────
    %  Mỗi khớp chiếm 1 hàng cao 130px
    %  Cột:  Ref=30  Sum=120  PID=220  Plant=380  Out=530

    row   = @(i) 30 + (i-1)*130;   % top-left y của hàng i
    mkPos = @(x,y,w,h) [x, y, x+w, y+h];

    pid_Kp = [Kp1,Kp2,Kp3];
    pid_Ki = [Ki1,Ki2,Ki3];
    pid_Kd = [Kd1,Kd2,Kd3];

    for i = 1:3
        sfx  = sprintf('_J%d',i);
        ph   = num2str(offsets3(i));   % dùng hàm khai báo dưới

        % 1. Sine Wave – tham chiếu
        bRef = [mdl '/Ref' sfx];
        add_block('simulink/Sources/Sine Wave', bRef, ...
            'Amplitude','0.5','Frequency','pi', ...
            'Phase', ph, 'SampleTime','0', ...
            'Position', mkPos(30, row(i), 60, 50));

        % 2. Sum – cộng sai số
        bSum = [mdl '/Sum' sfx];
        add_block('simulink/Math Operations/Sum', bSum, ...
            'Inputs','+-', ...
            'Position', mkPos(120, row(i)+5, 30, 30));

        % 3. PID Controller
        bPID = [mdl '/PID' sfx];
        add_block('simulink/Continuous/PID Controller', bPID, ...
            'P', num2str(pid_Kp(i)), ...
            'I', num2str(pid_Ki(i)), ...
            'D', num2str(pid_Kd(i)), ...
            'N','100', ...
            'LimitOutput','on', ...
            'UpperSaturationLimit','30', ...
            'LowerSaturationLimit','-30', ...
            'AntiWindupMode','clamping', ...
            'Position', mkPos(180, row(i), 100, 50));

        % 4. Transfer Function – Plant (1/(Js+b))
        bPlant = [mdl '/Plant' sfx];
        add_block('simulink/Continuous/Transfer Fcn', bPlant, ...
            'Numerator',   '[1]', ...
            'Denominator', sprintf('[%g %g]', J, b), ...
            'Position', mkPos(320, row(i), 90, 50));

        % 5. Integrator – tích phân vận tốc → góc
        bInt = [mdl '/Integrator' sfx];
        add_block('simulink/Continuous/Integrator', bInt, ...
            'InitialCondition','0.1', ...
            'Position', mkPos(450, row(i), 60, 50));

        % 6. Scope – hiển thị góc
        bScope = [mdl '/Scope' sfx];
        add_block('simulink/Sinks/Scope', bScope, ...
            'Position', mkPos(550, row(i), 50, 50));

        % 7. To Workspace
        bWS = [mdl '/ToWS' sfx];
        add_block('simulink/Sinks/To Workspace', bWS, ...
            'VariableName', sprintf('theta%d_sim',i), ...
            'SaveFormat','Array', ...
            'Position', mkPos(550, row(i)+60, 70, 30));

        % ── Kết nối ─────────────────────────────────────────────
        % Ref → Sum(+)
        add_line(mdl,[bRef(length(mdl)+2:end) '/1'], ...
                     [bSum(length(mdl)+2:end) '/1'],'autorouting','on');
        % Sum → PID
        add_line(mdl,[bSum(length(mdl)+2:end) '/1'], ...
                     [bPID(length(mdl)+2:end) '/1'],'autorouting','on');
        % PID → Plant
        add_line(mdl,[bPID(length(mdl)+2:end) '/1'], ...
                     [bPlant(length(mdl)+2:end) '/1'],'autorouting','on');
        % Plant (velocity) → Integrator
        add_line(mdl,[bPlant(length(mdl)+2:end) '/1'], ...
                     [bInt(length(mdl)+2:end) '/1'],'autorouting','on');
        % Integrator (angle) → Scope
        add_line(mdl,[bInt(length(mdl)+2:end) '/1'], ...
                     [bScope(length(mdl)+2:end) '/1'],'autorouting','on');
        % Integrator → ToWorkspace
        add_line(mdl,[bInt(length(mdl)+2:end) '/1'], ...
                     [bWS(length(mdl)+2:end) '/1'],'autorouting','on');
        % Feedback: Integrator → Sum(-)
        add_line(mdl,[bInt(length(mdl)+2:end) '/1'], ...
                     [bSum(length(mdl)+2:end) '/2'],'autorouting','on');
    end

    save_system(mdl, [mdl '.slx']);
    fprintf('[OK] Đã tạo: %s.slx – mở và nhấn Ctrl+T để chạy\n', mdl);
    open_system(mdl);
end

function v = offsets3(i)
    o = [0, 2*pi/3, 4*pi/3];
    v = o(i);
end
