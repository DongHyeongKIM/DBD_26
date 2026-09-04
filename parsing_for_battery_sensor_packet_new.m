%% this parsing code is with battery sensor packet
% Author KDH
% Date: 2026.08.14
% e-mail : dongh5290@postech.ac.kr
%%

clear; clc;
close all;

script_path = mfilename('fullpath');
if isempty(script_path)
    project_root = pwd;
else
    project_root = fileparts(script_path);
end

[data_file, data_dir] = uigetfile( ...
    {'*.csv', 'CSV files (*.csv)'}, ...
    'Select flightData.csv', ...
    fullfile(project_root, 'Dataset', 'flightData.csv'));

if isequal(data_file, 0)
    error('No flight data file was selected.');
end

data = readmatrix(fullfile(data_dir, data_file));

if size(data, 2) < 53
    error('Flight data must contain at least 53 columns.');
end

% optData = readmatrix(fullfile(data_dir, 'optitrackData.csv'), 4);
%% RMSE Parsing point selection
% [sec]
t0 = 0.0;
tf = 250.0;

%% Drone Parameters
ct = 1.157716790957343e-08;
cq = 2.756919454558223e-5;
m = 0.625;
gConst = 9.8066;
g = [0, 0, gConst]';
ixx = 0.001990948845221;
iyy = 0.002288026102605;
izz = 0.003863848336411;
Iv = diag([ixx, iyy, izz]);
dt = 0.004;

%% Data select
% IF serial message is changed, you have to change
droneIndex = data(: , 1);
mode = data(:, 2);
time = data(:, 3);
att = data(:, 4:6);             % XYZ
attRef = data(:, 26:28);        % target attitude roll pitch yaw
pos = data(:, 7:9);             % XYZ
posRef = data(:, 29:31);
omega = data(:, 10:12);
vel = data(:, 13:15);
updateCount = data(:, 16);
zForce=data(:,17);              % saved log name : battery
cpu = data(:,18);
gyro = data(:, 19:21);
acc = data(:, 22:24);
temp = data(:, 25);             % imu temperature
omegaRef = data(:, 32:34);
velRef = data(:, 35:37);        % target position dot
pwm = data(:, 38:41);           % saved log name : motor
% data 42~45 is pendata : not used

% battery sensor data (old)
% battery_time_stamp = data(:, 46);
% battery_read_out_time = data(:, 47);
% battery_status = data(:, 48);
% cell1_voltage = data(:, 49);
% cell2_voltage = data(:, 50);
% cell3_voltage = data(:, 51);
% cell4_voltage = data(:, 52);
% cell5_voltage = data(:, 53);
% cell6_voltage = data(:, 54);
% battery_total_voltage = data(:, 55);
% battery_current = data(:, 56);
% battery_temperature = data(:, 57);

% new : packet changed at 8/14

battery_time_stamp = data(:, 42);
battery_read_out_time = data(:, 43);
battery_status = data(:, 44);
cell1_voltage = data(:, 45);
cell2_voltage = data(:, 46);
cell3_voltage = data(:, 47);
cell4_voltage = data(:, 48);
cell5_voltage = data(:, 49);
cell6_voltage = data(:, 50);
battery_total_voltage = data(:, 51);
battery_current = data(:, 52);
battery_temperature = data(:, 53);


%% Plot the data
% Attitude

figure(1);
clf; subplot(3,1,1);
plot(time,att(:,1), 'b');
hold on
plot(time,attRef(:,1), 'r--');
title('Attitude','Interpreter','latex');
ylabel('$\phi$ [rad]','Interpreter','latex')
shadeModeBackground(gca, time, mode);
subplot(3,1,2);
plot(time,att(:,2), 'b');
hold on
plot(time,attRef(:,2), 'r--');
ylabel('$\theta$ [rad]','Interpreter','latex')
shadeModeBackground(gca, time, mode);
subplot(3,1,3);
plot(time,att(:,3), 'b');
hold on
plot(time,attRef(:,3), 'r--');
ylabel('$\psi$ [rad]','Interpreter','latex')
shadeModeBackground(gca, time, mode);

% Position
figure(2);
clf;
subplot(3,1,1);
plot(time,pos(:,1), 'b');
hold on
plot(time,posRef(:,1), 'r--');
title('Position',Interpreter='latex')
ylabel('$x$ [m]', 'Interpreter','latex')
shadeModeBackground(gca, time, mode);
subplot(3,1,2);
plot(time,pos(:,2), 'b');
hold on
plot(time,posRef(:,2), 'r--');
ylabel('$y$ [m]', 'Interpreter','latex')
shadeModeBackground(gca, time, mode);
subplot(3,1,3);
plot(time,pos(:,3), 'b');
hold on
plot(time,posRef(:,3), 'r--');
ylabel('$z$ [m]', 'Interpreter','latex')
shadeModeBackground(gca, time, mode);

% Omega
figure(3);
clf;
subplot(3,1,1);
plot(time,omega(:,1), 'b');
hold on
plot(time,omegaRef(:,1), 'r--');
title('Omega',Interpreter='latex')
ylabel('$Roll$ [rad/s]', 'Interpreter','latex')
shadeModeBackground(gca, time, mode);
subplot(3,1,2);
plot(time,omega(:,2), 'b');
hold on
plot(time,omegaRef(:,2), 'r--');
ylabel('$Pitch$ [rad/s]', 'Interpreter','latex')
shadeModeBackground(gca, time, mode);
subplot(3,1,3);
plot(time,omega(:,3), 'b');
hold on
plot(time,omegaRef(:,3), 'r--');
ylabel('$Yaw$ [rad/s]', 'Interpreter','latex')
shadeModeBackground(gca, time, mode);

% Velocity
figure(4);
clf;
subplot(3,1,1);
plot(time,vel(:,1), 'b');
hold on
plot(time,velRef(:,1), 'r--');
title('Velocity',Interpreter='latex')
ylabel('$x$ [m/s]', 'Interpreter','latex')
shadeModeBackground(gca, time, mode);
subplot(3,1,2);
plot(time,vel(:,2), 'b');
hold on
plot(time,velRef(:,2), 'r--');
ylabel('$y$ [m/s]', 'Interpreter','latex')
shadeModeBackground(gca, time, mode);
subplot(3,1,3);
plot(time,vel(:,3), 'b');
hold on
plot(time,velRef(:,3), 'r--');
ylabel('$z$ [m/s]', 'Interpreter','latex')
shadeModeBackground(gca, time, mode);


% PWM
figure(5); clf;
plot(time, pwm(:, 1));
hold on
plot(time, pwm(:, 2));
hold on
plot(time, pwm(:, 3));
hold on
plot(time, pwm(:, 4));
legend({'motor1', 'motor2', 'motor3', 'motor4'})
title('pwm');
shadeModeBackground(gca, time, mode);

figure(6); clf;
plot(time, mode);
shadeModeBackground(gca, time, mode);
title('mode');


%% Calculate RMSE
% computeRMSE(att, attRef, time, t0, tf, 'Attitude [rad]');
% computeRMSE(pos, posRef, time, t0, tf, 'Position [m]');
% computeRMSE(omega, omegaRef, time, t0, tf, 'Angular Velocity [rad/s]');
% computeRMSE(vel, velRef, time, t0, tf, 'Velocity [m/s]');

computeRMSE_by_mode_with_settle(att, attRef, time, mode, t0, tf, 3.0, 'Attitude [rad]');
computeRMSE_by_mode_with_settle(pos, posRef, time, mode, t0, tf, 3.0, 'Position [m]');
computeRMSE_by_mode_with_settle(omega, omegaRef, time, mode, t0, tf, 3.0, 'Angular Velocity [rad/s]');
computeRMSE_by_mode_with_settle(vel, velRef, time, mode, t0, tf, 3.0, 'Velocity [m/s]');

%% Plot battery data

% Low-pass filter cutoff frequency [Hz]
battery_lpf_fc = 0.1;

% 기존 코드에서 설정된 제어 주기
battery_lpf_dt = dt;

% Cell voltage array
cell_voltage = [
    cell1_voltage, ...
    cell2_voltage, ...
    cell3_voltage, ...
    cell4_voltage, ...
    cell5_voltage, ...
    cell6_voltage
];

%% Figure 7: Cell voltage
figure(7);
clf;

for i = 1:6
    subplot(3, 2, i);

    plot(time, cell_voltage(:, i), ...
         'LineWidth', 1.0);

    grid on;
    xlabel('Time [s]');
    ylabel('Voltage');

    title(sprintf('Cell %d Voltage', i));

    shadeModeBackground(gca, time, mode);
end

sgtitle('Battery Cell Voltages');


%% Figure 8: Total battery voltage
battery_total_voltage_lpf = firstOrderLowPass( ...
    battery_total_voltage, ...
    battery_lpf_fc, ...
    battery_lpf_dt);

figure(8);
clf;

subplot(2, 1, 1);
plot(time, battery_total_voltage, ...
     'LineWidth', 1.0);

grid on;
xlabel('Time [s]');
ylabel('Voltage');
title('Total Battery Voltage - Raw');

shadeModeBackground(gca, time, mode);


subplot(2, 1, 2);
plot(time, battery_total_voltage_lpf, ...
     'LineWidth', 1.2);

grid on;
xlabel('Time [s]');
ylabel('Voltage');
title(sprintf('Total Battery Voltage - LPF (%.1f Hz)', ...
              battery_lpf_fc));

shadeModeBackground(gca, time, mode);

sgtitle('Total Battery Voltage');


%% Figure 9: Battery current
battery_current_lpf = firstOrderLowPass( ...
    battery_current, ...
    battery_lpf_fc, ...
    battery_lpf_dt);

figure(9);
clf;

subplot(2, 1, 1);
plot(time, battery_current, ...
     'LineWidth', 1.0);

grid on;
xlabel('Time [s]');
ylabel('Current');
title('Battery Current - Raw');

shadeModeBackground(gca, time, mode);


subplot(2, 1, 2);
plot(time, battery_current_lpf, ...
     'LineWidth', 1.2);

grid on;
xlabel('Time [s]');
ylabel('Current');
title(sprintf('Battery Current - LPF (%.1f Hz)', ...
              battery_lpf_fc));

shadeModeBackground(gca, time, mode);

sgtitle('Battery Current');


%% Figure 10: Battery temperature
battery_temperature_lpf = firstOrderLowPass( ...
    battery_temperature, ...
    battery_lpf_fc, ...
    battery_lpf_dt);

figure(10);
clf;

subplot(2, 1, 1);
plot(time, battery_temperature, ...
     'LineWidth', 1.0);

grid on;
xlabel('Time [s]');
ylabel('Temperature');
title('Battery Temperature - Raw');

shadeModeBackground(gca, time, mode);


subplot(2, 1, 2);
plot(time, battery_temperature_lpf, ...
     'LineWidth', 1.2);

grid on;
xlabel('Time [s]');
ylabel('Temperature');
title(sprintf('Battery Temperature - LPF (%.1f Hz)', ...
              battery_lpf_fc));

shadeModeBackground(gca, time, mode);

sgtitle('Battery Temperature');





%% Helper functions
function shadeModeBackground(ax, time, mode,alpha)
% ax   : subplot axis handle
% time : time vector
% mode : mode vector (same length as time)

    if nargin < 4 || isempty(alpha)
    alpha = 0.1;
    end

    hold(ax, 'on');

    modes = unique(mode, 'stable');
    nMode = numel(modes);

    cmap = lines(nMode);
    yl = ylim(ax);

    for i = 1:nMode
        m = modes(i);
        idx = find(mode == m);

        if isempty(idx)
            continue;
        end

        % 연속 구간의 시작과 끝 찾기
        d = diff(idx);
        splitPoints = [0; find(d ~= 1); numel(idx)];

        for k = 1:length(splitPoints)-1
            iStart = idx(splitPoints(k)+1);
            iEnd   = idx(splitPoints(k+1));

            xPatch = [ time(iStart) ...
                       time(iEnd) ...
                       time(iEnd) ...
                       time(iStart) ];

            yPatch = [yl(1) yl(1) yl(2) yl(2)];

            patch(ax, xPatch, yPatch, cmap(i,:), ...
                  'FaceAlpha', alpha, ...
                  'EdgeColor', 'none');
        end
    end

    % plot 선을 항상 위로
    uistack(findobj(ax,'Type','line'),'top');
end

function computeRMSE(data, refData, time, t0, tf, label)
    % data, refData : Nx3 (or NxM)
    % time          : Nx1
    % t0, tf        : time window
    % label         : 출력용 문자열

    idx = (time >= t0) & (time <= tf);

    e = data(idx,:) - refData(idx,:);
    rmse = sqrt(mean(e.^2, 1));

    fprintf('\n===== RMSE (%s) =====\n', label);
    for i = 1:size(rmse,2)
        fprintf('Axis %d RMSE : %.6f\n', i, rmse(i));
    end
end

function computeRMSE_by_mode_with_settle(data, refData, time, mode, t0, tf, settle_time, label)
% data, refData : NxM
% time          : Nx1
% mode          : Nx1
% t0, tf        : 전체 시간 window
% settle_time   : mode 전환 후 제거할 시간 [sec]

    modes = unique(mode, 'stable');

    fprintf('\n========================================\n');
    fprintf('RMSE by Mode (settle %.2f s) - %s\n', settle_time, label);
    fprintf('========================================\n');

    % mode change index 찾기
    mode_change_idx = [1; find(diff(mode) ~= 0) + 1];

    % 각 샘플마다 "유효 시작 시간" 만들기
    valid_start_time = -inf(size(time));

    for i = 1:length(mode_change_idx)
        idx_start = mode_change_idx(i);

        if i < length(mode_change_idx)
            idx_end = mode_change_idx(i+1) - 1;
        else
            idx_end = length(time);
        end

        t_start_valid = time(idx_start) + settle_time;

        valid_start_time(idx_start:idx_end) = t_start_valid;
    end

    % mode별 RMSE 계산
    for i = 1:numel(modes)
        m = modes(i);

        idx = (mode == m) & ...
              (time >= t0) & (time <= tf) & ...
              (time >= valid_start_time);

        if sum(idx) < 5
            fprintf('\n--- Mode %d --- (skip: not enough data)\n', m);
            continue;
        end

        e = data(idx,:) - refData(idx,:);
        rmse = sqrt(mean(e.^2, 1));

        fprintf('\n--- Mode %d ---\n', m);
        for j = 1:size(rmse,2)
            fprintf('Axis %d RMSE : %.6f\n', j, rmse(j));
        end
    end
end


function out = compute_physical_disturbance_accpos_rotdiff(att, acc, gyro, pwm, time, dt_fixed, params, opts)
    % compute_physical_disturbance_accpos_rotdiff
    %
    % pos disturbance:
    %   pos_d = a_meas(acc) - a_pred(thrust, attitude, g)   (NO differentiation)
    %
    % rot disturbance:
    %   rot_d = J*omega_dot + omega x (J*omega) - tau_pred(thrust)
    %   where omega_dot is obtained by dirty derivative on gyro.

    if nargin < 8
        opts = struct();
    end

    % ---------------- defaults ----------------
    if ~isfield(opts, 'dt_fallback'),       opts.dt_fallback = 0.004; end
    if ~isfield(opts, 'thrust_min'),        opts.thrust_min  = 0.05;  end
    if ~isfield(opts, 'thrust_max'),        opts.thrust_max  = 6.0;   end
    if ~isfield(opts, 'use_continuity'),    opts.use_continuity = true; end
    if ~isfield(opts, 'deriv_fc_omega'),    opts.deriv_fc_omega = 30; end
    if ~isfield(opts, 'gravity_vec_world'), opts.gravity_vec_world = [0; 0; params.g]; end
    if ~isfield(opts, 'acc_frame'),         opts.acc_frame = 'world'; end  % 'world' or 'body'

    N = size(pwm, 1);

    % ---------------- dt safety ----------------
    dt = ones(N, 1) * dt_fixed;
    t = time(:);

    % 만약 time 벡터가 깨져있을 경우를 대비한 백업
    if any(diff(t) <= 0)
        for k = 2:N
            t(k) = t(k-1) + dt_fixed;
        end
    end

    % ---------------- 1) PWM(power) -> thrust (inverse) ----------------
    thrust = nan(N, 4);
    thrust_prev = nan(1, 4);

    for k = 1:N
        for mtr = 1:4
            P = pwm(k, mtr);

            % P = -A*T^4 + B*T^3 - C*T^2 + D*T + E
            polyCoeff = [-params.co_A, params.co_B, -params.co_C, params.co_D, (params.co_E - P)];
            r = roots(polyCoeff);

            r_real = r(abs(imag(r)) < 1e-9);
            r_real = real(r_real);
            r_real = r_real(r_real > opts.thrust_min & r_real < opts.thrust_max);

            if isempty(r_real)
                if isfinite(thrust_prev(mtr))
                    thrust(k, mtr) = thrust_prev(mtr);
                else
                    thrust(k, mtr) = opts.thrust_min;
                end
                continue;
            end

            % residual-min root
            resid = arrayfun(@(x) abs(polyval(polyCoeff, x)), r_real);
            [~, idx_res] = min(resid);
            cand = r_real(idx_res);

            % continuity (optional)
            if opts.use_continuity && isfinite(thrust_prev(mtr)) && numel(r_real) > 1
                thr = min(resid) + 1e-6;
                pool = r_real(resid <= 10 * thr);
                [~, idx_c] = min(abs(pool - thrust_prev(mtr)));
                cand = pool(idx_c);
            end

            thrust(k, mtr) = cand;
        end

        thrust_prev = thrust(k, :);
    end

    zForce_phys = sum(thrust, 2) ./ params.mass;  % [m/s^2]

    % ---------------- 2) predicted acceleration from thrust + attitude ----
    g_w = opts.gravity_vec_world(:);
    if numel(g_w) ~= 3
        error('opts.gravity_vec_world must be 3x1.');
    end

    a_pred = zeros(N, 3);
    R_cache = zeros(3, 3, N);

    for k = 1:N
        roll  = att(k, 1);
        pitch = att(k, 2);
        yaw   = att(k, 3);

        % body -> world rotation
        R = eul2rotm([yaw, pitch, roll], 'ZYX');
        R_cache(:, :, k) = R;

        a_pred(k, :) = (R * [0; 0; -zForce_phys(k)]).';
    end

    % ---------------- 3) measured acceleration from acc (NO diff) --------
    switch lower(opts.acc_frame)
        case 'world'
            a_meas = acc;
        case 'body'
            a_meas = zeros(N, 3);
            for k = 1:N
                a_meas(k, :) = (R_cache(:, :, k) * acc(k, :).').';
            end
        otherwise
            error('opts.acc_frame must be ''world'' or ''body''.');
    end

    pos_d = a_meas - a_pred;

    % ---------------- 4) predicted torque from thrust ---------------------
    L   = params.arm_length;
    TTM = params.THRUST_TO_MOMENT;

    tau_pred = zeros(N, 3);

    for k = 1:N
        T1 = thrust(k, 1);
        T2 = thrust(k, 2);
        T3 = thrust(k, 3);
        T4 = thrust(k, 4);

        % IMPORTANT: must match attctrl.c motor order & sign conventions
        tau_x = L * (T4 - T2);
        tau_y = L * (T1 - T3);
        tau_z = ((T2 + T4) - (T1 + T3)) / TTM;

        tau_pred(k, :) = [tau_x, tau_y, tau_z];
    end

    % ---------------- 5) rotational disturbance (same structure) ----------
    Iv = diag([params.Ixx, params.Iyy, params.Izz]);

    omega_dot = dirty_derivative(gyro, t, opts.deriv_fc_omega, dt_fixed);

    rot_d = zeros(N, 3);
    for k = 1:N
        w = gyro(k, :).';
        rot_d(k, :) = (Iv * omega_dot(k, :).' + cross(w, Iv * w) - tau_pred(k, :).').';
    end

    % ---------------- output ----------------
    out = struct();
    out.thrust      = thrust;
    out.zForce_phys = zForce_phys;

    out.a_meas = a_meas;
    out.a_pred = a_pred;
    out.pos_d  = pos_d;

    out.tau_pred  = tau_pred;
    out.omega_dot = omega_dot;
    out.rot_d     = rot_d;
end




function xdot = dirty_derivative(x, t, fc, dt_fixed)
    x = double(x);
    N = size(x, 1);
    xdot = zeros(N, size(x, 2));

    alpha = exp(-2 * pi * fc * dt_fixed);

    for k = 2:N
        dx = (x(k, :) - x(k-1, :)) / dt_fixed;
        xdot(k, :) = alpha * xdot(k-1, :) + (1 - alpha) * dx;
    end
end

function y = firstOrderLowPass(x, fc, dt)
% firstOrderLowPass
% x  : NxM input data
% fc : cutoff frequency [Hz]
% dt : sampling period [s]

    x = double(x);
    y = zeros(size(x));

    if isempty(x)
        return;
    end

    if fc <= 0
        error('LPF cutoff frequency must be positive.');
    end

    if dt <= 0
        error('Sampling period must be positive.');
    end

    % Exact discretization of a first-order LPF
    alpha = exp(-2*pi*fc*dt);

    y(1, :) = x(1, :);

    for k = 2:size(x, 1)
        if any(~isfinite(x(k, :)))
            y(k, :) = y(k-1, :);
        else
            y(k, :) = alpha*y(k-1, :) ...
                    + (1-alpha)*x(k, :);
        end
    end
end
