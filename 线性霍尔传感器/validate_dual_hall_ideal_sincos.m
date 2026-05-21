%% Validate ideal dual-linear-Hall sin/cos Simulink model

clear
clc
close all

scriptDir = fileparts(mfilename('fullpath'));
addpath(scriptDir);

init_dual_hall_ideal_params;
mdl = build_dual_hall_ideal_sincos_model();
% hall_pole_pairs	磁钢极对数
% hall_magnet_od_mm	磁钢外径
% hall_vref_v	ADC/Hall 参考电压
% hall_offset_v	Hall 中点偏置电压
% hall_amp_v	Hall 信号幅值
% hall_mech_speed_rpm	机械转速
% hall_f_mech_hz	机械转动频率
% hall_f_mag_hz	磁场信号频率
% hall_omega_mech_rad_s	机械角速度
% hall_theta0_rad	安装零位偏移
% hall_ts_sim_s	仿真步长
% hall_t_stop_s	仿真停止时间

modelPath = fullfile(scriptDir, 'models', [mdl '.slx']);
load_system(modelPath);
simOut = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
simout_dual_hall_ideal = simOut.get('simout_dual_hall_ideal');
%加载模型并运行仿真

t = simout_dual_hall_ideal.time;
y = simout_dual_hall_ideal.signals.values;
%提取时间和信号矩阵

theta_m = y(:,1);
theta_mag = y(:,2);
hall_sin_v = y(:,3);
hall_cos_v = y(:,4);
%拆分四路信号

hall_sin_norm = (hall_sin_v - hall_offset_v)/hall_amp_v;
hall_cos_norm = (hall_cos_v - hall_offset_v)/hall_amp_v;
%归一化两路 Hall 电压

theta_mag_est = unwrap(atan2(hall_sin_norm, hall_cos_norm));
%用 atan2 解算磁场角,atan2(y, x)=atan2(sin, cos)
theta_m_est = theta_mag_est / hall_pole_pairs;
%磁场角转换为机械角
theta_m_err = theta_m_est - theta_m;
%计算机械角误差


fprintf('\nIdeal dual-Hall sin/cos validation\n');
fprintf('Magnet: %.1f mm hollow magnet, pole pairs p = %d\n', ...
    hall_magnet_od_mm, hall_pole_pairs);
fprintf('Mechanical speed = %.3f rpm, f_mech = %.3f Hz, f_mag = %.3f Hz\n', ...
    hall_mech_speed_rpm, hall_f_mech_hz, hall_f_mag_hz);
fprintf('Hall signal: offset = %.3f V, amplitude = %.3f V\n', ...
    hall_offset_v, hall_amp_v);
fprintf('Max ideal angle decoding error = %.3e rad\n\n', ...
    max(abs(theta_m_err)));
%打印相关信息

figDir = fullfile(scriptDir, 'figures');
if ~exist(figDir, 'dir')
    mkdir(figDir);
end
%创建图像保存文件夹

fig = figure('Name', 'Ideal Dual Linear Hall SinCos Signals', ...
    'Color', 'w', 'Position', [80 80 1180 760]);
tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
%创建图窗和 2x2 布局

nexttile
plot(t, hall_sin_v, 'b', 'LineWidth', 1.2)
hold on
plot(t, hall_cos_v, 'r', 'LineWidth', 1.2)
grid on
xlabel('Time / s')
ylabel('Hall voltage / V')
title('(a) Ideal two-channel Hall voltage')
legend('H_s = V_0 + A sin(\theta_{mag})', ...
    'H_c = V_0 + A cos(\theta_{mag})', 'Location', 'best')
    %第一张图：Hall 电压随时间变化

nexttile
plot(theta_m, hall_sin_v, 'b', 'LineWidth', 1.0)
hold on
plot(theta_m, hall_cos_v, 'r', 'LineWidth', 1.0)
grid on
xlabel('\theta_m / rad')
ylabel('Hall voltage / V')
title('(b) Signal versus mechanical angle')
legend('H_s', 'H_c', 'Location', 'best')
%第二张图：Hall 电压随机械角变化

nexttile
plot(hall_cos_norm, hall_sin_norm, 'k', 'LineWidth', 1.1)
axis equal
grid on
xlabel('normalized H_c')
ylabel('normalized H_s')
title('(c) Lissajous circle after ideal normalization')
%第三张图：归一化后的相图

nexttile
plot(t, theta_m, 'k--', 'LineWidth', 1.1)
hold on
plot(t, theta_m_est, 'b', 'LineWidth', 1.0)
grid on
xlabel('Time / s')
ylabel('Mechanical angle / rad')
title('(d) atan2 angle decoding check')
legend('\theta_m true', '\theta_m decoded', 'Location', 'best')
%角度解算验证

pngPath = fullfile(figDir, 'dual_hall_ideal_sincos_validation.png');
exportgraphics(fig, pngPath, 'Resolution', 220);
fprintf('Saved validation figure:\n%s\n', pngPath);
%保存图像