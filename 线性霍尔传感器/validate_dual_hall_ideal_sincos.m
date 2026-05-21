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
% hall_amp_s_v	正弦通道实际幅值 A_s
% hall_amp_c_v	余弦通道实际幅值 A_c
% hall_amp_v	未补偿对比用的名义幅值
% hall_phase_err_rad	余弦通道相位不正交误差 Δφ
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

hall_sin_norm_raw = (hall_sin_v - hall_offset_v)/hall_amp_v;
hall_cos_norm_raw = (hall_cos_v - hall_offset_v)/hall_amp_v;
%未补偿：仍按理想 1.65 V 中点和名义幅值归一化，故会残留零偏和幅值误差

hall_sin_norm = (hall_sin_v - hall_offset_s_actual_v)/hall_amp_s_v;
hall_cos_norm = (hall_cos_v - hall_offset_c_actual_v)/hall_amp_c_v;
%已知参数补偿：按两路实际中点电压和实际幅值归一化

%% 实机估计模块：只根据采样到的 Hall 电压估计中点和幅值
% 实机上通常不知道 hall_offset_s_actual_v / hall_offset_c_actual_v。
% 因此先让电机/云台缓慢扫过至少一个完整磁周期，采集 H_s、H_c，
% 再用最大值和最小值估计每一路的零偏和幅值。
calib_idx = true(size(t));
hall_sin_calib_v = hall_sin_v(calib_idx);
hall_cos_calib_v = hall_cos_v(calib_idx);

hall_offset_s_est_v = 0.5*(max(hall_sin_calib_v) + min(hall_sin_calib_v));
hall_offset_c_est_v = 0.5*(max(hall_cos_calib_v) + min(hall_cos_calib_v));
hall_amp_s_est_v = 0.5*(max(hall_sin_calib_v) - min(hall_sin_calib_v));
hall_amp_c_est_v = 0.5*(max(hall_cos_calib_v) - min(hall_cos_calib_v));

if hall_amp_s_est_v <= eps || hall_amp_c_est_v <= eps
    error('Hall calibration failed: estimated amplitude is too small.');
end

hall_sin_norm_est = (hall_sin_v - hall_offset_s_est_v)/hall_amp_s_est_v;
hall_cos_norm_est = (hall_cos_v - hall_offset_c_est_v)/hall_amp_c_est_v;
% 实机估计补偿：后续上板时优先采用这一组估计量，而不是仿真已知量

%% 相位不正交补偿/椭圆校正
% 相位误差模型：H_c = V0c + A_c*cos(theta + Δφ)。
% 已完成零偏和幅值归一化后，仍有 x_c = cos(theta + Δφ)。
% 根据 cos(theta + Δφ)=cos(theta)cos(Δφ)-sin(theta)sin(Δφ)，可反推：
% cos(theta) = [x_c + x_s*sin(Δφ)]/cos(Δφ)。
if abs(cos(hall_phase_err_rad)) < 1e-6
    error('Phase compensation failed: cos(hall_phase_err_rad) is too small.');
end
%安全检查

hall_cos_norm_phasecorr = ...
    (hall_cos_norm + hall_sin_norm*sin(hall_phase_err_rad))/cos(hall_phase_err_rad);
% 已知相位误差补偿：用于验证补偿公式是否正确

%Δφ 从哪里来？
hall_sin_calib_norm = hall_sin_norm_est(calib_idx);
hall_cos_calib_norm = hall_cos_norm_est(calib_idx);
%取出用于标定的数据段实机可改为calib_idx = t > 0.2 & t < 1.8;避开启动瞬态，只用稳定转动的数据
hall_sin_calib_zero_mean = hall_sin_calib_norm - mean(hall_sin_calib_norm);
hall_cos_calib_zero_mean = hall_cos_calib_norm - mean(hall_cos_calib_norm);
%把标定数据减去平均值，变成零均值信号
hall_phase_corr_coef = mean(hall_sin_calib_zero_mean.*hall_cos_calib_zero_mean) / ...%计算相关系数
    sqrt(mean(hall_sin_calib_zero_mean.^2)*mean(hall_cos_calib_zero_mean.^2));
    %相关系数归一化分母
hall_phase_corr_coef = max(-1, min(1, hall_phase_corr_coef));
%限制相关系数范围
hall_phase_err_est_rad = asin(-hall_phase_corr_coef);
% 实机估计相位误差：归一化后两路相关系数 rho≈-sin(Δφ)

if abs(cos(hall_phase_err_est_rad)) < 1e-6
    error('Estimated phase compensation failed: cos(hall_phase_err_est_rad) is too small.');
end

hall_cos_norm_est_phasecorr = ...
    (hall_cos_norm_est + hall_sin_norm_est*sin(hall_phase_err_est_rad))/cos(hall_phase_err_est_rad);
% 实机估计相位补偿：不使用仿真注入的 Δφ，只使用采样估计出来的 Δφ

theta_mag_raw = unwrap(atan2(hall_sin_norm_raw, hall_cos_norm_raw));
theta_m_raw = theta_mag_raw / hall_pole_pairs;
theta_m_err_raw = theta_m_raw - theta_m;
%未补偿角度误差

theta_mag_est = unwrap(atan2(hall_sin_norm, hall_cos_norm));
%用 atan2 解算磁场角,atan2(y, x)=atan2(sin, cos)
theta_m_est = theta_mag_est / hall_pole_pairs;
%磁场角转换为机械角
theta_m_err = theta_m_est - theta_m;
%零偏补偿后的机械角误差

theta_mag_est_real = unwrap(atan2(hall_sin_norm_est, hall_cos_norm_est));
theta_m_est_real = theta_mag_est_real / hall_pole_pairs;
theta_m_err_real = theta_m_est_real - theta_m;
%实机估计补偿后的机械角误差

theta_mag_phasecorr = unwrap(atan2(hall_sin_norm, hall_cos_norm_phasecorr));
theta_m_phasecorr = theta_mag_phasecorr / hall_pole_pairs;
theta_m_err_phasecorr = theta_m_phasecorr - theta_m;
%已知相位误差补偿后的机械角误差

theta_mag_phasecorr_est = unwrap(atan2(hall_sin_norm_est, hall_cos_norm_est_phasecorr));
theta_m_phasecorr_est = theta_mag_phasecorr_est / hall_pole_pairs;
theta_m_err_phasecorr_est = theta_m_phasecorr_est - theta_m;
%实机估计零偏/幅值/相位补偿后的机械角误差

fprintf('\nDual-Hall sin/cos validation with offset, amplitude mismatch and phase error\n');
fprintf('Magnet: %.1f mm hollow magnet, pole pairs p = %d\n', ...
    hall_magnet_od_mm, hall_pole_pairs);
fprintf('Mechanical speed = %.3f rpm, f_mech = %.3f Hz, f_mag = %.3f Hz\n', ...
    hall_mech_speed_rpm, hall_f_mech_hz, hall_f_mag_hz);
fprintf('Nominal offset = %.3f V, nominal amplitude = %.3f V\n', ...
    hall_offset_v, hall_amp_v);
fprintf('Actual offsets: H_s %.3f V, H_c %.3f V\n', ...
    hall_offset_s_actual_v, hall_offset_c_actual_v);
fprintf('Actual amplitudes: A_s %.3f V, A_c %.3f V\n', ...
    hall_amp_s_v, hall_amp_c_v);
fprintf('Actual phase error: Δφ %.3f deg\n', hall_phase_err_deg);
fprintf('Sample-estimated offsets: H_s %.3f V, H_c %.3f V\n', ...
    hall_offset_s_est_v, hall_offset_c_est_v);
fprintf('Sample-estimated amplitudes: A_s %.3f V, A_c %.3f V\n', ...
    hall_amp_s_est_v, hall_amp_c_est_v);
fprintf('Sample-estimated phase error: Δφ %.3f deg\n', ...
    rad2deg(hall_phase_err_est_rad));
fprintf('Max raw angle decoding error = %.3e rad\n', ...
    max(abs(theta_m_err_raw)));
fprintf('Max offset/amplitude-compensated angle decoding error = %.3e rad\n', ...
    max(abs(theta_m_err)));
fprintf('Max sample-estimated offset/amplitude angle decoding error = %.3e rad\n', ...
    max(abs(theta_m_err_real)));
fprintf('Max known-phase-corrected angle decoding error = %.3e rad\n', ...
    max(abs(theta_m_err_phasecorr)));
fprintf('Max sample-estimated phase-corrected angle decoding error = %.3e rad\n\n', ...
    max(abs(theta_m_err_phasecorr_est)));
%打印相关信息

figDir = fullfile(scriptDir, 'figures');
if ~exist(figDir, 'dir')
    mkdir(figDir);
end
%创建图像保存文件夹

fig = figure('Name', 'Dual Linear Hall SinCos Signals With Offset, Amplitude and Phase Error', ...
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
title('(a) Hall voltage with offset, amplitude and phase error')
legend(sprintf('H_s: V0 %.2f V, A_s %.2f V', hall_offset_s_actual_v, hall_amp_s_v), ...
    sprintf('H_c: V0 %.2f V, A_c %.2f V, \\Delta\\phi %.1f^\\circ', ...
    hall_offset_c_actual_v, hall_amp_c_v, hall_phase_err_deg), ...
    'Location', 'best')
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
plot(hall_cos_norm_raw, hall_sin_norm_raw, 'r', 'LineWidth', 1.0)
hold on
plot(hall_cos_norm, hall_sin_norm, 'b--', 'LineWidth', 1.1)
plot(hall_cos_norm_est_phasecorr, hall_sin_norm_est, 'k', 'LineWidth', 1.0)
axis equal
grid on
xlabel('normalized H_c')
ylabel('normalized H_s')
title('(c) Lissajous: raw, tilted ellipse and phase correction')
legend('before compensation', 'offset/amplitude compensation', ...
    'sample-estimated phase correction', 'Location', 'best')
%第三张图：归一化后的相图

nexttile
plot(t, rad2deg(theta_m_err_raw), 'r','LineWidth', 0.9)
hold on
plot(t, rad2deg(theta_m_err_real), 'b--', 'LineWidth', 1.0)
plot(t, rad2deg(theta_m_err_phasecorr_est), 'k', 'LineWidth', 1.0)
grid on
xlabel('Time / s')
ylabel('Mechanical angle error / deg')
title('(d) atan2 angle decoding error comparison')
legend('raw decoded', 'offset/amplitude compensated', ...
    'phase-corrected by sample estimation', 'Location', 'best')
%角度解算验证

pngPath = fullfile(figDir, 'dual_hall_ideal_sincos_validation.png');
exportgraphics(fig, pngPath, 'Resolution', 220);
fprintf('Saved validation figure:\n%s\n', pngPath);
%保存图像
