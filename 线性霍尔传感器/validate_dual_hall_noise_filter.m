%% 双线性霍尔 ADC量化、白噪声和低通滤波验证
% 本脚本基于当前谐波补偿基准模型继续向实机采样靠近：
%   1. 先生成含零偏、幅值不一致、相位不正交和2/3/5/6次谐波的Hall信号；
%   2. 使用无噪声数据建立一组固定的离线补偿系数；
%   3. 分别验证无噪声、仅ADC、ADC+噪声、ADC+噪声+低通滤波四种情况；
%   4. 对比基础补偿、信号域谐波补偿和角度域傅里叶补偿后的解角误差。

clear
clc
close all

scriptDir = fileparts(mfilename('fullpath'));
addpath(scriptDir);

init_dual_hall_noise_filter_params;
P = collect_hall_params;

figDir = fullfile(scriptDir, 'figures');
if ~exist(figDir, 'dir')
    mkdir(figDir);
end

%% 生成无噪声基准Hall信号
t = (0:hall_ts_sim_s:hall_t_stop_s).';
theta_m = hall_omega_mech_rad_s*t;
theta_mag = hall_pole_pairs*theta_m + hall_theta0_rad;
calib_idx = t < hall_t_stop_s;
plot_idx = t <= min(hall_noise_fig_window_s, hall_t_stop_s);

[hall_s_clean_v, hall_c_clean_v] = generate_clean_hall_signal(theta_mag, P);

%% 构造ADC量化和白噪声信号
rng(hall_noise_seed);
hall_s_noise_v = hall_noise_rms_v*randn(size(hall_s_clean_v));
hall_c_noise_v = hall_noise_rms_v*randn(size(hall_c_clean_v));

hall_s_adc_v = adc_quantize(hall_s_clean_v, P);
hall_c_adc_v = adc_quantize(hall_c_clean_v, P);

hall_s_noisy_adc_v = adc_quantize(hall_s_clean_v + hall_s_noise_v, P);
hall_c_noisy_adc_v = adc_quantize(hall_c_clean_v + hall_c_noise_v, P);

hall_s_filt_v = first_order_lpf(hall_s_noisy_adc_v, hall_lpf_alpha);
hall_c_filt_v = first_order_lpf(hall_c_noisy_adc_v, hall_lpf_alpha);

%% 用无噪声数据建立固定补偿系数
cal = estimate_baseline_calibration(hall_s_clean_v, hall_c_clean_v, theta_mag, calib_idx, P);

%% 对四种采样情况执行同一套补偿链路
result_clean = apply_compensation_chain(hall_s_clean_v, hall_c_clean_v, theta_mag, cal, P);
result_adc = apply_compensation_chain(hall_s_adc_v, hall_c_adc_v, theta_mag, cal, P);
result_noisy = apply_compensation_chain(hall_s_noisy_adc_v, hall_c_noisy_adc_v, theta_mag, cal, P);
result_filt = apply_compensation_chain(hall_s_filt_v, hall_c_filt_v, theta_mag, cal, P);

scenarioNames = [
    "clean_baseline";
    "adc_quantization";
    "adc_plus_noise";
    "adc_noise_lowpass"
];
scenarioNamesCn = [
    "无噪声基准";
    "仅ADC量化";
    "ADC量化+白噪声";
    "ADC量化+白噪声+低通滤波"
];

results = {result_clean; result_adc; result_noisy; result_filt};
metrics = zeros(numel(results), 6);
for k = 1:numel(results)
    metrics(k,:) = [
        max(abs(rad2deg(results{k}.err_basic))), rms_local(rad2deg(results{k}.err_basic)), ...
        max(abs(rad2deg(results{k}.err_sigcomp))), rms_local(rad2deg(results{k}.err_sigcomp)), ...
        max(abs(rad2deg(results{k}.err_anglecomp))), rms_local(rad2deg(results{k}.err_anglecomp))
    ];
end

metricsTable = table(scenarioNames, scenarioNamesCn, ...
    metrics(:,1), metrics(:,2), metrics(:,3), metrics(:,4), metrics(:,5), metrics(:,6), ...
    'VariableNames', {'scenario_id', 'scenario_cn', ...
    'basic_max_deg', 'basic_rms_deg', ...
    'signal_harmonic_max_deg', 'signal_harmonic_rms_deg', ...
    'angle_fourier_max_deg', 'angle_fourier_rms_deg'});

csvPath = fullfile(figDir, 'dual_hall_noise_filter_metrics.csv');
writetable(metricsTable, csvPath);
prepend_utf8_bom(csvPath);

%% 绘图
fig = figure('Name', 'Dual Hall ADC Noise Filter Validation', ...
    'Color', 'w', 'Position', [70 70 1250 720]);
tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile
plot(t(plot_idx), hall_s_clean_v(plot_idx), 'k', 'LineWidth', 1.1)
hold on
plot(t(plot_idx), hall_s_noisy_adc_v(plot_idx), 'Color', [0.80 0.20 0.20], 'LineWidth', 0.8)
plot(t(plot_idx), hall_s_filt_v(plot_idx), 'b', 'LineWidth', 1.0)
grid on
xlabel('Time / s')
ylabel('Hall voltage / V')
title('(a) H_s voltage: clean, noisy ADC and low-pass filtered')
legend('clean', 'ADC + noise', 'ADC + noise + LPF', 'Location', 'southoutside', 'NumColumns', 3)

nexttile
plot(t(plot_idx), (hall_s_adc_v(plot_idx) - hall_s_clean_v(plot_idx))*1e3, 'Color', [0.45 0.45 0.45], 'LineWidth', 0.9)
hold on
plot(t(plot_idx), (hall_s_noisy_adc_v(plot_idx) - hall_s_clean_v(plot_idx))*1e3, 'r', 'LineWidth', 0.8)
plot(t(plot_idx), (hall_s_filt_v(plot_idx) - hall_s_clean_v(plot_idx))*1e3, 'b', 'LineWidth', 0.9)
grid on
xlabel('Time / s')
ylabel('Voltage error / mV')
title('(b) Sampling error relative to clean H_s')
legend('ADC only', 'ADC + noise', 'after LPF', 'Location', 'southoutside', 'NumColumns', 3)

nexttile
plot(t(plot_idx), rad2deg(result_clean.err_anglecomp(plot_idx)), 'k', 'LineWidth', 1.0)
hold on
plot(t(plot_idx), rad2deg(result_noisy.err_anglecomp(plot_idx)), 'r', 'LineWidth', 0.8)
plot(t(plot_idx), rad2deg(result_filt.err_anglecomp(plot_idx)), 'b', 'LineWidth', 1.0)
grid on
xlabel('Time / s')
ylabel('Mechanical angle error / deg')
title('(c) Final angle error after deterministic compensation')
legend('clean baseline', 'ADC + noise', 'ADC + noise + LPF', 'Location', 'southoutside', 'NumColumns', 3)

nexttile
barData = [metrics(:,5), metrics(:,6)];
b = bar(barData);
b(1).FaceColor = [0.30 0.55 0.90];
b(2).FaceColor = [0.90 0.55 0.25];
grid on
set(gca, 'XTickLabel', scenarioNamesCn)
xtickangle(20)
ylabel('Mechanical angle error / deg')
title('(d) Final angle-domain compensation metrics')
legend('max error', 'RMS error', 'Location', 'northwest')

axs = findall(fig, 'Type', 'Axes');
for ax = reshape(axs, 1, [])
    try
        ax.Toolbar.Visible = 'off';
    catch
    end
    try
        disableDefaultInteractivity(ax);
    catch
    end
end

pngPath = fullfile(figDir, 'dual_hall_noise_filter_validation.png');
exportgraphics(fig, pngPath, 'Resolution', 220);

fprintf('\nDual-Hall ADC/noise/filter validation\n');
fprintf('ADC resolution: %d bit, LSB = %.4g V\n', hall_adc_bits, hall_adc_lsb_v);
fprintf('Noise RMS: %.4g V, LPF cutoff: %.4g Hz\n', hall_noise_rms_v, hall_lpf_cutoff_hz);
fprintf('\nFinal angle-domain compensation metrics:\n');
for k = 1:height(metricsTable)
    fprintf('  %s: max %.4f deg, rms %.4f deg\n', ...
        metricsTable.scenario_cn(k), metricsTable.angle_fourier_max_deg(k), metricsTable.angle_fourier_rms_deg(k));
end
fprintf('\nSaved noise/filter validation figure:\n%s\n', pngPath);
fprintf('Saved noise/filter metrics:\n%s\n', csvPath);

%% 局部函数
function P = collect_hall_params
P.offset_s_actual_v = evalin('base', 'hall_offset_s_actual_v');
P.offset_c_actual_v = evalin('base', 'hall_offset_c_actual_v');
P.amp_s_v = evalin('base', 'hall_amp_s_v');
P.amp_c_v = evalin('base', 'hall_amp_c_v');
P.phase_err_rad = evalin('base', 'hall_phase_err_rad');
P.harm_orders = evalin('base', 'hall_harm_orders');
P.harm_s_sin_v = evalin('base', 'hall_harm_s_sin_v');
P.harm_s_cos_v = evalin('base', 'hall_harm_s_cos_v');
P.harm_c_sin_v = evalin('base', 'hall_harm_c_sin_v');
P.harm_c_cos_v = evalin('base', 'hall_harm_c_cos_v');
P.adc_enable = evalin('base', 'hall_adc_enable');
P.vref_v = evalin('base', 'hall_vref_v');
P.adc_bits = evalin('base', 'hall_adc_bits');
P.signal_comp_orders = evalin('base', 'hall_signal_comp_orders');
P.angle_comp_orders = evalin('base', 'hall_angle_comp_orders');
P.pole_pairs = evalin('base', 'hall_pole_pairs');
end

function [hall_s_v, hall_c_v] = generate_clean_hall_signal(theta_mag, P)
hall_s_fund_v = P.offset_s_actual_v + P.amp_s_v*sin(theta_mag);
hall_c_fund_v = P.offset_c_actual_v + P.amp_c_v*cos(theta_mag + P.phase_err_rad);

hall_s_harm_v = zeros(size(theta_mag));
hall_c_harm_v = zeros(size(theta_mag));
for idx = 1:numel(P.harm_orders)
    n = P.harm_orders(idx);
    hall_s_harm_v = hall_s_harm_v + ...
        P.harm_s_sin_v(idx)*sin(n*theta_mag) + ...
        P.harm_s_cos_v(idx)*cos(n*theta_mag);
    hall_c_harm_v = hall_c_harm_v + ...
        P.harm_c_sin_v(idx)*sin(n*theta_mag) + ...
        P.harm_c_cos_v(idx)*cos(n*theta_mag);
end

hall_s_v = hall_s_fund_v + hall_s_harm_v;
hall_c_v = hall_c_fund_v + hall_c_harm_v;
end

function y = adc_quantize(x, P)
if ~P.adc_enable
    y = x;
    return
end
x_clip = min(max(x, 0), P.vref_v);
adc_max = 2^P.adc_bits - 1;
adc_code = round(x_clip/P.vref_v*adc_max);
y = adc_code/adc_max*P.vref_v;
end

function y = first_order_lpf(x, alpha)
y = zeros(size(x));
y(1) = x(1);
for idx = 2:numel(x)
    y(idx) = alpha*y(idx-1) + (1 - alpha)*x(idx);
end
end

function cal = estimate_baseline_calibration(hall_s_v, hall_c_v, theta_mag, calib_idx, P)
cal.offset_s_v = 0.5*(max(hall_s_v(calib_idx)) + min(hall_s_v(calib_idx)));
cal.offset_c_v = 0.5*(max(hall_c_v(calib_idx)) + min(hall_c_v(calib_idx)));
cal.amp_s_v = 0.5*(max(hall_s_v(calib_idx)) - min(hall_s_v(calib_idx)));
cal.amp_c_v = 0.5*(max(hall_c_v(calib_idx)) - min(hall_c_v(calib_idx)));

if cal.amp_s_v <= eps || cal.amp_c_v <= eps
    error('Hall calibration failed: estimated amplitude is too small.');
end

sin_norm = (hall_s_v - cal.offset_s_v)/cal.amp_s_v;
cos_norm = (hall_c_v - cal.offset_c_v)/cal.amp_c_v;

sin_calib_zm = sin_norm(calib_idx) - mean(sin_norm(calib_idx));
cos_calib_zm = cos_norm(calib_idx) - mean(cos_norm(calib_idx));
phase_coef = mean(sin_calib_zm.*cos_calib_zm) / ...
    sqrt(mean(sin_calib_zm.^2)*mean(cos_calib_zm.^2));
phase_coef = max(-1, min(1, phase_coef));
cal.phase_err_rad = asin(-phase_coef);

cos_phasecorr = (cos_norm + sin_norm*sin(cal.phase_err_rad))/cos(cal.phase_err_rad);
theta_basic = align_angle(unwrap(atan2(sin_norm, cos_phasecorr)), theta_mag);

Phi_sig_calib = harmonic_matrix(theta_mag(calib_idx), P.signal_comp_orders);

res_s_calib = sin_norm(calib_idx) - sin(theta_mag(calib_idx));
res_c_calib = cos_phasecorr(calib_idx) - cos(theta_mag(calib_idx));
cal.coef_sig_s = Phi_sig_calib \ res_s_calib;
cal.coef_sig_c = Phi_sig_calib \ res_c_calib;

angle_err_mag_basic = wrap_pi(theta_basic - theta_mag);
Phi_ang_calib = harmonic_matrix(theta_basic(calib_idx), P.angle_comp_orders);
cal.coef_ang_err = Phi_ang_calib \ angle_err_mag_basic(calib_idx);
end

function result = apply_compensation_chain(hall_s_v, hall_c_v, theta_mag, cal, P)
sin_norm = (hall_s_v - cal.offset_s_v)/cal.amp_s_v;
cos_norm = (hall_c_v - cal.offset_c_v)/cal.amp_c_v;
cos_phasecorr = (cos_norm + sin_norm*sin(cal.phase_err_rad))/cos(cal.phase_err_rad);

theta_basic = align_angle(unwrap(atan2(sin_norm, cos_phasecorr)), theta_mag);
err_basic = wrap_pi(theta_basic - theta_mag)/P.pole_pairs;

Phi_sig_all = harmonic_matrix(theta_basic, P.signal_comp_orders);
sin_harm_est = Phi_sig_all*cal.coef_sig_s;
cos_harm_est = Phi_sig_all*cal.coef_sig_c;
sin_sigcomp = sin_norm - sin_harm_est;
cos_sigcomp = cos_phasecorr - cos_harm_est;

theta_sigcomp = align_angle(unwrap(atan2(sin_sigcomp, cos_sigcomp)), theta_mag);
err_sigcomp = wrap_pi(theta_sigcomp - theta_mag)/P.pole_pairs;

Phi_ang_all = harmonic_matrix(theta_basic, P.angle_comp_orders);
angle_err_mag_hat = Phi_ang_all*cal.coef_ang_err;
theta_anglecomp = align_angle(theta_basic - angle_err_mag_hat, theta_mag);
err_anglecomp = wrap_pi(theta_anglecomp - theta_mag)/P.pole_pairs;

result.theta_basic = theta_basic;
result.theta_sigcomp = theta_sigcomp;
result.theta_anglecomp = theta_anglecomp;
result.err_basic = err_basic;
result.err_sigcomp = err_sigcomp;
result.err_anglecomp = err_anglecomp;
result.sin_norm = sin_norm;
result.cos_norm = cos_phasecorr;
result.sin_sigcomp = sin_sigcomp;
result.cos_sigcomp = cos_sigcomp;
end

function Phi = harmonic_matrix(theta, orders)
theta = theta(:);
Phi = ones(numel(theta), 1);
for idx = 1:numel(orders)
    n = orders(idx);
    Phi = [Phi, sin(n*theta), cos(n*theta)]; %#ok<AGROW>
end
end

function y = wrap_pi(x)
y = mod(x + pi, 2*pi) - pi;
end

function theta_aligned = align_angle(theta_est, theta_ref)
offset = round((theta_est(1) - theta_ref(1))/(2*pi))*2*pi;
theta_aligned = theta_est - offset;
end

function value = rms_local(x)
x = x(:);
value = sqrt(mean(x.^2));
end

function prepend_utf8_bom(filePath)
fid = fopen(filePath, 'r');
if fid < 0
    return
end
data = fread(fid, '*uint8');
fclose(fid);

bom = uint8([239; 187; 191]);
if numel(data) >= 3 && all(data(1:3) == bom)
    return
end

fid = fopen(filePath, 'w');
if fid < 0
    return
end
fwrite(fid, bom, 'uint8');
fwrite(fid, data, 'uint8');
fclose(fid);
end
