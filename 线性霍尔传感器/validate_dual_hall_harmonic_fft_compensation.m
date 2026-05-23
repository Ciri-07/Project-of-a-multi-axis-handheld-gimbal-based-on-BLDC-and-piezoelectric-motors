%% Validate dual-linear-Hall harmonic identification and compensation
% Main validation chain:
%   1) Generate two Hall voltages with offset, amplitude mismatch, phase
%      non-orthogonality and 2/3/5/6 magnetic-angle harmonics.
%   2) Estimate offset, amplitude and phase from sampled data.
%   3) Apply signal-domain harmonic reconstruction to improve the Lissajous.
%   4) Use FFT/Fourier angle-error compensation to reduce atan2 error.

clear
clc
close all

scriptDir = fileparts(mfilename('fullpath'));
addpath(scriptDir);

init_dual_hall_harmonic_params;

figDir = fullfile(scriptDir, 'figures');
if ~exist(figDir, 'dir')
    mkdir(figDir);
end
%设置图片保存文件夹

%% Generate constant-speed Hall signals
t = (0:hall_ts_sim_s:hall_t_stop_s).';%生成时间列向量。采样间隔是 hall_ts_sim_s，总时间是 hall_t_stop_s
theta_m = hall_omega_mech_rad_s*t;
theta_mag = hall_pole_pairs*theta_m + hall_theta0_rad;%生成机械角、磁场角

hall_sin_fund_v = hall_offset_s_actual_v + hall_amp_s_v*sin(theta_mag);
hall_cos_fund_v = hall_offset_c_actual_v + hall_amp_c_v*cos(theta_mag + hall_phase_err_rad);
%生成两路 Hall 基波电压

hall_sin_harm_v = zeros(size(theta_mag));
hall_cos_harm_v = zeros(size(theta_mag));%初始化两路谐波电压，初始全为 0
for k = 1:numel(hall_harm_orders)
    n = hall_harm_orders(k);
    hall_sin_harm_v = hall_sin_harm_v + ...
        hall_harm_s_sin_v(k)*sin(n*theta_mag) + ...
        hall_harm_s_cos_v(k)*cos(n*theta_mag);
    hall_cos_harm_v = hall_cos_harm_v + ...
        hall_harm_c_sin_v(k)*sin(n*theta_mag) + ...
        hall_harm_c_cos_v(k)*cos(n*theta_mag);
end
%遍历要加入的谐波阶次，比如当前是 2、3、5、6 次

hall_sin_v = hall_sin_fund_v + hall_sin_harm_v;
hall_cos_v = hall_cos_fund_v + hall_cos_harm_v;
%最终 Hall 电压等于：基波 + 谐波

%% Basic offset, amplitude and phase calibration from sampled data
calib_idx = t < hall_t_stop_s;%选择用于标定的数据段。当前基本使用全程数据

hall_offset_s_est_v = 0.5*(max(hall_sin_v(calib_idx)) + min(hall_sin_v(calib_idx)));
hall_offset_c_est_v = 0.5*(max(hall_cos_v(calib_idx)) + min(hall_cos_v(calib_idx)));
%用最大值和最小值的平均值估计零偏：
hall_amp_s_est_v = 0.5*(max(hall_sin_v(calib_idx)) - min(hall_sin_v(calib_idx)));
hall_amp_c_est_v = 0.5*(max(hall_cos_v(calib_idx)) - min(hall_cos_v(calib_idx)));
%用最大值和最小值的差值一半估计幅值

if hall_amp_s_est_v <= eps || hall_amp_c_est_v <= eps
    error('Hall calibration failed: estimated amplitude is too small.');
end
%防止幅值太小导致后面归一化除以 0

hall_sin_norm_raw = (hall_sin_v - hall_offset_v)/hall_amp_v;
hall_cos_norm_raw = (hall_cos_v - hall_offset_v)/hall_amp_v;
%未补偿对比：只用理想零偏和名义幅值归一化。
%这会保留实际零偏误差、幅值误差和相位误差

hall_sin_norm_basic = (hall_sin_v - hall_offset_s_est_v)/hall_amp_s_est_v;
hall_cos_norm_basic = (hall_cos_v - hall_offset_c_est_v)/hall_amp_c_est_v;
%基础补偿：用估计出来的零偏和幅值进行归一化。

hall_sin_calib = hall_sin_norm_basic(calib_idx);
hall_cos_calib = hall_cos_norm_basic(calib_idx);
%取出标定段的归一化数据
hall_sin_calib_zm = hall_sin_calib - mean(hall_sin_calib);
hall_cos_calib_zm = hall_cos_calib - mean(hall_cos_calib);
%减去平均值，得到零均值信号。
%这样做是为了估计两路信号的相关性，而不受残余直流偏置影响
hall_phase_corr_coef = mean(hall_sin_calib_zm.*hall_cos_calib_zm) / ...
    sqrt(mean(hall_sin_calib_zm.^2)*mean(hall_cos_calib_zm.^2));
    %计算两路信号的归一化相关系数。
%理想正交时相关系数接近 0；不正交时会出现非零相关
hall_phase_corr_coef = max(-1, min(1, hall_phase_corr_coef));%把相关系数限制在 [-1, 1]，避免数值误差导致 asin 出错
hall_phase_err_est_rad = asin(-hall_phase_corr_coef);%由相关系数估计相位误差

if abs(cos(hall_phase_err_est_rad)) < 1e-6
    error('Estimated phase compensation failed: cosine of phase error is too small.');
end
%相位补偿公式中要除以 cos(Δφ)，这里防止分母太小

hall_cos_norm_phasecorr = ...
    (hall_cos_norm_basic + hall_sin_norm_basic*sin(hall_phase_err_est_rad))/ ...
    cos(hall_phase_err_est_rad);
%相位补偿

theta_mag_raw = align_angle(unwrap(atan2(hall_sin_norm_raw, hall_cos_norm_raw)), theta_mag);%用未补偿信号解磁场角
theta_mag_basic = align_angle(unwrap(atan2(hall_sin_norm_basic, hall_cos_norm_phasecorr)), theta_mag);%用零偏、幅值、相位补偿后的信号解磁场角


theta_m_err_raw = wrap_pi(theta_mag_raw - theta_mag)/hall_pole_pairs;%计算机械角误差
theta_m_err_basic = wrap_pi(theta_mag_basic - theta_mag)/hall_pole_pairs;%先得到磁场角误差，再除以极对数，转成机械角误差


%% Signal-domain harmonic reconstruction compensation
% This part compensates the normalized Hall pair itself, so the Lissajous
% curve becomes closer to a circle before atan2.
Phi_sig_calib = harmonic_matrix(theta_mag(calib_idx), hall_signal_comp_orders);%用真实磁场角生成标定段谐波基函数矩阵
Phi_sig_all = harmonic_matrix(theta_mag_basic, hall_signal_comp_orders);%用解算出来的角度生成全时段谐波基函数矩阵
%这更接近真实工程，因为实机中通常没有真实角，只能用估计角

res_s_calib = hall_sin_norm_basic(calib_idx) - sin(theta_mag(calib_idx));%计算两路信号相对理想正余弦的残差
res_c_calib = hall_cos_norm_phasecorr(calib_idx) - cos(theta_mag(calib_idx));%这些残差主要就是谐波畸变

coef_sig_s = Phi_sig_calib \ res_s_calib;
coef_sig_c = Phi_sig_calib \ res_c_calib;%用最小二乘拟合谐波残差系数

hall_sin_harm_est_norm = Phi_sig_all*coef_sig_s;
hall_cos_harm_est_norm = Phi_sig_all*coef_sig_c;%用拟合出的系数重构两路 Hall 的谐波残差

hall_sin_norm_sigcomp = hall_sin_norm_basic - hall_sin_harm_est_norm;
hall_cos_norm_sigcomp = hall_cos_norm_phasecorr - hall_cos_harm_est_norm;%从 Hall 信号中扣除估计出的谐波残差，这就是信号域谐波补偿

theta_mag_sigcomp = align_angle(unwrap(atan2(hall_sin_norm_sigcomp, hall_cos_norm_sigcomp)), theta_mag);
theta_m_err_sigcomp = wrap_pi(theta_mag_sigcomp - theta_mag)/hall_pole_pairs;%用信号域补偿后的两路 Hall 信号重新解角，并计算误差


%% FFT identification and angle-domain Fourier compensation
theta_m_err_basic_deg = rad2deg(theta_m_err_basic);%把基础补偿后的机械角误差从弧度转为角度，便于观察
[fft_order_amp_deg, fft_freq_hz] = order_fft_amplitude( ...
    theta_m_err_basic_deg(calib_idx), hall_ts_sim_s, hall_f_mag_hz, hall_angle_fft_orders);
%对基础补偿后的角度误差做 FFT，提取指定阶次的幅值和频率

Phi_ang_calib = harmonic_matrix(theta_mag_basic(calib_idx), hall_angle_comp_orders);
Phi_ang_all = harmonic_matrix(theta_mag_basic, hall_angle_comp_orders);%构造角度误差的傅里叶基函数矩阵，这里用的是 theta_mag_basic，也就是解算出来的角度

angle_err_mag_basic = wrap_pi(theta_mag_basic - theta_mag);%计算基础补偿后的磁场角误差
coef_ang_err = Phi_ang_calib \ angle_err_mag_basic(calib_idx);%用最小二乘拟合角度误差模型
angle_err_mag_hat = Phi_ang_all*coef_ang_err;%重构全时段角度误差

theta_mag_anglecomp = align_angle(theta_mag_basic - angle_err_mag_hat, theta_mag);%从原始解角结果中减去估计误差，这就是角度域傅里叶补偿
theta_m_err_anglecomp = wrap_pi(theta_mag_anglecomp - theta_mag)/hall_pole_pairs;%计算角度域补偿后的机械角误差

%% Metrics and CSV output
metrics = [
    max(abs(rad2deg(theta_m_err_raw))),       rms_local(rad2deg(theta_m_err_raw));
    max(abs(rad2deg(theta_m_err_basic))),     rms_local(rad2deg(theta_m_err_basic));
    max(abs(rad2deg(theta_m_err_sigcomp))),   rms_local(rad2deg(theta_m_err_sigcomp));
    max(abs(rad2deg(theta_m_err_anglecomp))), rms_local(rad2deg(theta_m_err_anglecomp))
];
%生成误差指标表。
%四行分别是：未补偿、基础补偿、信号域谐波补偿、角度域傅里叶补偿。
%两列分别是：最大误差、RMS 误差

fprintf('\nDual-Hall harmonic compensation validation\n');
fprintf('Injected Hall signal harmonic orders: %s\n', mat2str(hall_harm_orders));
fprintf('Estimated phase error = %.3f deg, actual = %.3f deg\n', ...
    rad2deg(hall_phase_err_est_rad), hall_phase_err_deg);
fprintf('\nMechanical angle error metrics:\n');
fprintf('  raw nominal compensation       max %.4f deg, rms %.4f deg\n', metrics(1,1), metrics(1,2));
fprintf('  offset/amplitude/phase comp    max %.4f deg, rms %.4f deg\n', metrics(2,1), metrics(2,2));
fprintf('  signal harmonic reconstruction max %.4f deg, rms %.4f deg\n', metrics(3,1), metrics(3,2));
fprintf('  angle Fourier compensation     max %.4f deg, rms %.4f deg\n\n', metrics(4,1), metrics(4,2));

orders = hall_angle_fft_orders(:);
fftTable = table(orders, fft_freq_hz(:), fft_order_amp_deg(:), ...
    ismember(orders, hall_angle_comp_orders(:)), ismember(orders, hall_harm_orders(:)), ...
    'VariableNames', {'angle_error_order', 'fft_frequency_hz', 'amplitude_deg', ...
    'used_for_angle_compensation', 'injected_signal_harmonic_order'});
csvPath = fullfile(figDir, 'dual_hall_harmonic_fft_orders.csv');
writetable(fftTable, csvPath);
%建立 FFT 结果表。
%内容包括：阶次、频率、幅值、是否用于角度补偿、是否为注入的 Hall 信号谐波

%% Plot
fig = figure('Name', 'Dual Hall Harmonic FFT Compensation Validation', ...
    'Color', 'w', 'Position', [60 60 1200 700]);
tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

time_plot_idx = t <= min(hall_fig3_time_window_s, hall_t_stop_s);
[rawSignalFftFreqHz, rawSignalFftDb] = single_sided_fft_db( ...
    hall_sin_norm_basic(calib_idx), hall_ts_sim_s);
[compSignalFftFreqHz, compSignalFftDb] = single_sided_fft_db( ...
    hall_sin_norm_sigcomp(calib_idx), hall_ts_sim_s);
hall_sin_v_basic = hall_offset_v + hall_amp_v*hall_sin_norm_basic;
hall_cos_v_basic = hall_offset_v + hall_amp_v*hall_cos_norm_phasecorr;
hall_sin_v_sigcomp = hall_offset_v + hall_amp_v*hall_sin_norm_sigcomp;
hall_cos_v_sigcomp = hall_offset_v + hall_amp_v*hall_cos_norm_sigcomp;

nexttile
plot(t(time_plot_idx), hall_sin_v(time_plot_idx), 'r', 'LineWidth', 1.2)
hold on
plot(t(time_plot_idx), hall_cos_v(time_plot_idx), 'Color', [0.45 0.20 0.75], 'LineWidth', 1.2)
plot(t(time_plot_idx), hall_sin_v_basic(time_plot_idx), '--', 'Color', [0.95 0.45 0.10], 'LineWidth', 1.0)
plot(t(time_plot_idx), hall_cos_v_basic(time_plot_idx), '--', 'Color', [0.10 0.55 0.95], 'LineWidth', 1.0)
plot(t(time_plot_idx), hall_sin_v_sigcomp(time_plot_idx), 'k', 'LineWidth', 1.1)
plot(t(time_plot_idx), hall_cos_v_sigcomp(time_plot_idx), 'Color', [0.00 0.55 0.20], 'LineWidth', 1.1)
grid on
xlabel('Time / s')
ylabel('Voltage / V')
title('(a) Hall voltage before and after compensation')
legend('Raw H_s', 'Raw H_c', 'Basic-comp H_s', 'Basic-comp H_c', ...
    'Harmonic-comp H_s', 'Harmonic-comp H_c', ...
    'Location', 'southoutside', 'NumColumns', 2)
text(0.07, max(hall_sin_v(time_plot_idx))*0.95, 'Unequal amplitude', ...
    'FontWeight', 'bold', 'BackgroundColor', 'w', 'EdgeColor', [0.4 0.4 0.4])

nexttile
plot(rawSignalFftFreqHz, rawSignalFftDb, 'Color', [0.45 0.20 0.75], 'LineWidth', 1.1)
hold on
plot(compSignalFftFreqHz, compSignalFftDb, 'k', 'LineWidth', 1.0)
grid on
xlabel('Frequency / Hz')
ylabel('Amplitude / dB')
title('(b) FFT of raw and harmonic-compensated Hall signal')
legend('FFT of raw normalized H_s', 'FFT after harmonic compensation', 'Location', 'northeast')
xlim([0 max(22, 1.2*max(hall_harm_orders)*hall_f_mag_hz)])
ylim([-90 5])
fftLabelOrders = unique([1 hall_harm_orders(:).']);
for i = 1:numel(fftLabelOrders)
    n = fftLabelOrders(i);
    targetFreq = n*hall_f_mag_hz;
    [~, fftIdx] = min(abs(rawSignalFftFreqHz - targetFreq));
    plot(rawSignalFftFreqHz(fftIdx), rawSignalFftDb(fftIdx), 'ko', ...
        'MarkerFaceColor', 'w', 'MarkerSize', 4, 'HandleVisibility', 'off')
    text(rawSignalFftFreqHz(fftIdx), rawSignalFftDb(fftIdx) + 5, ...
        order_label(n), 'FontSize', 9, 'HorizontalAlignment', 'center')
end

nexttile
plot(hall_cos_norm_raw, hall_sin_norm_raw, 'Color', [0.75 0.75 0.75], 'LineWidth', 0.9)
hold on
plot(hall_cos_norm_phasecorr, hall_sin_norm_basic, 'b', 'LineWidth', 1.0)
plot(hall_cos_norm_sigcomp, hall_sin_norm_sigcomp, 'k', 'LineWidth', 1.2)
axis equal
grid on
xlabel('normalized H_c')
ylabel('normalized H_s')
title('(c) Lissajous: harmonic compensation returns the signal pair toward circle')
legend('raw', 'offset/amplitude/phase compensation', ...
    'signal harmonic compensation', 'Location', 'southoutside', 'NumColumns', 1)


nexttile
yyaxis left
plot(t(time_plot_idx), wrap_pi(theta_mag_basic(time_plot_idx)), 'k', 'LineWidth', 1.1)
ylabel('Position / rad')
ylim([-pi pi])
yyaxis right
plot(t(time_plot_idx), theta_m_err_basic(time_plot_idx), 'r', 'LineWidth', 1.0)
hold on
plot(t(time_plot_idx), theta_m_err_anglecomp(time_plot_idx), 'b', 'LineWidth', 1.0)
ylabel('Position error / rad')
axD = gca;
axD.YAxis(1).Color = 'k';
axD.YAxis(2).Color = 'r';
grid on
xlabel('Time / s')
title('(d) Raw position and position error')
legend('Raw position', 'Position error before harmonic comp', ...
    'Position error after harmonic comp', 'Location', 'southoutside', 'NumColumns', 1)

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
%找到所有坐标轴，并关闭工具栏，避免导出图片时出现 MATLAB 工具按钮

pngPath = fullfile(figDir, 'dual_hall_harmonic_fft_compensation.png');
exportgraphics(fig, pngPath, 'Resolution', 220);

fprintf('Saved harmonic validation figure:\n%s\n', pngPath);
fprintf('Saved FFT order table:\n%s\n', csvPath);
%保存图片

%% Local helper functions
function Phi = harmonic_matrix(theta, orders)%生成谐波基函数矩阵
theta = theta(:);
Phi = ones(numel(theta), 1);
%把角度转成列向量，并加入常数项
for i = 1:numel(orders)
    n = orders(i);
    Phi = [Phi, sin(n*theta), cos(n*theta)]; %#ok<AGROW>
end
%对每个阶次加入 sin(nθ) 和 cos(nθ) 两列
end

function y = wrap_pi(x)%把角度限制到 [-π, π]
y = mod(x + pi, 2*pi) - pi;
end

function theta_aligned = align_angle(theta_est, theta_ref)%消除估计角和真实角之间的整圈偏移
offset = round((theta_est(1) - theta_ref(1))/(2*pi))*2*pi;
theta_aligned = theta_est - offset;
end

function value = rms_local(x)%计算 RMS 误差
x = x(:);
value = sqrt(mean(x.^2));
end

function [ampAtOrder, freqAtOrder] = order_fft_amplitude(errDeg, ts, fMag, orders)%计算角度误差在指定阶次上的 FFT 幅值
errDeg = errDeg(:) - mean(errDeg(:));
N = numel(errDeg);
Y = fft(errDeg);
%去掉直流分量，计算 FFT
P2 = abs(Y/N);
P1 = P2(1:floor(N/2)+1);
P1(2:end-1) = 2*P1(2:end-1);
%从双边频谱转换成单边频谱
freq = (0:floor(N/2)).'/(N*ts);
%生成频率轴

ampAtOrder = zeros(size(orders));
freqAtOrder = zeros(size(orders));
for i = 1:numel(orders)
    targetFreq = orders(i)*fMag;
    [~, idx] = min(abs(freq - targetFreq));
    ampAtOrder(i) = P1(idx);
    freqAtOrder(i) = freq(idx);
        %找到每个阶次对应频率处的 FFT 幅值
end
end

function [freq, ampDb] = single_sided_fft_db(signal, ts)%计算信号的单边 FFT，并转换成 dB
signal = signal(:) - mean(signal(:));
N = numel(signal);
Y = fft(signal);
P2 = abs(Y/N);
P1 = P2(1:floor(N/2)+1);
P1(2:end-1) = 2*P1(2:end-1);
freq = (0:floor(N/2)).'/(N*ts);
ampDb = 20*log10(P1/(max(P1) + eps) + eps);%把频谱幅值归一化到最大值，并转换为 dB
end

function txt = order_label(n)%根据阶次生成 1st、2nd、3rd、5th 这类文字标签
switch n
    case 1
        txt = '1st';
    case 2
        txt = '2nd';
    case 3
        txt = '3rd';
    otherwise
        txt = sprintf('%dth', n);
end
end
