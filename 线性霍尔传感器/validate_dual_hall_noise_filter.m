%% 双线性霍尔 ADC/噪声/低通滤波/alpha-beta 滤波对比验证
% 本脚本基于当前谐波补偿基准模型，继续向真实采样系统靠近：
%   1. 生成含零偏、幅值不一致、相位不正交和 2/3/5/6 次谐波的 Hall 信号；
%   2. 加入 ADC 量化和白噪声；
%   3. 对比两种噪声处理方式：
%      - 一阶低通：对采样后的 Hall 电压滤波，再进入补偿链路；
%      - alpha-beta：先完成确定性补偿和 atan2 解角，再对角度做状态估计。
%
% 重点观察：低通滤波会压制电压噪声，但可能引入相位滞后；
% alpha-beta 滤波利用角度-速度运动模型，通常比直接低通电压更适合云台角度估计。

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

%% 生成无噪声基准 Hall 信号
t = (0:hall_ts_sim_s:hall_t_stop_s).';
theta_m = hall_omega_mech_rad_s*t;
theta_mag = hall_pole_pairs*theta_m + hall_theta0_rad;
calib_idx = t < hall_t_stop_s;
plot_idx = t <= min(hall_noise_fig_window_s, hall_t_stop_s);

[hall_s_clean_v, hall_c_clean_v] = generate_clean_hall_signal(theta_mag, P);

%% 加入 ADC 量化和白噪声
rng(hall_noise_seed);
hall_s_noise_v = hall_noise_rms_v*randn(size(hall_s_clean_v));
hall_c_noise_v = hall_noise_rms_v*randn(size(hall_c_clean_v));

hall_s_adc_v = adc_quantize(hall_s_clean_v, P);
hall_c_adc_v = adc_quantize(hall_c_clean_v, P);

hall_s_noisy_adc_v = adc_quantize(hall_s_clean_v + hall_s_noise_v, P);
hall_c_noisy_adc_v = adc_quantize(hall_c_clean_v + hall_c_noise_v, P);

% 一阶低通作用在 Hall 电压上，是“补偿前”的信号调理。
hall_s_filt_v = first_order_lpf(hall_s_noisy_adc_v, hall_lpf_alpha);
hall_c_filt_v = first_order_lpf(hall_c_noisy_adc_v, hall_lpf_alpha);%调用lpffunc

%% 用无噪声数据建立固定离线补偿系数
cal = estimate_baseline_calibration(hall_s_clean_v, hall_c_clean_v, theta_mag, calib_idx, P);

%% 对不同采样链路执行同一套确定性补偿
result_clean = apply_compensation_chain(hall_s_clean_v, hall_c_clean_v, theta_mag, cal, P);
result_adc = apply_compensation_chain(hall_s_adc_v, hall_c_adc_v, theta_mag, cal, P);
result_noisy = apply_compensation_chain(hall_s_noisy_adc_v, hall_c_noisy_adc_v, theta_mag, cal, P);
result_lpf = apply_compensation_chain(hall_s_filt_v, hall_c_filt_v, theta_mag, cal, P);

%% alpha-beta 角度估计
% alpha-beta 不滤 Hall 电压，而是对补偿后的连续角度进行“角度-速度”状态估计。
[theta_ab_mag, omega_ab_mag] = alpha_beta_filter( ...
    result_noisy.theta_anglecomp, hall_ts_sim_s, hall_ab_alpha, hall_ab_beta);%调用αβfunc
theta_ab_mag = align_angle(theta_ab_mag, theta_mag);%对齐角度展开后的初始周期
err_ab = wrap_pi(theta_ab_mag - theta_mag)/P.pole_pairs;%把误差限制到 [−π,π]
omega_ab_mech = omega_ab_mag/P.pole_pairs;%从磁场角误差转换为机械角误差

%% ESO bandwidth tuning over steady and dynamic conditions
% The selected bandwidth is used by the steady-state and dynamic plots below.
dyn_t_stop_s = 2.0;
dyn_t = (0:hall_ts_sim_s:dyn_t_stop_s).';
dyn_plot_idx = dyn_t <= 1.2;
dyn_metric_idx = dyn_t >= 0.05;

dyn_angle_lpf_cutoff_hz = 40;
dyn_angle_lpf_tau_s = 1/(2*pi*dyn_angle_lpf_cutoff_hz);
dyn_angle_lpf_alpha = exp(-hall_ts_sim_s/dyn_angle_lpf_tau_s);

dynScenarioIds = ["variable_speed"; "impact"; "load_disturbance"];
dynScenarioLabels = ["Variable speed"; "Impact vibration"; "Load disturbance"];
dynMethodLabels = ["Raw"; "Angle LPF"; "Alpha-beta"; "Discrete ESO"];
dynLineColors = [
    0.80 0.10 0.10
    0.10 0.25 0.90
    0.00 0.55 0.20
    0.55 0.00 0.75
];

esoBandwidthCandidatesHz = [20 30 40 50 60 70 80 100 120 150];
[hall_eso_bandwidth_hz, esoSweepTable] = tune_eso_bandwidth( ...
    esoBandwidthCandidatesHz, hall_ts_sim_s, ...
    result_noisy.theta_anglecomp, theta_mag, calib_idx, ...
    dyn_t, dyn_metric_idx, dynScenarioIds, P.pole_pairs, hall_theta0_rad, hall_noise_seed);
[hall_eso_beta1, hall_eso_beta2, hall_eso_beta3, hall_eso_omega_o_rad_s] = ...
    eso_gains_from_bandwidth(hall_eso_bandwidth_hz);

esoSweepCsvPath = fullfile(figDir, 'dual_hall_eso_bandwidth_sweep.csv');
writetable(esoSweepTable, esoSweepCsvPath);
prepend_utf8_bom(esoSweepCsvPath);

esoSweepFig = figure('Name', 'Dual Hall ESO Bandwidth Sweep', ...
    'Color', 'w', 'Position', [150 150 980 420]);
tiledlayout(esoSweepFig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile
plot(esoSweepTable.bandwidth_hz, esoSweepTable.steady_rms_deg, '-o', 'LineWidth', 1.1)
hold on
plot(esoSweepTable.bandwidth_hz, esoSweepTable.dynamic_mean_rms_deg, '-s', 'LineWidth', 1.1)
xline(hall_eso_bandwidth_hz, '--k', 'LineWidth', 1.0)
grid on
xlabel('ESO bandwidth / Hz')
ylabel('RMS angle error / deg')
title('(a) RMS error versus ESO bandwidth')
legend(["steady noise", "dynamic mean", "selected"], 'Location', 'best')

nexttile
plot(esoSweepTable.bandwidth_hz, esoSweepTable.score_deg, '-o', 'LineWidth', 1.1)
hold on
xline(hall_eso_bandwidth_hz, '--k', 'LineWidth', 1.0)
grid on
xlabel('ESO bandwidth / Hz')
ylabel('composite score / deg')
title('(b) composite score for bandwidth selection')
legend(["score", "selected"], 'Location', 'best')

axs = findall(esoSweepFig, 'Type', 'Axes');
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

esoSweepPngPath = fullfile(figDir, 'dual_hall_eso_bandwidth_sweep.png');
exportgraphics(esoSweepFig, esoSweepPngPath, 'Resolution', 220);

%% discrete ESO angle estimator after Hall angle decoding
% ESO estimates z1=angle, z2=angular speed and z3=total disturbance.
[theta_eso_mag, omega_eso_mag, disturbance_eso_mag] = discrete_eso_filter( ...
    result_noisy.theta_anglecomp, hall_ts_sim_s, ...
    hall_eso_beta1, hall_eso_beta2, hall_eso_beta3);
theta_eso_mag = align_angle(theta_eso_mag, theta_mag);
err_eso = wrap_pi(theta_eso_mag - theta_mag)/P.pole_pairs;
omega_eso_mech = omega_eso_mag/P.pole_pairs;
disturbance_eso_mech = disturbance_eso_mag/P.pole_pairs;

%% 误差指标
scenarioNames = [
    "clean_baseline";
    "adc_quantization";
    "adc_plus_noise";
    "adc_noise_voltage_lpf";
    "adc_noise_alpha_beta";
    "adc_noise_discrete_eso"
];
scenarioNamesShort = [
    "Clean";
    "ADC";
    "ADC+noise";
    "Voltage LPF";
    "Alpha-beta";
    "Discrete ESO"
];

finalErrors = {
    result_clean.err_anglecomp;
    result_adc.err_anglecomp;
    result_noisy.err_anglecomp;
    result_lpf.err_anglecomp;
    err_ab;
    err_eso
};

metrics = zeros(numel(finalErrors), 2);
for k = 1:numel(finalErrors)
    err_deg = rad2deg(finalErrors{k});
    metrics(k,:) = [max(abs(err_deg)), rms_local(err_deg)];
end

metricsTable = table(scenarioNames, scenarioNamesShort, metrics(:,1), metrics(:,2), ...
    'VariableNames', {'scenario_id', 'scenario_label', 'final_max_deg', 'final_rms_deg'});

csvPath = fullfile(figDir, 'dual_hall_noise_lpf_ab_eso_metrics.csv');
writetable(metricsTable, csvPath);
prepend_utf8_bom(csvPath);

%% 绘图：重点比较 LPF 与 alpha-beta
fig = figure('Name', 'Dual Hall ADC Noise LPF Alpha-Beta Validation', ...
    'Color', 'w', 'Position', [70 70 1200 700]);
tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile
plot(t(plot_idx), hall_s_clean_v(plot_idx), 'k', 'LineWidth', 1.1)
hold on
plot(t(plot_idx), hall_s_noisy_adc_v(plot_idx), 'Color', [0.80 0.20 0.20], 'LineWidth', 0.8)
plot(t(plot_idx), hall_s_filt_v(plot_idx), 'b', 'LineWidth', 1.0)
grid on
xlabel('Time / s')
ylabel('Hall voltage / V')
title('(a) H_s voltage: clean, ADC+noise and voltage LPF')
legend('clean', 'ADC + noise', 'ADC + noise + LPF', ...
    'Location', 'southoutside', 'NumColumns', 3)

nexttile
plot(t(plot_idx), (hall_s_adc_v(plot_idx) - hall_s_clean_v(plot_idx))*1e3, ...
    'Color', [0.45 0.45 0.45], 'LineWidth', 0.9)
hold on
plot(t(plot_idx), (hall_s_noisy_adc_v(plot_idx) - hall_s_clean_v(plot_idx))*1e3, ...
    'r', 'LineWidth', 0.8)
plot(t(plot_idx), (hall_s_filt_v(plot_idx) - hall_s_clean_v(plot_idx))*1e3, ...
    'b', 'LineWidth', 0.9)
grid on
xlabel('Time / s')
ylabel('Voltage error / mV')
title('(b) Voltage-domain sampling error relative to clean H_s')
legend('ADC only', 'ADC + noise', 'after voltage LPF', ...
    'Location', 'southoutside', 'NumColumns', 3)

nexttile
plot(t(plot_idx), rad2deg(result_clean.err_anglecomp(plot_idx)), ...
    'k', 'LineWidth', 1.0)
hold on
plot(t(plot_idx), rad2deg(result_noisy.err_anglecomp(plot_idx)), ...
    'Color', [0.85 0.10 0.10], 'LineWidth', 0.8)
plot(t(plot_idx), rad2deg(result_lpf.err_anglecomp(plot_idx)), ...
    'b', 'LineWidth', 1.0)
plot(t(plot_idx), rad2deg(err_ab(plot_idx)), ...
    'Color', [0.00 0.55 0.20], 'LineWidth', 1.1)
plot(t(plot_idx), rad2deg(err_eso(plot_idx)), ...
    'Color', [0.55 0.00 0.75], 'LineWidth', 1.1)
grid on
xlabel('Time / s')
ylabel('Mechanical angle error / deg')
title('(c) Angle error: voltage LPF, alpha-beta and ESO')
legend('clean baseline', 'ADC + noise', 'voltage LPF', 'alpha-beta', 'discrete ESO', ...
    'Location', 'southoutside', 'NumColumns', 2)

nexttile
barData = [metrics(:,1), metrics(:,2)];
b = bar(barData);
b(1).FaceColor = [0.30 0.55 0.90];
b(2).FaceColor = [0.90 0.55 0.25];
grid on
set(gca, 'XTickLabel', scenarioNamesShort)
xtickangle(20)
ylabel('Mechanical angle error / deg')
title('(d) Final angle-estimation metrics')
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

pngPath = fullfile(figDir, 'dual_hall_noise_lpf_ab_eso_validation.png');
exportgraphics(fig, pngPath, 'Resolution', 220);

fprintf('\nDual-Hall ADC/noise/LPF/alpha-beta validation\n');
fprintf('ADC resolution: %d bit, LSB = %.4g V\n', hall_adc_bits, hall_adc_lsb_v);
fprintf('Noise RMS: %.4g V\n', hall_noise_rms_v);
fprintf('Voltage LPF cutoff: %.4g Hz\n', hall_lpf_cutoff_hz);
fprintf('Alpha-beta: alpha = %.4g, beta = %.4g\n', hall_ab_alpha, hall_ab_beta);
fprintf('Discrete ESO bandwidth: %.4g Hz\n', hall_eso_bandwidth_hz);
fprintf('\nFinal angle metrics:\n');
for k = 1:height(metricsTable)
    fprintf('  %-18s max %.4f deg, rms %.4f deg\n', ...
        metricsTable.scenario_label(k), metricsTable.final_max_deg(k), metricsTable.final_rms_deg(k));
end
fprintf('\nAlpha-beta final speed estimate mean: %.4f rad/s mechanical\n', mean(omega_ab_mech(calib_idx)));
fprintf('ESO final speed estimate mean: %.4f rad/s mechanical\n', mean(omega_eso_mech(calib_idx)));
fprintf('ESO disturbance estimate RMS: %.4f rad/s^2 mechanical\n', rms_local(disturbance_eso_mech(calib_idx)));
fprintf('Saved validation figure:\n%s\n', pngPath);
fprintf('Saved metrics:\n%s\n', csvPath);

%% Dynamic-condition validation: variable speed, impact and load disturbance
% This section keeps the original ADC/noise/filter baseline above unchanged.
% It tests the angle-domain estimators after Hall compensation and atan2:
% raw decoded angle, angle LPF, alpha-beta estimator and discrete ESO.
dynMetrics = table('Size', [0 5], ...
    'VariableTypes', {'string', 'string', 'string', 'double', 'double'}, ...
    'VariableNames', {'scenario_id', 'scenario_label', 'method', 'max_error_deg', 'rms_error_deg'});
dynData = struct([]);

for s = 1:numel(dynScenarioIds)
    data = make_dynamic_scenario(dynScenarioIds(s), dyn_t, hall_pole_pairs, hall_theta0_rad, hall_noise_seed + s);

    thetaRaw = data.theta_meas;
    thetaLpf = first_order_lpf(thetaRaw, dyn_angle_lpf_alpha);

    [thetaAbDyn, omegaAbDyn] = alpha_beta_filter(thetaRaw, hall_ts_sim_s, hall_ab_alpha, hall_ab_beta);
    [thetaEsoDyn, omegaEsoDyn, disturbanceEsoDyn] = discrete_eso_filter( ...
        thetaRaw, hall_ts_sim_s, hall_eso_beta1, hall_eso_beta2, hall_eso_beta3);

    thetaLpf = align_angle(thetaLpf, data.theta_true);
    thetaAbDyn = align_angle(thetaAbDyn, data.theta_true);
    thetaEsoDyn = align_angle(thetaEsoDyn, data.theta_true);

    errRawDyn = wrap_pi(thetaRaw - data.theta_true);
    errLpfDyn = wrap_pi(thetaLpf - data.theta_true);
    errAbDyn = wrap_pi(thetaAbDyn - data.theta_true);
    errEsoDyn = wrap_pi(thetaEsoDyn - data.theta_true);

    dynErrors = {errRawDyn; errLpfDyn; errAbDyn; errEsoDyn};
    dynEstimates = {thetaRaw; thetaLpf; thetaAbDyn; thetaEsoDyn};

    for m = 1:numel(dynMethodLabels)
        errDeg = rad2deg(dynErrors{m}(dyn_metric_idx));
        newRow = table( ...
            dynScenarioIds(s), dynScenarioLabels(s), dynMethodLabels(m), ...
            max(abs(errDeg)), rms_local(errDeg), ...
            'VariableNames', {'scenario_id', 'scenario_label', 'method', 'max_error_deg', 'rms_error_deg'});
        dynMetrics = [dynMetrics; newRow]; %#ok<AGROW>
    end

    dynData(s).id = dynScenarioIds(s);
    dynData(s).label = dynScenarioLabels(s);
    dynData(s).theta_true = data.theta_true;
    dynData(s).omega_true = data.omega_true;
    dynData(s).theta_meas = thetaRaw;
    dynData(s).estimates = dynEstimates;
    dynData(s).errors = dynErrors;
    dynData(s).omega_ab = omegaAbDyn;
    dynData(s).omega_eso = omegaEsoDyn;
    dynData(s).disturbance_eso = disturbanceEsoDyn;
end

dynCsvPath = fullfile(figDir, 'dual_hall_dynamic_conditions_metrics.csv');
writetable(dynMetrics, dynCsvPath);
prepend_utf8_bom(dynCsvPath);

dynFig = figure('Name', 'Dual Hall Dynamic Conditions Validation', ...
    'Color', 'w', 'Position', [50 50 1350 850]);
tiledlayout(dynFig, 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

for s = 1:numel(dynData)
    data = dynData(s);

    nexttile
    plot(dyn_t(dyn_plot_idx), data.omega_true(dyn_plot_idx)*60/(2*pi), 'k', 'LineWidth', 1.2)
    grid on
    xlabel('Time / s')
    ylabel('Speed / rpm')
    title(sprintf('(%c1) %s speed profile', char('a' + s - 1), data.label))

    nexttile
    hold on
    for m = 1:numel(dynMethodLabels)
        plot(dyn_t(dyn_plot_idx), rad2deg(data.errors{m}(dyn_plot_idx)), ...
            'Color', dynLineColors(m,:), 'LineWidth', 0.9)
    end
    grid on
    xlabel('Time / s')
    ylabel('Angle error / deg')
    title(sprintf('(%c2) angle-estimation error', char('a' + s - 1)))
    if s == 1
        legend(dynMethodLabels, 'Location', 'southoutside', 'NumColumns', 4)
    end

    nexttile
    plot(dyn_t(dyn_plot_idx), data.disturbance_eso(dyn_plot_idx), ...
        'Color', [0.55 0.00 0.75], 'LineWidth', 1.0)
    grid on
    xlabel('Time / s')
    ylabel('z_3 / rad*s^{-2}')
    title(sprintf('(%c3) ESO disturbance estimate', char('a' + s - 1)))
end

axs = findall(dynFig, 'Type', 'Axes');
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

dynPngPath = fullfile(figDir, 'dual_hall_dynamic_conditions_validation.png');
exportgraphics(dynFig, dynPngPath, 'Resolution', 220);

dynMetricFig = figure('Name', 'Dual Hall Dynamic Conditions Metrics', ...
    'Color', 'w', 'Position', [120 120 1050 520]);
tiledlayout(dynMetricFig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

dynMaxMat = zeros(numel(dynScenarioLabels), numel(dynMethodLabels));
dynRmsMat = zeros(numel(dynScenarioLabels), numel(dynMethodLabels));
for s = 1:numel(dynScenarioLabels)
    for m = 1:numel(dynMethodLabels)
        row = dynMetrics.scenario_label == dynScenarioLabels(s) & dynMetrics.method == dynMethodLabels(m);
        dynMaxMat(s,m) = dynMetrics.max_error_deg(row);
        dynRmsMat(s,m) = dynMetrics.rms_error_deg(row);
    end
end

nexttile
bar(dynMaxMat)
grid on
set(gca, 'XTickLabel', dynScenarioLabels)
xtickangle(15)
ylabel('Max angle error / deg')
title('(a) Maximum error under dynamic conditions')
legend(dynMethodLabels, 'Location', 'northoutside', 'NumColumns', 4)

nexttile
bar(dynRmsMat)
grid on
set(gca, 'XTickLabel', dynScenarioLabels)
xtickangle(15)
ylabel('RMS angle error / deg')
title('(b) RMS error under dynamic conditions')
legend(dynMethodLabels, 'Location', 'northoutside', 'NumColumns', 4)

dynMetricsPngPath = fullfile(figDir, 'dual_hall_dynamic_conditions_metrics.png');
exportgraphics(dynMetricFig, dynMetricsPngPath, 'Resolution', 220);

fprintf('\nDual-Hall dynamic-condition validation\n');
fprintf('Angle LPF cutoff: %.3g Hz\n', dyn_angle_lpf_cutoff_hz);
fprintf('Alpha-beta: alpha = %.4g, beta = %.4g\n', hall_ab_alpha, hall_ab_beta);
fprintf('Discrete ESO bandwidth: %.4g Hz\n', hall_eso_bandwidth_hz);
fprintf('\nDynamic metrics:\n');
for k = 1:height(dynMetrics)
    fprintf('  %-17s %-12s max %.4f deg, rms %.4f deg\n', ...
        dynMetrics.scenario_label(k), dynMetrics.method(k), ...
        dynMetrics.max_error_deg(k), dynMetrics.rms_error_deg(k));
end
fprintf('\nSaved dynamic figures:\n%s\n%s\n', dynPngPath, dynMetricsPngPath);
fprintf('Saved dynamic metrics:\n%s\n', dynCsvPath);

%% Hall + Gyro complementary/Kalman fusion validation
% Hall 给出补偿后的绝对/低频角度，Gyro 给出高频角速度。
% 互补滤波用 gyro 短时积分预测角度，再用 Hall 角度慢速拉回漂移；
% Kalman 滤波进一步把 gyro 零偏作为状态估计，适合后续 IMU 融合扩展。
fusionMethodLabels = [
    "Hall raw";
    "Alpha-beta";
    "Discrete ESO";
    "Gyro integration";
    "Complementary";
    "Kalman"
];
fusionLineColors = [
    0.80 0.10 0.10
    0.00 0.55 0.20
    0.55 0.00 0.75
    0.45 0.45 0.45
    0.10 0.25 0.90
    0.00 0.00 0.00
];

fusionMetrics = table('Size', [0 5], ...
    'VariableTypes', {'string', 'string', 'string', 'double', 'double'}, ...
    'VariableNames', {'scenario_id', 'scenario_label', 'method', 'max_error_deg', 'rms_error_deg'});
fusionData = struct([]);

for s = 1:numel(dynData)
    data = dynData(s);
    gyro = make_gyro_measurement( ...
        data.omega_true, dyn_t, ...
        hall_gyro_bias_rad_s, hall_gyro_bias_drift_rad_s2, ...
        hall_gyro_noise_rms_rad_s, hall_noise_seed + 100 + s);

    thetaGyro = integrate_gyro_angle(data.theta_meas(1), gyro, hall_ts_sim_s);
    thetaComp = complementary_hall_gyro_filter(data.theta_meas, gyro, hall_ts_sim_s, hall_comp_alpha);
    [thetaKalman, gyroBiasHat] = kalman_hall_gyro_filter( ...
        data.theta_meas, gyro, hall_ts_sim_s, ...
        hall_kalman_q_angle, hall_kalman_q_bias, hall_kalman_r_hall);

    thetaGyro = align_angle(thetaGyro, data.theta_true);
    thetaComp = align_angle(thetaComp, data.theta_true);
    thetaKalman = align_angle(thetaKalman, data.theta_true);

    fusionEstimates = {
        data.estimates{1};
        data.estimates{3};
        data.estimates{4};
        thetaGyro;
        thetaComp;
        thetaKalman
    };

    fusionErrors = cell(size(fusionEstimates));
    for m = 1:numel(fusionEstimates)
        fusionErrors{m} = wrap_pi(fusionEstimates{m} - data.theta_true);
        errDeg = rad2deg(fusionErrors{m}(dyn_metric_idx));
        newRow = table( ...
            data.id, data.label, fusionMethodLabels(m), ...
            max(abs(errDeg)), rms_local(errDeg), ...
            'VariableNames', {'scenario_id', 'scenario_label', 'method', 'max_error_deg', 'rms_error_deg'});
        fusionMetrics = [fusionMetrics; newRow]; %#ok<AGROW>
    end

    fusionData(s).id = data.id;
    fusionData(s).label = data.label;
    fusionData(s).theta_true = data.theta_true;
    fusionData(s).omega_true = data.omega_true;
    fusionData(s).gyro = gyro;
    fusionData(s).estimates = fusionEstimates;
    fusionData(s).errors = fusionErrors;
    fusionData(s).gyro_bias_hat = gyroBiasHat;
end

fusionCsvPath = fullfile(figDir, 'dual_hall_gyro_fusion_metrics.csv');
writetable(fusionMetrics, fusionCsvPath);
prepend_utf8_bom(fusionCsvPath);

fusionFig = figure('Name', 'Dual Hall Gyro Complementary Kalman Fusion Validation', ...
    'Color', 'w', 'Position', [40 40 1400 860]);
tiledlayout(fusionFig, 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

for s = 1:numel(fusionData)
    data = fusionData(s);

    nexttile
    plot(dyn_t(dyn_plot_idx), data.omega_true(dyn_plot_idx)*60/(2*pi), ...
        'k', 'LineWidth', 1.1)
    hold on
    plot(dyn_t(dyn_plot_idx), data.gyro(dyn_plot_idx)*60/(2*pi), ...
        'Color', [0.60 0.60 0.60], 'LineWidth', 0.7)
    grid on
    xlabel('Time / s')
    ylabel('Speed / rpm')
    title(sprintf('(%c1) %s true speed and gyro', char('a' + s - 1), data.label))
    if s == 1
        legend('true', 'gyro meas.', 'Location', 'southoutside', 'NumColumns', 2)
    end

    nexttile
    hold on
    for m = 1:numel(fusionMethodLabels)
        plot(dyn_t(dyn_plot_idx), rad2deg(data.errors{m}(dyn_plot_idx)), ...
            'Color', fusionLineColors(m,:), 'LineWidth', 0.9)
    end
    grid on
    xlabel('Time / s')
    ylabel('Angle error / deg')
    title(sprintf('(%c2) Hall/Gyro fusion angle error', char('a' + s - 1)))
    if s == 1
        legend(fusionMethodLabels, 'Location', 'southoutside', 'NumColumns', 3)
    end

    nexttile
    plot(dyn_t(dyn_plot_idx), rad2deg(data.gyro_bias_hat(dyn_plot_idx)), ...
        'Color', [0.00 0.00 0.00], 'LineWidth', 1.0)
    hold on
    plot(dyn_t(dyn_plot_idx), rad2deg( ...
        hall_gyro_bias_rad_s + hall_gyro_bias_drift_rad_s2*dyn_t(dyn_plot_idx)), ...
        '--', 'Color', [0.70 0.10 0.10], 'LineWidth', 1.0)
    grid on
    xlabel('Time / s')
    ylabel('Gyro bias / deg*s^{-1}')
    title(sprintf('(%c3) Kalman gyro-bias estimate', char('a' + s - 1)))
    if s == 1
        legend('estimated', 'injected', 'Location', 'southoutside', 'NumColumns', 2)
    end
end

axs = findall(fusionFig, 'Type', 'Axes');
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

fusionPngPath = fullfile(figDir, 'dual_hall_gyro_fusion_validation.png');
exportgraphics(fusionFig, fusionPngPath, 'Resolution', 220);

fusionMetricFig = figure('Name', 'Dual Hall Gyro Fusion Metrics', ...
    'Color', 'w', 'Position', [120 120 1150 560]);
tiledlayout(fusionMetricFig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

fusionMaxMat = zeros(numel(dynScenarioLabels), numel(fusionMethodLabels));
fusionRmsMat = zeros(numel(dynScenarioLabels), numel(fusionMethodLabels));
for s = 1:numel(dynScenarioLabels)
    for m = 1:numel(fusionMethodLabels)
        row = fusionMetrics.scenario_label == dynScenarioLabels(s) & ...
            fusionMetrics.method == fusionMethodLabels(m);
        fusionMaxMat(s,m) = fusionMetrics.max_error_deg(row);
        fusionRmsMat(s,m) = fusionMetrics.rms_error_deg(row);
    end
end

nexttile
bar(fusionMaxMat)
grid on
set(gca, 'XTickLabel', dynScenarioLabels)
xtickangle(15)
ylabel('Max angle error / deg')
title('(a) Maximum error: Hall-only versus Hall/Gyro fusion')
legend(fusionMethodLabels, 'Location', 'northoutside', 'NumColumns', 3)

nexttile
bar(fusionRmsMat)
grid on
set(gca, 'XTickLabel', dynScenarioLabels)
xtickangle(15)
ylabel('RMS angle error / deg')
title('(b) RMS error: Hall-only versus Hall/Gyro fusion')
legend(fusionMethodLabels, 'Location', 'northoutside', 'NumColumns', 3)

fusionMetricsPngPath = fullfile(figDir, 'dual_hall_gyro_fusion_metrics.png');
exportgraphics(fusionMetricFig, fusionMetricsPngPath, 'Resolution', 220);

fprintf('\nDual-Hall + Gyro fusion validation\n');
fprintf('Gyro noise RMS: %.4g rad/s, bias: %.4g rad/s, drift: %.4g rad/s^2\n', ...
    hall_gyro_noise_rms_rad_s, hall_gyro_bias_rad_s, hall_gyro_bias_drift_rad_s2);
fprintf('Complementary alpha: %.4g\n', hall_comp_alpha);
fprintf('Kalman Q angle: %.4g, Q bias: %.4g, R Hall: %.4g\n', ...
    hall_kalman_q_angle, hall_kalman_q_bias, hall_kalman_r_hall);
fprintf('\nHall/Gyro fusion metrics:\n');
for k = 1:height(fusionMetrics)
    fprintf('  %-17s %-16s max %.4f deg, rms %.4f deg\n', ...
        fusionMetrics.scenario_label(k), fusionMetrics.method(k), ...
        fusionMetrics.max_error_deg(k), fusionMetrics.rms_error_deg(k));
end
fprintf('\nSaved Hall/Gyro fusion figures:\n%s\n%s\n', fusionPngPath, fusionMetricsPngPath);
fprintf('Saved Hall/Gyro fusion metrics:\n%s\n', fusionCsvPath);

%% Low-speed sweep detection-accuracy validation
% 该段对应“检测精度约为 <= ±0.005 deg @1sigma”的准静态工况：
% 低速扫角、无冲击、无大加减速，统计角度误差的标准差 sigma。
low_t = (0:hall_low_speed_ts_s:hall_low_speed_t_stop_s).';
low_plot_idx = low_t <= min(5, hall_low_speed_t_stop_s);
low_metric_idx = low_t >= hall_low_speed_metric_start_s;

lowMethodLabels = fusionMethodLabels;
lowMetrics = table('Size', [0 7], ...
    'VariableTypes', {'double', 'string', 'double', 'double', 'double', 'double', 'logical'}, ...
    'VariableNames', {'sweep_rpm', 'method', 'mean_error_deg', 'sigma_1deg', ...
    'rms_error_deg', 'max_abs_error_deg', 'pass_1sigma'});
lowData = struct([]);

for s = 1:numel(hall_low_speed_sweep_rpm)
    sweepRpm = hall_low_speed_sweep_rpm(s);
    omegaTrue = sweepRpm*2*pi/60*ones(size(low_t));
    thetaTrue = cumtrapz(low_t, omegaTrue);
    thetaMagLow = P.pole_pairs*thetaTrue + hall_theta0_rad;

    [hall_s_low_clean_v, hall_c_low_clean_v] = generate_clean_hall_signal(thetaMagLow, P);
    rng(hall_noise_seed + 300 + s);
    hall_s_low_noisy_adc_v = adc_quantize( ...
        hall_s_low_clean_v + hall_noise_rms_v*randn(size(low_t)), P);
    hall_c_low_noisy_adc_v = adc_quantize( ...
        hall_c_low_clean_v + hall_noise_rms_v*randn(size(low_t)), P);

    resultLow = apply_compensation_chain( ...
        hall_s_low_noisy_adc_v, hall_c_low_noisy_adc_v, thetaMagLow, cal, P);
    thetaHall = align_angle((resultLow.theta_anglecomp - hall_theta0_rad)/P.pole_pairs, thetaTrue);

    [thetaAbLow, ~] = alpha_beta_filter(thetaHall, hall_low_speed_ts_s, hall_ab_alpha, hall_ab_beta);
    [thetaEsoLow, ~, ~] = discrete_eso_filter( ...
        thetaHall, hall_low_speed_ts_s, hall_eso_beta1, hall_eso_beta2, hall_eso_beta3);

    gyroLow = make_gyro_measurement( ...
        omegaTrue, low_t, ...
        hall_gyro_bias_rad_s, hall_gyro_bias_drift_rad_s2, ...
        hall_gyro_noise_rms_rad_s, hall_noise_seed + 400 + s);
    thetaGyroLow = integrate_gyro_angle(thetaHall(1), gyroLow, hall_low_speed_ts_s);
    thetaCompLow = complementary_hall_gyro_filter(thetaHall, gyroLow, hall_low_speed_ts_s, hall_comp_alpha);
    [thetaKalmanLow, gyroBiasHatLow] = kalman_hall_gyro_filter( ...
        thetaHall, gyroLow, hall_low_speed_ts_s, ...
        hall_kalman_q_angle, hall_kalman_q_bias, hall_kalman_r_hall);

    thetaAbLow = align_angle(thetaAbLow, thetaTrue);
    thetaEsoLow = align_angle(thetaEsoLow, thetaTrue);
    thetaGyroLow = align_angle(thetaGyroLow, thetaTrue);
    thetaCompLow = align_angle(thetaCompLow, thetaTrue);
    thetaKalmanLow = align_angle(thetaKalmanLow, thetaTrue);

    lowEstimates = {
        thetaHall;
        thetaAbLow;
        thetaEsoLow;
        thetaGyroLow;
        thetaCompLow;
        thetaKalmanLow
    };

    lowErrors = cell(size(lowEstimates));
    for m = 1:numel(lowMethodLabels)
        lowErrors{m} = wrap_pi(lowEstimates{m} - thetaTrue);
        errDeg = rad2deg(lowErrors{m}(low_metric_idx));
        sigmaDeg = std(errDeg);
        newRow = table( ...
            sweepRpm, lowMethodLabels(m), mean(errDeg), sigmaDeg, ...
            rms_local(errDeg), max(abs(errDeg)), sigmaDeg <= hall_detection_sigma_target_deg, ...
            'VariableNames', {'sweep_rpm', 'method', 'mean_error_deg', 'sigma_1deg', ...
            'rms_error_deg', 'max_abs_error_deg', 'pass_1sigma'});
        lowMetrics = [lowMetrics; newRow]; %#ok<AGROW>
    end

    lowData(s).rpm = sweepRpm;
    lowData(s).theta_true = thetaTrue;
    lowData(s).omega_true = omegaTrue;
    lowData(s).hall_s_clean_v = hall_s_low_clean_v;
    lowData(s).hall_s_noisy_adc_v = hall_s_low_noisy_adc_v;
    lowData(s).gyro = gyroLow;
    lowData(s).errors = lowErrors;
    lowData(s).gyro_bias_hat = gyroBiasHatLow;
end

lowCsvPath = fullfile(figDir, 'dual_hall_low_speed_detection_metrics.csv');
writetable(lowMetrics, lowCsvPath);
prepend_utf8_bom(lowCsvPath);

lowPlotCase = min(2, numel(lowData));
lowFig = figure('Name', 'Dual Hall Low-Speed Detection Accuracy Validation', ...
    'Color', 'w', 'Position', [60 60 1350 780]);
tiledlayout(lowFig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile
plot(low_t(low_plot_idx), lowData(lowPlotCase).hall_s_clean_v(low_plot_idx), ...
    'k', 'LineWidth', 1.0)
hold on
plot(low_t(low_plot_idx), lowData(lowPlotCase).hall_s_noisy_adc_v(low_plot_idx), ...
    'Color', [0.80 0.10 0.10], 'LineWidth', 0.75)
grid on
xlabel('Time / s')
ylabel('Hall voltage / V')
title(sprintf('(a) %.1f rpm low-speed Hall voltage', lowData(lowPlotCase).rpm))
legend('clean', 'ADC + noise', 'Location', 'southoutside', 'NumColumns', 2)

nexttile
hold on
for m = 1:numel(lowMethodLabels)
    plot(low_t(low_plot_idx), rad2deg(lowData(lowPlotCase).errors{m}(low_plot_idx)), ...
        'Color', fusionLineColors(m,:), 'LineWidth', 0.9)
end
yline(hall_detection_sigma_target_deg, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 0.9)
yline(-hall_detection_sigma_target_deg, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 0.9)
grid on
xlabel('Time / s')
ylabel('Angle error / deg')
title(sprintf('(b) %.1f rpm angle error, target band shown', lowData(lowPlotCase).rpm))
legend([lowMethodLabels; "±0.005 deg"], 'Location', 'southoutside', 'NumColumns', 3)

lowSigmaMat = zeros(numel(hall_low_speed_sweep_rpm), numel(lowMethodLabels));
lowMaxMat = zeros(numel(hall_low_speed_sweep_rpm), numel(lowMethodLabels));
for s = 1:numel(hall_low_speed_sweep_rpm)
    for m = 1:numel(lowMethodLabels)
        row = lowMetrics.sweep_rpm == hall_low_speed_sweep_rpm(s) & ...
            lowMetrics.method == lowMethodLabels(m);
        lowSigmaMat(s,m) = lowMetrics.sigma_1deg(row);
        lowMaxMat(s,m) = lowMetrics.max_abs_error_deg(row);
    end
end

nexttile
bar(lowSigmaMat)
hold on
yline(hall_detection_sigma_target_deg, '--r', 'LineWidth', 1.2)
grid on
set(gca, 'XTickLabel', string(hall_low_speed_sweep_rpm) + " rpm")
xtickangle(15)
ylabel('1\sigma / deg')
title('(c) Detection precision: standard deviation')
legend([lowMethodLabels; "target"], 'Location', 'northoutside', 'NumColumns', 3)

nexttile
bar(lowMaxMat)
grid on
set(gca, 'XTickLabel', string(hall_low_speed_sweep_rpm) + " rpm")
xtickangle(15)
ylabel('Max abs error / deg')
title('(d) Maximum absolute error during low-speed sweep')
legend(lowMethodLabels, 'Location', 'northoutside', 'NumColumns', 3)

axs = findall(lowFig, 'Type', 'Axes');
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

lowPngPath = fullfile(figDir, 'dual_hall_low_speed_detection_validation.png');
exportgraphics(lowFig, lowPngPath, 'Resolution', 220);

fprintf('\nDual-Hall low-speed detection-accuracy validation\n');
fprintf('Detection target: sigma <= %.4f deg @1sigma\n', hall_detection_sigma_target_deg);
fprintf('Sweep speeds: %s rpm\n', mat2str(hall_low_speed_sweep_rpm));
fprintf('\nLow-speed detection metrics:\n');
for k = 1:height(lowMetrics)
    passText = "NO";
    if lowMetrics.pass_1sigma(k)
        passText = "YES";
    end
    fprintf('  %-5.1f rpm %-16s mean %+8.4f deg, sigma %.4f deg, rms %.4f deg, max %.4f deg, pass %s\n', ...
        lowMetrics.sweep_rpm(k), lowMetrics.method(k), lowMetrics.mean_error_deg(k), ...
        lowMetrics.sigma_1deg(k), lowMetrics.rms_error_deg(k), lowMetrics.max_abs_error_deg(k), passText);
end
fprintf('\nSaved low-speed detection figure:\n%s\n', lowPngPath);
fprintf('Saved low-speed detection metrics:\n%s\n', lowCsvPath);

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
y = zeros(size(x));     %初始化输出数组，长度与输入一致
y(1) = x(1);            %第一拍没有历史滤波值，直接用当前输入作为初值
for idx = 2:numel(x)    %从第二个采样点开始递推
    y(idx) = alpha*y(idx-1) + (1 - alpha)*x(idx);
end
end

function [theta_hat, omega_hat] = alpha_beta_filter(theta_meas, Ts, alpha, beta)
theta_meas = theta_meas(:);
theta_hat = zeros(size(theta_meas));
omega_hat = zeros(size(theta_meas));

theta_hat(1) = theta_meas(1);%初始角度估计等于第一拍测量角度
if numel(theta_meas) >= 2
    omega_hat(1) = (theta_meas(2) - theta_meas(1))/Ts;%用前两拍角度差估计初始角速度
end

for idx = 2:numel(theta_meas)
    theta_pred = theta_hat(idx-1) + Ts*omega_hat(idx-1);%角度预测
    omega_pred = omega_hat(idx-1);%假设短时间角速度不变

    innovation = theta_meas(idx) - theta_pred;%当前测量和预测的差
    theta_hat(idx) = theta_pred + alpha*innovation;%用 α 修正角度
    omega_hat(idx) = omega_pred + beta/Ts*innovation;%用 β 修正角速度
end
end

function [bestBandwidthHz, sweepTable] = tune_eso_bandwidth( ...
    candidatesHz, Ts, steadyThetaMeas, steadyThetaTrue, steadyMetricIdx, ...
    dynT, dynMetricIdx, dynScenarioIds, polePairs, theta0, seed)
% Sweep ESO bandwidth and choose a balanced setting for steady and dynamic tests.
nCandidates = numel(candidatesHz);
steadyMaxDeg = zeros(nCandidates, 1);
steadyRmsDeg = zeros(nCandidates, 1);
dynamicMeanMaxDeg = zeros(nCandidates, 1);
dynamicMeanRmsDeg = zeros(nCandidates, 1);
dynamicWorstMaxDeg = zeros(nCandidates, 1);
scoreDeg = zeros(nCandidates, 1);

for idx = 1:nCandidates
    bandwidthHz = candidatesHz(idx);
    [beta1, beta2, beta3] = eso_gains_from_bandwidth(bandwidthHz);

    thetaEsoSteady = discrete_eso_filter(steadyThetaMeas, Ts, beta1, beta2, beta3);
    thetaEsoSteady = align_angle(thetaEsoSteady, steadyThetaTrue);
    errSteadyDeg = rad2deg(wrap_pi(thetaEsoSteady - steadyThetaTrue)/polePairs);
    errSteadyDeg = errSteadyDeg(steadyMetricIdx);
    steadyMaxDeg(idx) = max(abs(errSteadyDeg));
    steadyRmsDeg(idx) = rms_local(errSteadyDeg);

    dynMaxEach = zeros(numel(dynScenarioIds), 1);
    dynRmsEach = zeros(numel(dynScenarioIds), 1);
    for s = 1:numel(dynScenarioIds)
        data = make_dynamic_scenario(dynScenarioIds(s), dynT, polePairs, theta0, seed + s);
        thetaEsoDyn = discrete_eso_filter(data.theta_meas, Ts, beta1, beta2, beta3);
        thetaEsoDyn = align_angle(thetaEsoDyn, data.theta_true);
        errDynDeg = rad2deg(wrap_pi(thetaEsoDyn - data.theta_true));
        errDynDeg = errDynDeg(dynMetricIdx);
        dynMaxEach(s) = max(abs(errDynDeg));
        dynRmsEach(s) = rms_local(errDynDeg);
    end

    dynamicMeanMaxDeg(idx) = mean(dynMaxEach);
    dynamicMeanRmsDeg(idx) = mean(dynRmsEach);
    dynamicWorstMaxDeg(idx) = max(dynMaxEach);

    % RMS is the main target, while peak errors are softly constrained.
    steadyMaxGuardDeg = 0.42;
    dynamicWorstGuardDeg = 0.16;
    scoreDeg(idx) = 0.65*(steadyRmsDeg(idx) + dynamicMeanRmsDeg(idx)) + ...
        0.35*max(0, steadyMaxDeg(idx) - steadyMaxGuardDeg) + ...
        0.20*max(0, dynamicWorstMaxDeg(idx) - dynamicWorstGuardDeg);
end

sweepTable = table(candidatesHz(:), steadyMaxDeg, steadyRmsDeg, ...
    dynamicMeanMaxDeg, dynamicMeanRmsDeg, dynamicWorstMaxDeg, scoreDeg, ...
    'VariableNames', {'bandwidth_hz', 'steady_max_deg', 'steady_rms_deg', ...
    'dynamic_mean_max_deg', 'dynamic_mean_rms_deg', 'dynamic_worst_max_deg', 'score_deg'});

[~, bestIdx] = min(scoreDeg);
bestBandwidthHz = candidatesHz(bestIdx);
end

function [beta1, beta2, beta3, omegaO] = eso_gains_from_bandwidth(bandwidthHz)
omegaO = 2*pi*bandwidthHz;
beta1 = 3*omegaO;
beta2 = 3*omegaO^2;
beta3 = omegaO^3;
end

function [z1, z2, z3] = discrete_eso_filter(theta_meas, Ts, beta1, beta2, beta3)
theta_meas = theta_meas(:);
z1 = zeros(size(theta_meas));
z2 = zeros(size(theta_meas));
z3 = zeros(size(theta_meas));

z1(1) = theta_meas(1);
z2(1) = 0;
z3(1) = 0;

for idx = 2:numel(theta_meas)
    err = theta_meas(idx-1) - z1(idx-1);
    z1(idx) = z1(idx-1) + Ts*(z2(idx-1) + beta1*err);
    z2(idx) = z2(idx-1) + Ts*(z3(idx-1) + beta2*err);
    z3(idx) = z3(idx-1) + Ts*(beta3*err);
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

function data = make_dynamic_scenario(scenarioId, t, polePairs, theta0, seed)
Ts = t(2) - t(1);

switch scenarioId
    case "variable_speed"
        rpm0 = 30;
        rpm1 = 180;
        speedBlend = smooth_step(t, 0.25, 1.15);
        rpm = rpm0 + (rpm1 - rpm0)*speedBlend;
        rpm = rpm + 8*sin(2*pi*0.8*t).*speedBlend;
        omega = rpm*2*pi/60;
        theta = cumtrapz(t, omega);

    case "impact"
        rpm = 100*ones(size(t));
        omegaBase = rpm*2*pi/60;
        thetaBase = cumtrapz(t, omegaBase);
        tImpact = 0.75;
        active = t >= tImpact;
        impact = zeros(size(t));
        impact(active) = deg2rad(2.0)*exp(-14*(t(active)-tImpact)).* ...
            sin(2*pi*16*(t(active)-tImpact));
        theta = thetaBase + impact;
        omega = gradient(theta, Ts);

    case "load_disturbance"
        rpmBase = 120;
        tLoad = 0.70;
        loadShape = zeros(size(t));
        active = t >= tLoad;
        loadShape(active) = 1 - exp(-5*(t(active)-tLoad));
        rpm = rpmBase - 35*loadShape + 7*sin(2*pi*3.0*t).*loadShape;
        omega = rpm*2*pi/60;
        theta = cumtrapz(t, omega);

    otherwise
        error('Unknown dynamic scenario: %s', scenarioId);
end

thetaMag = polePairs*theta + theta0;
residual = deg2rad(0.08)*sin(2*thetaMag + 0.3) + ...
    deg2rad(0.04)*sin(5*thetaMag - 0.5);

rng(seed);
noise = deg2rad(0.06)*randn(size(t));
thetaMeas = theta + residual/polePairs + noise;

data.theta_true = theta;
data.omega_true = omega;
data.theta_meas = thetaMeas;
end

function omega_gyro = make_gyro_measurement(omega_true, t, bias, biasDrift, noiseRms, seed)
% 生成陀螺测量角速度：真实角速度 + 固定零偏 + 慢漂移 + 白噪声。
rng(seed);
omega_gyro = omega_true(:) + bias + biasDrift*(t(:) - t(1)) + noiseRms*randn(size(t(:)));
end

function theta_gyro = integrate_gyro_angle(theta0, omega_gyro, Ts)
% 只积分 gyro，短时间动态好，但只要存在零偏就会长期漂移。
omega_gyro = omega_gyro(:);
theta_gyro = zeros(size(omega_gyro));
theta_gyro(1) = theta0;
for idx = 2:numel(omega_gyro)
    theta_gyro(idx) = theta_gyro(idx-1) + Ts*omega_gyro(idx);
end
end

function theta_fused = complementary_hall_gyro_filter(theta_hall, omega_gyro, Ts, alpha)
% 互补滤波：gyro 积分负责短时预测，Hall 角度负责低频拉回漂移。
theta_hall = theta_hall(:);
omega_gyro = omega_gyro(:);
theta_fused = zeros(size(theta_hall));
theta_fused(1) = theta_hall(1);
for idx = 2:numel(theta_hall)
    theta_pred = theta_fused(idx-1) + Ts*omega_gyro(idx);
    theta_fused(idx) = alpha*theta_pred + (1 - alpha)*theta_hall(idx);
end
end

function [theta_hat, bias_hat] = kalman_hall_gyro_filter(theta_hall, omega_gyro, Ts, qTheta, qBias, rHall)
% 二状态 Hall/Gyro Kalman：状态为 [角度; gyro 零偏]。
% 预测：theta[k+1] = theta[k] + Ts*(omega_gyro - bias)
% 修正：用 Hall 解角测量 theta_hall 修正角度和零偏。
theta_hall = theta_hall(:);
omega_gyro = omega_gyro(:);

n = numel(theta_hall);
theta_hat = zeros(n, 1);
bias_hat = zeros(n, 1);

x = [theta_hall(1); 0];
Pcov = diag([rHall, deg2rad(0.5)^2]);
Q = diag([qTheta, qBias]);
H = [1 0];
R = rHall;

theta_hat(1) = x(1);
bias_hat(1) = x(2);

for idx = 2:n
    F = [1, -Ts; 0, 1];
    B = [Ts; 0];

    xPred = F*x + B*omega_gyro(idx);
    PPred = F*Pcov*F' + Q;

    innovation = theta_hall(idx) - H*xPred;
    S = H*PPred*H' + R;
    K = PPred*H'/S;

    x = xPred + K*innovation;
    Pcov = (eye(2) - K*H)*PPred;

    theta_hat(idx) = x(1);
    bias_hat(idx) = x(2);
end
end

function y = smooth_step(t, t0, t1)
u = (t - t0)/(t1 - t0);
u = min(max(u, 0), 1);
y = u.^2.*(3 - 2*u);
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
