function mdl = build_dual_hall_harmonic_baseline_model(forceRebuild)
%BUILD_DUAL_HALL_HARMONIC_BASELINE_MODEL 构建双线性霍尔谐波畸变基准模型。
%
% 这个模型用于固定当前阶段的仿真基准：
%   1. 生成含零偏、幅值不一致、相位不正交和 2/3/5/6 次谐波的 Hall 电压；
%   2. 输出基础补偿后的等效电压；
%   3. 输出已知谐波模型补偿后的等效电压；
%   4. 将关键变量写入 simout_dual_hall_harmonic_baseline。
%
% 说明：
%   - 该 Simulink 模型用于观察信号链路和固定基准波形；
%   - FFT 识别、最小二乘拟合和角度域傅里叶补偿仍在
%     validate_dual_hall_harmonic_fft_compensation.m 中完成；
%   - 默认不重建已有模型，避免覆盖手动调整过的布局。
%
% 用法：
%   build_dual_hall_harmonic_baseline_model       % 只加载并刷新参数
%   build_dual_hall_harmonic_baseline_model(true) % 强制重建模型

if nargin < 1
    forceRebuild = false;
end

scriptDir = fileparts(mfilename('fullpath'));
modelDir = fullfile(scriptDir, 'models');
if ~exist(modelDir, 'dir')
    mkdir(modelDir);
end

init_dual_hall_harmonic_params;

mdl = 'dual_hall_harmonic_baseline';
modelPath = fullfile(modelDir, [mdl '.slx']);

if exist(modelPath, 'file') == 2 && ~forceRebuild
    if ~bdIsLoaded(mdl)
        load_system(modelPath);
    end
    put_params_in_model_workspace(mdl);
    set_param(mdl, ...
        'StopTime', 'hall_t_stop_s', ...
        'SolverType', 'Fixed-step', ...
        'Solver', 'ode4', ...
        'FixedStep', 'hall_ts_sim_s', ...
        'SaveOutput', 'on');
    fprintf('Loaded existing harmonic baseline model, layout preserved:\n%s\n', modelPath);
    fprintf('Model workspace parameters refreshed in memory.\n');
    fprintf('The core MATLAB Function keeps the fixed baseline constants saved in the .slx.\n');
    fprintf('To update constants and rebuild blocks, run build_dual_hall_harmonic_baseline_model(true).\n');
    return
end

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

new_system(mdl);
put_params_in_model_workspace(mdl);

set_param(mdl, ...
    'StopTime', 'hall_t_stop_s', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'ode4', ...
    'FixedStep', 'hall_ts_sim_s', ...
    'SaveOutput', 'on');

%% 输入时间与核心计算模块
add_block('simulink/Sources/Clock', [mdl '/time_t'], ...
    'Position', [60 175 90 205]);

coreBlock = [mdl '/dual_hall_harmonic_core'];
add_block('simulink/User-Defined Functions/MATLAB Function', coreBlock, ...
    'Position', [165 85 455 295]);
set_matlab_function_script(coreBlock);

%% 输出记录与示波显示
add_block('simulink/Signal Routing/Mux', [mdl '/baseline_log_mux'], ...
    'Inputs', '8', ...
    'Position', [535 80 565 300]);
add_block('simulink/Signal Routing/Mux', [mdl '/voltage_scope_mux'], ...
    'Inputs', '6', ...
    'Position', [535 335 565 485]);

add_block('simulink/Sinks/To Workspace', [mdl '/simout_dual_hall_harmonic_baseline'], ...
    'VariableName', 'simout_dual_hall_harmonic_baseline', ...
    'SaveFormat', 'StructureWithTime', ...
    'Position', [640 150 870 190]);
add_block('simulink/Sinks/Scope', [mdl '/scope_hall_voltage_baseline'], ...
    'Position', [640 375 740 435]);

%% 说明注释
annotationText = sprintf([ ...
    'Dual linear Hall harmonic baseline model\\n', ...
    'Raw: offset + amplitude mismatch + phase error + harmonic distortion\\n', ...
    'Basic comp: offset / amplitude / phase compensation\\n', ...
    'Harmonic comp: known 2/3/5/6 order signal-domain compensation\\n', ...
    'p = %d, f_mag = %.3g Hz, injected orders = %s'], ...
    hall_pole_pairs, hall_f_mag_hz, mat2str(hall_harm_orders));
try
    note = Simulink.Annotation(mdl, annotationText);
    note.Position = [55 20 890 75];
catch
    % 不同 MATLAB 版本的 Annotation API 可能略有差异；不影响模型逻辑。
end

%% 连线
add_line(mdl, 'time_t/1', 'dual_hall_harmonic_core/1', 'autorouting', 'on');

set_param(mdl, 'SimulationCommand', 'update');
for idx = 1:8
    add_line(mdl, sprintf('dual_hall_harmonic_core/%d', idx), ...
        sprintf('baseline_log_mux/%d', idx), 'autorouting', 'on');
end
add_line(mdl, 'baseline_log_mux/1', 'simout_dual_hall_harmonic_baseline/1', 'autorouting', 'on');

% 示波器只显示 6 路 Hall 电压：原始、基础补偿、谐波补偿各两路。
for idx = 3:8
    add_line(mdl, sprintf('dual_hall_harmonic_core/%d', idx), ...
        sprintf('voltage_scope_mux/%d', idx - 2), 'autorouting', 'on');
end
add_line(mdl, 'voltage_scope_mux/1', 'scope_hall_voltage_baseline/1', 'autorouting', 'on');

save_system(mdl, modelPath);
fprintf('Saved harmonic baseline Simulink model:\n%s\n', modelPath);

end

function set_matlab_function_script(blockPath)
% 将核心公式写入 MATLAB Function Block。
% 输出顺序：
%   1 theta_m       机械角
%   2 theta_mag     磁场角
%   3 Hs_raw        正弦通道原始电压
%   4 Hc_raw        余弦通道原始电压
%   5 Hs_basic_v    基础补偿后的正弦通道等效电压
%   6 Hc_basic_v    基础补偿后的余弦通道等效电压
%   7 Hs_comp_v     谐波补偿后的正弦通道等效电压
%   8 Hc_comp_v     谐波补偿后的余弦通道等效电压

omegaText = fmt_value(evalin('caller', 'hall_omega_mech_rad_s'));
pText = fmt_value(evalin('caller', 'hall_pole_pairs'));
theta0Text = fmt_value(evalin('caller', 'hall_theta0_rad'));
offsetText = fmt_value(evalin('caller', 'hall_offset_v'));
ampNomText = fmt_value(evalin('caller', 'hall_amp_v'));
ampSText = fmt_value(evalin('caller', 'hall_amp_s_v'));
ampCText = fmt_value(evalin('caller', 'hall_amp_c_v'));
offsetSText = fmt_value(evalin('caller', 'hall_offset_s_actual_v'));
offsetCText = fmt_value(evalin('caller', 'hall_offset_c_actual_v'));
phaseText = fmt_value(evalin('caller', 'hall_phase_err_rad'));
ordersText = mat2str(evalin('caller', 'hall_harm_orders'), 16);
harmSSinText = mat2str(evalin('caller', 'hall_harm_s_sin_v'), 16);
harmSCosText = mat2str(evalin('caller', 'hall_harm_s_cos_v'), 16);
harmCSinText = mat2str(evalin('caller', 'hall_harm_c_sin_v'), 16);
harmCCosText = mat2str(evalin('caller', 'hall_harm_c_cos_v'), 16);

scriptText = [
"function [theta_m, theta_mag, Hs_raw, Hc_raw, Hs_basic_v, Hc_basic_v, Hs_comp_v, Hc_comp_v] = f(t)"
"%#codegen"
"% 双线性霍尔谐波基准信号源。"
"% 该模块用已知注入参数演示 raw -> basic comp -> harmonic comp 的信号变化。"
"omega_m = " + omegaText + ";"
"pole_pairs = " + pText + ";"
"theta0 = " + theta0Text + ";"
"offset_v = " + offsetText + ";"
"amp_nom_v = " + ampNomText + ";"
"amp_s_v = " + ampSText + ";"
"amp_c_v = " + ampCText + ";"
"offset_s_v = " + offsetSText + ";"
"offset_c_v = " + offsetCText + ";"
"phase_err = " + phaseText + ";"
"harm_orders = " + ordersText + ";"
"harm_s_sin = " + harmSSinText + ";"
"harm_s_cos = " + harmSCosText + ";"
"harm_c_sin = " + harmCSinText + ";"
"harm_c_cos = " + harmCCosText + ";"
""
"theta_m = omega_m*t;"
"theta_mag = pole_pairs*theta_m + theta0;"
""
"Hs_harm = 0;"
"Hc_harm = 0;"
"for idx = 1:4"
"    n = harm_orders(idx);"
"    Hs_harm = Hs_harm + harm_s_sin(idx)*sin(n*theta_mag) + harm_s_cos(idx)*cos(n*theta_mag);"
"    Hc_harm = Hc_harm + harm_c_sin(idx)*sin(n*theta_mag) + harm_c_cos(idx)*cos(n*theta_mag);"
"end"
""
"Hs_raw = offset_s_v + amp_s_v*sin(theta_mag) + Hs_harm;"
"Hc_raw = offset_c_v + amp_c_v*cos(theta_mag + phase_err) + Hc_harm;"
""
"% 基础补偿：去除已知零偏和幅值不一致，并校正两路相位不正交。"
"xs_basic = (Hs_raw - offset_s_v)/amp_s_v;"
"xc_basic = (Hc_raw - offset_c_v)/amp_c_v;"
"xc_phasecorr = (xc_basic + xs_basic*sin(phase_err))/cos(phase_err);"
"Hs_basic_v = offset_v + amp_nom_v*xs_basic;"
"Hc_basic_v = offset_v + amp_nom_v*xc_phasecorr;"
""
"% 谐波补偿：扣除已知的信号域谐波残差，用于固定当前基准模型。"
"xs_harm_norm = Hs_harm/amp_s_v;"
"xc_harm_norm = Hc_harm/amp_c_v;"
"xc_harm_phasecorr = (xc_harm_norm + xs_harm_norm*sin(phase_err))/cos(phase_err);"
"xs_comp = xs_basic - xs_harm_norm;"
"xc_comp = xc_phasecorr - xc_harm_phasecorr;"
"Hs_comp_v = offset_v + amp_nom_v*xs_comp;"
"Hc_comp_v = offset_v + amp_nom_v*xc_comp;"
];

rt = sfroot;
chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
if isempty(chart)
    error('Cannot find MATLAB Function block chart: %s', blockPath);
end
chart.Script = strjoin(scriptText, newline);
end

function valueText = fmt_value(value)
% 用足够有效数字写入 MATLAB Function Block，保证基准模型可复现。
valueText = string(mat2str(value, 16));
end

function put_params_in_model_workspace(mdl)
% 将 hall_ 开头的参数写入模型工作区，避免依赖 base workspace。
vars = evalin('caller', 'who');
mw = get_param(mdl, 'ModelWorkspace');
for k = 1:numel(vars)
    name = vars{k};
    if startsWith(name, 'hall_')
        assignin(mw, name, evalin('caller', name));
    end
end
end
