function mdl = build_dual_hall_discrete_eso_model(forceRebuild)
%BUILD_DUAL_HALL_DISCRETE_ESO_MODEL Build a discrete ESO Simulink demo.
%
% This model focuses on the estimator stage after Hall angle decoding:
%
%   compensated angle measurement theta_meas
%       -> angle LPF
%       -> alpha-beta estimator
%       -> discrete ESO estimator
%
% The model is intentionally independent from the hand-arranged Hall voltage
% models. By default, an existing .slx is loaded and its workspace data is
% refreshed, so manual layout edits are preserved. Use forceRebuild=true only
% when you want to regenerate the blocks.

if nargin < 1
    forceRebuild = false;
end

scriptDir = fileparts(mfilename('fullpath'));
modelDir = fullfile(scriptDir, 'models');
if ~exist(modelDir, 'dir')
    mkdir(modelDir);
end

init_dual_hall_noise_filter_params;

mdl = 'dual_hall_discrete_eso';
modelPath = fullfile(modelDir, [mdl '.slx']);

if exist(modelPath, 'file') == 2 && ~forceRebuild
    if ~bdIsLoaded(mdl)
        load_system(modelPath);
    end
    put_eso_data_in_model_workspace(mdl);
    set_solver_params(mdl);
    fprintf('Loaded existing discrete ESO model, layout preserved:\n%s\n', modelPath);
    return
end

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

new_system(mdl);
put_eso_data_in_model_workspace(mdl);
set_solver_params(mdl);

%% Sources and constants
add_block('simulink/Sources/From Workspace', [mdl '/theta_meas_from_hall'], ...
    'VariableName', 'eso_theta_meas_ts', ...
    'Position', [55 115 185 145]);
add_block('simulink/Sources/From Workspace', [mdl '/theta_true_reference'], ...
    'VariableName', 'eso_theta_true_ts', ...
    'Position', [55 315 185 345]);

add_block('simulink/Sources/Constant', [mdl '/Ts'], ...
    'Value', 'eso_Ts', ...
    'Position', [225 65 285 95]);
add_block('simulink/Sources/Constant', [mdl '/alpha_lpf'], ...
    'Value', 'eso_angle_lpf_alpha', ...
    'Position', [225 170 285 200]);
add_block('simulink/Sources/Constant', [mdl '/alpha_ab'], ...
    'Value', 'eso_ab_alpha', ...
    'Position', [420 60 485 90]);
add_block('simulink/Sources/Constant', [mdl '/beta_ab'], ...
    'Value', 'eso_ab_beta', ...
    'Position', [420 100 485 130]);
add_block('simulink/Sources/Constant', [mdl '/beta1_eso'], ...
    'Value', 'eso_beta1', ...
    'Position', [420 190 485 220]);
add_block('simulink/Sources/Constant', [mdl '/beta2_eso'], ...
    'Value', 'eso_beta2', ...
    'Position', [420 230 485 260]);
add_block('simulink/Sources/Constant', [mdl '/beta3_eso'], ...
    'Value', 'eso_beta3', ...
    'Position', [420 270 485 300]);

%% Estimator blocks
lpfBlock = [mdl '/angle_LPF_after_decoding'];
add_block('simulink/Ports & Subsystems/Subsystem', lpfBlock, ...
    'Position', [335 135 520 205]);
build_angle_lpf_principle_subsystem(lpfBlock);

abBlock = [mdl '/alpha_beta_estimator'];
add_block('simulink/Ports & Subsystems/Subsystem', abBlock, ...
    'Position', [560 60 765 150]);
build_alpha_beta_principle_subsystem(abBlock);

esoBlock = [mdl '/discrete_ESO_estimator'];
add_block('simulink/Ports & Subsystems/Subsystem', esoBlock, ...
    'Position', [560 205 765 315]);
build_discrete_eso_principle_subsystem(esoBlock);

errBlock = [mdl '/angle_error_calc'];
add_block('simulink/User-Defined Functions/MATLAB Function', errBlock, ...
    'Position', [850 130 1060 300]);
set_chart_script(errBlock, error_calc_script());

%% Output scopes
% Keep small Mux blocks only in front of Scopes for visual comparison.
add_block('simulink/Signal Routing/Mux', [mdl '/estimate_scope_mux'], ...
    'Inputs', '4', ...
    'Position', [1125 65 1155 185]);
add_block('simulink/Signal Routing/Mux', [mdl '/error_scope_mux'], ...
    'Inputs', '4', ...
    'Position', [1125 240 1155 360]);
add_block('simulink/Signal Routing/Mux', [mdl '/eso_state_mux'], ...
    'Inputs', '3', ...
    'Position', [850 365 880 455]);

add_block('simulink/Sinks/Scope', [mdl '/scope_angle_estimates'], ...
    'Position', [1210 90 1310 150]);
add_block('simulink/Sinks/Scope', [mdl '/scope_angle_errors'], ...
    'Position', [1210 265 1310 325]);
add_block('simulink/Sinks/Scope', [mdl '/scope_eso_states'], ...
    'Position', [930 385 1030 445]);

%% Annotation
demoEsoBandwidthHz = 80;
annotationText = sprintf([ ...
    'Discrete ESO demo after Hall angle decoding\\n', ...
    'Input: theta_{meas}; outputs: z1=theta_hat, z2=omega_hat, z3=disturbance_hat\\n', ...
    'ESO bandwidth = %.3g Hz, alpha-beta = [%.3g, %.3g], noise RMS = %.3g V'], ...
    demoEsoBandwidthHz, hall_ab_alpha, hall_ab_beta, hall_noise_rms_v);
try
    note = Simulink.Annotation(mdl, annotationText);
    note.Position = [45 15 920 55];
catch
end

%% Wiring
add_line(mdl, 'theta_meas_from_hall/1', 'angle_LPF_after_decoding/1', 'autorouting', 'on');
add_line(mdl, 'alpha_lpf/1', 'angle_LPF_after_decoding/2', 'autorouting', 'on');

add_line(mdl, 'theta_meas_from_hall/1', 'alpha_beta_estimator/1', 'autorouting', 'on');
add_line(mdl, 'Ts/1', 'alpha_beta_estimator/2', 'autorouting', 'on');
add_line(mdl, 'alpha_ab/1', 'alpha_beta_estimator/3', 'autorouting', 'on');
add_line(mdl, 'beta_ab/1', 'alpha_beta_estimator/4', 'autorouting', 'on');

add_line(mdl, 'theta_meas_from_hall/1', 'discrete_ESO_estimator/1', 'autorouting', 'on');
add_line(mdl, 'Ts/1', 'discrete_ESO_estimator/2', 'autorouting', 'on');
add_line(mdl, 'beta1_eso/1', 'discrete_ESO_estimator/3', 'autorouting', 'on');
add_line(mdl, 'beta2_eso/1', 'discrete_ESO_estimator/4', 'autorouting', 'on');
add_line(mdl, 'beta3_eso/1', 'discrete_ESO_estimator/5', 'autorouting', 'on');

add_line(mdl, 'theta_true_reference/1', 'angle_error_calc/1', 'autorouting', 'on');
add_line(mdl, 'theta_meas_from_hall/1', 'angle_error_calc/2', 'autorouting', 'on');
add_line(mdl, 'angle_LPF_after_decoding/1', 'angle_error_calc/3', 'autorouting', 'on');
add_line(mdl, 'alpha_beta_estimator/1', 'angle_error_calc/4', 'autorouting', 'on');
add_line(mdl, 'discrete_ESO_estimator/1', 'angle_error_calc/5', 'autorouting', 'on');

% Estimate scope: true, measured, alpha-beta, ESO.
add_line(mdl, 'theta_true_reference/1', 'estimate_scope_mux/1', 'autorouting', 'on');
add_line(mdl, 'theta_meas_from_hall/1', 'estimate_scope_mux/2', 'autorouting', 'on');
add_line(mdl, 'alpha_beta_estimator/1', 'estimate_scope_mux/3', 'autorouting', 'on');
add_line(mdl, 'discrete_ESO_estimator/1', 'estimate_scope_mux/4', 'autorouting', 'on');
add_line(mdl, 'estimate_scope_mux/1', 'scope_angle_estimates/1', 'autorouting', 'on');

% Error scope: raw, angle LPF, alpha-beta, ESO.
for idx = 1:4
    add_line(mdl, sprintf('angle_error_calc/%d', idx), ...
        sprintf('error_scope_mux/%d', idx), 'autorouting', 'on');
end
add_line(mdl, 'error_scope_mux/1', 'scope_angle_errors/1', 'autorouting', 'on');

% ESO states.
for idx = 1:3
    add_line(mdl, sprintf('discrete_ESO_estimator/%d', idx), ...
        sprintf('eso_state_mux/%d', idx), 'autorouting', 'on');
end
add_line(mdl, 'eso_state_mux/1', 'scope_eso_states/1', 'autorouting', 'on');

set_param(mdl, 'SimulationCommand', 'update');
save_system(mdl, modelPath);
fprintf('Saved discrete ESO Simulink model:\n%s\n', modelPath);
end

function put_eso_data_in_model_workspace(mdl)
init_dual_hall_noise_filter_params;

eso_Ts = hall_ts_sim_s;
eso_t_stop_s = hall_noise_fig_window_s;
eso_t = (0:eso_Ts:eso_t_stop_s).';
eso_theta_true = hall_omega_mech_rad_s*eso_t;

rng(hall_noise_seed);
angle_noise_deg = max(0.15, 18*hall_noise_rms_v);
angle_noise = deg2rad(angle_noise_deg)*randn(size(eso_t));
angle_ripple = deg2rad(0.16)*sin(2*pi*hall_f_mag_hz*eso_t) + ...
    deg2rad(0.06)*sin(2*pi*2*hall_f_mag_hz*eso_t + 0.4);
eso_theta_meas = eso_theta_true + angle_ripple + angle_noise;

eso_theta_true_ts = timeseries(eso_theta_true, eso_t);
eso_theta_meas_ts = timeseries(eso_theta_meas, eso_t);
eso_initial_theta = eso_theta_meas(1);

eso_angle_lpf_cutoff_hz = 40;
eso_angle_lpf_tau_s = 1/(2*pi*eso_angle_lpf_cutoff_hz);
eso_angle_lpf_alpha = exp(-eso_Ts/eso_angle_lpf_tau_s);

eso_ab_alpha = hall_ab_alpha;
eso_ab_beta = hall_ab_beta;

eso_bandwidth_hz = hall_eso_bandwidth_hz;
eso_omega_o = 2*pi*eso_bandwidth_hz;
eso_beta1 = 3*eso_omega_o;
eso_beta2 = 3*eso_omega_o^2;
eso_beta3 = eso_omega_o^3;

mw = get_param(mdl, 'ModelWorkspace');
assignin(mw, 'eso_Ts', eso_Ts);
assignin(mw, 'eso_t_stop_s', eso_t_stop_s);
assignin(mw, 'eso_theta_true_ts', eso_theta_true_ts);
assignin(mw, 'eso_theta_meas_ts', eso_theta_meas_ts);
assignin(mw, 'eso_initial_theta', eso_initial_theta);
assignin(mw, 'eso_angle_lpf_alpha', eso_angle_lpf_alpha);
assignin(mw, 'eso_ab_alpha', eso_ab_alpha);
assignin(mw, 'eso_ab_beta', eso_ab_beta);
assignin(mw, 'eso_beta1', eso_beta1);
assignin(mw, 'eso_beta2', eso_beta2);
assignin(mw, 'eso_beta3', eso_beta3);
assignin(mw, 'eso_bandwidth_hz', eso_bandwidth_hz);
end

function set_solver_params(mdl)
set_param(mdl, ...
    'StopTime', 'eso_t_stop_s', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', 'eso_Ts', ...
    'SaveOutput', 'on');
end

function set_chart_script(blockPath, scriptText)
rt = sfroot;
chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
if isempty(chart)
    error('Cannot find MATLAB Function block chart: %s', blockPath);
end
chart.Script = scriptText;
end

function build_angle_lpf_principle_subsystem(subsys)
% Build the angle-domain first-order low-pass filter from visible blocks.
% Equation:
%   theta_lpf[k] = alpha*theta_lpf[k-1] + (1-alpha)*theta_meas[k]

try
    Simulink.SubSystem.deleteContents(subsys);
catch
    oldBlocks = find_system(subsys, 'SearchDepth', 1, 'Type', 'Block');
    for k = 1:numel(oldBlocks)
        if ~strcmp(oldBlocks{k}, subsys)
            delete_block(oldBlocks{k});
        end
    end
end

add_block('simulink/Sources/In1', [subsys '/theta_meas'], ...
    'Position', [35 75 65 95]);
add_block('simulink/Sources/In1', [subsys '/alpha'], ...
    'Port', '2', ...
    'Position', [35 175 65 195]);
add_block('simulink/Sinks/Out1', [subsys '/theta_lpf'], ...
    'Position', [620 115 650 135]);

add_block('simulink/Sources/Constant', [subsys '/one'], ...
    'Value', '1', ...
    'Position', [115 150 145 175]);
add_block('simulink/Math Operations/Sum', [subsys '/1 - alpha'], ...
    'Inputs', '+-', ...
    'Position', [185 160 215 195]);
add_block('simulink/Discrete/Unit Delay', [subsys '/Unit Delay theta_lpf'], ...
    'InitialCondition', 'eso_initial_theta', ...
    'SampleTime', 'eso_Ts', ...
    'Position', [255 60 305 110]);
add_block('simulink/Math Operations/Product', [subsys '/alpha_times_prev'], ...
    'Inputs', '2', ...
    'Position', [370 65 405 100]);
add_block('simulink/Math Operations/Product', [subsys '/one_minus_alpha_times_meas'], ...
    'Inputs', '2', ...
    'Position', [370 155 405 190]);
add_block('simulink/Math Operations/Sum', [subsys '/theta_lpf_next'], ...
    'Inputs', '++', ...
    'Position', [480 110 510 145]);

try
    note = Simulink.Annotation(subsys, sprintf([ ...
        'Angle-domain LPF principle module\n', ...
        'theta_lpf[k] = alpha*theta_lpf[k-1] + (1-alpha)*theta_meas[k]\n', ...
        'It smooths angle measurement but can introduce phase lag.']));
    note.Position = [80 15 560 45];
catch
end

add_line(subsys, 'one/1', '1 - alpha/1', 'autorouting', 'on');
add_line(subsys, 'alpha/1', '1 - alpha/2', 'autorouting', 'on');
add_line(subsys, 'alpha/1', 'alpha_times_prev/1', 'autorouting', 'on');
add_line(subsys, 'Unit Delay theta_lpf/1', 'alpha_times_prev/2', 'autorouting', 'on');
add_line(subsys, '1 - alpha/1', 'one_minus_alpha_times_meas/1', 'autorouting', 'on');
add_line(subsys, 'theta_meas/1', 'one_minus_alpha_times_meas/2', 'autorouting', 'on');
add_line(subsys, 'alpha_times_prev/1', 'theta_lpf_next/1', 'autorouting', 'on');
add_line(subsys, 'one_minus_alpha_times_meas/1', 'theta_lpf_next/2', 'autorouting', 'on');
add_line(subsys, 'theta_lpf_next/1', 'Unit Delay theta_lpf/1', 'autorouting', 'on');
add_line(subsys, 'theta_lpf_next/1', 'theta_lpf/1', 'autorouting', 'on');
end

function build_alpha_beta_principle_subsystem(subsys)
% Build the alpha-beta estimator from visible prediction/correction blocks.
% Equations:
%   theta_pred[k] = theta_hat[k-1] + Ts*omega_hat[k-1]
%   e[k]          = theta_meas[k] - theta_pred[k]
%   theta_hat[k]  = theta_pred[k] + alpha*e[k]
%   omega_hat[k]  = omega_hat[k-1] + beta/Ts*e[k]

try
    Simulink.SubSystem.deleteContents(subsys);
catch
    oldBlocks = find_system(subsys, 'SearchDepth', 1, 'Type', 'Block');
    for k = 1:numel(oldBlocks)
        if ~strcmp(oldBlocks{k}, subsys)
            delete_block(oldBlocks{k});
        end
    end
end

add_block('simulink/Sources/In1', [subsys '/theta_meas'], ...
    'Position', [35 65 65 85]);
add_block('simulink/Sources/In1', [subsys '/Ts'], ...
    'Port', '2', ...
    'Position', [35 135 65 155]);
add_block('simulink/Sources/In1', [subsys '/alpha'], ...
    'Port', '3', ...
    'Position', [35 230 65 250]);
add_block('simulink/Sources/In1', [subsys '/beta'], ...
    'Port', '4', ...
    'Position', [35 305 65 325]);

add_block('simulink/Sinks/Out1', [subsys '/theta_hat'], ...
    'Position', [720 100 750 120]);
add_block('simulink/Sinks/Out1', [subsys '/omega_hat'], ...
    'Port', '2', ...
    'Position', [720 275 750 295]);

add_block('simulink/Discrete/Unit Delay', [subsys '/Unit Delay theta_hat'], ...
    'InitialCondition', 'eso_initial_theta', ...
    'SampleTime', 'eso_Ts', ...
    'Position', [140 80 190 130]);
add_block('simulink/Discrete/Unit Delay', [subsys '/Unit Delay omega_hat'], ...
    'InitialCondition', '0', ...
    'SampleTime', 'eso_Ts', ...
    'Position', [140 260 190 310]);

add_block('simulink/Math Operations/Product', [subsys '/Ts_times_omega'], ...
    'Inputs', '2', ...
    'Position', [245 175 280 210]);
add_block('simulink/Math Operations/Sum', [subsys '/theta_pred'], ...
    'Inputs', '++', ...
    'Position', [340 105 370 140]);
add_block('simulink/Math Operations/Sum', [subsys '/e = theta_meas - theta_pred'], ...
    'Inputs', '+-', ...
    'Position', [430 75 460 110]);

add_block('simulink/Math Operations/Product', [subsys '/alpha_e'], ...
    'Inputs', '2', ...
    'Position', [520 135 555 170]);
add_block('simulink/Math Operations/Sum', [subsys '/theta_hat_next'], ...
    'Inputs', '++', ...
    'Position', [610 95 640 130]);

add_block('simulink/Math Operations/Product', [subsys '/beta_e'], ...
    'Inputs', '2', ...
    'Position', [520 255 555 290]);
add_block('simulink/Math Operations/Product', [subsys '/beta_e_over_Ts'], ...
    'Inputs', '*/', ...
    'Position', [590 255 625 290]);
add_block('simulink/Math Operations/Sum', [subsys '/omega_hat_next'], ...
    'Inputs', '++', ...
    'Position', [655 260 685 295]);

try
    note = Simulink.Annotation(subsys, sprintf([ ...
        'Alpha-beta estimator principle module\n', ...
        'theta_pred = theta_hat[k-1] + Ts*omega_hat[k-1]\n', ...
        'e = theta_meas - theta_pred\n', ...
        'theta_hat = theta_pred + alpha*e,  omega_hat = omega_hat[k-1] + beta/Ts*e']));
    note.Position = [85 15 700 50];
catch
end

add_line(subsys, 'Ts/1', 'Ts_times_omega/1', 'autorouting', 'on');
add_line(subsys, 'Unit Delay omega_hat/1', 'Ts_times_omega/2', 'autorouting', 'on');
add_line(subsys, 'Unit Delay theta_hat/1', 'theta_pred/1', 'autorouting', 'on');
add_line(subsys, 'Ts_times_omega/1', 'theta_pred/2', 'autorouting', 'on');
add_line(subsys, 'theta_meas/1', 'e = theta_meas - theta_pred/1', 'autorouting', 'on');
add_line(subsys, 'theta_pred/1', 'e = theta_meas - theta_pred/2', 'autorouting', 'on');

add_line(subsys, 'alpha/1', 'alpha_e/1', 'autorouting', 'on');
add_line(subsys, 'e = theta_meas - theta_pred/1', 'alpha_e/2', 'autorouting', 'on');
add_line(subsys, 'theta_pred/1', 'theta_hat_next/1', 'autorouting', 'on');
add_line(subsys, 'alpha_e/1', 'theta_hat_next/2', 'autorouting', 'on');

add_line(subsys, 'beta/1', 'beta_e/1', 'autorouting', 'on');
add_line(subsys, 'e = theta_meas - theta_pred/1', 'beta_e/2', 'autorouting', 'on');
add_line(subsys, 'beta_e/1', 'beta_e_over_Ts/1', 'autorouting', 'on');
add_line(subsys, 'Ts/1', 'beta_e_over_Ts/2', 'autorouting', 'on');
add_line(subsys, 'Unit Delay omega_hat/1', 'omega_hat_next/1', 'autorouting', 'on');
add_line(subsys, 'beta_e_over_Ts/1', 'omega_hat_next/2', 'autorouting', 'on');

add_line(subsys, 'theta_hat_next/1', 'Unit Delay theta_hat/1', 'autorouting', 'on');
add_line(subsys, 'omega_hat_next/1', 'Unit Delay omega_hat/1', 'autorouting', 'on');
add_line(subsys, 'theta_hat_next/1', 'theta_hat/1', 'autorouting', 'on');
add_line(subsys, 'omega_hat_next/1', 'omega_hat/1', 'autorouting', 'on');
end

function build_discrete_eso_principle_subsystem(subsys)
% Build the discrete ESO from visible Simulink principle blocks.
% Equations:
%   e[k]       = theta_meas[k] - z1[k]
%   z1[k+1]   = z1[k] + Ts*(z2[k] + beta1*e[k])
%   z2[k+1]   = z2[k] + Ts*(z3[k] + beta2*e[k])
%   z3[k+1]   = z3[k] + Ts*(beta3*e[k])

try
    Simulink.SubSystem.deleteContents(subsys);
catch
    oldBlocks = find_system(subsys, 'SearchDepth', 1, 'Type', 'Block');
    for k = 1:numel(oldBlocks)
        if ~strcmp(oldBlocks{k}, subsys)
            delete_block(oldBlocks{k});
        end
    end
end

add_block('simulink/Sources/In1', [subsys '/theta_meas'], ...
    'Position', [35 55 65 75]);
add_block('simulink/Sources/In1', [subsys '/Ts'], ...
    'Port', '2', ...
    'Position', [35 115 65 135]);
add_block('simulink/Sources/In1', [subsys '/beta1'], ...
    'Port', '3', ...
    'Position', [35 175 65 195]);
add_block('simulink/Sources/In1', [subsys '/beta2'], ...
    'Port', '4', ...
    'Position', [35 235 65 255]);
add_block('simulink/Sources/In1', [subsys '/beta3'], ...
    'Port', '5', ...
    'Position', [35 295 65 315]);

add_block('simulink/Sinks/Out1', [subsys '/z1'], ...
    'Position', [720 60 750 80]);
add_block('simulink/Sinks/Out1', [subsys '/z2'], ...
    'Port', '2', ...
    'Position', [720 175 750 195]);
add_block('simulink/Sinks/Out1', [subsys '/z3'], ...
    'Port', '3', ...
    'Position', [720 290 750 310]);

add_block('simulink/Discrete/Unit Delay', [subsys '/Unit Delay z1'], ...
    'InitialCondition', 'eso_initial_theta', ...
    'SampleTime', 'eso_Ts', ...
    'Position', [585 45 635 95]);
add_block('simulink/Discrete/Unit Delay', [subsys '/Unit Delay z2'], ...
    'InitialCondition', '0', ...
    'SampleTime', 'eso_Ts', ...
    'Position', [585 160 635 210]);
add_block('simulink/Discrete/Unit Delay', [subsys '/Unit Delay z3'], ...
    'InitialCondition', '0', ...
    'SampleTime', 'eso_Ts', ...
    'Position', [585 275 635 325]);

add_block('simulink/Math Operations/Sum', [subsys '/e = theta_meas - z1'], ...
    'Inputs', '+-', ...
    'Position', [130 60 160 90]);

add_block('simulink/Math Operations/Product', [subsys '/beta1_e'], ...
    'Inputs', '2', ...
    'Position', [225 145 260 180]);
add_block('simulink/Math Operations/Product', [subsys '/beta2_e'], ...
    'Inputs', '2', ...
    'Position', [225 220 260 255]);
add_block('simulink/Math Operations/Product', [subsys '/beta3_e'], ...
    'Inputs', '2', ...
    'Position', [225 295 260 330]);

add_block('simulink/Math Operations/Sum', [subsys '/z2 + beta1e'], ...
    'Inputs', '++', ...
    'Position', [330 120 360 150]);
add_block('simulink/Math Operations/Sum', [subsys '/z3 + beta2e'], ...
    'Inputs', '++', ...
    'Position', [330 220 360 250]);

add_block('simulink/Math Operations/Product', [subsys '/Ts_times_z1_update'], ...
    'Inputs', '2', ...
    'Position', [410 115 445 150]);
add_block('simulink/Math Operations/Product', [subsys '/Ts_times_z2_update'], ...
    'Inputs', '2', ...
    'Position', [410 215 445 250]);
add_block('simulink/Math Operations/Product', [subsys '/Ts_times_z3_update'], ...
    'Inputs', '2', ...
    'Position', [410 295 445 330]);

add_block('simulink/Math Operations/Sum', [subsys '/z1_next'], ...
    'Inputs', '++', ...
    'Position', [505 60 535 90]);
add_block('simulink/Math Operations/Sum', [subsys '/z2_next'], ...
    'Inputs', '++', ...
    'Position', [505 175 535 205]);
add_block('simulink/Math Operations/Sum', [subsys '/z3_next'], ...
    'Inputs', '++', ...
    'Position', [505 290 535 320]);

try
    note = Simulink.Annotation(subsys, sprintf([ ...
        'Discrete ESO principle module\n', ...
        'e = theta_meas - z1\n', ...
        'z1[k+1] = z1[k] + Ts*(z2[k] + beta1*e)\n', ...
        'z2[k+1] = z2[k] + Ts*(z3[k] + beta2*e)\n', ...
        'z3[k+1] = z3[k] + Ts*beta3*e']));
    note.Position = [85 10 650 42];
catch
end

add_line(subsys, 'theta_meas/1', 'e = theta_meas - z1/1', 'autorouting', 'on');
add_line(subsys, 'Unit Delay z1/1', 'e = theta_meas - z1/2', 'autorouting', 'on');

add_line(subsys, 'e = theta_meas - z1/1', 'beta1_e/2', 'autorouting', 'on');
add_line(subsys, 'e = theta_meas - z1/1', 'beta2_e/2', 'autorouting', 'on');
add_line(subsys, 'e = theta_meas - z1/1', 'beta3_e/2', 'autorouting', 'on');
add_line(subsys, 'beta1/1', 'beta1_e/1', 'autorouting', 'on');
add_line(subsys, 'beta2/1', 'beta2_e/1', 'autorouting', 'on');
add_line(subsys, 'beta3/1', 'beta3_e/1', 'autorouting', 'on');

add_line(subsys, 'Unit Delay z2/1', 'z2 + beta1e/1', 'autorouting', 'on');
add_line(subsys, 'beta1_e/1', 'z2 + beta1e/2', 'autorouting', 'on');
add_line(subsys, 'Unit Delay z3/1', 'z3 + beta2e/1', 'autorouting', 'on');
add_line(subsys, 'beta2_e/1', 'z3 + beta2e/2', 'autorouting', 'on');

add_line(subsys, 'Ts/1', 'Ts_times_z1_update/1', 'autorouting', 'on');
add_line(subsys, 'z2 + beta1e/1', 'Ts_times_z1_update/2', 'autorouting', 'on');
add_line(subsys, 'Ts/1', 'Ts_times_z2_update/1', 'autorouting', 'on');
add_line(subsys, 'z3 + beta2e/1', 'Ts_times_z2_update/2', 'autorouting', 'on');
add_line(subsys, 'Ts/1', 'Ts_times_z3_update/1', 'autorouting', 'on');
add_line(subsys, 'beta3_e/1', 'Ts_times_z3_update/2', 'autorouting', 'on');

add_line(subsys, 'Unit Delay z1/1', 'z1_next/1', 'autorouting', 'on');
add_line(subsys, 'Ts_times_z1_update/1', 'z1_next/2', 'autorouting', 'on');
add_line(subsys, 'Unit Delay z2/1', 'z2_next/1', 'autorouting', 'on');
add_line(subsys, 'Ts_times_z2_update/1', 'z2_next/2', 'autorouting', 'on');
add_line(subsys, 'Unit Delay z3/1', 'z3_next/1', 'autorouting', 'on');
add_line(subsys, 'Ts_times_z3_update/1', 'z3_next/2', 'autorouting', 'on');

add_line(subsys, 'z1_next/1', 'Unit Delay z1/1', 'autorouting', 'on');
add_line(subsys, 'z2_next/1', 'Unit Delay z2/1', 'autorouting', 'on');
add_line(subsys, 'z3_next/1', 'Unit Delay z3/1', 'autorouting', 'on');

add_line(subsys, 'Unit Delay z1/1', 'z1/1', 'autorouting', 'on');
add_line(subsys, 'Unit Delay z2/1', 'z2/1', 'autorouting', 'on');
add_line(subsys, 'Unit Delay z3/1', 'z3/1', 'autorouting', 'on');
end

function scriptText = error_calc_script()
scriptText = sprintf([ ...
    'function [err_meas_deg, err_lpf_deg, err_ab_deg, err_eso_deg] = f(theta_true, theta_meas, theta_lpf, theta_ab, theta_eso)\n', ...
    '%%#codegen\n', ...
    'rad2deg_gain = 180/pi;\n', ...
    'err_meas_deg = wrap_pi_local(theta_meas - theta_true)*rad2deg_gain;\n', ...
    'err_lpf_deg = wrap_pi_local(theta_lpf - theta_true)*rad2deg_gain;\n', ...
    'err_ab_deg = wrap_pi_local(theta_ab - theta_true)*rad2deg_gain;\n', ...
    'err_eso_deg = wrap_pi_local(theta_eso - theta_true)*rad2deg_gain;\n', ...
    'end\n', ...
    '\n', ...
    'function y = wrap_pi_local(x)\n', ...
    'y = mod(x + pi, 2*pi) - pi;\n', ...
    'end\n']);
end
