function mdl = build_dual_hall_ideal_sincos_model()
%BUILD_DUAL_HALL_IDEAL_SINCOS_MODEL Build an ideal dual-Hall Simulink model.
%
% Output model:
%   ./models/dual_hall_ideal_sincos.slx
%
% Signal equations:
%   theta_m(t) = omega_m*t
%   theta_mag(t) = p*theta_m(t) + theta0
%   Hall_sin = V0 + A*sin(theta_mag)
%   Hall_cos = V0 + A*cos(theta_mag)

scriptDir = fileparts(mfilename('fullpath'));
modelDir = fullfile(scriptDir, 'models');
if ~exist(modelDir, 'dir')
    mkdir(modelDir);
end

init_dual_hall_ideal_params;

mdl = 'dual_hall_ideal_sincos';
modelPath = fullfile(modelDir, [mdl '.slx']);

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

%% Mechanical angle and magnetic angle
add_block('simulink/Sources/Clock', [mdl '/time'], ...
    'Position', [45 90 75 120]);
add_block('simulink/Math Operations/Gain', [mdl '/omega_mech_rad_s'], ...
    'Gain', 'hall_omega_mech_rad_s', ...
    'Position', [115 82 225 128]);
add_block('simulink/Math Operations/Gain', [mdl '/pole_pairs_p'], ...
    'Gain', 'hall_pole_pairs', ...
    'Position', [270 82 365 128]);
add_block('simulink/Math Operations/Bias', [mdl '/theta0_rad'], ...
    'Bias', 'hall_theta0_rad', ...
    'Position', [405 82 500 128]);

%% Ideal sin/cos Hall channels
add_block('simulink/Math Operations/Trigonometric Function', [mdl '/sin_theta_mag'], ...
    'Operator', 'sin', ...
    'Position', [560 45 615 95]);
add_block('simulink/Math Operations/Trigonometric Function', [mdl '/cos_theta_mag'], ...
    'Operator', 'cos', ...
    'Position', [560 135 615 185]);

add_block('simulink/Math Operations/Gain', [mdl '/Hall_sin_amplitude'], ...
    'Gain', 'hall_amp_v', ...
    'Position', [655 45 745 95]);
add_block('simulink/Math Operations/Gain', [mdl '/Hall_cos_amplitude'], ...
    'Gain', 'hall_amp_v', ...
    'Position', [655 135 745 185]);

add_block('simulink/Math Operations/Bias', [mdl '/Hall_sin_offset'], ...
    'Bias', 'hall_offset_v', ...
    'Position', [785 45 875 95]);
add_block('simulink/Math Operations/Bias', [mdl '/Hall_cos_offset'], ...
    'Bias', 'hall_offset_v', ...
    'Position', [785 135 875 185]);

%% Logging
add_block('simulink/Signal Routing/Mux', [mdl '/hall_log_mux'], ...
    'Inputs', '4', ...
    'Position', [945 70 975 170]);
add_block('simulink/Signal Routing/Mux', [mdl '/hall_scope_mux'], ...
    'Inputs', '2', ...
    'Position', [945 220 975 260]);
add_block('simulink/Sinks/To Workspace', [mdl '/simout_dual_hall_ideal'], ...
    'VariableName', 'simout_dual_hall_ideal', ...
    'SaveFormat', 'StructureWithTime', ...
    'Position', [1035 82 1185 118]);
add_block('simulink/Sinks/Scope', [mdl '/scope_dual_hall_ideal'], ...
    'Position', [1035 212 1115 262]);

%% Wiring
add_line(mdl, 'time/1', 'omega_mech_rad_s/1', 'autorouting', 'on');
add_line(mdl, 'omega_mech_rad_s/1', 'pole_pairs_p/1', 'autorouting', 'on');
add_line(mdl, 'pole_pairs_p/1', 'theta0_rad/1', 'autorouting', 'on');

add_line(mdl, 'theta0_rad/1', 'sin_theta_mag/1', 'autorouting', 'on');
add_line(mdl, 'theta0_rad/1', 'cos_theta_mag/1', 'autorouting', 'on');

add_line(mdl, 'sin_theta_mag/1', 'Hall_sin_amplitude/1', 'autorouting', 'on');
add_line(mdl, 'cos_theta_mag/1', 'Hall_cos_amplitude/1', 'autorouting', 'on');
add_line(mdl, 'Hall_sin_amplitude/1', 'Hall_sin_offset/1', 'autorouting', 'on');
add_line(mdl, 'Hall_cos_amplitude/1', 'Hall_cos_offset/1', 'autorouting', 'on');

add_line(mdl, 'omega_mech_rad_s/1', 'hall_log_mux/1', 'autorouting', 'on');
add_line(mdl, 'theta0_rad/1', 'hall_log_mux/2', 'autorouting', 'on');
add_line(mdl, 'Hall_sin_offset/1', 'hall_log_mux/3', 'autorouting', 'on');
add_line(mdl, 'Hall_cos_offset/1', 'hall_log_mux/4', 'autorouting', 'on');
add_line(mdl, 'hall_log_mux/1', 'simout_dual_hall_ideal/1', 'autorouting', 'on');

add_line(mdl, 'Hall_sin_offset/1', 'hall_scope_mux/1', 'autorouting', 'on');
add_line(mdl, 'Hall_cos_offset/1', 'hall_scope_mux/2', 'autorouting', 'on');
add_line(mdl, 'hall_scope_mux/1', 'scope_dual_hall_ideal/1', 'autorouting', 'on');

annotationText = sprintf([ ...
    'Ideal dual linear Hall signal source\\n', ...
    'H_s = V0 + A sin(p theta_m + theta0)\\n', ...
    'H_c = V0 + A cos(p theta_m + theta0)\\n', ...
    'Default: p = %d, V0 = %.2f V, A = %.2f V'], ...
    hall_pole_pairs, hall_offset_v, hall_amp_v);
try
    note = Simulink.Annotation(mdl, annotationText);
    note.Position = [45 220 520 300];
catch
    % Annotation API differs across MATLAB releases; the model logic is unchanged.
end

try
    Simulink.BlockDiagram.arrangeSystem(mdl);
catch
end

save_system(mdl, modelPath);
fprintf('Saved Simulink model:\n%s\n', modelPath);

end

function put_params_in_model_workspace(mdl)
vars = evalin('caller', 'who');
mw = get_param(mdl, 'ModelWorkspace');
for k = 1:numel(vars)
    name = vars{k};
    if startsWith(name, 'hall_')
        assignin(mw, name, evalin('caller', name));
    end
end
end
