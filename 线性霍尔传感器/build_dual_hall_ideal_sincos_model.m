function mdl = build_dual_hall_ideal_sincos_model(forceRebuild)
%BUILD_DUAL_HALL_IDEAL_SINCOS_MODEL Build an ideal dual-Hall Simulink model.
%
% Default behavior preserves an existing model layout. After you manually
% adjust and save the .slx layout, calling this function without arguments
% only loads the existing model and refreshes parameters in memory. It does
% not force-save the .slx file, so manual layout edits are not disturbed.
%
% To regenerate all blocks and reset layout intentionally:
%   build_dual_hall_ideal_sincos_model(true)
% When forceRebuild is true, the function first captures block positions
% from the existing model and reapplies them after regeneration.
%
% Output model:
%   ./models/dual_hall_ideal_sincos.slx
%
% Signal equations:
%   theta_m(t) = omega_m*t
%   theta_mag(t) = p*theta_m(t) + theta0
%   Hall_sin = V0s + A_s*sin(theta_mag)
%   Hall_cos = V0c + A_c*cos(theta_mag + dphi)

if nargin < 1
    forceRebuild = false;
end

scriptDir = fileparts(mfilename('fullpath'));
modelDir = fullfile(scriptDir, 'models');
if ~exist(modelDir, 'dir')
    mkdir(modelDir);
end

init_dual_hall_ideal_params;

mdl = 'dual_hall_ideal_sincos';
modelPath = fullfile(modelDir, [mdl '.slx']);
savedLayout = struct('Name', {}, 'Position', {});

if exist(modelPath, 'file') == 2 && ~forceRebuild
    if ~bdIsLoaded(mdl)
        load_system(modelPath);
    end
    put_params_in_model_workspace(mdl);
    ensure_cos_phase_error_block(mdl);
    refresh_existing_model_params(mdl);
    set_param(mdl, ...
        'StopTime', 'hall_t_stop_s', ...
        'SolverType', 'Fixed-step', ...
        'Solver', 'ode4', ...
        'FixedStep', 'hall_ts_sim_s', ...
        'SaveOutput', 'on');
    fprintf('Loaded existing Simulink model, layout preserved:\n%s\n', modelPath);
    fprintf('Parameters refreshed in memory; existing .slx file was not force-saved.\n');
    fprintf('To rebuild and reset layout, run build_dual_hall_ideal_sincos_model(true).\n');
    return
end

if forceRebuild && exist(modelPath, 'file') == 2
    savedLayout = capture_existing_block_layout(mdl, modelPath);
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

%% Mechanical angle and magnetic angle
add_block('simulink/Sources/Clock', [mdl '/t'], ...
    'Position', [45 90 75 120]);
add_block('simulink/Math Operations/Gain', [mdl '/ωₘ'], ...
    'Gain', fmt_value(hall_omega_mech_rad_s), ...
    'Position', [115 82 225 128]);
add_block('simulink/Math Operations/Gain', [mdl '/p'], ...
    'Gain', fmt_value(hall_pole_pairs), ...
    'Position', [270 82 365 128]);
add_block('simulink/Math Operations/Bias', [mdl '/θ₀'], ...
    'Bias', fmt_value(hall_theta0_rad), ...
    'Position', [405 82 500 128]);

%% Ideal sin/cos Hall channels
add_block('simulink/Math Operations/Trigonometric Function', [mdl '/sin'], ...
    'Operator', 'sin', ...
    'Position', [560 45 615 95]);
add_block('simulink/Math Operations/Trigonometric Function', [mdl '/cos'], ...
    'Operator', 'cos', ...
    'Position', [560 135 615 185]);
add_block('simulink/Math Operations/Bias', [mdl '/Δφ'], ...
    'Bias', fmt_value(hall_phase_err_rad), ...
    'Position', [465 135 535 185]);

add_block('simulink/Math Operations/Gain', [mdl '/A_s'], ...
    'Gain', fmt_value(hall_amp_s_v), ...
    'Position', [655 45 745 95]);
add_block('simulink/Math Operations/Gain', [mdl '/A_c'], ...
    'Gain', fmt_value(hall_amp_c_v), ...
    'Position', [655 135 745 185]);

add_block('simulink/Math Operations/Bias', [mdl '/V₀s'], ...
    'Bias', fmt_value(hall_offset_s_actual_v), ...
    'Position', [785 45 875 95]);
add_block('simulink/Math Operations/Bias', [mdl '/V₀c'], ...
    'Bias', fmt_value(hall_offset_c_actual_v), ...
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
add_line(mdl, 't/1', 'ωₘ/1', 'autorouting', 'on');
add_line(mdl, 'ωₘ/1', 'p/1', 'autorouting', 'on');
add_line(mdl, 'p/1', 'θ₀/1', 'autorouting', 'on');

add_line(mdl, 'θ₀/1', 'sin/1', 'autorouting', 'on');
add_line(mdl, 'θ₀/1', 'Δφ/1', 'autorouting', 'on');
add_line(mdl, 'Δφ/1', 'cos/1', 'autorouting', 'on');

add_line(mdl, 'sin/1', 'A_s/1', 'autorouting', 'on');
add_line(mdl, 'cos/1', 'A_c/1', 'autorouting', 'on');
add_line(mdl, 'A_s/1', 'V₀s/1', 'autorouting', 'on');
add_line(mdl, 'A_c/1', 'V₀c/1', 'autorouting', 'on');

add_line(mdl, 'ωₘ/1', 'hall_log_mux/1', 'autorouting', 'on');
add_line(mdl, 'θ₀/1', 'hall_log_mux/2', 'autorouting', 'on');
add_line(mdl, 'V₀s/1', 'hall_log_mux/3', 'autorouting', 'on');
add_line(mdl, 'V₀c/1', 'hall_log_mux/4', 'autorouting', 'on');
add_line(mdl, 'hall_log_mux/1', 'simout_dual_hall_ideal/1', 'autorouting', 'on');

add_line(mdl, 'V₀s/1', 'hall_scope_mux/1', 'autorouting', 'on');
add_line(mdl, 'V₀c/1', 'hall_scope_mux/2', 'autorouting', 'on');
add_line(mdl, 'hall_scope_mux/1', 'scope_dual_hall_ideal/1', 'autorouting', 'on');

annotationText = sprintf([ ...
    'Ideal dual linear Hall signal source\\n', ...
    'theta_m = omega_m*t\\n', ...
    'theta_mag = p*theta_m + theta0\\n', ...
    'H_s = V0 + A_s sin(theta_mag)\\n', ...
    'H_c = V0 + A_c cos(theta_mag + dphi)\\n', ...
    'V0s = %.2f V, A_s = %.2f V\\n', ...
    'V0c = %.2f V, A_c = %.2f V\\n', ...
    'dphi = %.2f deg'], ...
    hall_offset_s_actual_v, hall_amp_s_v, ...
    hall_offset_c_actual_v, hall_amp_c_v, ...
    hall_phase_err_deg);
try
    note = Simulink.Annotation(mdl, annotationText);
    note.Position = [45 220 520 320];
catch
    % Annotation API differs across MATLAB releases; the model logic is unchanged.
end

if ~isempty(savedLayout)
    apply_saved_block_layout(mdl, savedLayout);
else
    try
        Simulink.BlockDiagram.arrangeSystem(mdl);
    catch
    end
end
% 在余弦通道前加入相位误差 Δφ，用于模拟两个 Hall 传感器不是严格 90° 正交安装。
% 正弦通道保持 H_s = V0s + A_s*sin(theta_mag)。
% 余弦通道变为 H_c = V0c + A_c*cos(theta_mag + Δφ)。
% 当 Δφ 不为 0 时，即使完成零偏和幅值补偿，相图仍会表现为倾斜椭圆，atan2 解角仍会有周期性误差。

save_system(mdl, modelPath);
fprintf('Saved Simulink model:\n%s\n', modelPath);

end

function valueText = fmt_value(value)
if abs(value - round(value)) < 1e-12
    valueText = sprintf('%d', round(value));
else
    valueText = sprintf('%.6g', value);
end
end

function refresh_existing_model_params(mdl)
set_param_if_exists(mdl, 'ωₘ', 'Gain', fmt_value(evalin('caller', 'hall_omega_mech_rad_s')));
set_param_if_exists(mdl, 'p', 'Gain', fmt_value(evalin('caller', 'hall_pole_pairs')));
set_param_if_exists(mdl, 'θ₀', 'Bias', fmt_value(evalin('caller', 'hall_theta0_rad')));
set_param_if_exists(mdl, 'Δφ', 'Bias', fmt_value(evalin('caller', 'hall_phase_err_rad')));
set_param_if_exists(mdl, 'A_s', 'Gain', fmt_value(evalin('caller', 'hall_amp_s_v')));
set_param_if_exists(mdl, 'A_c', 'Gain', fmt_value(evalin('caller', 'hall_amp_c_v')));
set_param_if_exists(mdl, 'V₀s', 'Bias', fmt_value(evalin('caller', 'hall_offset_s_actual_v')));
set_param_if_exists(mdl, 'V₀c', 'Bias', fmt_value(evalin('caller', 'hall_offset_c_actual_v')));

% Backward-compatible names from previous versions.
set_param_if_exists(mdl, 'Hall_sin_amplitude', 'Gain', fmt_value(evalin('caller', 'hall_amp_s_v')));
set_param_if_exists(mdl, 'Hall_cos_amplitude', 'Gain', fmt_value(evalin('caller', 'hall_amp_c_v')));
set_param_if_exists(mdl, 'H_s = V_0 + A sin(\theta_{mag})', 'Bias', fmt_value(evalin('caller', 'hall_offset_s_actual_v')));
set_param_if_exists(mdl, 'H_c = V_0 + A cos(\theta_{mag})', 'Bias', fmt_value(evalin('caller', 'hall_offset_c_actual_v')));
end

function ensure_cos_phase_error_block(mdl)
phaseBlock = [mdl '/Δφ'];
phaseValue = fmt_value(evalin('caller', 'hall_phase_err_rad'));

if block_exists(phaseBlock)
    set_param(phaseBlock, 'Bias', phaseValue);
else
    cosBlock = [mdl '/cos'];
    if block_exists(cosBlock)
        cosPos = get_param(cosBlock, 'Position');
        phasePos = [cosPos(1)-95 cosPos(2) cosPos(1)-25 cosPos(4)];
    else
        phasePos = [465 135 535 185];
    end
    add_block('simulink/Math Operations/Bias', phaseBlock, ...
        'Bias', phaseValue, ...
        'Position', phasePos);
end

if ~block_exists([mdl '/cos'])
    return
end

phaseHandles = get_param(phaseBlock, 'PortHandles');
cosHandles = get_param([mdl '/cos'], 'PortHandles');
cosInLine = get_param(cosHandles.Inport(1), 'Line');

if cosInLine ~= -1
    srcBlock = get_param(get_param(cosInLine, 'SrcBlockHandle'), 'Name');
    if ~strcmp(srcBlock, 'Δφ')
        srcPort = get_param(cosInLine, 'SrcPortHandle');
        delete_line(cosInLine);
        try
            add_line(mdl, srcPort, phaseHandles.Inport(1), 'autorouting', 'on');
        catch
        end
    end
end

phaseOutLine = get_param(phaseHandles.Outport(1), 'Line');
if phaseOutLine == -1
    try
        add_line(mdl, phaseHandles.Outport(1), cosHandles.Inport(1), 'autorouting', 'on');
    catch
    end
end

if get_param(phaseHandles.Inport(1), 'Line') == -1 && block_exists([mdl '/θ₀'])
    try
        add_line(mdl, 'θ₀/1', 'Δφ/1', 'autorouting', 'on');
    catch
    end
end
end

function tf = block_exists(blockPath)
try
    get_param(blockPath, 'Handle');
    tf = true;
catch
    tf = false;
end
end

function layout = capture_existing_block_layout(mdl, modelPath)
try
    if ~bdIsLoaded(mdl)
        load_system(modelPath);
    end
    blocks = find_system(mdl, 'SearchDepth', 1, 'Type', 'Block');
    layout = repmat(struct('Name', '', 'Position', []), numel(blocks), 1);
    for k = 1:numel(blocks)
        layout(k).Name = get_param(blocks{k}, 'Name');
        layout(k).Position = get_param(blocks{k}, 'Position');
    end
    fprintf('Captured %d block positions from existing layout.\n', numel(layout));
catch ME
    warning('dualHall:LayoutCaptureFailed', ...
        'Could not capture existing layout positions: %s', ME.message);
    layout = struct('Name', {}, 'Position', {});
end
end

function apply_saved_block_layout(mdl, layout)
appliedCount = 0;
for k = 1:numel(layout)
    blockPath = [mdl '/' layout(k).Name];
    if ~isempty(layout(k).Position) && block_exists(blockPath)
        try
            set_param(blockPath, 'Position', layout(k).Position);
            appliedCount = appliedCount + 1;
        catch
        end
    end
end
fprintf('Reapplied %d saved block positions after rebuild.\n', appliedCount);
end

function set_param_if_exists(mdl, blockName, paramName, valueText)
blockPath = [mdl '/' blockName];
try
    get_param(blockPath, 'Handle');
    set_param(blockPath, paramName, valueText);
catch
end
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
