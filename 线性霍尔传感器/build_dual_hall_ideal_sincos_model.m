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
%定义模型名称和路径

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
%如果模型已经打开，就先关闭

new_system(mdl);
put_params_in_model_workspace(mdl);
%新建 Simulink 模型

set_param(mdl, ...
    'StopTime', 'hall_t_stop_s', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'ode23tb', ...
    'FixedStep', 'hall_ts_sim_s', ...
    'SaveOutput', 'on');
%设置仿真参数
%StopTime	仿真停止时间
%SolverType	求解器类型
%Solver	具体求解器
%FixedStep	固定步长
%SaveOutput	是否保存输出

%% Mechanical angle and magnetic angle
%创建机械角和磁场角模块
add_block('simulink/Sources/Clock', [mdl '/t'], ...
    'Position', [45 90 75 120]);
add_block('simulink/Math Operations/Gain', [mdl '/\theta_m = \omega_m t'], ...
    'Gain', 'hall_omega_mech_rad_s', ...
    'Position', [115 82 225 128]);
add_block('simulink/Math Operations/Gain', [mdl '/p\theta_m'], ...
    'Gain', 'hall_pole_pairs', ...
    'Position', [270 82 365 128]);
add_block('simulink/Math Operations/Bias', [mdl '/\theta_{mag} = p\theta_m + \theta_0'], ...
    'Bias', 'hall_theta0_rad', ...
    'Position', [405 82 500 128]);

%% Ideal sin/cos Hall channels
%创建理想 sin/cos Hall 通道
add_block('simulink/Math Operations/Trigonometric Function', [mdl '/sin(\theta_{mag})'], ...
    'Operator', 'sin', ...
    'Position', [560 45 615 95]);
add_block('simulink/Math Operations/Trigonometric Function', [mdl '/cos(\theta_{mag})'], ...
    'Operator', 'cos', ...
    'Position', [560 135 615 185]);

add_block('simulink/Math Operations/Gain', [mdl '/A sin(\theta_{mag})'], ...
    'Gain', 'hall_amp_v', ...
    'Position', [655 45 745 95]);
add_block('simulink/Math Operations/Gain', [mdl '/A cos(\theta_{mag})'], ...
    'Gain', 'hall_amp_v', ...
    'Position', [655 135 745 185]);

add_block('simulink/Math Operations/Bias', [mdl '/H_s = V_0 + A sin(\theta_{mag})'], ...
    'Bias', 'hall_offset_v', ...
    'Position', [785 45 875 95]);
add_block('simulink/Math Operations/Bias', [mdl '/H_c = V_0 + A cos(\theta_{mag})'], ...
    'Bias', 'hall_offset_v', ...
    'Position', [785 135 875 185]);

%% Logging
%添加日志和显示模块
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
%连接模块
add_line(mdl, 't/1', '\theta_m = \omega_m t/1', 'autorouting', 'on');
add_line(mdl, '\theta_m = \omega_m t/1', 'p\theta_m/1', 'autorouting', 'on');
add_line(mdl, 'p\theta_m/1', '\theta_{mag} = p\theta_m + \theta_0/1', 'autorouting', 'on');

add_line(mdl, '\theta_{mag} = p\theta_m + \theta_0/1', 'sin(\theta_{mag})/1', 'autorouting', 'on');
add_line(mdl, '\theta_{mag} = p\theta_m + \theta_0/1', 'cos(\theta_{mag})/1', 'autorouting', 'on');

add_line(mdl, 'sin(\theta_{mag})/1', 'A sin(\theta_{mag})/1', 'autorouting', 'on');
add_line(mdl, 'cos(\theta_{mag})/1', 'A cos(\theta_{mag})/1', 'autorouting', 'on');
add_line(mdl, 'A sin(\theta_{mag})/1', 'H_s = V_0 + A sin(\theta_{mag})/1', 'autorouting', 'on');
add_line(mdl, 'A cos(\theta_{mag})/1', 'H_c = V_0 + A cos(\theta_{mag})/1', 'autorouting', 'on');

add_line(mdl, '\theta_m = \omega_m t/1', 'hall_log_mux/1', 'autorouting', 'on');
add_line(mdl, '\theta_{mag} = p\theta_m + \theta_0/1', 'hall_log_mux/2', 'autorouting', 'on');
add_line(mdl, 'H_s = V_0 + A sin(\theta_{mag})/1', 'hall_log_mux/3', 'autorouting', 'on');
add_line(mdl, 'H_c = V_0 + A cos(\theta_{mag})/1', 'hall_log_mux/4', 'autorouting', 'on');
add_line(mdl, 'hall_log_mux/1', 'simout_dual_hall_ideal/1', 'autorouting', 'on');

add_line(mdl, 'H_s = V_0 + A sin(\theta_{mag})/1', 'hall_scope_mux/1', 'autorouting', 'on');
add_line(mdl, 'H_c = V_0 + A cos(\theta_{mag})/1', 'hall_scope_mux/2', 'autorouting', 'on');
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
%添加模型注释

save_system(mdl, modelPath);
fprintf('Saved Simulink model:\n%s\n', modelPath);
%保存模型

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
