function mdl = build_bo1808_svpwm_sfunc_model()
%BUILD_BO1808_SVPWM_SFUNC_MODEL Create the SVPWM S-Function model.
%
% Generated model:
%   models/bo1808_svpwm_sfunc_sim.slx
%
% The SVPWM core is implemented by svpwm_bo1808_sfunc.m:
%   [Valpha; Vbeta; Udc; Tpwm] -> [Tcm1; Tcm2; Tcm3; sector; T1; T2; T0]

init_bo1808_svpwm_params;

mdl = 'bo1808_svpwm_sfunc_sim';
if bdIsLoaded(mdl)
    close_system(mdl,0);
end
new_system(mdl);
open_system(mdl);
put_svpwm_sfunc_vars_in_model_workspace(mdl);

set_param(mdl, ...
    'StopTime','t_stop_svpwm', ...
    'Solver','ode23tb', ...
    'MaxStep','Ts_sim_svpwm', ...
    'SaveOutput','on');

%% Reference voltage vector in alpha-beta frame
add_block('simulink/Sources/Sine Wave', [mdl '/Ualpha_ref'], ...
    'Amplitude','Uref_svpwm', ...
    'Frequency','2*pi*f_ref_svpwm', ...
    'Phase','pi/2', ...
    'SampleTime','0', ...
    'Position',[40 65 95 95]);
add_block('simulink/Sources/Sine Wave', [mdl '/Ubeta_ref'], ...
    'Amplitude','Uref_svpwm', ...
    'Frequency','2*pi*f_ref_svpwm', ...
    'Phase','0', ...
    'SampleTime','0', ...
    'Position',[40 125 95 155]);
add_block('simulink/Sources/Constant', [mdl '/Vdc'], ...
    'Value','Vdc', 'Position',[40 190 95 220]);
add_block('simulink/Sources/Constant', [mdl '/Ts_pwm'], ...
    'Value','Ts_pwm_svpwm', 'Position',[40 250 95 280]);

%% SVPWM S-Function core
add_block('simulink/Signal Routing/Mux', [mdl '/svpwm_sfunc_input_mux'], ...
    'Inputs','4', 'Position',[145 105 175 245]);
add_block('simulink/User-Defined Functions/S-Function', [mdl '/svpwm_bo1808_sfunc_core'], ...
    'FunctionName','svpwm_bo1808_sfunc', 'Position',[225 115 380 235]);
add_block('simulink/Signal Routing/Demux', [mdl '/svpwm_sfunc_output_demux'], ...
    'Outputs','7', 'Position',[420 75 450 275]);

%% Triangular carrier and comparators
add_block('simulink/Sources/Repeating Sequence', [mdl '/triangular_carrier_time'], ...
    'rep_seq_t','[0 Ts_pwm_svpwm/2 Ts_pwm_svpwm]', ...
    'rep_seq_y','[0 Ts_pwm_svpwm/2 0]', ...
    'Position',[500 330 595 370]);

add_block('simulink/Logic and Bit Operations/Relational Operator', [mdl '/compare_duty_a'], ...
    'Operator','<=', 'Position',[610 130 650 165]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [mdl '/compare_duty_b'], ...
    'Operator','<=', 'Position',[610 190 650 225]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [mdl '/compare_duty_c'], ...
    'Operator','<=', 'Position',[610 250 650 285]);
add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/Sa_double'], ...
    'OutDataTypeStr','double', 'Position',[675 132 705 162]);
add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/Sb_double'], ...
    'OutDataTypeStr','double', 'Position',[675 192 705 222]);
add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/Sc_double'], ...
    'OutDataTypeStr','double', 'Position',[675 252 705 282]);

%% Inverter voltage reconstruction
recon = [mdl '/two_level_inverter_reconstruction'];
add_block('simulink/User-Defined Functions/MATLAB Function', recon, ...
    'Position',[750 160 930 310]);
set_matlab_function_script(recon, inverter_reconstruction_script());

%% Logging and scopes
add_block('simulink/Signal Routing/Mux', [mdl '/svpwm_sfunc_log_mux'], ...
    'Inputs','18', 'Position',[990 50 1020 430]);
add_block('simulink/Sinks/To Workspace', [mdl '/simout_svpwm_sfunc'], ...
    'VariableName','simout_svpwm_sfunc', ...
    'SaveFormat','StructureWithTime', ...
    'Position',[1075 195 1205 225]);
add_block('simulink/Sinks/Scope', [mdl '/scope_svpwm_sfunc'], ...
    'Position',[1075 270 1150 320]);

%% Wiring: reference to S-Function
add_line(mdl,'Ualpha_ref/1','svpwm_sfunc_input_mux/1','autorouting','on');
add_line(mdl,'Ubeta_ref/1','svpwm_sfunc_input_mux/2','autorouting','on');
add_line(mdl,'Vdc/1','svpwm_sfunc_input_mux/3','autorouting','on');
add_line(mdl,'Ts_pwm/1','svpwm_sfunc_input_mux/4','autorouting','on');
add_line(mdl,'svpwm_sfunc_input_mux/1','svpwm_bo1808_sfunc_core/1','autorouting','on');
add_line(mdl,'svpwm_bo1808_sfunc_core/1','svpwm_sfunc_output_demux/1','autorouting','on');

% Demux outputs: 1 Tcm1, 2 Tcm2, 3 Tcm3, 4 sector, 5 T1, 6 T2, 7 T0.
add_line(mdl,'svpwm_sfunc_output_demux/1','compare_duty_a/1','autorouting','on');
add_line(mdl,'svpwm_sfunc_output_demux/2','compare_duty_b/1','autorouting','on');
add_line(mdl,'svpwm_sfunc_output_demux/3','compare_duty_c/1','autorouting','on');
add_line(mdl,'triangular_carrier_time/1','compare_duty_a/2','autorouting','on');
add_line(mdl,'triangular_carrier_time/1','compare_duty_b/2','autorouting','on');
add_line(mdl,'triangular_carrier_time/1','compare_duty_c/2','autorouting','on');

add_line(mdl,'compare_duty_a/1','Sa_double/1','autorouting','on');
add_line(mdl,'compare_duty_b/1','Sb_double/1','autorouting','on');
add_line(mdl,'compare_duty_c/1','Sc_double/1','autorouting','on');
add_line(mdl,'Sa_double/1','two_level_inverter_reconstruction/1','autorouting','on');
add_line(mdl,'Sb_double/1','two_level_inverter_reconstruction/2','autorouting','on');
add_line(mdl,'Sc_double/1','two_level_inverter_reconstruction/3','autorouting','on');
add_line(mdl,'Vdc/1','two_level_inverter_reconstruction/4','autorouting','on');

%% Logging order, kept identical to validate_bo1808_svpwm where possible
add_line(mdl,'Ualpha_ref/1','svpwm_sfunc_log_mux/1','autorouting','on');
add_line(mdl,'Ubeta_ref/1','svpwm_sfunc_log_mux/2','autorouting','on');
add_line(mdl,'two_level_inverter_reconstruction/1','svpwm_sfunc_log_mux/3','autorouting','on');
add_line(mdl,'two_level_inverter_reconstruction/2','svpwm_sfunc_log_mux/4','autorouting','on');
add_line(mdl,'svpwm_sfunc_output_demux/4','svpwm_sfunc_log_mux/5','autorouting','on');
add_line(mdl,'svpwm_sfunc_output_demux/5','svpwm_sfunc_log_mux/6','autorouting','on');
add_line(mdl,'svpwm_sfunc_output_demux/6','svpwm_sfunc_log_mux/7','autorouting','on');
add_line(mdl,'svpwm_sfunc_output_demux/7','svpwm_sfunc_log_mux/8','autorouting','on');
add_line(mdl,'svpwm_sfunc_output_demux/1','svpwm_sfunc_log_mux/9','autorouting','on');
add_line(mdl,'svpwm_sfunc_output_demux/2','svpwm_sfunc_log_mux/10','autorouting','on');
add_line(mdl,'svpwm_sfunc_output_demux/3','svpwm_sfunc_log_mux/11','autorouting','on');
add_line(mdl,'triangular_carrier_time/1','svpwm_sfunc_log_mux/12','autorouting','on');
add_line(mdl,'Sa_double/1','svpwm_sfunc_log_mux/13','autorouting','on');
add_line(mdl,'Sb_double/1','svpwm_sfunc_log_mux/14','autorouting','on');
add_line(mdl,'Sc_double/1','svpwm_sfunc_log_mux/15','autorouting','on');
add_line(mdl,'two_level_inverter_reconstruction/3','svpwm_sfunc_log_mux/16','autorouting','on');
add_line(mdl,'two_level_inverter_reconstruction/4','svpwm_sfunc_log_mux/17','autorouting','on');
add_line(mdl,'two_level_inverter_reconstruction/5','svpwm_sfunc_log_mux/18','autorouting','on');
add_line(mdl,'svpwm_sfunc_log_mux/1','simout_svpwm_sfunc/1','autorouting','on');
add_line(mdl,'svpwm_sfunc_log_mux/1','scope_svpwm_sfunc/1','autorouting','on');

add_model_annotation(mdl, ...
    ['BO1808NBH2B SVPWM model using a Level-1 MATLAB S-Function core. ' ...
     'The core follows the Section 2.4.2 Tcm calculation interface: ' ...
     'Valpha, Vbeta, Udc, Tpwm -> Tcm1, Tcm2, Tcm3, sector.']);

model_file = fullfile(models_dir,[mdl '.slx']);
save_system(mdl,model_file);
fprintf('Created %s\n', model_file);
end

function txt = inverter_reconstruction_script()
txt = [
"function [Valpha,Vbeta,Vab,Vbc,Vca] = fcn(Sa,Sb,Sc,Vdc)"
"%#codegen"
"sa = double(Sa);"
"sb = double(Sb);"
"sc = double(Sc);"
"Va0 = (sa - 0.5)*Vdc;"
"Vb0 = (sb - 0.5)*Vdc;"
"Vc0 = (sc - 0.5)*Vdc;"
"Vab = Va0 - Vb0;"
"Vbc = Vb0 - Vc0;"
"Vca = Vc0 - Va0;"
"Valpha = (2/3)*(Va0 - 0.5*Vb0 - 0.5*Vc0);"
"Vbeta = (2/3)*(sqrt(3)/2)*(Vb0 - Vc0);"
"end"
];
txt = strjoin(txt,newline);
end

function set_matlab_function_script(block, scriptText)
rt = sfroot;
chart = rt.find('-isa','Stateflow.EMChart','Path',block);
chart.Script = scriptText;
end

function add_model_annotation(sys, txt)
try
    Simulink.Annotation(sys, txt);
catch
end
end

function put_svpwm_sfunc_vars_in_model_workspace(mdl)
names = { ...
    'Vdc','fs_pwm','Ts_pwm_svpwm','Ts_sim_svpwm','t_stop_svpwm', ...
    'fft_cycles_svpwm','f_ref_svpwm','omega_ref_svpwm', ...
    'Vref_max_svpwm','m_svpwm','Uref_svpwm', ...
    'id_ref_svpwm','iq_ref_svpwm','ud_ref_svpwm','uq_ref_svpwm'};
mws = get_param(mdl,'ModelWorkspace');
for k = 1:numel(names)
    assignin(mws,names{k},evalin('caller',names{k}));
end
end
