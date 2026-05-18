function mdl = build_bo1808_svpwm_model()
%BUILD_BO1808_SVPWM_MODEL Create a standalone Simulink SVPWM model.
%
% Generated model:
%   bo1808_svpwm_sim.slx
%
% Contents:
%   alpha-beta rotating voltage reference
%   N-coded sector, X/Y/Z, T1/T2, and Tcm1/Tcm2/Tcm3 calculation
%   triangular carrier comparison
%   three-phase two-level inverter voltage reconstruction

init_bo1808_svpwm_params;

mdl = 'bo1808_svpwm_sim';
if bdIsLoaded(mdl)
    close_system(mdl,0);
end
new_system(mdl);
open_system(mdl);
put_svpwm_vars_in_model_workspace(mdl);

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

%% SVPWM core
core = [mdl '/SVPWM_core_sector_T1_T2_T0'];
add_block('simulink/User-Defined Functions/MATLAB Function', core, ...
    'Position',[175 75 345 280]);
set_matlab_function_script(core, svpwm_core_script());

%% Triangular carrier and comparators
add_block('simulink/Sources/Repeating Sequence', [mdl '/triangular_carrier_time'], ...
    'rep_seq_t','[0 Ts_pwm_svpwm/2 Ts_pwm_svpwm]', ...
    'rep_seq_y','[0 Ts_pwm_svpwm/2 0]', ...
    'Position',[405 330 500 370]);

add_block('simulink/Logic and Bit Operations/Relational Operator', [mdl '/compare_duty_a'], ...
    'Operator','<=', 'Position',[555 180 595 215]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [mdl '/compare_duty_b'], ...
    'Operator','<=', 'Position',[555 240 595 275]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [mdl '/compare_duty_c'], ...
    'Operator','<=', 'Position',[555 300 595 335]);
add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/Sa_double'], ...
    'OutDataTypeStr','double', 'Position',[620 182 650 212]);
add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/Sb_double'], ...
    'OutDataTypeStr','double', 'Position',[620 242 650 272]);
add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/Sc_double'], ...
    'OutDataTypeStr','double', 'Position',[620 302 650 332]);

%% Inverter voltage reconstruction
recon = [mdl '/two_level_inverter_reconstruction'];
add_block('simulink/User-Defined Functions/MATLAB Function', recon, ...
    'Position',[705 190 885 340]);
set_matlab_function_script(recon, inverter_reconstruction_script());

%% Logging and scopes
add_block('simulink/Signal Routing/Mux', [mdl '/svpwm_log_mux'], ...
    'Inputs','18', 'Position',[930 50 960 430]);
add_block('simulink/Sinks/To Workspace', [mdl '/simout_svpwm'], ...
    'VariableName','simout_svpwm', ...
    'SaveFormat','StructureWithTime', ...
    'Position',[1015 195 1125 225]);
add_block('simulink/Sinks/Scope', [mdl '/scope_svpwm'], ...
    'Position',[1015 270 1090 320]);

%% Wiring: reference to SVPWM core
add_line(mdl,'Ualpha_ref/1','SVPWM_core_sector_T1_T2_T0/1','autorouting','on');
add_line(mdl,'Ubeta_ref/1','SVPWM_core_sector_T1_T2_T0/2','autorouting','on');
add_line(mdl,'Vdc/1','SVPWM_core_sector_T1_T2_T0/3','autorouting','on');
add_line(mdl,'Ts_pwm/1','SVPWM_core_sector_T1_T2_T0/4','autorouting','on');

% MATLAB Function outputs:
% 1 N-coded sector, 2 T1, 3 T2, 4 T0, 5 Tcm1, 6 Tcm2, 7 Tcm3
add_line(mdl,'SVPWM_core_sector_T1_T2_T0/5','compare_duty_a/1','autorouting','on');
add_line(mdl,'SVPWM_core_sector_T1_T2_T0/6','compare_duty_b/1','autorouting','on');
add_line(mdl,'SVPWM_core_sector_T1_T2_T0/7','compare_duty_c/1','autorouting','on');
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

%% Logging order
add_line(mdl,'Ualpha_ref/1','svpwm_log_mux/1','autorouting','on');
add_line(mdl,'Ubeta_ref/1','svpwm_log_mux/2','autorouting','on');
add_line(mdl,'two_level_inverter_reconstruction/1','svpwm_log_mux/3','autorouting','on');
add_line(mdl,'two_level_inverter_reconstruction/2','svpwm_log_mux/4','autorouting','on');
add_line(mdl,'SVPWM_core_sector_T1_T2_T0/1','svpwm_log_mux/5','autorouting','on');
add_line(mdl,'SVPWM_core_sector_T1_T2_T0/2','svpwm_log_mux/6','autorouting','on');
add_line(mdl,'SVPWM_core_sector_T1_T2_T0/3','svpwm_log_mux/7','autorouting','on');
add_line(mdl,'SVPWM_core_sector_T1_T2_T0/4','svpwm_log_mux/8','autorouting','on');
add_line(mdl,'SVPWM_core_sector_T1_T2_T0/5','svpwm_log_mux/9','autorouting','on');
add_line(mdl,'SVPWM_core_sector_T1_T2_T0/6','svpwm_log_mux/10','autorouting','on');
add_line(mdl,'SVPWM_core_sector_T1_T2_T0/7','svpwm_log_mux/11','autorouting','on');
add_line(mdl,'triangular_carrier_time/1','svpwm_log_mux/12','autorouting','on');
add_line(mdl,'Sa_double/1','svpwm_log_mux/13','autorouting','on');
add_line(mdl,'Sb_double/1','svpwm_log_mux/14','autorouting','on');
add_line(mdl,'Sc_double/1','svpwm_log_mux/15','autorouting','on');
add_line(mdl,'two_level_inverter_reconstruction/3','svpwm_log_mux/16','autorouting','on');
add_line(mdl,'two_level_inverter_reconstruction/4','svpwm_log_mux/17','autorouting','on');
add_line(mdl,'two_level_inverter_reconstruction/5','svpwm_log_mux/18','autorouting','on');
add_line(mdl,'svpwm_log_mux/1','simout_svpwm/1','autorouting','on');
add_line(mdl,'svpwm_log_mux/1','scope_svpwm/1','autorouting','on');

add_model_annotation(mdl, ...
    ['SVPWM Simulink model using the Tcm method: N-coded sector sequence ' ...
     '3-1-5-4-6-2, X/Y/Z, T1/T2/T0, Tcm1/Tcm2/Tcm3, triangular carrier ' ...
     'comparison, and inverter voltage reconstruction. Reference frequency ' ...
     'and voltage magnitude are derived from the BO1808NBH2B rated point.']);

model_file = fullfile(models_dir,[mdl '.slx']);
save_system(mdl,model_file);
fprintf('Created %s\n', model_file);
end

function txt = svpwm_core_script()
txt = [
"function [sector,T1,T2,T0,Tcm1,Tcm2,Tcm3] = fcn(Ualpha,Ubeta,Vdc,Ts)"
"%#codegen"
"Vref1 = Ubeta;"
"Vref2 = (sqrt(3)*Ualpha - Ubeta)/2;"
"Vref3 = (-sqrt(3)*Ualpha - Ubeta)/2;"
"A = 0;"
"B = 0;"
"C = 0;"
"if Vref1 >= 0"
"    A = 1;"
"end"
"if Vref2 >= 0"
"    B = 1;"
"end"
"if Vref3 >= 0"
"    C = 1;"
"end"
"sector = 4*C + 2*B + A;"
"if sector < 1"
"    sector = 1;"
"end"
"X = sqrt(3)*Ubeta*Ts/Vdc;"
"Y = Ts/Vdc*(1.5*Ualpha + sqrt(3)/2*Ubeta);"
"Z = Ts/Vdc*(-1.5*Ualpha + sqrt(3)/2*Ubeta);"
"if sector == 1"
"    T1 = Z;"
"    T2 = Y;"
"elseif sector == 2"
"    T1 = Y;"
"    T2 = -X;"
"elseif sector == 3"
"    T1 = -Z;"
"    T2 = X;"
"elseif sector == 4"
"    T1 = -X;"
"    T2 = Z;"
"elseif sector == 5"
"    T1 = X;"
"    T2 = -Y;"
"else"
"    T1 = -Y;"
"    T2 = -Z;"
"end"
"if T1 < 0"
"    T1 = 0;"
"end"
"if T2 < 0"
"    T2 = 0;"
"end"
"if T1 + T2 > Ts"
"    scale = Ts/(T1 + T2);"
"    T1 = T1*scale;"
"    T2 = T2*scale;"
"end"
"T0 = Ts - T1 - T2;"
"ta = T0/4;"
"tb = ta + T1/2;"
"tc = tb + T2/2;"
"if sector == 1"
"    Tcm1 = tb;"
"    Tcm2 = ta;"
"    Tcm3 = tc;"
"elseif sector == 2"
"    Tcm1 = ta;"
"    Tcm2 = tc;"
"    Tcm3 = tb;"
"elseif sector == 3"
"    Tcm1 = ta;"
"    Tcm2 = tb;"
"    Tcm3 = tc;"
"elseif sector == 4"
"    Tcm1 = tc;"
"    Tcm2 = tb;"
"    Tcm3 = ta;"
"elseif sector == 5"
"    Tcm1 = tc;"
"    Tcm2 = ta;"
"    Tcm3 = tb;"
"else"
"    Tcm1 = tb;"
"    Tcm2 = tc;"
"    Tcm3 = ta;"
"end"
"Tcm1 = min(max(Tcm1,0),Ts/2);"
"Tcm2 = min(max(Tcm2,0),Ts/2);"
"Tcm3 = min(max(Tcm3,0),Ts/2);"
"end"
];
txt = strjoin(txt,newline);
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

function put_svpwm_vars_in_model_workspace(mdl)
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
