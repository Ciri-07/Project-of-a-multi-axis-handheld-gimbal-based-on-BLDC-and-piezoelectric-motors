function mdl = build_bo1808_foc_svpwm_model(loadMode, controlMode)
%BUILD_BO1808_FOC_SVPWM_MODEL Create the full BO1808 closed-loop drive model.
%
% Generated model:
%   models/bo1808_foc_svpwm_bo1808.slx
%
% Control and power-chain structure:
%   speed PI -> iq*; id* = 0;
%   d/q current PI + decoupling -> ud/uq;
%   inverse Park -> u_alpha/u_beta;
%   SVPWM Level-1 S-Function -> switching times;
%   carrier comparison -> Sa/Sb/Sc;
%   two-level inverter reconstruction -> alpha/beta voltage;
%   Park transform -> d/q voltage;
%   BO1808 PMSM Level-1 S-Function plant.
%
% Optional:
%   build_bo1808_foc_svpwm_model([], 'position') adds an outer mechanical
%   position loop. The position loop outputs the speed reference for the
%   existing speed/current/SVPWM drive chain.

if nargin < 1
    loadMode = [];
end
if nargin < 2
    controlMode = '';
end
init_bo1808_params;
if nargin >= 1 && ~isempty(loadMode)
    TL_load_mode = loadMode;
end
if isempty(controlMode)
    controlMode = foc_control_mode;
else
    foc_control_mode = controlMode;
end
usePositionLoop = strcmpi(controlMode,'position') || strcmpi(controlMode,'pos');

mdl = 'bo1808_foc_svpwm_bo1808';
if bdIsLoaded(mdl)
    close_system(mdl,0);
end
new_system(mdl);
open_system(mdl);
put_full_foc_svpwm_vars_in_model_workspace(mdl);
set_model_path_callbacks(mdl);

if usePositionLoop
    stop_time_expr = 't_stop_position_loop';
else
    stop_time_expr = 't_stop_foc_svpwm';
end
set_param(mdl, ...
    'StopTime',stop_time_expr, ...
    'Solver','ode23tb', ...
    'MaxStep','Ts_sim_foc_svpwm', ...
    'SaveOutput','on');

%% Reference and load
if usePositionLoop
    add_block('simulink/Sources/Step', [mdl '/theta_m_ref_step'], ...
        'Time','position_ref_step_time', 'Before','0', 'After','theta_m_ref_step', ...
        'Position',[25 25 85 55]);
    add_block('simulink/Math Operations/Sum', [mdl '/sum_theta_ref_minus_theta_m'], ...
        'Inputs','+-', 'Position',[125 25 150 55]);
    add_block('simulink/Commonly Used Blocks/Gain', [mdl '/P_position'], ...
        'Gain','Kp_pos', 'Position',[190 25 255 55]);
    add_block('simulink/Discontinuities/Saturation', [mdl '/sat_speed_ref_from_position'], ...
        'UpperLimit','omega_ref_pos_max', 'LowerLimit','-omega_ref_pos_max', ...
        'Position',[295 25 370 55]);
    add_block('simulink/Continuous/Integrator', [mdl '/int_theta_m'], ...
        'InitialCondition','theta_m0', 'Position',[125 430 160 460]);
else
    add_block('simulink/Sources/Step', [mdl '/speed_ref_rpm_step'], ...
        'Time','0.005', 'Before','0', 'After','speed_ref_fig3_9_rpm', ...
        'Position',[30 70 75 100]);
    add_block('simulink/Commonly Used Blocks/Gain', [mdl '/rpm_to_radps'], ...
        'Gain','2*pi/60', 'Position',[115 70 180 100]);
end
add_block('simulink/Sources/Constant', [mdl '/id_ref_0'], ...
    'Value','0', 'Position',[30 245 75 275]);
add_load_source(mdl,TL_load_mode,TL_noise_enable);

%% Speed and current PI loops
add_block('simulink/Math Operations/Sum', [mdl '/sum_speed_ref_minus_omega'], ...
    'Inputs','+-', 'Position',[225 70 250 100]);
add_block('built-in/Subsystem', [mdl '/PI_speed'], 'Position',[295 58 415 112]);
clear_subsystem([mdl '/PI_speed']);
build_pi_subsystem([mdl '/PI_speed'], 'Kp_w', 'Ki_w', '0');
add_block('simulink/Discontinuities/Saturation', [mdl '/sat_iq_ref'], ...
    'UpperLimit','Iq_max', 'LowerLimit','-Iq_max', ...
    'Position',[455 70 515 100]);

add_block('simulink/Math Operations/Sum', [mdl '/sum_iq_ref_minus_iq'], ...
    'Inputs','+-', 'Position',[225 155 250 185]);
add_block('simulink/Math Operations/Sum', [mdl '/sum_id_ref_minus_id'], ...
    'Inputs','+-', 'Position',[225 245 250 275]);
add_block('built-in/Subsystem', [mdl '/PI_iq'], 'Position',[295 142 415 196]);
clear_subsystem([mdl '/PI_iq']);
build_pi_subsystem([mdl '/PI_iq'], 'Kp_iq', 'Ki_iq', '0');
add_block('built-in/Subsystem', [mdl '/PI_id'], 'Position',[295 232 415 286]);
clear_subsystem([mdl '/PI_id']);
build_pi_subsystem([mdl '/PI_id'], 'Kp_id', 'Ki_id', '0');

%% Decoupling and d/q voltage saturation
add_block('simulink/Commonly Used Blocks/Gain', [mdl '/pn_omega_m_to_omega_e'], ...
    'Gain','pn', 'Position',[520 330 580 360]);

add_block('simulink/Commonly Used Blocks/Gain', [mdl '/gain_Lq_iq'], ...
    'Gain','Lq', 'Position',[535 225 590 255]);
add_block('simulink/Math Operations/Product', [mdl '/product_omegae_Lq_iq'], ...
    'Inputs','**', 'Position',[625 214 660 254]);
add_block('simulink/Math Operations/Sum', [mdl '/sum_ud_pi_minus_decoupling'], ...
    'Inputs','+-', 'Position',[695 235 720 265]);
add_block('simulink/Discontinuities/Saturation', [mdl '/sat_ud'], ...
    'UpperLimit','Vdq_max', 'LowerLimit','-Vdq_max', ...
    'Position',[760 235 820 265]);

add_block('simulink/Commonly Used Blocks/Gain', [mdl '/gain_Ld_id'], ...
    'Gain','Ld', 'Position',[535 285 590 315]);
add_block('simulink/Sources/Constant', [mdl '/const_psi_f'], ...
    'Value','psi_f', 'Position',[535 385 590 415]);
add_block('simulink/Math Operations/Sum', [mdl '/sum_Ld_id_plus_psi_f'], ...
    'Inputs','++', 'Position',[625 307 650 360]);
add_block('simulink/Math Operations/Product', [mdl '/product_omegae_flux'], ...
    'Inputs','**', 'Position',[695 310 730 350]);
add_block('simulink/Math Operations/Sum', [mdl '/sum_uq_pi_plus_feedforward'], ...
    'Inputs','++', 'Position',[760 155 785 185]);
add_block('simulink/Discontinuities/Saturation', [mdl '/sat_uq'], ...
    'UpperLimit','Vdq_max', 'LowerLimit','-Vdq_max', ...
    'Position',[830 155 890 185]);

%% Electrical angle, inverse Park, SVPWM, and inverter
add_block('simulink/Continuous/Integrator', [mdl '/int_theta_e'], ...
    'InitialCondition','theta_e0', 'Position',[650 405 685 435]);

inv_park = [mdl '/inverse_park_udq_to_alpha_beta'];
add_block('simulink/User-Defined Functions/MATLAB Function', inv_park, ...
    'Position',[935 145 1095 255]);
set_matlab_function_script(inv_park, inverse_park_script());

add_block('simulink/Sources/Constant', [mdl '/Vdc'], ...
    'Value','Vdc', 'Position',[935 305 990 335]);
add_block('simulink/Sources/Constant', [mdl '/Ts_pwm'], ...
    'Value','Ts_pwm', 'Position',[935 365 990 395]);
add_block('simulink/Signal Routing/Mux', [mdl '/svpwm_input_mux'], ...
    'Inputs','4', 'Position',[1140 190 1170 350]);
add_block('simulink/User-Defined Functions/S-Function', [mdl '/svpwm_bo1808_sfunc_core'], ...
    'FunctionName','svpwm_bo1808_sfunc', 'Position',[1215 215 1385 330]);
add_block('simulink/Signal Routing/Demux', [mdl '/svpwm_output_demux'], ...
    'Outputs','7', 'Position',[1425 180 1455 360]);

add_block('simulink/Sources/Repeating Sequence', [mdl '/triangular_carrier_time'], ...
    'rep_seq_t','[0 Ts_pwm/2 Ts_pwm]', ...
    'rep_seq_y','[0 Ts_pwm/2 0]', ...
    'Position',[1500 465 1605 505]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [mdl '/compare_duty_a'], ...
    'Operator','<=', 'Position',[1545 205 1585 240]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [mdl '/compare_duty_b'], ...
    'Operator','<=', 'Position',[1545 265 1585 300]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [mdl '/compare_duty_c'], ...
    'Operator','<=', 'Position',[1545 325 1585 360]);
add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/Sa_double'], ...
    'OutDataTypeStr','double', 'Position',[1620 207 1650 237]);
add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/Sb_double'], ...
    'OutDataTypeStr','double', 'Position',[1620 267 1650 297]);
add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/Sc_double'], ...
    'OutDataTypeStr','double', 'Position',[1620 327 1650 357]);

inv_recon = [mdl '/two_level_inverter_reconstruction'];
add_block('simulink/User-Defined Functions/MATLAB Function', inv_recon, ...
    'Position',[1695 235 1875 385]);
set_matlab_function_script(inv_recon, inverter_reconstruction_script());

park = [mdl '/park_alpha_beta_to_dq'];
add_block('simulink/User-Defined Functions/MATLAB Function', park, ...
    'Position',[1925 245 2075 355]);
set_matlab_function_script(park, park_script());

%% BO1808 plant and measured quantity reconstruction
add_block('simulink/Signal Routing/Mux', [mdl '/plant_input_mux'], ...
    'Inputs','3', 'Position',[2130 260 2160 370]);
add_block('simulink/User-Defined Functions/S-Function', [mdl '/pmsm_bo1808_sfunc'], ...
    'FunctionName','pmsm_bo1808', 'Position',[2205 280 2325 340]);
add_block('simulink/Signal Routing/Demux', [mdl '/plant_output_demux'], ...
    'Outputs','3', 'Position',[2370 280 2400 340]);
add_block('simulink/Commonly Used Blocks/Gain', [mdl '/radps_to_rpm'], ...
    'Gain','60/(2*pi)', 'Position',[2450 360 2530 390]);
add_block('simulink/Commonly Used Blocks/Gain', [mdl '/iq_to_Te'], ...
    'Gain','Kt_q', 'Position',[2450 210 2530 240]);

cur_abc = [mdl '/current_dq_to_abc'];
add_block('simulink/User-Defined Functions/MATLAB Function', cur_abc, ...
    'Position',[2450 460 2605 570]);
set_matlab_function_script(cur_abc, current_dq_to_abc_script());

%% Logging
add_block('simulink/Signal Routing/Mux', [mdl '/log_mux'], ...
    'Inputs','24', 'Position',[2660 45 2690 610]);
add_block('simulink/Sinks/To Workspace', [mdl '/simout_foc_svpwm'], ...
    'VariableName','simout_foc_svpwm', ...
    'SaveFormat','StructureWithTime', ...
    'Position',[2740 280 2885 310]);
add_block('simulink/Sinks/Scope', [mdl '/scope_foc_svpwm'], ...
    'Position',[2740 360 2820 410]);

%% Wiring: reference and PI loops
if usePositionLoop
    add_line(mdl,'theta_m_ref_step/1','sum_theta_ref_minus_theta_m/1','autorouting','on');
    add_line(mdl,'sum_theta_ref_minus_theta_m/1','P_position/1','autorouting','on');
    add_line(mdl,'P_position/1','sat_speed_ref_from_position/1','autorouting','on');
    add_line(mdl,'sat_speed_ref_from_position/1','sum_speed_ref_minus_omega/1','autorouting','on');
else
    add_line(mdl,'speed_ref_rpm_step/1','rpm_to_radps/1','autorouting','on');
    add_line(mdl,'rpm_to_radps/1','sum_speed_ref_minus_omega/1','autorouting','on');
end
add_line(mdl,'sum_speed_ref_minus_omega/1','PI_speed/1','autorouting','on');
add_line(mdl,'PI_speed/1','sat_iq_ref/1','autorouting','on');
add_line(mdl,'sat_iq_ref/1','sum_iq_ref_minus_iq/1','autorouting','on');
add_line(mdl,'id_ref_0/1','sum_id_ref_minus_id/1','autorouting','on');
add_line(mdl,'sum_iq_ref_minus_iq/1','PI_iq/1','autorouting','on');
add_line(mdl,'sum_id_ref_minus_id/1','PI_id/1','autorouting','on');

%% Wiring: decoupled d/q voltage command
add_line(mdl,'PI_id/1','sum_ud_pi_minus_decoupling/1','autorouting','on');
add_line(mdl,'product_omegae_Lq_iq/1','sum_ud_pi_minus_decoupling/2','autorouting','on');
add_line(mdl,'sum_ud_pi_minus_decoupling/1','sat_ud/1','autorouting','on');
add_line(mdl,'PI_iq/1','sum_uq_pi_plus_feedforward/1','autorouting','on');
add_line(mdl,'product_omegae_flux/1','sum_uq_pi_plus_feedforward/2','autorouting','on');
add_line(mdl,'sum_uq_pi_plus_feedforward/1','sat_uq/1','autorouting','on');
add_line(mdl,'sat_ud/1','inverse_park_udq_to_alpha_beta/1','autorouting','on');
add_line(mdl,'sat_uq/1','inverse_park_udq_to_alpha_beta/2','autorouting','on');
add_line(mdl,'int_theta_e/1','inverse_park_udq_to_alpha_beta/3','autorouting','on');

%% Wiring: SVPWM and inverter chain
add_line(mdl,'inverse_park_udq_to_alpha_beta/1','svpwm_input_mux/1','autorouting','on');
add_line(mdl,'inverse_park_udq_to_alpha_beta/2','svpwm_input_mux/2','autorouting','on');
add_line(mdl,'Vdc/1','svpwm_input_mux/3','autorouting','on');
add_line(mdl,'Ts_pwm/1','svpwm_input_mux/4','autorouting','on');
add_line(mdl,'svpwm_input_mux/1','svpwm_bo1808_sfunc_core/1','autorouting','on');
add_line(mdl,'svpwm_bo1808_sfunc_core/1','svpwm_output_demux/1','autorouting','on');

add_line(mdl,'svpwm_output_demux/1','compare_duty_a/1','autorouting','on');
add_line(mdl,'svpwm_output_demux/2','compare_duty_b/1','autorouting','on');
add_line(mdl,'svpwm_output_demux/3','compare_duty_c/1','autorouting','on');
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
add_line(mdl,'two_level_inverter_reconstruction/1','park_alpha_beta_to_dq/1','autorouting','on');
add_line(mdl,'two_level_inverter_reconstruction/2','park_alpha_beta_to_dq/2','autorouting','on');
add_line(mdl,'int_theta_e/1','park_alpha_beta_to_dq/3','autorouting','on');

%% Wiring: plant
add_line(mdl,'park_alpha_beta_to_dq/1','plant_input_mux/1','autorouting','on');
add_line(mdl,'park_alpha_beta_to_dq/2','plant_input_mux/2','autorouting','on');
add_line(mdl,'TL_load/1','plant_input_mux/3','autorouting','on');
add_line(mdl,'plant_input_mux/1','pmsm_bo1808_sfunc/1','autorouting','on');
add_line(mdl,'pmsm_bo1808_sfunc/1','plant_output_demux/1','autorouting','on');

% Demux outputs: 1=id, 2=iq, 3=omega_m.
add_line(mdl,'plant_output_demux/1','sum_id_ref_minus_id/2','autorouting','on');
add_line(mdl,'plant_output_demux/2','sum_iq_ref_minus_iq/2','autorouting','on');
add_line(mdl,'plant_output_demux/3','sum_speed_ref_minus_omega/2','autorouting','on');
if usePositionLoop
    add_line(mdl,'plant_output_demux/3','int_theta_m/1','autorouting','on');
    add_line(mdl,'int_theta_m/1','sum_theta_ref_minus_theta_m/2','autorouting','on');
end
add_line(mdl,'plant_output_demux/3','pn_omega_m_to_omega_e/1','autorouting','on');
add_line(mdl,'pn_omega_m_to_omega_e/1','int_theta_e/1','autorouting','on');
add_line(mdl,'plant_output_demux/3','radps_to_rpm/1','autorouting','on');
add_line(mdl,'plant_output_demux/2','iq_to_Te/1','autorouting','on');

%% Wiring: decoupling feedback and current reconstruction
add_line(mdl,'pn_omega_m_to_omega_e/1','product_omegae_Lq_iq/1','autorouting','on');
add_line(mdl,'plant_output_demux/2','gain_Lq_iq/1','autorouting','on');
add_line(mdl,'gain_Lq_iq/1','product_omegae_Lq_iq/2','autorouting','on');
add_line(mdl,'plant_output_demux/1','gain_Ld_id/1','autorouting','on');
add_line(mdl,'gain_Ld_id/1','sum_Ld_id_plus_psi_f/1','autorouting','on');
add_line(mdl,'const_psi_f/1','sum_Ld_id_plus_psi_f/2','autorouting','on');
add_line(mdl,'pn_omega_m_to_omega_e/1','product_omegae_flux/1','autorouting','on');
add_line(mdl,'sum_Ld_id_plus_psi_f/1','product_omegae_flux/2','autorouting','on');

add_line(mdl,'plant_output_demux/1','current_dq_to_abc/1','autorouting','on');
add_line(mdl,'plant_output_demux/2','current_dq_to_abc/2','autorouting','on');
add_line(mdl,'int_theta_e/1','current_dq_to_abc/3','autorouting','on');

%% Logging order
if usePositionLoop
    add_line(mdl,'sat_speed_ref_from_position/1','log_mux/1','autorouting','on');       % omega_ref
else
    add_line(mdl,'rpm_to_radps/1','log_mux/1','autorouting','on');                     % omega_ref
end
add_line(mdl,'plant_output_demux/3','log_mux/2','autorouting','on');                    % omega_m
add_line(mdl,'radps_to_rpm/1','log_mux/3','autorouting','on');                          % Nr
add_line(mdl,'plant_output_demux/1','log_mux/4','autorouting','on');                    % id
add_line(mdl,'plant_output_demux/2','log_mux/5','autorouting','on');                    % iq
add_line(mdl,'sat_iq_ref/1','log_mux/6','autorouting','on');                            % iq_ref
add_line(mdl,'sat_ud/1','log_mux/7','autorouting','on');                                % ud_cmd
add_line(mdl,'sat_uq/1','log_mux/8','autorouting','on');                                % uq_cmd
add_line(mdl,'inverse_park_udq_to_alpha_beta/1','log_mux/9','autorouting','on');         % Ualpha_ref
add_line(mdl,'inverse_park_udq_to_alpha_beta/2','log_mux/10','autorouting','on');        % Ubeta_ref
add_line(mdl,'two_level_inverter_reconstruction/1','log_mux/11','autorouting','on');     % Valpha_sw
add_line(mdl,'two_level_inverter_reconstruction/2','log_mux/12','autorouting','on');     % Vbeta_sw
add_line(mdl,'park_alpha_beta_to_dq/1','log_mux/13','autorouting','on');                 % ud_actual
add_line(mdl,'park_alpha_beta_to_dq/2','log_mux/14','autorouting','on');                 % uq_actual
add_line(mdl,'int_theta_e/1','log_mux/15','autorouting','on');                           % theta_e
add_line(mdl,'iq_to_Te/1','log_mux/16','autorouting','on');                              % Te
add_line(mdl,'TL_load/1','log_mux/17','autorouting','on');                               % TL
add_line(mdl,'current_dq_to_abc/1','log_mux/18','autorouting','on');                     % ia
add_line(mdl,'current_dq_to_abc/2','log_mux/19','autorouting','on');                     % ib
add_line(mdl,'current_dq_to_abc/3','log_mux/20','autorouting','on');                     % ic
add_line(mdl,'svpwm_output_demux/4','log_mux/21','autorouting','on');                    % sector
add_line(mdl,'Sa_double/1','log_mux/22','autorouting','on');                             % Sa
add_line(mdl,'Sb_double/1','log_mux/23','autorouting','on');                             % Sb
add_line(mdl,'Sc_double/1','log_mux/24','autorouting','on');                             % Sc
add_line(mdl,'log_mux/1','simout_foc_svpwm/1','autorouting','on');
add_line(mdl,'log_mux/1','scope_foc_svpwm/1','autorouting','on');

if usePositionLoop
    annotation_text = ['BO1808NBH2B position-servo drive: position P loop + speed PI + current PI ' ...
        '+ inverse Park + SVPWM + two-level inverter + BO1808 d-q plant. ' ...
        'All motor parameters come from init_bo1808_params.m.'];
else
    annotation_text = ['BO1808NBH2B full closed-loop drive: FOC + inverse Park + SVPWM ' ...
        '+ two-level inverter + BO1808 d-q plant. The load source is selected by TL_load_mode, ' ...
        'and all motor parameters come from init_bo1808_params.m.'];
end
add_model_annotation(mdl, annotation_text);

model_file = fullfile(models_dir,[mdl '.slx']);
save_system(mdl,model_file);
fprintf('Created %s\n', model_file);
end

function add_load_source(mdl, mode, noiseEnable)
if strcmpi(mode,'sine')
    add_block('simulink/Sources/Sine Wave', [mdl '/TL_sine_wave'], ...
        'Amplitude','TL_amp_sine', ...
        'Bias','TL_mean_sine', ...
        'Frequency','2*pi*f_load_sine', ...
        'Phase','0', ...
        'SampleTime','0', ...
        'Position',[1110 390 1165 420]);
    add_block('simulink/Sources/Step', [mdl '/TL_sine_enable'], ...
        'Time','TL_sine_start', 'Before','0', 'After','1', ...
        'Position',[1110 450 1165 480]);
    add_block('simulink/Math Operations/Product', [mdl '/TL_load_base'], ...
        'Inputs','**', ...
        'Position',[1210 415 1250 455]);
    add_line(mdl,'TL_sine_wave/1','TL_load_base/1','autorouting','on');
    add_line(mdl,'TL_sine_enable/1','TL_load_base/2','autorouting','on');
else
    add_block('simulink/Sources/Step', [mdl '/TL_load_base'], ...
        'Time','TL_step_time_fig3_9', 'Before','0', 'After','TL_nom', ...
        'Position',[1200 405 1250 435]);
end
if noiseEnable
    add_block('simulink/Sources/Random Number', [mdl '/TL_white_noise'], ...
        'Mean','0', ...
        'Variance','TL_noise_std^2', ...
        'Seed','TL_noise_seed', ...
        'SampleTime','TL_noise_sample_time', ...
        'Position',[1200 505 1265 535]);
    add_block('simulink/Math Operations/Sum', [mdl '/sum_TL_base_plus_noise'], ...
        'Inputs','++', ...
        'Position',[1315 430 1340 470]);
    add_block('simulink/Discontinuities/Saturation', [mdl '/TL_load'], ...
        'LowerLimit','0', ...
        'UpperLimit','inf', ...
        'Position',[1385 430 1435 470]);
    add_line(mdl,'TL_load_base/1','sum_TL_base_plus_noise/1','autorouting','on');
    add_line(mdl,'TL_white_noise/1','sum_TL_base_plus_noise/2','autorouting','on');
    add_line(mdl,'sum_TL_base_plus_noise/1','TL_load/1','autorouting','on');
else
    add_block('simulink/Commonly Used Blocks/Gain', [mdl '/TL_load'], ...
        'Gain','1', ...
        'Position',[1315 420 1365 450]);
    add_line(mdl,'TL_load_base/1','TL_load/1','autorouting','on');
end
end

function build_pi_subsystem(ss, kp_name, ki_name, ic_value)
add_block('simulink/Ports & Subsystems/In1', [ss '/e'], 'Position',[30 85 60 105]);
add_block('simulink/Ports & Subsystems/Out1', [ss '/u'], 'Position',[410 85 440 105]);

add_block('simulink/Commonly Used Blocks/Gain', [ss '/Kp'], ...
    'Gain',kp_name, 'Position',[120 45 180 75]);
add_block('simulink/Commonly Used Blocks/Gain', [ss '/Ki'], ...
    'Gain',ki_name, 'Position',[120 125 180 155]);
add_block('simulink/Continuous/Integrator', [ss '/integrator_1_over_s'], ...
    'InitialCondition',ic_value, 'Position',[225 125 255 155]);
add_block('simulink/Math Operations/Sum', [ss '/sum_P_plus_I'], ...
    'Inputs','++', 'Position',[320 78 345 112]);

add_line(ss,'e/1','Kp/1','autorouting','on');
add_line(ss,'e/1','Ki/1','autorouting','on');
add_line(ss,'Kp/1','sum_P_plus_I/1','autorouting','on');
add_line(ss,'Ki/1','integrator_1_over_s/1','autorouting','on');
add_line(ss,'integrator_1_over_s/1','sum_P_plus_I/2','autorouting','on');
add_line(ss,'sum_P_plus_I/1','u/1','autorouting','on');
end

function txt = inverse_park_script()
txt = [
"function [Ualpha,Ubeta] = fcn(ud,uq,theta_e)"
"%#codegen"
"c = cos(theta_e);"
"s = sin(theta_e);"
"Ualpha = ud*c - uq*s;"
"Ubeta = ud*s + uq*c;"
"end"
];
txt = strjoin(txt,newline);
end

function txt = park_script()
txt = [
"function [ud,uq] = fcn(Ualpha,Ubeta,theta_e)"
"%#codegen"
"c = cos(theta_e);"
"s = sin(theta_e);"
"ud = Ualpha*c + Ubeta*s;"
"uq = -Ualpha*s + Ubeta*c;"
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

function txt = current_dq_to_abc_script()
txt = [
"function [ia,ib,ic] = fcn(id,iq,theta_e)"
"%#codegen"
"c = cos(theta_e);"
"s = sin(theta_e);"
"ialpha = id*c - iq*s;"
"ibeta = id*s + iq*c;"
"ia = ialpha;"
"ib = -0.5*ialpha + sqrt(3)/2*ibeta;"
"ic = -0.5*ialpha - sqrt(3)/2*ibeta;"
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

function set_model_path_callbacks(mdl)
cb = [ ...
    "try, " + ...
    "mf=get_param(bdroot,'FileName'); " + ...
    "if ~isempty(mf), addpath(fileparts(fileparts(mf))); end, " + ...
    "catch, end" ...
    ];
cb = char(cb);
set_param(mdl,'PostLoadFcn',cb,'InitFcn',cb);
end

function clear_subsystem(ss)
try
    Simulink.SubSystem.deleteContents(ss);
catch
    lines = find_system(ss,'FindAll','on','Type','line');
    if ~isempty(lines)
        delete_line(lines);
    end
    blocks = find_system(ss,'SearchDepth',1,'Type','Block');
    blocks = setdiff(blocks,{ss});
    for k = 1:numel(blocks)
        delete_block(blocks{k});
    end
end
end

function put_full_foc_svpwm_vars_in_model_workspace(mdl)
names = { ...
    'pn','Rs','Ld','Lq','psi_f','J','B','Vdc','TL_nom','Vdq_max', ...
    'Iq_max','Id_max','theta_e0','theta_m0','Ts_pwm','Ts_sim_foc_svpwm', ...
    't_stop_foc_svpwm','t_stop_position_loop','speed_ref_fig3_9_rpm','TL_step_time_fig3_9', ...
    'theta_m_ref_step','position_ref_step_time','Kp_pos','omega_ref_pos_max', ...
    'TL_mean_sine','TL_amp_sine','f_load_sine','TL_sine_start','TL_load_mode', ...
    'TL_noise_enable','TL_noise_std','TL_noise_sample_time','TL_noise_seed','foc_control_mode', ...
    'Kp_id','Ki_id','Kp_iq','Ki_iq','Kp_w','Ki_w','Kt_q'};
mws = get_param(mdl,'ModelWorkspace');
for k = 1:numel(names)
    assignin(mws,names{k},evalin('caller',names{k}));
end
end
