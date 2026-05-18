function mdl = build_bo1808_foc_id0_model()
%BUILD_BO1808_FOC_ID0_MODEL Create an id=0 closed-loop vector-control model.
%
% Generated model:
%   bo1808_foc_id0.slx
%
% This is a d-q equivalent vector-control simulation:
%   speed PI -> iq*;
%   id* = 0;
%   d/q current PI loops;
%   d/q voltage decoupling feedforward;
%   BO1808 PMSM Level-1 S-Function plant.

init_bo1808_params;

mdl = 'bo1808_foc_id0';
if bdIsLoaded(mdl)
    close_system(mdl,0);
end
new_system(mdl);
open_system(mdl);
put_bo1808_vars_in_model_workspace(mdl);

set_param(mdl, ...
    'StopTime','t_stop_foc', ...
    'Solver','ode23tb', ...
    'MaxStep','1e-5', ...
    'SaveOutput','on');

%% Reference and load
add_block('simulink/Sources/Step', [mdl '/speed_ref_rpm_step'], ...
    'Time','0.005', 'Before','0', 'After','speed_ref_rpm', ...
    'Position',[30 70 75 100]);
add_block('simulink/Commonly Used Blocks/Gain', [mdl '/rpm_to_radps'], ...
    'Gain','2*pi/60', 'Position',[115 70 180 100]);
add_block('simulink/Sources/Constant', [mdl '/id_ref_0'], ...
    'Value','0', 'Position',[30 245 75 275]);
add_block('simulink/Sources/Step', [mdl '/TL_step'], ...
    'Time','TL_step_time', 'Before','0', 'After','TL_nom', ...
    'Position',[700 375 745 405]);

%% Speed loop
add_block('simulink/Math Operations/Sum', [mdl '/sum_speed_ref_minus_omega'], ...
    'Inputs','+-', 'Position',[225 70 250 100]);
add_block('built-in/Subsystem', [mdl '/PI_speed'], 'Position',[295 58 415 112]);
clear_subsystem([mdl '/PI_speed']);
build_pi_subsystem([mdl '/PI_speed'], 'Kp_w', 'Ki_w', '0');
add_block('simulink/Discontinuities/Saturation', [mdl '/sat_iq_ref'], ...
    'UpperLimit','Iq_max', 'LowerLimit','-Iq_max', ...
    'Position',[455 70 515 100]);

%% Current loops
add_block('simulink/Math Operations/Sum', [mdl '/sum_id_ref_minus_id'], ...
    'Inputs','+-', 'Position',[225 245 250 275]);
add_block('simulink/Math Operations/Sum', [mdl '/sum_iq_ref_minus_iq'], ...
    'Inputs','+-', 'Position',[225 155 250 185]);

add_block('built-in/Subsystem', [mdl '/PI_id'], 'Position',[295 232 415 286]);
clear_subsystem([mdl '/PI_id']);
build_pi_subsystem([mdl '/PI_id'], 'Kp_id', 'Ki_id', '0');
add_block('built-in/Subsystem', [mdl '/PI_iq'], 'Position',[295 142 415 196]);
clear_subsystem([mdl '/PI_iq']);
build_pi_subsystem([mdl '/PI_iq'], 'Kp_iq', 'Ki_iq', '0');

%% Decoupling and voltage saturation
add_block('simulink/Commonly Used Blocks/Gain', [mdl '/pn_omega_m_to_omega_e'], ...
    'Gain','pn', 'Position',[750 185 805 215]);

add_block('simulink/Commonly Used Blocks/Gain', [mdl '/gain_Lq_iq'], ...
    'Gain','Lq', 'Position',[535 225 590 255]);
add_block('simulink/Math Operations/Product', [mdl '/product_omegae_Lq_iq'], ...
    'Inputs','**', 'Position',[625 214 660 254]);
add_block('simulink/Math Operations/Sum', [mdl '/sum_ud_pi_minus_decoupling'], ...
    'Inputs','+-', 'Position',[690 235 715 265]);
add_block('simulink/Discontinuities/Saturation', [mdl '/sat_ud'], ...
    'UpperLimit','Vdq_max', 'LowerLimit','-Vdq_max', ...
    'Position',[755 235 815 265]);

add_block('simulink/Commonly Used Blocks/Gain', [mdl '/gain_Ld_id'], ...
    'Gain','Ld', 'Position',[535 300 590 330]);
add_block('simulink/Sources/Constant', [mdl '/const_psi_f'], ...
    'Value','psi_f', 'Position',[535 342 590 372]);
add_block('simulink/Math Operations/Sum', [mdl '/sum_Ld_id_plus_psi_f'], ...
    'Inputs','++', 'Position',[625 307 650 360]);
add_block('simulink/Math Operations/Product', [mdl '/product_omegae_flux'], ...
    'Inputs','**', 'Position',[690 310 725 350]);
add_block('simulink/Math Operations/Sum', [mdl '/sum_uq_pi_plus_feedforward'], ...
    'Inputs','++', 'Position',[760 155 785 185]);
add_block('simulink/Discontinuities/Saturation', [mdl '/sat_uq'], ...
    'UpperLimit','Vdq_max', 'LowerLimit','-Vdq_max', ...
    'Position',[830 155 890 185]);

%% S-Function plant
add_block('simulink/Signal Routing/Mux', [mdl '/plant_input_mux'], ...
    'Inputs','3', 'Position',[930 180 960 250]);
add_block('simulink/User-Defined Functions/S-Function', [mdl '/pmsm_bo1808_sfunc'], ...
    'FunctionName','pmsm_bo1808', 'Position',[1005 185 1125 245]);
add_block('simulink/Signal Routing/Demux', [mdl '/plant_output_demux'], ...
    'Outputs','3', 'Position',[1170 185 1200 245]);

add_block('simulink/Commonly Used Blocks/Gain', [mdl '/radps_to_rpm'], ...
    'Gain','60/(2*pi)', 'Position',[1265 275 1345 305]);
add_block('simulink/Continuous/Integrator', [mdl '/int_theta_e'], ...
    'InitialCondition','theta_e0', 'Position',[900 285 930 315]);

%% Logging
add_block('simulink/Signal Routing/Mux', [mdl '/log_mux'], ...
    'Inputs','10', 'Position',[1390 55 1420 285]);
add_block('simulink/Sinks/To Workspace', [mdl '/simout_foc'], ...
    'VariableName','simout_foc', ...
    'SaveFormat','StructureWithTime', ...
    'Position',[1465 135 1570 165]);
add_block('simulink/Sinks/Scope', [mdl '/scope_foc'], ...
    'Position',[1465 205 1540 255]);

%% Wiring: reference and speed PI
add_line(mdl,'speed_ref_rpm_step/1','rpm_to_radps/1','autorouting','on');
add_line(mdl,'rpm_to_radps/1','sum_speed_ref_minus_omega/1','autorouting','on');
add_line(mdl,'sum_speed_ref_minus_omega/1','PI_speed/1','autorouting','on');
add_line(mdl,'PI_speed/1','sat_iq_ref/1','autorouting','on');

%% Wiring: current PI
add_line(mdl,'id_ref_0/1','sum_id_ref_minus_id/1','autorouting','on');
add_line(mdl,'sat_iq_ref/1','sum_iq_ref_minus_iq/1','autorouting','on');
add_line(mdl,'sum_id_ref_minus_id/1','PI_id/1','autorouting','on');
add_line(mdl,'sum_iq_ref_minus_iq/1','PI_iq/1','autorouting','on');

%% Wiring: d-axis voltage command
add_line(mdl,'PI_id/1','sum_ud_pi_minus_decoupling/1','autorouting','on');
add_line(mdl,'product_omegae_Lq_iq/1','sum_ud_pi_minus_decoupling/2','autorouting','on');
add_line(mdl,'sum_ud_pi_minus_decoupling/1','sat_ud/1','autorouting','on');

%% Wiring: q-axis voltage command
add_line(mdl,'PI_iq/1','sum_uq_pi_plus_feedforward/1','autorouting','on');
add_line(mdl,'product_omegae_flux/1','sum_uq_pi_plus_feedforward/2','autorouting','on');
add_line(mdl,'sum_uq_pi_plus_feedforward/1','sat_uq/1','autorouting','on');

%% Wiring: plant input/output
add_line(mdl,'sat_ud/1','plant_input_mux/1','autorouting','on');
add_line(mdl,'sat_uq/1','plant_input_mux/2','autorouting','on');
add_line(mdl,'TL_step/1','plant_input_mux/3','autorouting','on');
add_line(mdl,'plant_input_mux/1','pmsm_bo1808_sfunc/1','autorouting','on');
add_line(mdl,'pmsm_bo1808_sfunc/1','plant_output_demux/1','autorouting','on');

% Demux outputs: 1=id, 2=iq, 3=omega_m.
add_line(mdl,'plant_output_demux/1','sum_id_ref_minus_id/2','autorouting','on');
add_line(mdl,'plant_output_demux/2','sum_iq_ref_minus_iq/2','autorouting','on');
add_line(mdl,'plant_output_demux/3','sum_speed_ref_minus_omega/2','autorouting','on');
add_line(mdl,'plant_output_demux/3','pn_omega_m_to_omega_e/1','autorouting','on');
add_line(mdl,'plant_output_demux/3','radps_to_rpm/1','autorouting','on');

%% Wiring: decoupling feedback
add_line(mdl,'pn_omega_m_to_omega_e/1','product_omegae_Lq_iq/1','autorouting','on');
add_line(mdl,'plant_output_demux/2','gain_Lq_iq/1','autorouting','on');
add_line(mdl,'gain_Lq_iq/1','product_omegae_Lq_iq/2','autorouting','on');

add_line(mdl,'plant_output_demux/1','gain_Ld_id/1','autorouting','on');
add_line(mdl,'gain_Ld_id/1','sum_Ld_id_plus_psi_f/1','autorouting','on');
add_line(mdl,'const_psi_f/1','sum_Ld_id_plus_psi_f/2','autorouting','on');
add_line(mdl,'pn_omega_m_to_omega_e/1','product_omegae_flux/1','autorouting','on');
add_line(mdl,'sum_Ld_id_plus_psi_f/1','product_omegae_flux/2','autorouting','on');
add_line(mdl,'pn_omega_m_to_omega_e/1','int_theta_e/1','autorouting','on');

%% Logging signals
add_line(mdl,'rpm_to_radps/1','log_mux/1','autorouting','on');
add_line(mdl,'plant_output_demux/3','log_mux/2','autorouting','on');
add_line(mdl,'radps_to_rpm/1','log_mux/3','autorouting','on');
add_line(mdl,'plant_output_demux/1','log_mux/4','autorouting','on');
add_line(mdl,'plant_output_demux/2','log_mux/5','autorouting','on');
add_line(mdl,'sat_iq_ref/1','log_mux/6','autorouting','on');
add_line(mdl,'sat_ud/1','log_mux/7','autorouting','on');
add_line(mdl,'sat_uq/1','log_mux/8','autorouting','on');
add_line(mdl,'TL_step/1','log_mux/9','autorouting','on');
add_line(mdl,'int_theta_e/1','log_mux/10','autorouting','on');
add_line(mdl,'log_mux/1','simout_foc/1','autorouting','on');
add_line(mdl,'log_mux/1','scope_foc/1','autorouting','on');

add_model_annotation(mdl, ...
    ['BO1808NBH2B id=0 vector-control model. ' ...
     'ud = ud_PI - omega_e*Lq*iq; ' ...
     'uq = uq_PI + omega_e*(Ld*id + psi_f); ' ...
     'plant is pmsm_bo1808 Level-1 S-Function.']);

model_file = fullfile(models_dir,[mdl '.slx']);
save_system(mdl,model_file);
fprintf('Created %s\n', model_file);
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

function add_model_annotation(sys, txt)
try
    Simulink.Annotation(sys, txt);
catch
end
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

function put_bo1808_vars_in_model_workspace(mdl)
names = { ...
    'pn','Rs','Ld','Lq','psi_f','J','B','Vdc','TL_nom','Vdq_max', ...
    'Iq_max','Id_max','theta_e0','t_stop_foc','speed_ref_rpm', ...
    'TL_step_time','Kp_id','Ki_id','Kp_iq','Ki_iq','Kp_w','Ki_w'};
mws = get_param(mdl,'ModelWorkspace');
for k = 1:numel(names)
    assignin(mws,names{k},evalin('caller',names{k}));
end
end
