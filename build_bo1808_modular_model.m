function mdl = build_bo1808_modular_model()
%BUILD_BO1808_MODULAR_MODEL Create the textbook Fig.1-9 style PMSM model.
%
% Generated model:
%   bo1808_fig1_9_modular.slx
%
% The model contains three explicit subsystems:
%   A_dq_axis_current_calculation
%   B_electromagnetic_torque
%   C_mechanical_speed_angle

init_bo1808_params;

mdl = 'bo1808_fig1_9_modular';
if bdIsLoaded(mdl)
    close_system(mdl,0);
end
new_system(mdl);
open_system(mdl);
put_bo1808_vars_in_model_workspace(mdl);

set_param(mdl, ...
    'StopTime','t_stop_open_loop', ...
    'Solver','ode23tb', ...
    'MaxStep','1e-5', ...
    'SaveOutput','on');

%% Top-level sources
add_block('simulink/Sources/Step', [mdl '/ud_step'], ...
    'Time','0.005', 'Before','0', 'After','ud_step_value', ...
    'Position',[35 95 80 125]);
add_block('simulink/Sources/Step', [mdl '/uq_step'], ...
    'Time','0.005', 'Before','0', 'After','uq_step_value', ...
    'Position',[35 155 80 185]);
add_block('simulink/Sources/Step', [mdl '/TL_step'], ...
    'Time','TL_step_time', 'Before','0', 'After','TL_nom', ...
    'Position',[435 285 480 315]);

add_block('built-in/Subsystem', [mdl '/A_dq_axis_current_calculation'], ...
    'Position',[165 80 355 205]);
add_block('built-in/Subsystem', [mdl '/B_electromagnetic_torque'], ...
    'Position',[450 95 640 185]);
add_block('built-in/Subsystem', [mdl '/C_mechanical_speed_angle'], ...
    'Position',[735 120 925 255]);
clear_subsystem([mdl '/A_dq_axis_current_calculation']);
clear_subsystem([mdl '/B_electromagnetic_torque']);
clear_subsystem([mdl '/C_mechanical_speed_angle']);

build_current_subsystem([mdl '/A_dq_axis_current_calculation']);
build_torque_subsystem([mdl '/B_electromagnetic_torque']);
build_mechanical_subsystem([mdl '/C_mechanical_speed_angle']);

add_block('simulink/Commonly Used Blocks/Gain', [mdl '/pn_omega_m_to_omega_e'], ...
    'Gain','pn', 'Position',[1010 175 1060 205]);

add_block('simulink/Signal Routing/Mux', [mdl '/plant_signal_mux'], ...
    'Inputs','7', 'Position',[1030 315 1060 455]);
add_block('simulink/Sinks/To Workspace', [mdl '/simout_open_loop'], ...
    'VariableName','simout_open_loop', ...
    'SaveFormat','StructureWithTime', ...
    'Position',[1115 360 1225 390]);
add_block('simulink/Sinks/Scope', [mdl '/scope_open_loop'], ...
    'Position',[1115 415 1190 465]);

%% Top-level wiring
add_line(mdl,'ud_step/1','A_dq_axis_current_calculation/1','autorouting','on');
add_line(mdl,'uq_step/1','A_dq_axis_current_calculation/2','autorouting','on');
add_line(mdl,'pn_omega_m_to_omega_e/1','A_dq_axis_current_calculation/3','autorouting','on');

add_line(mdl,'A_dq_axis_current_calculation/1','B_electromagnetic_torque/1','autorouting','on');
add_line(mdl,'A_dq_axis_current_calculation/2','B_electromagnetic_torque/2','autorouting','on');

add_line(mdl,'B_electromagnetic_torque/1','C_mechanical_speed_angle/1','autorouting','on');
add_line(mdl,'TL_step/1','C_mechanical_speed_angle/2','autorouting','on');
add_line(mdl,'C_mechanical_speed_angle/1','pn_omega_m_to_omega_e/1','autorouting','on');

add_line(mdl,'A_dq_axis_current_calculation/1','plant_signal_mux/1','autorouting','on');
add_line(mdl,'A_dq_axis_current_calculation/2','plant_signal_mux/2','autorouting','on');
add_line(mdl,'B_electromagnetic_torque/1','plant_signal_mux/3','autorouting','on');
add_line(mdl,'C_mechanical_speed_angle/1','plant_signal_mux/4','autorouting','on');
add_line(mdl,'C_mechanical_speed_angle/2','plant_signal_mux/5','autorouting','on');
add_line(mdl,'C_mechanical_speed_angle/3','plant_signal_mux/6','autorouting','on');
add_line(mdl,'pn_omega_m_to_omega_e/1','plant_signal_mux/7','autorouting','on');
add_line(mdl,'plant_signal_mux/1','simout_open_loop/1','autorouting','on');
add_line(mdl,'plant_signal_mux/1','scope_open_loop/1','autorouting','on');

add_block('simulink/Ports & Subsystems/Out1', [mdl '/id'], ...
    'Position',[1115 75 1145 95]);
add_block('simulink/Ports & Subsystems/Out1', [mdl '/iq'], ...
    'Position',[1115 110 1145 130]);
add_block('simulink/Ports & Subsystems/Out1', [mdl '/omega_m'], ...
    'Position',[1115 145 1145 165]);
add_block('simulink/Ports & Subsystems/Out1', [mdl '/theta_e'], ...
    'Position',[1115 215 1145 235]);
add_line(mdl,'A_dq_axis_current_calculation/1','id/1','autorouting','on');
add_line(mdl,'A_dq_axis_current_calculation/2','iq/1','autorouting','on');
add_line(mdl,'C_mechanical_speed_angle/1','omega_m/1','autorouting','on');
add_line(mdl,'C_mechanical_speed_angle/2','theta_e/1','autorouting','on');

add_model_annotation(mdl, ...
    ['BO1808NBH2B d-q modular PMSM model, textbook Fig.1-9 style. ' ...
     'Equations: did/dt=(ud-Rs*id+omega_e*Lq*iq)/Ld; ' ...
     'diq/dt=(uq-Rs*iq-omega_e*(Ld*id+psi_f))/Lq; ' ...
     'J*domega_m/dt=Te-B*omega_m-TL.']);

model_file = fullfile(models_dir,[mdl '.slx']);
save_system(mdl,model_file);
fprintf('Created %s\n', model_file);
end

function build_current_subsystem(ss)
add_block('simulink/Ports & Subsystems/In1', [ss '/ud'], 'Position',[25 55 55 75]);
add_block('simulink/Ports & Subsystems/In1', [ss '/uq'], 'Position',[25 215 55 235]);
add_block('simulink/Ports & Subsystems/In1', [ss '/omega_e'], 'Position',[25 135 55 155]);
add_block('simulink/Ports & Subsystems/Out1', [ss '/id'], 'Position',[675 65 705 85]);
add_block('simulink/Ports & Subsystems/Out1', [ss '/iq'], 'Position',[675 225 705 245]);

% d-axis current: did/dt = (ud - Rs*id + omega_e*Lq*iq)/Ld
add_block('simulink/Math Operations/Sum', [ss '/sum_d_ud_minus_Rsid_plus_coupling'], ...
    'Inputs','+-+', 'Position',[210 46 235 84]);
add_block('simulink/Commonly Used Blocks/Gain', [ss '/gain_1_over_Ld'], ...
    'Gain','1/Ld', 'Position',[280 50 340 80]);
add_block('simulink/Continuous/Integrator', [ss '/int_id_1_over_s'], ...
    'InitialCondition','id0', 'Position',[385 50 415 80]);
add_block('simulink/Commonly Used Blocks/Gain', [ss '/gain_Rs_for_id'], ...
    'Gain','Rs', 'Position',[485 105 535 135]);
add_block('simulink/Commonly Used Blocks/Gain', [ss '/gain_Lq_for_iq'], ...
    'Gain','Lq', 'Position',[105 310 155 340]);
add_block('simulink/Math Operations/Product', [ss '/product_omegae_Lq_iq'], ...
    'Inputs','**', 'Position',[170 125 205 165]);

% q-axis current: diq/dt = (uq - Rs*iq - omega_e*(Ld*id + psi_f))/Lq
add_block('simulink/Math Operations/Sum', [ss '/sum_q_uq_minus_Rsiq_minus_emf'], ...
    'Inputs','+--', 'Position',[210 206 235 244]);
add_block('simulink/Commonly Used Blocks/Gain', [ss '/gain_1_over_Lq'], ...
    'Gain','1/Lq', 'Position',[280 210 340 240]);
add_block('simulink/Continuous/Integrator', [ss '/int_iq_1_over_s'], ...
    'InitialCondition','iq0', 'Position',[385 210 415 240]);
add_block('simulink/Commonly Used Blocks/Gain', [ss '/gain_Rs_for_iq'], ...
    'Gain','Rs', 'Position',[485 260 535 290]);
add_block('simulink/Commonly Used Blocks/Gain', [ss '/gain_Ld_for_id'], ...
    'Gain','Ld', 'Position',[485 155 535 185]);
add_block('simulink/Sources/Constant', [ss '/const_psi_f'], ...
    'Value','psi_f', 'Position',[485 200 535 230]);
add_block('simulink/Math Operations/Sum', [ss '/sum_Ld_id_plus_psi_f'], ...
    'Inputs','++', 'Position',[570 172 595 218]);
add_block('simulink/Math Operations/Product', [ss '/product_omegae_flux_q'], ...
    'Inputs','**', 'Position',[625 158 660 198]);

add_line(ss,'ud/1','sum_d_ud_minus_Rsid_plus_coupling/1','autorouting','on');
add_line(ss,'sum_d_ud_minus_Rsid_plus_coupling/1','gain_1_over_Ld/1','autorouting','on');
add_line(ss,'gain_1_over_Ld/1','int_id_1_over_s/1','autorouting','on');
add_line(ss,'int_id_1_over_s/1','id/1','autorouting','on');
add_line(ss,'int_id_1_over_s/1','gain_Rs_for_id/1','autorouting','on');
add_line(ss,'gain_Rs_for_id/1','sum_d_ud_minus_Rsid_plus_coupling/2','autorouting','on');

add_line(ss,'omega_e/1','product_omegae_Lq_iq/1','autorouting','on');
add_line(ss,'int_iq_1_over_s/1','gain_Lq_for_iq/1','autorouting','on');
add_line(ss,'gain_Lq_for_iq/1','product_omegae_Lq_iq/2','autorouting','on');
add_line(ss,'product_omegae_Lq_iq/1','sum_d_ud_minus_Rsid_plus_coupling/3','autorouting','on');

add_line(ss,'uq/1','sum_q_uq_minus_Rsiq_minus_emf/1','autorouting','on');
add_line(ss,'sum_q_uq_minus_Rsiq_minus_emf/1','gain_1_over_Lq/1','autorouting','on');
add_line(ss,'gain_1_over_Lq/1','int_iq_1_over_s/1','autorouting','on');
add_line(ss,'int_iq_1_over_s/1','iq/1','autorouting','on');
add_line(ss,'int_iq_1_over_s/1','gain_Rs_for_iq/1','autorouting','on');
add_line(ss,'gain_Rs_for_iq/1','sum_q_uq_minus_Rsiq_minus_emf/2','autorouting','on');

add_line(ss,'int_id_1_over_s/1','gain_Ld_for_id/1','autorouting','on');
add_line(ss,'gain_Ld_for_id/1','sum_Ld_id_plus_psi_f/1','autorouting','on');
add_line(ss,'const_psi_f/1','sum_Ld_id_plus_psi_f/2','autorouting','on');
add_line(ss,'omega_e/1','product_omegae_flux_q/1','autorouting','on');
add_line(ss,'sum_Ld_id_plus_psi_f/1','product_omegae_flux_q/2','autorouting','on');
add_line(ss,'product_omegae_flux_q/1','sum_q_uq_minus_Rsiq_minus_emf/3','autorouting','on');

add_model_annotation(ss, 'Gain labels: 1/Ld, 1/Lq, Rs, Lq, Ld, psi_f.');
end

function build_torque_subsystem(ss)
add_block('simulink/Ports & Subsystems/In1', [ss '/id'], 'Position',[35 55 65 75]);
add_block('simulink/Ports & Subsystems/In1', [ss '/iq'], 'Position',[35 125 65 145]);
add_block('simulink/Ports & Subsystems/Out1', [ss '/Te'], 'Position',[455 118 485 138]);

add_block('simulink/Sources/Constant', [ss '/const_psi_f'], ...
    'Value','psi_f', 'Position',[120 75 170 105]);
add_block('simulink/Math Operations/Product', [ss '/product_psi_f_iq'], ...
    'Inputs','**', 'Position',[230 105 265 145]);
add_block('simulink/Commonly Used Blocks/Gain', [ss '/gain_3_over_2_pn'], ...
    'Gain','1.5*pn', 'Position',[315 112 385 142]);
add_block('simulink/Sinks/Terminator', [ss '/id_terminated_Ld_equals_Lq'], ...
    'Position',[130 48 150 82]);

add_line(ss,'id/1','id_terminated_Ld_equals_Lq/1','autorouting','on');
add_line(ss,'const_psi_f/1','product_psi_f_iq/1','autorouting','on');
add_line(ss,'iq/1','product_psi_f_iq/2','autorouting','on');
add_line(ss,'product_psi_f_iq/1','gain_3_over_2_pn/1','autorouting','on');
add_line(ss,'gain_3_over_2_pn/1','Te/1','autorouting','on');

add_model_annotation(ss, 'Surface PMSM: Te = (3/2)*pn*psi_f*iq because Ld = Lq and id is controlled to 0.');
end

function build_mechanical_subsystem(ss)
add_block('simulink/Ports & Subsystems/In1', [ss '/Te'], 'Position',[35 65 65 85]);
add_block('simulink/Ports & Subsystems/In1', [ss '/TL'], 'Position',[35 165 65 185]);
add_block('simulink/Ports & Subsystems/Out1', [ss '/omega_m'], 'Position',[610 70 640 90]);
add_block('simulink/Ports & Subsystems/Out1', [ss '/theta_e'], 'Position',[610 155 640 175]);
add_block('simulink/Ports & Subsystems/Out1', [ss '/Nr_rpm'], 'Position',[610 240 640 260]);

add_block('simulink/Math Operations/Sum', [ss '/sum_Te_minus_Bomega_minus_TL'], ...
    'Inputs','+--', 'Position',[170 75 195 125]);
add_block('simulink/Commonly Used Blocks/Gain', [ss '/gain_1_over_J'], ...
    'Gain','1/J', 'Position',[245 85 305 115]);
add_block('simulink/Continuous/Integrator', [ss '/int_omega_m_1_over_s'], ...
    'InitialCondition','omega_m0', 'Position',[355 85 385 115]);
add_block('simulink/Discrete/Memory', [ss '/memory_omega_m_feedback'], ...
    'Position',[355 175 385 205]);
add_block('simulink/Commonly Used Blocks/Gain', [ss '/gain_B'], ...
    'Gain','B', 'Position',[245 175 305 205]);
add_block('simulink/Commonly Used Blocks/Gain', [ss '/gain_pn_for_theta_e'], ...
    'Gain','pn', 'Position',[430 145 480 175]);
add_block('simulink/Continuous/Integrator', [ss '/int_theta_e_1_over_s'], ...
    'InitialCondition','theta_e0', 'Position',[520 145 550 175]);
add_block('simulink/Commonly Used Blocks/Gain', [ss '/gain_radps_to_rpm'], ...
    'Gain','60/(2*pi)', 'Position',[430 235 520 265]);

add_line(ss,'Te/1','sum_Te_minus_Bomega_minus_TL/1','autorouting','on');
add_line(ss,'TL/1','sum_Te_minus_Bomega_minus_TL/3','autorouting','on');
add_line(ss,'sum_Te_minus_Bomega_minus_TL/1','gain_1_over_J/1','autorouting','on');
add_line(ss,'gain_1_over_J/1','int_omega_m_1_over_s/1','autorouting','on');
add_line(ss,'int_omega_m_1_over_s/1','omega_m/1','autorouting','on');

add_line(ss,'int_omega_m_1_over_s/1','memory_omega_m_feedback/1','autorouting','on');
add_line(ss,'memory_omega_m_feedback/1','gain_B/1','autorouting','on');
add_line(ss,'gain_B/1','sum_Te_minus_Bomega_minus_TL/2','autorouting','on');

add_line(ss,'int_omega_m_1_over_s/1','gain_pn_for_theta_e/1','autorouting','on');
add_line(ss,'gain_pn_for_theta_e/1','int_theta_e_1_over_s/1','autorouting','on');
add_line(ss,'int_theta_e_1_over_s/1','theta_e/1','autorouting','on');

add_line(ss,'int_omega_m_1_over_s/1','gain_radps_to_rpm/1','autorouting','on');
add_line(ss,'gain_radps_to_rpm/1','Nr_rpm/1','autorouting','on');

add_model_annotation(ss, 'Mechanical part: d(omega_m)/dt = (Te - B*omega_m - TL)/J; theta_e = integral(pn*omega_m).');
end

function add_model_annotation(sys, txt)
try
    Simulink.Annotation(sys, txt);
catch
    % Annotation support differs slightly across MATLAB releases.
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
    'Iq_max','Id_max','id0','iq0','omega_m0','theta_e0', ...
    't_stop_open_loop','ud_step_value','uq_step_value','TL_step_time'};
mws = get_param(mdl,'ModelWorkspace');
for k = 1:numel(names)
    assignin(mws,names{k},evalin('caller',names{k}));
end
end
