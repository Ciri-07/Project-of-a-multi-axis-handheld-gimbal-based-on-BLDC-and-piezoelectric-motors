%% Validate the full BO1808 FOC + SVPWM + inverter closed-loop model
%
% Baseline validation script for the BO1808NBH2B drive:
%   speed PI -> iq*
%   id* = 0
%   d/q current PI + decoupling -> ud/uq
%   inverse Park -> u_alpha/u_beta
%   SVPWM -> inverter reconstruction
%   BO1808 d-q plant
%
% The load is the rated-load step defined in init_bo1808_params.m.

clear
clc

init_bo1808_params;
TL_load_mode = 'step';

mdl = 'bo1808_foc_svpwm_bo1808';
if bdIsLoaded(mdl)
    close_system(mdl,0);
end
build_bo1808_foc_svpwm_model(TL_load_mode);

load_system(mdl);
set_param(mdl,'StopTime','t_stop_foc_svpwm');

simOut = sim(mdl,'ReturnWorkspaceOutputs','on');
simout_foc_svpwm = simOut.get('simout_foc_svpwm');

t = simout_foc_svpwm.time;
y = simout_foc_svpwm.signals.values;

omega_ref = y(:,1);
omega_m = y(:,2);
Nr = y(:,3);
id = y(:,4);
iq = y(:,5);
iq_ref = y(:,6);
ud_cmd = y(:,7);
uq_cmd = y(:,8);
Ualpha_ref = y(:,9);
Ubeta_ref = y(:,10);
Valpha_sw = y(:,11);
Vbeta_sw = y(:,12);
Te = y(:,16);
TL = y(:,17);
ia = y(:,18);
ib = y(:,19);
ic = y(:,20);
sector = y(:,21);
Sa = y(:,22);
Sb = y(:,23);
Sc = y(:,24);

Nref = omega_ref*60/(2*pi);
TL_event_time = TL_step_time_fig3_9;

idx_pre = t > max(0,TL_event_time - 0.035) & t < TL_event_time - 0.005;
idx_post = t > max(0,t_stop_foc_svpwm - 0.035);
if ~any(idx_pre)
    idx_pre = t > 0.04 & t < TL_event_time;
end
if ~any(idx_post)
    idx_post = t > 0.8*t_stop_foc_svpwm;
end

pre_speed_mean = mean(Nr(idx_pre));
post_speed_mean = mean(Nr(idx_post));
post_speed_err = mean(Nref(idx_post) - Nr(idx_post));
post_id_mean = mean(id(idx_post));
post_iq_mean = mean(iq(idx_post));
post_te_mean = mean(Te(idx_post));
phase_current_rms_post = sqrt(mean(ia(idx_post).^2));
sector_set = unique(round(sector(:))).';

fprintf('\nBO1808 full FOC + inverse Park + SVPWM + inverter validation\n');
fprintf('Model: %s\n', fullfile(models_dir,[mdl '.slx']));
fprintf('Speed reference: %.4g rpm\n', speed_ref_fig3_9_rpm);
fprintf('Load mode: step, %.4g N*m at %.4g s\n', TL_nom, TL_step_time_fig3_9);
fprintf('PWM period: %.4g s, solver max step: %.4g s\n', Ts_pwm, Ts_sim_foc_svpwm);
fprintf('Pre-load average speed: %.4g rpm\n', pre_speed_mean);
fprintf('Post-load average speed: %.4g rpm, average speed error: %.4g rpm\n', ...
    post_speed_mean, post_speed_err);
fprintf('Post-load id mean: %.4g A, iq mean: %.4g A\n', post_id_mean, post_iq_mean);
fprintf('Post-load Te mean: %.4g N*m, TL mean: %.4g N*m\n', ...
    post_te_mean, mean(TL(idx_post)));
fprintf('Post-load A-phase current RMS: %.4g A\n', phase_current_rms_post);
fprintf('Voltage command max |ud,uq|: %.4g V, %.4g V\n', max(abs(ud_cmd)), max(abs(uq_cmd)));
fprintf('Inverter alpha-beta voltage max |Valpha,Vbeta|: %.4g V, %.4g V\n', ...
    max(abs(Valpha_sw)), max(abs(Vbeta_sw)));
fprintf('SVPWM sector set: %s\n', mat2str(sector_set));
fprintf('Switch values: Sa=%s, Sb=%s, Sc=%s\n\n', ...
    mat2str(unique(Sa).'), mat2str(unique(Sb).'), mat2str(unique(Sc).'));

%% Fig.3-9-style result figure
fig = figure('Name','BO1808 Full FOC SVPWM Closed-Loop Validation','Color','w');

subplot(3,1,1)
plot(t,Nref,'k--','LineWidth',1.0)
hold on
plot(t,Nr,'b','LineWidth',1.2)
xline(TL_event_time,'r--','Load step','LabelVerticalAlignment','bottom')
grid on
xlabel('Time / s')
ylabel('Speed / rpm')
title('(a) Speed response')
legend('N^*','N','Location','best')

subplot(3,1,2)
plot(t,Te*1e3,'b','LineWidth',1.1)
hold on
plot(t,TL*1e3,'r--','LineWidth',1.0)
xline(TL_event_time,'r--','Load step','HandleVisibility','off')
grid on
xlabel('Time / s')
ylabel('Torque / mN*m')
title('(b) Electromagnetic torque and load torque')
legend('T_e','T_L','Location','best')

subplot(3,1,3)
plot(t,ia,'b','LineWidth',0.9)
hold on
plot(t,ib,'r','LineWidth',0.9)
plot(t,ic,'g','LineWidth',0.9)
xline(TL_event_time,'r--','Load step','HandleVisibility','off')
grid on
xlabel('Time / s')
ylabel('Current / A')
title('(c) Three-phase stator currents reconstructed from d-q current')
legend('i_a','i_b','i_c','Location','best')

fig_file = fullfile(figures_dir,'bo1808_fig3_9_foc_svpwm_validation.png');
saveas(fig,fig_file);
fprintf('Saved plot: %s\n', fig_file);

%% Extra diagnostic figure for the complete chain
fig_diag = figure('Name','BO1808 Full FOC SVPWM Diagnostics','Color','w');

subplot(4,1,1)
plot(t,id,'b','LineWidth',1.0)
hold on
plot(t,iq,'r','LineWidth',1.0)
plot(t,iq_ref,'k--','LineWidth',1.0)
xline(TL_event_time,'r--','Load step','HandleVisibility','off')
grid on
xlabel('Time / s')
ylabel('Current / A')
title('(a) d-q current tracking')
legend('i_d','i_q','i_q^*','Location','best')

subplot(4,1,2)
plot(t,ud_cmd,'b','LineWidth',1.0)
hold on
plot(t,uq_cmd,'r','LineWidth',1.0)
grid on
xlabel('Time / s')
ylabel('Voltage / V')
title('(b) FOC d-q voltage commands before inverse Park')
legend('u_d^*','u_q^*','Location','best')

subplot(4,1,3)
idx_zoom = t <= min(t_stop_foc_svpwm,0.03);
plot(t(idx_zoom),Ualpha_ref(idx_zoom),'b','LineWidth',1.0)
hold on
plot(t(idx_zoom),Ubeta_ref(idx_zoom),'r','LineWidth',1.0)
grid on
xlabel('Time / s')
ylabel('Voltage / V')
title('(c) Inverse-Park alpha-beta voltage reference, startup zoom')
legend('u_\alpha^*','u_\beta^*','Location','best')

subplot(4,1,4)
stairs(t(idx_zoom),sector(idx_zoom),'k','LineWidth',1.0)
grid on
xlabel('Time / s')
ylabel('N')
title('(d) SVPWM sector sequence in the full closed loop')
ylim([0.5 6.5])

fig_diag_file = fullfile(figures_dir,'bo1808_fig3_9_foc_svpwm_diagnostics.png');
saveas(fig_diag,fig_diag_file);
fprintf('Saved plot: %s\n', fig_diag_file);

set_param(mdl,'Dirty','off');
