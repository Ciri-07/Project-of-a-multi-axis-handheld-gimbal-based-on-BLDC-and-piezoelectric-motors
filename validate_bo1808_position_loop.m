%% Validate BO1808 position loop with full FOC + SVPWM drive
%
% This script keeps validate_bo1808_fig3_9.m as the speed-loop baseline and
% uses the position-loop branch already implemented in
% build_bo1808_foc_svpwm_model.m.

clear
clc

init_bo1808_params;
TL_load_mode = 'step';
controlMode = 'position';

mdl = 'bo1808_foc_svpwm_bo1808';
if bdIsLoaded(mdl)
    close_system(mdl,0);
end
build_bo1808_foc_svpwm_model(TL_load_mode, controlMode);

load_system(mdl);
set_param(mdl,'StopTime','t_stop_position_loop');

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
theta_e = y(:,15);
Te = y(:,16);
TL = y(:,17);
ia = y(:,18);
ib = y(:,19);
ic = y(:,20);
sector = y(:,21);

Nref = omega_ref*60/(2*pi);
theta_m = theta_e/pn;
theta_ref = zeros(size(t));
theta_ref(t >= position_ref_step_time) = theta_m_ref_step;
theta_err = theta_ref - theta_m;

idx_post = t > max(0,t_stop_position_loop - 0.05);
if ~any(idx_post)
    idx_post = t > 0.8*t_stop_position_loop;
end
idx_after_load = t >= TL_step_time_fig3_9;

final_pos_err_deg = mean(theta_err(idx_post))*180/pi;
max_pos_err_after_load_deg = max(abs(theta_err(idx_after_load)))*180/pi;
max_speed_abs = max(abs(Nr));
post_iq_mean = mean(iq(idx_post));
post_te_mean = mean(Te(idx_post));
sector_set = unique(round(sector(:))).';

fprintf('\nBO1808 position-loop validation with full FOC + SVPWM drive\n');
fprintf('Model: %s\n', fullfile(models_dir,[mdl '.slx']));
fprintf('Position reference: %.4g rad = %.4g deg at %.4g s\n', ...
    theta_m_ref_step, theta_m_ref_step*180/pi, position_ref_step_time);
fprintf('Position P gain: %.4g 1/s\n', Kp_pos);
fprintf('Position-loop speed limit: %.4g rpm\n', omega_ref_pos_max_rpm);
fprintf('Load mode: step, %.4g N*m at %.4g s\n', TL_nom, TL_step_time_fig3_9);
fprintf('Final average position error: %.4g deg\n', final_pos_err_deg);
fprintf('Max |position error| after load step: %.4g deg\n', max_pos_err_after_load_deg);
fprintf('Max |speed|: %.4g rpm\n', max_speed_abs);
fprintf('Post-load iq mean: %.4g A\n', post_iq_mean);
fprintf('Post-load Te mean: %.4g N*m, TL mean: %.4g N*m\n', ...
    post_te_mean, mean(TL(idx_post)));
fprintf('Voltage command max |ud,uq|: %.4g V, %.4g V\n', ...
    max(abs(ud_cmd)), max(abs(uq_cmd)));
fprintf('SVPWM sector set: %s\n\n', mat2str(sector_set));

%% Position-loop result figure
fig = figure('Name','BO1808 Position Loop Validation','Color','w', ...
    'Position',[100 100 920 900]);

subplot(5,1,1)
plot(t,theta_ref*180/pi,'k--','LineWidth',1.0)
hold on
plot(t,theta_m*180/pi,'b','LineWidth',1.2)
xline(TL_step_time_fig3_9,'r--')
grid on
xlabel('Time / s')
ylabel('Position / deg')
title('(a) Mechanical position response')
legend('\theta_m^*','\theta_m','Location','best')

subplot(5,1,2)
plot(t,theta_err*180/pi,'m','LineWidth',1.1)
xline(TL_step_time_fig3_9,'r--','HandleVisibility','off')
grid on
xlabel('Time / s')
ylabel('Error / deg')
title('(b) Position error')
legend('\theta_m^* - \theta_m','Location','best')

subplot(5,1,3)
plot(t,Nref,'k--','LineWidth',1.0)
hold on
plot(t,Nr,'b','LineWidth',1.2)
xline(TL_step_time_fig3_9,'r--','HandleVisibility','off')
grid on
xlabel('Time / s')
ylabel('Speed / rpm')
title('(c) Speed command from position loop and actual speed')
legend('N^*','N','Location','best')

subplot(5,1,4)
plot(t,Te*1e3,'b','LineWidth',1.1)
hold on
plot(t,TL*1e3,'r--','LineWidth',1.0)
xline(TL_step_time_fig3_9,'r--','HandleVisibility','off')
grid on
xlabel('Time / s')
ylabel('Torque / mN*m')
title('(d) Electromagnetic torque and load torque')
legend('T_e','T_L','Location','best')

subplot(5,1,5)
plot(t,id,'b','LineWidth',1.0)
hold on
plot(t,iq,'r','LineWidth',1.0)
plot(t,iq_ref,'k--','LineWidth',1.0)
xline(TL_step_time_fig3_9,'r--','HandleVisibility','off')
grid on
xlabel('Time / s')
ylabel('Current / A')
title('(e) d-q current tracking')
legend('i_d','i_q','i_q^*','Location','best')

fig_file = fullfile(figures_dir,'bo1808_position_loop_validation.png');
saveas(fig,fig_file);
fprintf('Saved plot: %s\n', fig_file);

%% Three-phase current detail figure
fig_current = figure('Name','BO1808 Position Loop Three-Phase Currents','Color','w', ...
    'Position',[120 120 920 620]);

subplot(2,1,1)
plot(t,ia,'b','LineWidth',0.9)
hold on
plot(t,ib,'r','LineWidth',0.9)
plot(t,ic,'g','LineWidth',0.9)
xline(TL_step_time_fig3_9,'r--','HandleVisibility','off')
grid on
xlabel('Time / s')
ylabel('Current / A')
title('(a) Three-phase stator currents')
legend('i_a','i_b','i_c','Location','best')

subplot(2,1,2)
stairs(t,sector,'k','LineWidth',1.0)
grid on
xlabel('Time / s')
ylabel('N')
title('(b) SVPWM sector sequence')
ylim([0.5 6.5])

fig_current_file = fullfile(figures_dir,'bo1808_position_loop_currents_and_sector.png');
saveas(fig_current,fig_current_file);
fprintf('Saved plot: %s\n', fig_current_file);

set_param(mdl,'Dirty','off');
