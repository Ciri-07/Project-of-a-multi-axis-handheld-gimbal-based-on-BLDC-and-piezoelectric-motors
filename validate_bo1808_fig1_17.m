%% BO1808NBH2B validation in the style of textbook Fig.1-17
% Textbook Fig.1-17 validates the PMSM vector-control model using:
%   Nref = 1500 r/min
%   TL = 0 initially
%   TL steps at t = 0.05 s
%
% The textbook example uses a large-motor load step of 10 N*m. That is not
% physically meaningful for BO1808NBH2B, whose rated torque is 1.2 mN*m.
% This validation therefore keeps the Fig.1-17 structure but uses TL_nom.

clear
clc

init_bo1808_params;

mdl = 'bo1808_foc_id0';
model_file = fullfile(models_dir,[mdl '.slx']);
if ~isfile(model_file)
    build_bo1808_foc_id0_model;
end

load_system(model_file);

% Fig.1-17 style validation settings adapted to BO1808NBH2B.
speed_ref_rpm = 1500;
speed_ref = speed_ref_rpm*2*pi/60;
TL_step_time = 0.05;
TL_validate = TL_nom;
t_stop_foc = 0.10;

% Write validation values into the model workspace because the generated
% model is intentionally self-contained.
mws = get_param(mdl,'ModelWorkspace');
assignin(mws,'speed_ref_rpm',speed_ref_rpm);
assignin(mws,'speed_ref',speed_ref);
assignin(mws,'TL_step_time',TL_step_time);
assignin(mws,'TL_nom',TL_validate);
assignin(mws,'t_stop_foc',t_stop_foc);

set_param(mdl,'StopTime','t_stop_foc');

simOut = sim(mdl,'ReturnWorkspaceOutputs','on');
simout_foc = simOut.get('simout_foc');

t = simout_foc.time;
y = simout_foc.signals.values;

omega_ref = y(:,1);
omega_m = y(:,2);
Nr = y(:,3);
id = y(:,4);
iq = y(:,5);
iq_ref = y(:,6);
ud = y(:,7);
uq = y(:,8);
TL = y(:,9);

Nr_ref = omega_ref*60/(2*pi);
Te = Kt_q*iq;

idx_preload = t >= 0.035 & t < TL_step_time;
idx_afterload = t >= 0.08 & t <= t_stop_foc;

speed_err_pre_rpm = max(abs(Nr(idx_preload) - Nr_ref(idx_preload)));
speed_err_after_rpm = max(abs(Nr(idx_afterload) - Nr_ref(idx_afterload)));
id_abs_max = max(abs(id));
iq_abs_max = max(abs(iq));
voltage_abs_max = max(max(abs([ud uq])));
torque_after_mean = mean(Te(idx_afterload));
load_after_mean = mean(TL(idx_afterload));

fprintf('\nFig.1-17 style BO1808 validation\n');
fprintf('Nref = %.1f r/min, TL step = %.4g N*m at %.3f s\n', ...
    speed_ref_rpm, TL_validate, TL_step_time);
fprintf('Pre-load speed max error, 0.035-0.05 s: %.3f r/min\n', speed_err_pre_rpm);
fprintf('After-load speed max error, 0.08-0.10 s: %.3f r/min\n', speed_err_after_rpm);
fprintf('Max |id|: %.6g A\n', id_abs_max);
fprintf('Max |iq|: %.6g A, Max |iq_ref|: %.6g A\n', iq_abs_max, max(abs(iq_ref)));
fprintf('Mean Te after load: %.6g N*m, mean TL after load: %.6g N*m\n', ...
    torque_after_mean, load_after_mean);
fprintf('Max |ud,uq|: %.6g V, Vdq_max: %.6g V\n\n', voltage_abs_max, Vdq_max);

fig = figure('Name','BO1808 Fig.1-17 Style Validation','Color','w');

subplot(3,1,1)
plot(t,Nr_ref,'k--','LineWidth',1.0)
hold on
plot(t,Nr,'b','LineWidth',1.2)
grid on
xlabel('Time / s')
ylabel('Speed / r/min')
title('(a) Speed response')
legend('N_{ref}','N_r','Location','best')

subplot(3,1,2)
plot(t,iq,'r','LineWidth',1.2)
hold on
plot(t,iq_ref,'k--','LineWidth',1.0)
grid on
xlabel('Time / s')
ylabel('i_q / A')
title('(b) q-axis stator current')
legend('i_q','i_q^*','Location','best')

subplot(3,1,3)
plot(t,id,'b','LineWidth',1.2)
grid on
xlabel('Time / s')
ylabel('i_d / A')
title('(c) d-axis stator current')

fig_file = fullfile(figures_dir,'bo1808_fig1_17_validation.png');
saveas(fig,fig_file);
fprintf('Saved plot: %s\n', fig_file);

% Validation settings are temporary; avoid a save prompt caused by model
% workspace assignments.
set_param(mdl,'Dirty','off');
