%% Validate the full BO1808 FOC + SVPWM model with sinusoidal load

clear
clc

init_bo1808_params;
TL_load_mode = 'sine';

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

Nref = y(:,1)*60/(2*pi);
Nr = y(:,3);
id = y(:,4);
iq = y(:,5);
iq_ref = y(:,6);
Te = y(:,16);
TL = y(:,17);
ia = y(:,18);
ib = y(:,19);
ic = y(:,20);

fprintf('\nBO1808 full FOC + SVPWM validation with sinusoidal load\n');
fprintf('Load: TL = %.4g + %.4g*sin(2*pi*%.4g*t), enabled at %.4g s\n', ...
    TL_mean_sine, TL_amp_sine, f_load_sine, TL_sine_start);
if TL_noise_enable
    fprintf('White-noise disturbance: std %.4g N*m, sample time %.4g s, seed %d\n', ...
        TL_noise_std, TL_noise_sample_time, TL_noise_seed);
else
    fprintf('White-noise disturbance: disabled\n');
end
fprintf('TL range after enable: [%.4g, %.4g] N*m\n', ...
    min(TL(t >= TL_sine_start)), max(TL(t >= TL_sine_start)));
fprintf('iq range after enable: [%.4g, %.4g] A\n', ...
    min(iq(t >= TL_sine_start)), max(iq(t >= TL_sine_start)));
fprintf('Speed range after enable: [%.4g, %.4g] rpm\n\n', ...
    min(Nr(t >= TL_sine_start)), max(Nr(t >= TL_sine_start)));

fig = figure('Name','BO1808 Full FOC SVPWM Sinusoidal Load Validation','Color','w');

subplot(4,1,1)
plot(t,Nref,'k--','LineWidth',1.0)
hold on
plot(t,Nr,'b','LineWidth',1.2)
xline(TL_sine_start,'r--','Load start','LabelVerticalAlignment','bottom')
grid on
xlabel('Time / s')
ylabel('Speed / rpm')
if TL_noise_enable
    load_title_suffix = 'sinusoidal load plus white noise';
else
    load_title_suffix = 'sinusoidal load';
end
title(['(a) Speed response under ' load_title_suffix])
legend('N^*','N','Location','best')

subplot(4,1,2)
plot(t,Te*1e3,'b','LineWidth',1.1)
hold on
plot(t,TL*1e3,'r--','LineWidth',1.0)
xline(TL_sine_start,'r--','Load start','HandleVisibility','off')
grid on
xlabel('Time / s')
ylabel('Torque / mN*m')
title('(b) Electromagnetic torque and load torque')
legend('T_e','T_L','Location','best')

subplot(4,1,3)
plot(t,id,'b','LineWidth',1.0)
hold on
plot(t,iq,'r','LineWidth',1.0)
plot(t,iq_ref,'k--','LineWidth',1.0)
xline(TL_sine_start,'r--','Load start','HandleVisibility','off')
grid on
xlabel('Time / s')
ylabel('Current / A')
title('(c) d-q current response')
legend('i_d','i_q','i_q^*','Location','best')

subplot(4,1,4)
plot(t,ia,'b','LineWidth',0.9)
hold on
plot(t,ib,'r','LineWidth',0.9)
plot(t,ic,'g','LineWidth',0.9)
xline(TL_sine_start,'r--','Load start','HandleVisibility','off')
grid on
xlabel('Time / s')
ylabel('Current / A')
title('(d) Three-phase stator currents')
legend('i_a','i_b','i_c','Location','best')

fig_file = fullfile(figures_dir,'bo1808_fig3_9_foc_svpwm_sine_load.png');
saveas(fig,fig_file);
fprintf('Saved plot: %s\n', fig_file);

set_param(mdl,'Dirty','off');
