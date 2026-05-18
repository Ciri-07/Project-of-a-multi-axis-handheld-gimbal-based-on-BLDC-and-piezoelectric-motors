%% Validate the BO1808 SVPWM S-Function Simulink model

clear
clc

init_bo1808_svpwm_params;

mdl = 'bo1808_svpwm_sfunc_sim';
if bdIsLoaded(mdl)
    close_system(mdl,0);
end
build_bo1808_svpwm_sfunc_model;

load_system(mdl);
set_param(mdl,'StopTime','t_stop_svpwm');

simOut = sim(mdl,'ReturnWorkspaceOutputs','on');
simout_svpwm_sfunc = simOut.get('simout_svpwm_sfunc');

t = simout_svpwm_sfunc.time;
y = simout_svpwm_sfunc.signals.values;

Ualpha_ref = y(:,1);
Ubeta_ref = y(:,2);
Valpha_sw = y(:,3);
Vbeta_sw = y(:,4);
sector = y(:,5);
T1 = y(:,6);
T2 = y(:,7);
T0 = y(:,8);
Tcm1 = y(:,9);
Tcm2 = y(:,10);
Tcm3 = y(:,11);
carrier_time = y(:,12);
Sa = y(:,13);
Sb = y(:,14);
Sc = y(:,15);
Vab = y(:,16);
Vbc = y(:,17);
Vca = y(:,18);

Va_pole = (Sa - 0.5)*Vdc;
Vb_pole = (Sb - 0.5)*Vdc;
Vc_pole = (Sc - 0.5)*Vdc;
V_common = (Va_pole + Vb_pole + Vc_pole)/3;
Van = Va_pole - V_common;

fprintf('\nBO1808 SVPWM S-Function validation\n');
fprintf('Vdc = %.4g V, fs_pwm = %.4g Hz, f_ref = %.4g Hz, Uref = %.4g V, m = %.4g\n', ...
    Vdc, fs_pwm, f_ref_svpwm, Uref_svpwm, m_svpwm);
fprintf('S-Function core: [Valpha Vbeta Udc Tpwm] -> [Tcm1 Tcm2 Tcm3 sector T1 T2 T0]\n');

sector_step = round(sector(:));
sector_changes = sector_step([true; diff(sector_step) ~= 0]);
sector_order = sector_changes(1:min(6,numel(sector_changes)));
fprintf('Sector set: %s\n', mat2str(unique(sector_step).'));
fprintf('Sector transition order, first cycle: %s\n', mat2str(sector_order.'));
fprintf('Tcm range: Tcm1 [%.4g, %.4g], Tcm2 [%.4g, %.4g], Tcm3 [%.4g, %.4g] s\n', ...
    min(Tcm1), max(Tcm1), min(Tcm2), max(Tcm2), min(Tcm3), max(Tcm3));
fprintf('Time check: max |T1+T2+T0-Ts| = %.4g s\n', ...
    max(abs(T1 + T2 + T0 - Ts_pwm_svpwm)));
fprintf('Switch values: Sa=%s, Sb=%s, Sc=%s\n', ...
    mat2str(unique(Sa).'), mat2str(unique(Sb).'), mat2str(unique(Sc).'));
fprintf('Line voltage values are within +/-Vdc: max |Vab,Vbc,Vca| = %.4g V\n\n', ...
    max(max(abs([Vab Vbc Vca]))));

%% Alpha-beta reference voltage waveforms
fig_ab = figure('Name','BO1808 SVPWM S-Function Alpha-Beta Reference','Color','w');
idx_ab_zoom = t <= 2/f_ref_svpwm;

subplot(2,1,1)
plot(t(idx_ab_zoom),Ualpha_ref(idx_ab_zoom),'b','LineWidth',1.2)
hold on
plot(t(idx_ab_zoom),Ubeta_ref(idx_ab_zoom),'r','LineWidth',1.2)
grid on
xlabel('Time / s')
ylabel('Voltage / V')
title(sprintf('(a) Reference voltage waveforms, f = %.2f Hz, Uref = %.3f V', ...
    f_ref_svpwm, Uref_svpwm))
legend('u_\alpha','u_\beta','Location','best')

subplot(2,1,2)
plot(Ualpha_ref,Ubeta_ref,'k','LineWidth',1.1)
hold on
plot(Ualpha_ref(1),Ubeta_ref(1),'ro','MarkerFaceColor','r')
grid on
axis equal
xlabel('u_\alpha / V')
ylabel('u_\beta / V')
title('(b) Alpha-beta voltage vector trajectory')
xlim(1.15*Uref_svpwm*[-1 1])
ylim(1.15*Uref_svpwm*[-1 1])

fig_ab_file = fullfile(figures_dir,'bo1808_svpwm_sfunc_alpha_beta_ref.png');
saveas(fig_ab,fig_ab_file);
fprintf('Saved plot: %s\n', fig_ab_file);

%% A-phase voltage FFT analysis
fft_duration = fft_cycles_svpwm/f_ref_svpwm;
Ts_fft = min(Ts_sim_svpwm,Ts_pwm_svpwm/100);
t_fft = (0:Ts_fft:fft_duration-Ts_fft).';
Tcm1_fft = interp1(t,Tcm1,t_fft,'linear','extrap');
Tcm2_fft = interp1(t,Tcm2,t_fft,'linear','extrap');
Tcm3_fft = interp1(t,Tcm3,t_fft,'linear','extrap');
tau_pwm = mod(t_fft,Ts_pwm_svpwm);
carrier_fft = min(tau_pwm,Ts_pwm_svpwm - tau_pwm);
Sa_fft = double(Tcm1_fft <= carrier_fft);
Sb_fft = double(Tcm2_fft <= carrier_fft);
Sc_fft = double(Tcm3_fft <= carrier_fft);
Va_pole_fft = (Sa_fft - 0.5)*Vdc;
Vb_pole_fft = (Sb_fft - 0.5)*Vdc;
Vc_pole_fft = (Sc_fft - 0.5)*Vdc;
V_common_fft = (Va_pole_fft + Vb_pole_fft + Vc_pole_fft)/3;
van_fft_signal = Va_pole_fft - V_common_fft;
van_fft_signal = van_fft_signal - mean(van_fft_signal);

Fs_fft = 1/Ts_fft;
N_fft = numel(van_fft_signal);
Y_fft = fft(van_fft_signal);
P2 = abs(Y_fft/N_fft);
P1 = P2(1:floor(N_fft/2)+1);
P1(2:end-1) = 2*P1(2:end-1);
f_fft = Fs_fft*(0:floor(N_fft/2))/N_fft;

[~,idx_fund] = min(abs(f_fft - f_ref_svpwm));
[~,idx_carrier] = min(abs(f_fft - fs_pwm));
idx_carrier_band = f_fft >= (fs_pwm - 3*f_ref_svpwm) & f_fft <= (fs_pwm + 3*f_ref_svpwm);
if any(idx_carrier_band)
    carrier_band_freqs = f_fft(idx_carrier_band);
    carrier_band_amps = P1(idx_carrier_band);
    [carrier_band_peak,idx_carrier_band_peak_local] = max(carrier_band_amps);
    f_carrier_band_peak = carrier_band_freqs(idx_carrier_band_peak_local);
else
    carrier_band_peak = P1(idx_carrier);
    f_carrier_band_peak = f_fft(idx_carrier);
end

fprintf('A-phase voltage FFT:\n');
fprintf('Fundamental %.1f Hz amplitude: %.4g V\n', f_fft(idx_fund), P1(idx_fund));
fprintf('Exact carrier-bin %.1f Hz amplitude: %.4g V\n', f_fft(idx_carrier), P1(idx_carrier));
fprintf('Carrier-band peak %.1f Hz amplitude: %.4g V\n', f_carrier_band_peak, carrier_band_peak);
fprintf('RMS Van: %.4g V\n\n', sqrt(mean(van_fft_signal.^2)));

low_freq_limit = max(1000,ceil(5*f_ref_svpwm/100)*100);

fig = figure('Name','BO1808 Rated-Point SVPWM S-Function Validation','Color','w');

subplot(4,1,1)
stairs(t,sector,'k','LineWidth',1.1)
grid on
xlabel('Time / s')
ylabel('N')
title('(a) S-Function sector N calculation, display order 3-1-5-4-6-2')
ylim([0.5 6.5])

subplot(4,1,2)
plot(t,Tcm1,'b','LineWidth',1.0)
hold on
plot(t,Tcm2,'r','LineWidth',1.0)
plot(t,Tcm3,'g','LineWidth',1.0)
grid on
xlabel('Time / s')
ylabel('Tcm / s')
title('(b) S-Function saddle-shaped Tcm1/Tcm2/Tcm3')
legend('Tcm1','Tcm2','Tcm3','Location','best')

subplot(4,1,3)
stairs(t,Van,'b','LineWidth',0.9)
grid on
xlabel('Time / s')
ylabel('u_a / V')
title('(c) A-phase voltage u_a')

subplot(4,1,4)
idx_freq_main = f_fft <= low_freq_limit;
stem(f_fft(idx_freq_main),P1(idx_freq_main),'Marker','none','LineWidth',0.9)
hold on
plot(f_fft(idx_fund),P1(idx_fund),'ro','MarkerFaceColor','r')
grid on
xlabel('Frequency / Hz')
ylabel('Amplitude / V')
title(sprintf('(d) FFT of u_a, fundamental %.1f Hz = %.4g V', ...
    f_fft(idx_fund), P1(idx_fund)))
xlim([0 low_freq_limit])
ylim([0 max(P1(idx_fund)*1.2,1e-3)])

fig_file = fullfile(figures_dir,'bo1808_svpwm_sfunc_validation.png');
saveas(fig,fig_file);
fprintf('Saved plot: %s\n', fig_file);

fig_fft = figure('Name','BO1808 SVPWM S-Function A-Phase Voltage FFT','Color','w');

subplot(3,1,1)
idx_time_zoom = t <= 2e-3;
stairs(t(idx_time_zoom),Van(idx_time_zoom),'b','LineWidth',1.0)
grid on
xlabel('Time / s')
ylabel('V_{an} / V')
title('(a) A-phase voltage to motor neutral')

subplot(3,1,2)
idx_low_freq = f_fft <= low_freq_limit;
stem(f_fft(idx_low_freq),P1(idx_low_freq),'Marker','none','LineWidth',0.9)
hold on
plot(f_fft(idx_fund),P1(idx_fund),'ro','MarkerFaceColor','r')
grid on
xlabel('Frequency / Hz')
ylabel('Amplitude / V')
title(sprintf('(b) Low-frequency FFT zoom, %.1f Hz = %.4g V', ...
    f_fft(idx_fund), P1(idx_fund)))
xlim([0 low_freq_limit])
ylim([0 max(P1(idx_fund)*1.2,1e-3)])

subplot(3,1,3)
idx_freq = f_fft <= 2.5*fs_pwm;
plot(f_fft(idx_freq),20*log10(P1(idx_freq) + 1e-6),'k','LineWidth',0.9)
hold on
plot(f_carrier_band_peak,20*log10(carrier_band_peak + 1e-6),'ro','MarkerFaceColor','r')
grid on
xlabel('Frequency / Hz')
ylabel('Amplitude / dBV')
title('(c) Wideband FFT spectrum of V_{an}, carrier-band peak marked')
xline(f_ref_svpwm,'r--','Fundamental','LabelVerticalAlignment','bottom')
xline(fs_pwm,'k--','Carrier','LabelVerticalAlignment','bottom')

fig_fft_file = fullfile(figures_dir,'bo1808_svpwm_sfunc_phase_voltage_fft.png');
saveas(fig_fft,fig_fft_file);
fprintf('Saved plot: %s\n', fig_fft_file);

set_param(mdl,'Dirty','off');
