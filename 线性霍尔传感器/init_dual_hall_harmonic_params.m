%% Dual-linear-Hall harmonic compensation validation parameters
% This file is independent from init_dual_hall_ideal_params.m.
% It keeps the previous offset/amplitude/phase case and adds 2/3/5/6
% magnetic-angle harmonics for compensation validation.

hall_pole_pairs = 6;             % 12 magnetic poles -> 6 pole pairs
hall_magnet_od_mm = 18;          % mm, hollow magnet outer diameter

hall_adc_bits = 16;              % bit, project ADC resolution
hall_vref_v = 3.3;               % V, ADC/Hall reference voltage
hall_offset_v = hall_vref_v/2;   % V, nominal mid-supply offset

hall_amp_s_v = 1.0;              % V, sine-channel fundamental amplitude
hall_amp_c_v = 0.95;             % V, cosine-channel fundamental amplitude
hall_amp_v = hall_amp_s_v;       % V, nominal amplitude for raw comparison

hall_phase_err_deg = 2;          % deg, cosine channel non-orthogonal phase error
hall_phase_err_rad = deg2rad(hall_phase_err_deg);

hall_offset_err_s_v = 0.01;      % V, sine-channel zero-offset error
hall_offset_err_c_v = -0.015;     % V, cosine-channel zero-offset error
hall_offset_s_actual_v = hall_offset_v + hall_offset_err_s_v;
hall_offset_c_actual_v = hall_offset_v + hall_offset_err_c_v;

hall_mech_speed_rpm = 100;        % rpm, constant-speed calibration motion
hall_f_mech_hz = hall_mech_speed_rpm/60;
hall_f_mag_hz = hall_pole_pairs*hall_f_mech_hz;
hall_omega_mech_rad_s = 2*pi*hall_f_mech_hz;
hall_theta0_rad = 0;             % rad, mechanical installation phase offset
hall_ts_sim_s = 1e-4;            % s, simulation sample time
hall_t_stop_s = 2.0;             % s, total simulation time

% Harmonic orders are relative to magnetic angle theta_mag.
% The values below are voltage amplitudes. They are intentionally visible
% but still small relative to the 1 V fundamental.
hall_harm_orders = [2 3 5 6];
hall_harm_s_sin_v = [0.050 -0.035 0.024 0.016];
hall_harm_s_cos_v = [0.018  0.028 -0.014 0.010];
hall_harm_c_sin_v = [-0.032 0.022 0.016 -0.012];
hall_harm_c_cos_v = [0.040  0.030 -0.018 0.013];

% Orders used by the signal-domain reconstruction. These match the
% injected Hall-voltage harmonics.
hall_signal_comp_orders = hall_harm_orders;

% Orders used by the angle-domain Fourier compensation. Because atan2 mixes
% a k-th signal harmonic into roughly k-1 and k+1 angle-error components,
% this set covers the dominant neighboring angle-error orders.
hall_angle_fft_orders = 1:12;
hall_angle_comp_orders = 1:8;
%角度域补偿：先看 1-12 阶 FFT，实际补偿 1-8 阶

% Time window used only for Fig.3-style waveform display. The full
% simulation still lasts hall_t_stop_s; this just makes several magnetic
% periods visible in the voltage and position plots.
hall_fig3_time_window_s = 0.5;   % s, about 5 magnetic periods at 10 Hz
