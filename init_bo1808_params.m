%% BO1808NBH2B PMSM parameter initialization
% Synchronous rotating d-q model parameters for the 11.4 V winding.
% Default interpretation:
%   Datasheet terminal R/L are line-to-line values of a Y-connected motor.
%   Therefore phase Rs = R_line/2 and Ld = Lq = L_line/2.

clearvars -except ans loadMode controlMode controlModeOverride

%% Project folders
project_root = fileparts(mfilename('fullpath'));
if isempty(project_root)
    project_root = pwd;
end
data_dir = fullfile(project_root,'data');
references_dir = fullfile(project_root,'references');
docs_dir = fullfile(project_root,'docs');
models_dir = fullfile(project_root,'models');
results_dir = fullfile(project_root,'results');
figures_dir = fullfile(results_dir,'figures');
reports_dir = fullfile(results_dir,'reports');
cache_dir = fullfile(project_root,'cache');
project_dirs = {data_dir,references_dir,docs_dir,models_dir,results_dir,figures_dir,reports_dir,cache_dir};
for k_dir = 1:numel(project_dirs)
    if ~exist(project_dirs{k_dir},'dir')
        mkdir(project_dirs{k_dir});
    end
end
try
    Simulink.fileGenControl('set', ...
        'CacheFolder',cache_dir, ...
        'CodeGenFolder',cache_dir, ...
        'createDir',true);
catch
end

%% Motor electrical parameters
pn = 4;                         % pole pairs
R_line = 14.4;                  % ohm, terminal/line resistance
L_line = 0.74e-3;               % H, terminal/line inductance
use_line_to_line_RL = true;

if use_line_to_line_RL
    Rs = R_line/2;              % ohm, phase resistance
    Ld = L_line/2;              % H, d-axis inductance
    Lq = L_line/2;              % H, q-axis inductance
else
    Rs = R_line;                % ohm, if datasheet values are phase values
    Ld = L_line;                % H
    Lq = L_line;                % H
end

Kt = 6.4e-3;                    % N*m/A
psi_f = Kt/(1.5*pn);            % Wb, permanent-magnet flux linkage
Ke_mV_per_rpm = 0.67;           % mV/rpm, for reference only

%% Rated data and operating limits
Vdc = 11.4;                     % V
TL_nom = 1.2e-3;                % N*m
TL_mean_sine = TL_nom;      % N*m, 正弦负载平均值
TL_amp_sine = 0.4e-3;       % N*m, 正弦负载波动幅值
f_load_sine = 50;            % Hz, 正弦负载频率
TL_sine_start = 0.12;       % s, sinusoidal load enable time
TL_load_mode = 'step';      % 'step' or 'sine'
TL_noise_enable = false;     % add white-noise load disturbance
TL_noise_std = 0.05e-3;     % N*m, noise standard deviation
TL_noise_sample_time = 1e-4;% s, random load update interval
TL_noise_seed = 1808;       % reproducible random load seed
foc_control_mode = 'position'; % 'speed' or 'position' for validate_bo1808_fig3_9

Nrated = 7790;                  % rpm
N0 = 14300;                     % rpm
I0 = 0.12;                      % A

omega_rated = Nrated*2*pi/60;   % rad/s mechanical
omega0 = N0*2*pi/60;            % rad/s mechanical
omegae_rated = pn*omega_rated;  % rad/s electrical

%% Motor mechanical parameters
J = 1.4e-7;                     % kg*m^2
B_no_load_equiv = Kt*I0/omega0; % N*m*s, calibrated from no-load point
B = B_no_load_equiv;            % N*m*s, default damping used by simulations

Kt_q = 1.5*pn*psi_f;            % N*m/A, torque per q-axis ampere
Iq_rated = TL_nom/Kt_q;         % A, q-axis current for rated load torque

% d-q voltage vector limits. Use the SVPWM value by default.
Vdq_max_svpwm = Vdc/sqrt(3);
Vdq_max_spwm = Vdc/2;
Vdq_max = Vdq_max_svpwm;

% Conservative current limit for this small high-resistance motor.
%Iq_max = 0.10；                 带不动负载
Iq_max = 0.3;                  % A最低限幅
%Iq_max = 1.00;
%Iq_max = 0.25;                  % A
Id_max = 0.60;                  % A

%% Simulation setup
Ts_ctrl = 1e-5;                 % s, suggested control sample time
Ts_pwm = 2e-5;                  % s, suggested PWM/update period
t_stop_open_loop = 0.08;        % s
t_stop_foc = 0.10;              % s
Ts_sim_foc_svpwm = 2e-6;        % s, max solver step for switching FOC model
t_stop_foc_svpwm = 0.25;        % s, BO1808 Fig.3-9-style validation window
t_stop_position_loop = 0.40;    % s, longer window for position settling

id0 = 0;
iq0 = 0;
omega_m0 = 0;
theta_e0 = 0;
theta_m0 = 0;

%% PI controller default tuning
% Current-loop pole-zero cancellation:
%   Kp_i = L*wci, Ki_i = Rs*wci
% The BO1808 has high Rs and very small L, so avoid an unnecessarily large
% current bandwidth unless the solver/PWM step is also very small.
%wci = 2*pi*20;
wci = 2*pi*300;                 % rad/s, conservative current-loop bandwidth
%wci = 2*pi*1500;
Kp_id = Ld*wci;
Ki_id = Rs*wci;
Kp_iq = Lq*wci;
Ki_iq = Rs*wci;

% Speed loop assumes the current loop is much faster:
%   omega plant approx: Kt_q/(J*s + B)
%   Kp_w = (2*zeta*wn*J - B)/Kt_q, Ki_w = J*wn^2/Kt_q
zeta_w = 0.90;
wn_w = 2*pi*18;                 % rad/s, about one decade below current loop
Kp_w = max((2*zeta_w*wn_w*J - B)/Kt_q, 0);
Ki_w = J*wn_w^2/Kt_q;

% Example references used by the generated models.
speed_ref_rpm = 3000;
speed_ref = speed_ref_rpm*2*pi/60;
ud_step_value = 0;
uq_step_value = 1.0;
TL_step_time = 0.04;
speed_ref_fig3_9_rpm = 1000;    % rpm, Fig.3-9-style closed-loop validation
TL_step_time_fig3_9 = 0.12;     % s, load disturbance time for full FOC+SVPWM

%% Position-loop defaults for servo validation
% Position control is implemented as an outer P loop:
%   theta_m_error -> Kp_pos -> omega_ref
% The existing speed PI then rejects load torque and drives steady-state
% speed to the position-loop command.
theta_m_ref_step = pi/6;        % rad, 30 deg mechanical position command
position_ref_step_time = 0.005; % s
Kp_pos = 2*pi*6;                % 1/s, position-loop proportional gain
omega_ref_pos_max_rpm = 1000;   % rpm, speed limit from position loop
omega_ref_pos_max = omega_ref_pos_max_rpm*2*pi/60;

%% Pack parameters for workspace use and logging
BO1808 = struct();
BO1808.pn = pn;
BO1808.R_line = R_line;
BO1808.L_line = L_line;
BO1808.use_line_to_line_RL = use_line_to_line_RL;
BO1808.Rs = Rs;
BO1808.Ld = Ld;
BO1808.Lq = Lq;
BO1808.Kt = Kt;
BO1808.psi_f = psi_f;
BO1808.Ke_mV_per_rpm = Ke_mV_per_rpm;
BO1808.J = J;
BO1808.B = B;
BO1808.B_no_load_equiv = B_no_load_equiv;
BO1808.Vdc = Vdc;
BO1808.TL_nom = TL_nom;
BO1808.TL_mean_sine = TL_mean_sine;
BO1808.TL_amp_sine = TL_amp_sine;
BO1808.f_load_sine = f_load_sine;
BO1808.TL_sine_start = TL_sine_start;
BO1808.TL_load_mode = TL_load_mode;
BO1808.TL_noise_enable = TL_noise_enable;
BO1808.TL_noise_std = TL_noise_std;
BO1808.TL_noise_sample_time = TL_noise_sample_time;
BO1808.TL_noise_seed = TL_noise_seed;
BO1808.foc_control_mode = foc_control_mode;
BO1808.Nrated = Nrated;
BO1808.N0 = N0;
BO1808.I0 = I0;
BO1808.omega_rated = omega_rated;
BO1808.omega0 = omega0;
BO1808.omegae_rated = omegae_rated;
BO1808.Kt_q = Kt_q;
BO1808.Iq_rated = Iq_rated;
BO1808.Vdq_max = Vdq_max;
BO1808.Iq_max = Iq_max;
BO1808.Id_max = Id_max;
BO1808.Ts_ctrl = Ts_ctrl;
BO1808.Ts_pwm = Ts_pwm;
BO1808.Ts_sim_foc_svpwm = Ts_sim_foc_svpwm;
BO1808.t_stop_foc_svpwm = t_stop_foc_svpwm;
BO1808.t_stop_position_loop = t_stop_position_loop;
BO1808.theta_m_ref_step = theta_m_ref_step;
BO1808.position_ref_step_time = position_ref_step_time;
BO1808.Kp_pos = Kp_pos;
BO1808.omega_ref_pos_max_rpm = omega_ref_pos_max_rpm;
BO1808.omega_ref_pos_max = omega_ref_pos_max;
BO1808.Kp_id = Kp_id;
BO1808.Ki_id = Ki_id;
BO1808.Kp_iq = Kp_iq;
BO1808.Ki_iq = Ki_iq;
BO1808.Kp_w = Kp_w;
BO1808.Ki_w = Ki_w;
BO1808.speed_ref_fig3_9_rpm = speed_ref_fig3_9_rpm;
BO1808.TL_step_time_fig3_9 = TL_step_time_fig3_9;
BO1808.project_root = project_root;
BO1808.data_dir = data_dir;
BO1808.references_dir = references_dir;
BO1808.docs_dir = docs_dir;
BO1808.models_dir = models_dir;
BO1808.figures_dir = figures_dir;
BO1808.reports_dir = reports_dir;
BO1808.cache_dir = cache_dir;

fprintf('BO1808NBH2B parameters loaded.\n');
fprintf('Rs = %.4g ohm, Ld = Lq = %.4g H, psi_f = %.6g Wb\n', Rs, Ld, psi_f);
fprintf('Kt_q = %.6g N*m/A, Iq_rated = %.4g A, Vdq_max = %.4g V\n', Kt_q, Iq_rated, Vdq_max);
fprintf('B = %.6g N*m*s, calibrated from I0 = %.4g A and N0 = %.4g rpm\n', B, I0, N0);
fprintf('Current PI: Kp = %.4g, Ki = %.4g; Speed PI: Kp = %.4g, Ki = %.4g\n', Kp_iq, Ki_iq, Kp_w, Ki_w);
