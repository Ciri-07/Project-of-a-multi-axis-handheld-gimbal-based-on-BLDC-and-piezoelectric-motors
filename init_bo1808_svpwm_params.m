%% BO1808NBH2B SVPWM simulation parameters
% Standalone SVPWM verification parameters for the 11.4 V DC bus.

if ~exist('Vdc','var')
    init_bo1808_params;
end

fs_pwm = 20e3;                  % Hz, practical carrier assumption for this model
Ts_pwm_svpwm = 1/fs_pwm;        % s
Ts_sim_svpwm = 1e-6;            % s, maximum simulation step
fft_cycles_svpwm = 20;          % integer cycles for clean FFT bins

f_ref_svpwm = pn*Nrated/60;     % Hz, electrical frequency at rated speed
omega_ref_svpwm = 2*pi*f_ref_svpwm;
t_stop_svpwm = fft_cycles_svpwm/f_ref_svpwm;
Vref_max_svpwm = Vdc/sqrt(3);   % V, linear SVPWM voltage vector limit

% Rated-load d-q voltage magnitude used as the alpha-beta reference vector.
id_ref_svpwm = 0;
iq_ref_svpwm = Iq_rated;
ud_ref_svpwm = -omega_ref_svpwm*Lq*iq_ref_svpwm;
uq_ref_svpwm = Rs*iq_ref_svpwm + omega_ref_svpwm*(Ld*id_ref_svpwm + psi_f);
Uref_svpwm = hypot(ud_ref_svpwm,uq_ref_svpwm);
m_svpwm = Uref_svpwm/Vref_max_svpwm;

SVPWM = struct();
SVPWM.fs_pwm = fs_pwm;
SVPWM.Ts_pwm = Ts_pwm_svpwm;
SVPWM.Ts_sim = Ts_sim_svpwm;
SVPWM.t_stop = t_stop_svpwm;
SVPWM.fft_cycles = fft_cycles_svpwm;
SVPWM.f_ref = f_ref_svpwm;
SVPWM.omega_ref = omega_ref_svpwm;
SVPWM.Vdc = Vdc;
SVPWM.Vref_max = Vref_max_svpwm;
SVPWM.m = m_svpwm;
SVPWM.Uref = Uref_svpwm;
SVPWM.id_ref = id_ref_svpwm;
SVPWM.iq_ref = iq_ref_svpwm;
SVPWM.ud_ref = ud_ref_svpwm;
SVPWM.uq_ref = uq_ref_svpwm;

fprintf(['SVPWM parameters loaded: Vdc = %.4g V, fs_pwm = %.4g Hz, ' ...
    'f_ref = %.4g Hz, Uref = %.4g V, m = %.4g\n'], ...
    Vdc, fs_pwm, f_ref_svpwm, Uref_svpwm, m_svpwm);
