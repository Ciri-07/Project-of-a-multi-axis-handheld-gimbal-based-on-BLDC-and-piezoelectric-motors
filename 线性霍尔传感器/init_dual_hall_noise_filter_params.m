%% 双线性霍尔 ADC 量化、白噪声、低通滤波和 alpha-beta 滤波参数
% 本文件在谐波补偿基准参数基础上，继续加入真实采样链路误差：
%   1. ADC 量化；
%   2. Hall/电源/采样白噪声；
%   3. 一阶低通滤波；
%   4. alpha-beta 角度/速度估计器。
%
% 低通滤波用于对采样电压做平滑；alpha-beta 滤波用于对补偿后的角度
% 做状态估计，二者的作用对象不同。

init_dual_hall_harmonic_params;

hall_noise_seed = 1808;          % 随机种子，保证每次仿真可复现
hall_noise_rms_v = 1.0e-2;       % V，单通道白噪声 RMS 值

hall_adc_enable = true;          % 是否启用 ADC 量化
hall_adc_lsb_v = hall_vref_v/(2^hall_adc_bits - 1);  % V，ADC 最低有效位

hall_lpf_cutoff_hz = 300;        % Hz，一阶低通滤波截止频率
hall_lpf_tau_s = 1/(2*pi*hall_lpf_cutoff_hz);        % s，滤波时间常数
hall_lpf_alpha = exp(-hall_ts_sim_s/hall_lpf_tau_s); % 离散一阶低通系数

% alpha-beta 滤波参数。alpha 修正角度，beta 修正角速度。
% alpha、beta 越大，跟随越快但抑噪越弱；越小，输出越平滑但动态滞后越大。
hall_ab_alpha = 0.22;            % 角度修正系数
hall_ab_beta = 0.018;            % 速度修正系数

% 离散 ESO 参数。ESO 在角度域估计 z1=角度、z2=角速度、z3=总扰动。
% 带宽越高，跟踪越快，但对噪声越敏感；带宽越低，输出越平滑但动态滞后越大。
hall_eso_bandwidth_hz = 60;      % Hz，ESO 观测器带宽，按当前噪声/动态工况折中整定
hall_eso_omega_o_rad_s = 2*pi*hall_eso_bandwidth_hz;  % rad/s，ESO 观测器带宽
hall_eso_beta1 = 3*hall_eso_omega_o_rad_s;            % ESO 角度误差反馈增益
hall_eso_beta2 = 3*hall_eso_omega_o_rad_s^2;          % ESO 角速度误差反馈增益
hall_eso_beta3 = hall_eso_omega_o_rad_s^3;            % ESO 总扰动误差反馈增益

hall_noise_fig_window_s = 0.5;   % s，结果图显示窗口
