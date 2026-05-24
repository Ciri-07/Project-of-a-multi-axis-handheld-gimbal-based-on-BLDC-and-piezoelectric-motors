%% 双线性霍尔噪声、ADC量化和滤波验证参数
% 本文件在谐波补偿基准参数基础上增加实机采样误差：
%   1. ADC量化；
%   2. Hall/电源/采样白噪声；
%   3. 一阶低通滤波。
%
% 先运行 init_dual_hall_harmonic_params，保证零偏、幅值、相位和谐波
% 参数与当前基准仿真一致。

init_dual_hall_harmonic_params;

hall_noise_seed = 1808;          % 随机种子，保证每次仿真结果可复现
hall_noise_rms_v = 1.0e-3;       % V，单通道白噪声RMS值
hall_adc_enable = true;          % 是否启用ADC量化
hall_adc_lsb_v = hall_vref_v/(2^hall_adc_bits - 1);  % V，ADC最低有效位

hall_lpf_cutoff_hz = 300;        % Hz，一阶低通滤波截止频率
hall_lpf_tau_s = 1/(2*pi*hall_lpf_cutoff_hz);        % s，滤波时间常数
hall_lpf_alpha = exp(-hall_ts_sim_s/hall_lpf_tau_s); % 离散一阶低通系数

hall_noise_fig_window_s = 0.5;   % s，结果图显示窗口
