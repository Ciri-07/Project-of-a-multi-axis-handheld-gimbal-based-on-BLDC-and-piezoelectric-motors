%% Ideal dual-linear-Hall signal source parameters
% Selected project assumption:
%   18 mm hollow magnet, 12 magnetic poles -> 6 pole pairs.
%   Two linear Hall channels are expected to form an ideal sin/cos pair.

hall_pole_pairs = 6;            % 12 poles -> 6 pole pairs
hall_magnet_od_mm = 18;         % mm, 18mm中空磁钢
%机械角和磁场角的关系是：theta_mag = 6 * theta_m

hall_adc_bits = 16;             % ADC resolution used by the project note
hall_vref_v = 3.3;              % V, ADC / Hall 参考电压
hall_offset_v = hall_vref_v/2;  % V, 理想中点偏置电压
hall_amp_v = 1.0;               % V, ideal AC amplitude, keep away from rails
% Hall 输出电压以 1.65 V 为中心，上下波动 1 V
%所以两路理想电压是：H_s = 1.65 + 1.0 * sin(theta_mag),H_c = 1.65 + 1.0 * cos(theta_mag)
%N 极方向磁场增强 -> 输出高于 1.65 V
%S 极方向磁场增强 -> 输出低于 1.65 V
%上下波动 1 V 是为了避免碰到电源轨。
%真实建模时，1 V 可以换成实际 Hall 灵敏度、磁钢气隙、磁场强度算出来的幅值。

hall_mech_speed_rpm = 30;       % rpm, demonstration mechanical speed
hall_f_mech_hz = hall_mech_speed_rpm/60;
hall_f_mag_hz = hall_pole_pairs*hall_f_mech_hz;%磁场信号频率
hall_omega_mech_rad_s = 2*pi*hall_f_mech_hz;%机械角速度2Πf
hall_theta0_rad = 0;            % rad, 安装零位移偏角

hall_ts_sim_s = 1e-4;           % s, fixed-step simulation sample time
hall_t_stop_s = 2.0;            % s, one mechanical revolution at 30 rpm
%仿真步长是 0.0001 s，仿真总时间是 2 s。因为 30 rpm = 0.5 r/s，所以 2 s 正好是机械转一圈。
