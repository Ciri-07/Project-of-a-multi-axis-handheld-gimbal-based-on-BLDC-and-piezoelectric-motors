# 双线性 Hall 位置检测中的 LPF、alpha-beta、ESO、互补滤波、Kalman 与残差 LUT 说明

本文档用于说明当前“双线性 Hall 位置检测”仿真中几类角度处理方法的原理、代码实现方式，以及 Simulink 原理模型中的模块含义。

对应工程文件如下。

| 文件 | 作用 |
|---|---|
| `init_dual_hall_noise_filter_params.m` | 设置 ADC、白噪声、LPF、alpha-beta、ESO、Gyro、互补滤波、Kalman 等参数 |
| `validate_dual_hall_noise_filter.m` | 主验证脚本，对比 Hall 原始解角、残差 LUT、LPF、alpha-beta、ESO、互补滤波、Kalman 等方法 |
| `build_dual_hall_discrete_eso_model.m` | 生成可展开查看的 Simulink 原理模型 |
| `models/dual_hall_discrete_eso.slx` | Simulink 原理模型，包含 LPF、alpha-beta、ESO 等模块 |

当前处理链路可以理解为：

```text
两路线性 Hall 电压
    -> ADC 量化 + 白噪声
    -> 零偏 / 幅值 / 相位 / 谐波等确定性补偿
    -> atan2 解角得到 theta_meas
    -> 残差 LUT 抵消剩余周期性自身残差
    -> 角度域估计或融合：LPF / alpha-beta / ESO / Hall+Gyro 互补 / Kalman
    -> 输出角度估计、角速度估计和误差指标
```

需要注意：零偏、幅值、相位、谐波、残差 LUT 属于“确定性误差补偿”；LPF、alpha-beta、ESO、互补滤波、Kalman 更偏向“随机噪声抑制、动态估计和多传感器融合”。二者不是替代关系，而是串联关系。

## 1. 总体方法定位

| 方法 | 输入 | 输出 | 核心作用 | 主要优势 | 主要局限 |
|---|---|---|---|---|---|
| 残差 LUT | 已完成基础补偿后的 Hall 磁场角 | 抵消后的 Hall 磁场角 | 查表抵消周期性自身残差 | 对固定结构误差非常有效 | 依赖标定数据，温漂和装配变化后需重新标定或在线修正 |
| 一阶 LPF | Hall 电压或角度测量值 | 平滑后的电压或角度 | 低通滤波抑制高频噪声 | 简单直观 | 会引入相位滞后，动态时可能变差 |
| alpha-beta | 补偿后的角度 `theta_meas` | 角度 `theta_hat`、角速度 `omega_hat` | 用“角度-速度”模型做预测-校正 | 比直接低通更适合运动角度估计 | 参数依赖运动工况，强冲击下有局限 |
| ESO | 补偿后的角度 `theta_meas` | `z1` 角度、`z2` 角速度、`z3` 总扰动 | 把未知扰动扩展成状态估计 | 可用于未建模扰动和变速工况 | 对带宽敏感，纯白噪声场景不一定优于 alpha-beta |
| Hall+Gyro 互补滤波 | Hall 角度、Gyro 角速度 | 融合角度 | Gyro 短时动态 + Hall 长期校正 | 简单、实时性好 | 权重固定，不能自适应噪声变化 |
| Hall/Gyro Kalman | Hall 角度、Gyro 角速度 | 融合角度、Gyro 零偏估计 | 概率意义上的预测-校正 | 可估计 Gyro 零偏 | 参数 Q/R 不好整定时效果可能一般 |

## 2. 残差 LUT 补偿

### 2.1 为什么需要残差 LUT

前面的零偏、幅值、相位和谐波补偿可以消除大部分确定性误差，但在真实结构中仍可能存在周期性残差，例如：

| 来源 | 表现 |
|---|---|
| 磁钢偏心 | 解角误差随角度周期变化 |
| Hall 安装偏差 | 相图不完全为标准圆 |
| 磁场非理想分布 | 角度误差中存在高阶周期项 |
| PCB / 机械结构不对称 | 残余误差随转角重复出现 |

这类误差不是随机噪声，而是“每转到同一角度都会重复出现”的自身残差。滤波无法真正消除它，只能把曲线变平滑；更合适的方法是用标定数据建立误差表。

### 2.2 核心公式

设基础补偿后的磁场角为：

$$
\theta_{comp}
$$

真实磁场角为：

$$
\theta_{ref}
$$

残差为：

$$
e_{res}(\theta) = wrap(\theta_{comp} - \theta_{ref})
$$

建立 LUT 后，在线补偿为：

$$
\theta_{lut} = \theta_{comp} - \hat e_{res}(\theta_{comp})
$$

机械角误差为：

$$
e_m = \frac{wrap(\theta_{lut} - \theta_{ref})}{p}
$$

其中，`p` 是磁极对数。

### 2.3 代码实现位置

在 `validate_dual_hall_noise_filter.m` 中，先用无噪声标定数据建立基础补偿系数：

```matlab
cal = estimate_baseline_calibration(hall_s_clean_v, hall_c_clean_v, theta_mag, calib_idx, P);
```

然后用基础补偿后的干净角度建立残差表：

```matlab
result_clean_for_lut = apply_compensation_chain(hall_s_clean_v, hall_c_clean_v, theta_mag, cal, P);
cal.residual_lut = build_residual_lut(result_clean_for_lut.theta_anglecomp, theta_mag, calib_idx);
```

后续不同采样链路都使用同一套标定补偿：

```matlab
result_clean = apply_compensation_chain(hall_s_clean_v, hall_c_clean_v, theta_mag, cal, P);
result_adc = apply_compensation_chain(hall_s_adc_v, hall_c_adc_v, theta_mag, cal, P);
result_noisy = apply_compensation_chain(hall_s_noisy_adc_v, hall_c_noisy_adc_v, theta_mag, cal, P);
result_lpf = apply_compensation_chain(hall_s_filt_v, hall_c_filt_v, theta_mag, cal, P);
```

`apply_compensation_chain` 内部执行 LUT 补偿：

```matlab
if isfield(cal, 'residual_lut')
    residual_mag_hat = lookup_residual_lut(theta_anglecomp, cal.residual_lut);
    theta_lutcomp = align_angle(theta_anglecomp - residual_mag_hat, theta_mag);
else
    residual_mag_hat = zeros(size(theta_anglecomp));
    theta_lutcomp = theta_anglecomp;
end
err_lutcomp = wrap_pi(theta_lutcomp - theta_mag)/P.pole_pairs;
```

这里的 `theta_anglecomp` 是零偏、幅值、相位和角度域谐波补偿后的结果；`theta_lutcomp` 是进一步抵消自身残差后的结果。

### 2.4 当前效果

当前仿真中，残差 LUT 把干净信号的自身残差显著压低：

| 方法 | 最大误差 | RMS 误差 |
|---|---:|---:|
| Clean raw | 约 0.1250 deg | 约 0.0418 deg |
| Clean+LUT | 约 0.0019 deg | 约 0.0004 deg |

这说明原来主要限制精度的是周期性确定性残差，而不是 ADC 或白噪声。

## 3. 一阶 LPF

### 3.1 基本原理

LPF 是 Low-Pass Filter，即低通滤波器。它允许低频信号通过，削弱高频噪声。

在 Hall 角度检测中：

| 信号成分 | 典型表现 | LPF 作用 |
|---|---|---|
| 真实低速角度变化 | 低频 | 保留 |
| ADC 量化抖动 | 高频小幅误差 | 削弱 |
| 白噪声 | 宽频随机扰动 | 削弱高频部分 |
| 电源噪声 / 采样噪声 | 高频成分较多 | 削弱 |

但 LPF 会引入滞后。对云台角度反馈来说，滞后会直接变成动态角度误差。

### 3.2 连续形式

$$
\tau \frac{dy(t)}{dt} + y(t) = x(t)
$$

截止频率和时间常数关系为：

$$
f_c = \frac{1}{2\pi\tau}
$$

### 3.3 离散形式

当前工程采用的一阶离散低通形式为：

$$
y[k] = \alpha y[k-1] + (1-\alpha)x[k]
$$

其中：

| 符号 | 含义 |
|---|---|
| `x[k]` | 当前输入 |
| `y[k]` | 当前滤波输出 |
| `y[k-1]` | 上一拍滤波输出 |
| `alpha` | 滤波系数 |

`alpha` 越接近 1，滤波越强，但滞后越大；`alpha` 越接近 0，响应越快，但噪声抑制越弱。

### 3.4 代码实现

参数文件中：

```matlab
hall_lpf_cutoff_hz = 300;
hall_lpf_tau_s = 1/(2*pi*hall_lpf_cutoff_hz);
hall_lpf_alpha = exp(-hall_ts_sim_s/hall_lpf_tau_s);
```

主脚本中，LPF 先作用在采样后的 Hall 电压上：

```matlab
hall_s_filt_v = first_order_lpf(hall_s_noisy_adc_v, hall_lpf_alpha);
hall_c_filt_v = first_order_lpf(hall_c_noisy_adc_v, hall_lpf_alpha);
```

函数实现：

```matlab
function y = first_order_lpf(x, alpha)
y = zeros(size(x));
y(1) = x(1);
for idx = 2:numel(x)
    y(idx) = alpha*y(idx-1) + (1 - alpha)*x(idx);
end
end
```

### 3.5 Simulink 模块对应

角度域 LPF 原理模块对应公式：

$$
\theta_{lpf}[k] = \alpha\theta_{lpf}[k-1] + (1-\alpha)\theta_{meas}[k]
$$

| Block | 类型 | 对应含义 |
|---|---|---|
| `theta_meas` | Inport | 当前测量角度 |
| `alpha` | Inport / Gain | 滤波系数 |
| `1-alpha` | Sum / Gain | 当前测量权重 |
| `Unit Delay theta_lpf` | Unit Delay | 保存上一拍输出 |
| `alpha_times_prev` | Gain | 计算 `alpha*theta_lpf[k-1]` |
| `one_minus_alpha_times_meas` | Gain | 计算 `(1-alpha)*theta_meas[k]` |
| `theta_lpf_next` | Sum | 两项相加得到当前滤波输出 |
| `theta_lpf` | Outport | 输出滤波角度 |

## 4. alpha-beta 估计

### 4.1 基本思想

alpha-beta 估计器不是简单低通，而是用“位置-速度”模型做预测，然后用当前测量修正。

内部状态为：

| 状态 | 含义 |
|---|---|
| `theta_hat` | 角度估计 |
| `omega_hat` | 角速度估计 |

### 4.2 核心公式

预测：

$$
\theta_{pred}[k] = \theta_{hat}[k-1] + T_s\omega_{hat}[k-1]
$$

$$
\omega_{pred}[k] = \omega_{hat}[k-1]
$$

测量残差：

$$
e[k] = \theta_{meas}[k] - \theta_{pred}[k]
$$

校正：

$$
\theta_{hat}[k] = \theta_{pred}[k] + \alpha e[k]
$$

$$
\omega_{hat}[k] = \omega_{pred}[k] + \frac{\beta}{T_s}e[k]
$$

### 4.3 代码实现

主脚本调用：

```matlab
[theta_ab_mag, omega_ab_mag] = alpha_beta_filter( ...
    result_noisy.theta_lutcomp, hall_ts_sim_s, hall_ab_alpha, hall_ab_beta);
theta_ab_mag = align_angle(theta_ab_mag, theta_mag);
err_ab = wrap_pi(theta_ab_mag - theta_mag)/P.pole_pairs;
```

这里注意：当前输入已经改成 `result_noisy.theta_lutcomp`，即先经过残差 LUT 抵消自身残差，再进入 alpha-beta。

函数实现：

```matlab
function [theta_hat, omega_hat] = alpha_beta_filter(theta_meas, Ts, alpha, beta)
theta_meas = theta_meas(:);
theta_hat = zeros(size(theta_meas));
omega_hat = zeros(size(theta_meas));

theta_hat(1) = theta_meas(1);
if numel(theta_meas) >= 2
    omega_hat(1) = (theta_meas(2) - theta_meas(1))/Ts;
end

for idx = 2:numel(theta_meas)
    theta_pred = theta_hat(idx-1) + Ts*omega_hat(idx-1);
    omega_pred = omega_hat(idx-1);

    innovation = theta_meas(idx) - theta_pred;
    theta_hat(idx) = theta_pred + alpha*innovation;
    omega_hat(idx) = omega_pred + beta/Ts*innovation;
end
end
```

### 4.4 Simulink 模块对应

| Block | 对应公式 | 作用 |
|---|---|---|
| `Unit Delay theta_hat` | `theta_hat[k-1]` | 保存上一拍角度估计 |
| `Unit Delay omega_hat` | `omega_hat[k-1]` | 保存上一拍角速度估计 |
| `theta_pred` | `theta_hat[k-1] + Ts*omega_hat[k-1]` | 角度预测 |
| `e` | `theta_meas - theta_pred` | 测量残差 |
| `alpha` | `alpha*e` | 角度修正量 |
| `eso_ab_beta` | `beta*e` | 速度修正前半部分 |
| `1/eso_Ts` | `beta/Ts*e` | 速度修正量 |
| `theta_hat_next` | `theta_pred + alpha*e` | 当前角度估计 |
| `omega_hat_next` | `omega_hat[k-1] + beta/Ts*e` | 当前角速度估计 |

## 5. 离散 ESO

### 5.1 基本思想

ESO 是 Extended State Observer，扩张状态观测器。它把未建模误差、外部扰动、残余动态影响统一看成一个“总扰动”状态。

当前离散 ESO 维护三个状态：

| 状态 | 含义 |
|---|---|
| `z1` | 角度估计，近似 `theta` |
| `z2` | 角速度估计，近似 `omega` |
| `z3` | 总扰动估计，近似 `d` |

其中 `d` 不是单独测出来的物理量，而是模型无法解释的剩余变化，例如剩余谐波、噪声、安装微小变化、外部振动、未建模动态等。

### 5.2 核心公式

测量误差：

$$
e[k] = \theta_{meas}[k] - z_1[k]
$$

状态更新：

$$
z_1[k+1] = z_1[k] + T_s(z_2[k] + \beta_1 e[k])
$$

$$
z_2[k+1] = z_2[k] + T_s(z_3[k] + \beta_2 e[k])
$$

$$
z_3[k+1] = z_3[k] + T_s\beta_3 e[k]
$$

带宽整定：

$$
\beta_1 = 3\omega_o
$$

$$
\beta_2 = 3\omega_o^2
$$

$$
\beta_3 = \omega_o^3
$$

$$
\omega_o = 2\pi f_o
$$

### 5.3 代码实现

带宽转换：

```matlab
function [beta1, beta2, beta3, omegaO] = eso_gains_from_bandwidth(bandwidthHz)
omegaO = 2*pi*bandwidthHz;
beta1 = 3*omegaO;
beta2 = 3*omegaO^2;
beta3 = omegaO^3;
end
```

ESO 滤波：

```matlab
function [z1, z2, z3] = discrete_eso_filter(theta_meas, Ts, beta1, beta2, beta3)
theta_meas = theta_meas(:);
z1 = zeros(size(theta_meas));
z2 = zeros(size(theta_meas));
z3 = zeros(size(theta_meas));

z1(1) = theta_meas(1);
z2(1) = 0;
z3(1) = 0;

for idx = 2:numel(theta_meas)
    err = theta_meas(idx-1) - z1(idx-1);
    z1(idx) = z1(idx-1) + Ts*(z2(idx-1) + beta1*err);
    z2(idx) = z2(idx-1) + Ts*(z3(idx-1) + beta2*err);
    z3(idx) = z3(idx-1) + Ts*(beta3*err);
end
end
```

主脚本调用：

```matlab
[theta_eso_mag, omega_eso_mag, disturbance_eso_mag] = discrete_eso_filter( ...
    result_noisy.theta_lutcomp, hall_ts_sim_s, ...
    hall_eso_beta1, hall_eso_beta2, hall_eso_beta3);
```

同样，ESO 的输入也是 `theta_lutcomp`，即先做自身残差抵消，再做动态估计。

### 5.4 Simulink 模块对应

| Block | 公式 | 作用 |
|---|---|---|
| `theta_meas` | 输入 | 当前测量角度 |
| `e = theta_meas - z1` | `e[k]` | 测量残差 |
| `Unit Delay z1` | `z1[k]` | 保存上一拍角度估计 |
| `Unit Delay z2` | `z2[k]` | 保存上一拍角速度估计 |
| `Unit Delay z3` | `z3[k]` | 保存上一拍扰动估计 |
| `eso_beta1` | `beta1*e` | 角度误差反馈 |
| `eso_beta2` | `beta2*e` | 角速度误差反馈 |
| `eso_beta3` | `beta3*e` | 扰动误差反馈 |
| `eso_Ts1` | `Ts*beta1*e` | 角度通道离散积分项 |
| `eso_Ts2` | `Ts*(z3 + beta2*e)` | 角速度通道离散积分项 |
| `eso_Ts3` | `Ts*beta3*e` | 扰动通道离散积分项 |
| `z1_next` | `z1 + Ts*(z2 + beta1*e)` | 下一拍角度 |
| `z2_next` | `z2 + Ts*(z3 + beta2*e)` | 下一拍角速度 |
| `z3_next` | `z3 + Ts*beta3*e` | 下一拍扰动 |

## 6. Hall + Gyro 互补滤波

### 6.1 为什么要互补滤波

Hall 和 Gyro 的优势互补：

| 传感器 | 优点 | 缺点 |
|---|---|---|
| Hall | 能给出绝对或准绝对角度，不会无限漂移 | 受磁场、安装、ADC、噪声和残差影响 |
| Gyro | 高频动态好，短时间角速度响应快 | 有零偏，积分后会长期漂移 |

互补滤波的核心思想是：

```text
短时间相信 Gyro 的动态响应
长时间用 Hall 把漂移拉回来
```

### 6.2 核心公式

先用 Gyro 积分预测角度：

$$
\theta_{pred}[k] = \theta_{fused}[k-1] + T_s\omega_{gyro}[k]
$$

再用 Hall 角度校正：

$$
\theta_{fused}[k] = \alpha\theta_{pred}[k] + (1-\alpha)\theta_{hall}[k]
$$

其中：

| 符号 | 含义 |
|---|---|
| `theta_hall` | Hall 解算角度 |
| `omega_gyro` | Gyro 测得角速度 |
| `theta_pred` | Gyro 积分预测角度 |
| `theta_fused` | 融合输出角度 |
| `alpha` | Gyro 预测权重 |

`alpha` 越接近 1，越相信 Gyro 的短期动态；`alpha` 越小，越快被 Hall 拉回。

### 6.3 代码实现

生成 Gyro 测量：

```matlab
gyro = make_gyro_measurement( ...
    data.omega_true, dyn_t, ...
    hall_gyro_bias_rad_s, hall_gyro_bias_drift_rad_s2, ...
    hall_gyro_noise_rms_rad_s, hall_noise_seed + 100 + s);
```

互补滤波调用：

```matlab
thetaComp = complementary_hall_gyro_filter(data.theta_meas, gyro, hall_ts_sim_s, hall_comp_alpha);
thetaComp = align_angle(thetaComp, data.theta_true);
```

函数实现：

```matlab
function theta_fused = complementary_hall_gyro_filter(theta_hall, omega_gyro, Ts, alpha)
theta_hall = theta_hall(:);
omega_gyro = omega_gyro(:);
theta_fused = zeros(size(theta_hall));
theta_fused(1) = theta_hall(1);
for idx = 2:numel(theta_hall)
    theta_pred = theta_fused(idx-1) + Ts*omega_gyro(idx);
    theta_fused(idx) = alpha*theta_pred + (1 - alpha)*theta_hall(idx);
end
end
```

### 6.4 当前仿真结论

在变速、冲击、负载扰动等动态场景中，互补滤波通常比单独 Hall、alpha-beta、ESO 更稳，因为它引入了 Gyro 的高频动态信息。

但互补滤波不是万能的：

| 情况 | 表现 |
|---|---|
| Gyro 零偏较大 | 长时间仍会有残余漂移，需要 Hall 拉回 |
| Hall 残差未补偿 | Hall 会把周期性误差带入融合结果 |
| 权重 `alpha` 不合适 | 可能过度相信 Gyro 或过度相信 Hall |

因此当前推荐链路是：

```text
Hall 确定性补偿 + 残差 LUT -> Hall 角度
Gyro -> 角速度
Hall + Gyro 互补滤波 -> 动态姿态角反馈
```

## 7. Hall/Gyro Kalman 滤波

### 7.1 基本思想

Kalman 滤波也是预测-校正结构，但它用噪声协方差来决定“相信模型还是相信测量”。当前脚本使用的是二状态 Hall/Gyro Kalman：

| 状态 | 含义 |
|---|---|
| `theta` | 角度 |
| `bias` | Gyro 零偏 |

### 7.2 状态模型

Gyro 测量可写为：

$$
\omega_{gyro} = \omega + b + n_g
$$

其中 `b` 是 Gyro 零偏。

预测角度：

$$
\theta_{pred}[k] = \theta[k-1] + T_s(\omega_{gyro}[k] - b[k-1])
$$

零偏近似慢变：

$$
b_{pred}[k] = b[k-1]
$$

写成状态形式：

$$
x[k] =
\begin{bmatrix}
\theta[k] \\
b[k]
\end{bmatrix}
$$

$$
x_{pred}[k] =
\begin{bmatrix}
\theta[k-1] + T_s(\omega_{gyro}[k] - b[k-1]) \\
b[k-1]
\end{bmatrix}
$$

Hall 测量模型：

$$
z[k] = \theta_{hall}[k] + v
$$

### 7.3 代码实现

主脚本调用：

```matlab
[thetaKalman, gyroBiasHat] = kalman_hall_gyro_filter( ...
    data.theta_meas, gyro, hall_ts_sim_s, ...
    hall_kalman_q_angle, hall_kalman_q_bias, hall_kalman_r_hall);
thetaKalman = align_angle(thetaKalman, data.theta_true);
```

函数中主要逻辑：

```matlab
theta_pred = x(1) + Ts*(omega_gyro(idx) - x(2));
bias_pred = x(2);
x_pred = [theta_pred; bias_pred];
```

这一步是用 Gyro 做预测，并扣除估计到的零偏。

协方差预测：

```matlab
F = [1, -Ts; 0, 1];
Q = diag([qTheta, qBias]);
Pcov = F*Pcov*F' + Q;
```

Hall 测量校正：

```matlab
H = [1, 0];
innovation = wrap_pi(theta_hall(idx) - x_pred(1));
S = H*Pcov*H' + rHall;
K = Pcov*H'/S;
x = x_pred + K*innovation;
Pcov = (eye(2) - K*H)*Pcov;
```

### 7.4 参数含义

| 参数 | 含义 | 调大后的效果 |
|---|---|---|
| `hall_kalman_q_angle` | 角度模型过程噪声 | 更不相信模型，更容易跟随 Hall |
| `hall_kalman_q_bias` | Gyro 零偏变化噪声 | 允许零偏估计变化更快 |
| `hall_kalman_r_hall` | Hall 测量噪声 | 越大越不相信 Hall，越依赖 Gyro |

如果 Q/R 设置不合理，Kalman 不一定比互补滤波好。当前仿真中互补滤波表现更直观，Kalman 更适合作为后续实机阶段的可扩展方案。

## 8. 几种方法的关系

### 8.1 推荐工程链路

```mermaid
flowchart LR
    A[两路 Hall 电压] --> B[ADC 采样]
    B --> C[零偏/幅值/相位补偿]
    C --> D[谐波补偿]
    D --> E[atan2 解角]
    E --> F[残差 LUT]
    F --> G[Hall 角度]
    H[Gyro 角速度] --> I[互补滤波或 Kalman]
    G --> I
    I --> J[姿态/位置反馈]
```

### 8.2 方法优先级

| 阶段 | 推荐方法 | 目的 |
|---|---|---|
| 离线标定 | 零偏、幅值、相位、谐波、残差 LUT | 先消除确定性误差 |
| 单 Hall 角度估计 | alpha-beta 或 ESO | 抑制随机噪声并输出角速度 |
| 动态云台控制 | Hall + Gyro 互补滤波 | 兼顾低频绝对角和高频动态 |
| 后续高精度实机 | Kalman / EKF | 估计 Gyro 零偏、支持更多传感器融合 |

## 9. 当前仿真结果如何理解

### 9.1 自身残差被 LUT 抵消

当前结果中：

| 项目 | 说明 |
|---|---|
| `Clean raw` | 已做基础补偿，但仍保留周期性自身残差 |
| `Clean+LUT` | 用残差 LUT 抵消周期性残差后，误差明显降低 |
| `Noise+LUT` | 在随机噪声存在时，确定性残差已不是主导项 |

这说明：要达到 `≤±0.005° @1σ`，不能只靠滤波，必须先把确定性残差压下去。

### 9.2 LPF 为什么可能变差

LPF 在电压域会让波形变平滑，但对运动信号会引入相位滞后。低速时滞后较小，高速或动态时滞后会直接变成角度误差。

因此 LPF 更适合做轻量级前端抗噪，不适合单独作为高精度角度估计核心。

### 9.3 alpha-beta 为什么低速表现好

低速扫角时，角速度变化缓慢，alpha-beta 的“角度-速度”假设成立，因此可以在不引入明显相位滞后的情况下压低随机噪声。

### 9.4 ESO 为什么不一定更好

ESO 的优势在于估计未建模扰动和动态变化。但在当前“恒速 + 白噪声”场景中，主要问题是随机噪声，而不是强外部扰动，所以 ESO 不一定明显优于 alpha-beta。

### 9.5 互补滤波为什么适合云台

云台实际工作中存在手持抖动、冲击、姿态变化等动态问题。此时 Gyro 的高频动态响应很有价值，Hall 则负责长期绝对角约束。互补滤波结构简单，适合先作为工程可行方案。

## 10. 后续建议

后续可以按以下顺序推进：

1. 保留残差 LUT 作为 Hall 角度链路的基础补偿。
2. 用低速扫角验证 `≤±0.005° @1σ`，重点看 `Alpha-beta`、`Kalman` 和 `Complementary`。
3. 加入温漂和分段标定，验证 LUT 在不同温度和装配偏差下是否仍有效。
4. 引入 IMU 姿态角，进一步从单轴 Hall+Gyro 扩展到多轴姿态融合。
5. 实机阶段优先做离线标定表，再考虑在线自适应补偿。

