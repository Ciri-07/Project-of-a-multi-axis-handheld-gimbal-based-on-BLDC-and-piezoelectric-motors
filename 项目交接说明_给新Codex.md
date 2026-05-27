# 项目交接说明：多轴手持云台与双线性 Hall 位置检测

更新时间：2026-05-27  
项目目录：`D:\Desktop\基于无刷电机和压电马达的多轴手机云台\BO1808 NBH2B Simulation Modeling`

这份文档用于在更换账号或开启新对话后快速恢复当前协作状态。新的 AI/Codex 读取本文后，应尽量延续当前对话的协作习惯、工程上下文、文件路径、仿真路线和讲解风格。

---

## 1. 用户协作习惯

用户偏好：

- 主要用中文沟通，解释和思考过程尽量用中文。
- 用户正在做项目汇报和理论准备，很多内容需要能直接放进 PPT 或作为汇报讲稿。
- 回答不要只给结论，要能解释“为什么”，尤其是电机控制、Simulink 模型、传感器补偿算法、结果图含义。
- 对代码解释时，用户经常要求“每一行什么意思”，需要逐行说明变量、公式和物理意义。
- 用户很在意 Simulink 布局，不要随便强制重建或自动排版模型。
- 如果必须新增模块，可以新增，但要说明会影响布局；用户可以自己手动调整布局。
- 线性霍尔传感器相关文件默认都保存到 `线性霍尔传感器` 文件夹，除非用户明确说保存到根目录。
- 用户经常用截图询问“这一页能说清楚吗”“怎么讲”，需要给出简洁讲述文案。
- 用户喜欢先从直观现象理解，再过渡到公式。
- 如果 VS Code 显示内容和磁盘文件不一致，要提醒可能是编辑器缓存，不要直接 `Ctrl+S` 覆盖新文件。

建议回复风格：

- 先给判断，再给原因，再给可直接使用的表述。
- 复杂公式尽量配一句“它在做什么”。
- 对图解释按“信号现象 -> 相图形状 -> 解角误差 -> 工程结论”的顺序。
- 对代码解释按“变量含义 -> 数学公式 -> 为什么这样做 -> 实机意义”的顺序。

---

## 2. 项目总体背景

项目主题：基于无刷电机和压电马达的多轴手持云台，目标是为后续特制电机和多自由度大角度级联系统做高精度检测与控制算法准备。

当前主要方向包括：

- 直流无刷电机干扰环境匀速控制算法。
- 三轴耦合条件下的动态云台控制算法。
- 传感器方向：陀螺、加速度计、线性 Hall、互补滤波/融合算法，用于实现手持相机云台高精度控制。

用户目前主要负责第三个方向，即：

```text
双线性 Hall 位置检测 + IMU/陀螺/加速度计融合 + 后续云台控制接口
```

---

## 3. 当前重点：双线性 Hall 位置检测与融合仿真

当前主线选择：

```text
归一化 + 谐波重构 + ESO 复合信号提取
```

当前已经完成的仿真链路已经从“理想 Hall 信号”推进到“接近真实采样链路和融合算法验证”：

```text
理想正余弦信号
-> 零偏误差
-> 幅值不一致
-> 相位不正交误差
-> 采样估计零偏/幅值/相位
-> atan2 解角误差对比
-> 2/3/5/6 次谐波注入、FFT 识别与谐波补偿
-> ADC 量化、白噪声、一阶低通滤波
-> 角度域 alpha-beta 估计
-> 离散 ESO 估计
-> Hall + Gyro 互补滤波和 Kalman 融合
-> 0.5 / 1 / 5 rpm 低速扫角精度验证
-> 变速、冲击、负载扰动工况
-> 更强冲击、gyro 零偏漂移、温漂、轴间耦合和综合强扰动工况
```

当前阶段的重要结论：

```text
1. 在低速准静态扫角下，当前 0.5 mV Hall 噪声设置中，Hall raw 的 1σ 约 0.0055°，略高于 ≤±0.005° 指标。
2. alpha-beta、离散 ESO、互补滤波和 Kalman 在低速指标验证中可以压到 0.005° 以内。
3. 电压域 LPF 虽能压高频噪声，但会引入相位滞后，在角度估计中不一定更好。
4. alpha-beta 简单、稳定，适合作为低速角度估计基线。
5. Hall + Gyro 互补滤波对动态跟踪更直观，适合后续云台姿态/速度融合接口。
6. Kalman 有扩展潜力，但依赖 Q/R、gyro 零偏模型和真实 IMU 数据标定。
7. ESO 对未建模扰动有意义，但在当前恒速+白噪声工况下优势不明显，带宽需要随工况调。
8. 强冲击、温漂和轴间耦合下，仅靠滤波算法不能消除确定性污染，需要机械标定、轴间解耦、温度补偿和实测数据闭环。
```

---

## 4. 关键文件位置

双线性 Hall 当前核心文件：

```text
线性霍尔传感器/init_dual_hall_ideal_params.m
线性霍尔传感器/build_dual_hall_ideal_sincos_model.m
线性霍尔传感器/validate_dual_hall_ideal_sincos.m
线性霍尔传感器/models/dual_hall_ideal_sincos.slx
线性霍尔传感器/figures/dual_hall_ideal_sincos_validation.png
线性霍尔传感器/init_dual_hall_noise_filter_params.m
线性霍尔传感器/validate_dual_hall_noise_filter.m
线性霍尔传感器/build_dual_hall_discrete_eso_model.m
线性霍尔传感器/models/dual_hall_discrete_eso.slx
线性霍尔传感器/build_dual_hall_harmonic_baseline_model.m
线性霍尔传感器/validate_dual_hall_harmonic_fft_compensation.m
线性霍尔传感器/init_dual_hall_harmonic_params.m
线性霍尔传感器/LPF_alpha-beta_ESO原理与Simulink实现说明.md
线性霍尔传感器/双线性霍尔位置检测方案规划与学习路线.md
线性霍尔传感器/线性霍尔高精度位置检测与补偿方法章节.md
线性霍尔传感器/README.md
```

当前最重要的验证入口是：

```matlab
cd('D:/Desktop/基于无刷电机和压电马达的多轴手机云台/BO1808 NBH2B Simulation Modeling/线性霍尔传感器')
validate_dual_hall_noise_filter
```

该脚本会生成低速精度、噪声/滤波、动态扰动、Hall+Gyro 融合和强扰动工况的主要结果图与 CSV 指标。

近期关键输出文件：

```text
线性霍尔传感器/figures/dual_hall_noise_lpf_ab_eso_validation.png
线性霍尔传感器/figures/dual_hall_low_speed_detection_validation.png
线性霍尔传感器/figures/dual_hall_dynamic_conditions_validation.png
线性霍尔传感器/figures/dual_hall_gyro_fusion_validation.png
线性霍尔传感器/figures/dual_hall_stress_conditions_validation.png
线性霍尔传感器/figures/dual_hall_stress_conditions_metrics.csv
```

线性 Hall 参考资料：

```text
线性霍尔传感器/01_TI_Linear_Hall_Effect_Sensor_Angle_Measurement_SLYA036B.pdf
线性霍尔传感器/02_Self_Calibrating_Position_Measurements_Imperfect_Hall_Sensors_2025.pdf
线性霍尔传感器/03_TIE2021_Position_Extraction_Ultralow_Speed_Gimbal_Linear_Hall.pdf
线性霍尔传感器/04_TMECH2022_3D_Printed_Halbach_Cylinder_Motor_Self_Position_Sensing.pdf
```

BO1808 电机建模相关文件：

```text
init_bo1808_params.m
build_bo1808_modular_model.m
build_bo1808_foc_id0_model.m
build_bo1808_foc_svpwm_model.m
validate_bo1808_fig3_9.m
validate_bo1808_model_against_datasheet.m
README_BO1808_PMSM.md
```

---

## 5. 当前双线性 Hall 参数

文件：`线性霍尔传感器/init_dual_hall_ideal_params.m`

当前关键参数：

```matlab
hall_pole_pairs = 6;            % 12 poles -> 6 pole pairs
hall_magnet_od_mm = 18;         % mm，18 mm 中空磁钢

hall_adc_bits = 16;             % ADC resolution
hall_vref_v = 3.3;              % V，ADC / Hall 参考电压
hall_offset_v = hall_vref_v/2;  % V，理想中点偏置电压，1.65 V

hall_amp_s_v = 1.0;             % V，正弦通道实际幅值 A_s
hall_amp_c_v = 0.85;            % V，余弦通道实际幅值 A_c
hall_amp_v = hall_amp_s_v;      % V，未补偿对比用名义幅值

hall_phase_err_deg = 5;         % deg，余弦通道相位不正交误差 Δφ
hall_phase_err_rad = deg2rad(hall_phase_err_deg);

hall_offset_err_s_v = 0.05;     % V，正弦通道零偏误差
hall_offset_err_c_v = -0.08;    % V，余弦通道零偏误差

hall_offset_s_actual_v = hall_offset_v + hall_offset_err_s_v;  % 1.70 V
hall_offset_c_actual_v = hall_offset_v + hall_offset_err_c_v;  % 1.57 V

hall_mech_speed_rpm = 30;       % rpm
hall_f_mech_hz = hall_mech_speed_rpm/60;
hall_f_mag_hz = hall_pole_pairs*hall_f_mech_hz;
hall_omega_mech_rad_s = 2*pi*hall_f_mech_hz;
hall_theta0_rad = 0;

hall_ts_sim_s = 1e-4;
hall_t_stop_s = 2.0;
```

当前信号模型：

```math
\theta_m(t)=\omega_m t
```

```math
\theta_{mag}(t)=p\theta_m(t)+\theta_0
```

```math
H_s = V_{0s}+A_s\sin(\theta_{mag})
```

```math
H_c = V_{0c}+A_c\cos(\theta_{mag}+\Delta\phi)
```

---

## 6. 当前补偿算法逻辑

当前验证脚本：`线性霍尔传感器/validate_dual_hall_ideal_sincos.m`

### 6.1 未补偿

按理想中点和名义幅值归一化：

```matlab
hall_sin_norm_raw = (hall_sin_v - hall_offset_v)/hall_amp_v;
hall_cos_norm_raw = (hall_cos_v - hall_offset_v)/hall_amp_v;
```

含义：

```text
仍假设两路中点都是 1.65 V，幅值都是 1.0 V。
此时会同时残留零偏、幅值不一致、相位不正交误差。
```

### 6.2 已知参数补偿

用仿真中已知的真实中点和真实幅值归一化：

```matlab
hall_sin_norm = (hall_sin_v - hall_offset_s_actual_v)/hall_amp_s_v;
hall_cos_norm = (hall_cos_v - hall_offset_c_actual_v)/hall_amp_c_v;
```

含义：

```text
用于验证补偿理论是否正确。
这一步可以消除零偏和幅值不一致，但不能消除相位不正交。
```

### 6.3 实机估计零偏和幅值

用采样数据估计：

```matlab
hall_offset_s_est_v = 0.5*(max(hall_sin_calib_v) + min(hall_sin_calib_v));
hall_offset_c_est_v = 0.5*(max(hall_cos_calib_v) + min(hall_cos_calib_v));

hall_amp_s_est_v = 0.5*(max(hall_sin_calib_v) - min(hall_sin_calib_v));
hall_amp_c_est_v = 0.5*(max(hall_cos_calib_v) - min(hall_cos_calib_v));
```

含义：

```text
实机上通常不知道真实零偏和真实幅值。
因此让电机/云台慢速扫过至少一个完整磁周期，通过最大值和最小值估计中点和幅值。
```

### 6.4 相位误差估计

相位误差模型：

```math
x_s=\sin\theta
```

```math
x_c=\cos(\theta+\Delta\phi)
```

计算两路归一化信号相关系数：

```matlab
hall_phase_corr_coef = mean(hall_sin_calib_zero_mean.*hall_cos_calib_zero_mean) / ...
    sqrt(mean(hall_sin_calib_zero_mean.^2)*mean(hall_cos_calib_zero_mean.^2));
```

再估计相位误差：

```matlab
hall_phase_err_est_rad = asin(-hall_phase_corr_coef);
```

解释：

```text
理想正交的 sin 和 cos 在完整周期内相关系数应接近 0。
如果余弦通道变成 cos(theta + Δφ)，两路之间会出现非零相关性。
近似关系为 rho ≈ -sin(Δφ)，所以 Δφ ≈ asin(-rho)。
```

### 6.5 相位补偿

根据恒等式：

```math
\cos(\theta+\Delta\phi)
=\cos\theta\cos\Delta\phi-\sin\theta\sin\Delta\phi
```

可反推：

```math
\cos\theta =
\frac{x_c+x_s\sin\Delta\phi}{\cos\Delta\phi}
```

代码：

```matlab
hall_cos_norm_est_phasecorr = ...
    (hall_cos_norm_est + hall_sin_norm_est*sin(hall_phase_err_est_rad))/cos(hall_phase_err_est_rad);
```

含义：

```text
用采样估计得到的 Δφ_est 修正余弦通道，
恢复更接近理想 cos(theta) 的信号，再交给 atan2 解角。
```

---

## 7. 当前仿真结果

运行：

```matlab
validate_dual_hall_ideal_sincos
```

最近一次结果：

```text
Actual offsets: H_s 1.700 V, H_c 1.570 V
Actual amplitudes: A_s 1.000 V, A_c 0.850 V
Actual phase error: Δφ 5.000 deg

Sample-estimated offsets: H_s 1.700 V, H_c 1.570 V
Sample-estimated amplitudes: A_s 1.000 V, A_c 0.850 V
Sample-estimated phase error: Δφ 5.000 deg

Max raw angle decoding error = 3.916e-02 rad
Max offset/amplitude-compensated angle decoding error = 1.455e-02 rad
Max sample-estimated offset/amplitude angle decoding error = 1.455e-02 rad
Max known-phase-corrected angle decoding error = 6.245e-09 rad
Max sample-estimated phase-corrected angle decoding error = 7.124e-07 rad
```

换算为角度：

```text
未补偿最大误差：约 2.24°
补偿零偏 + 幅值后：约 0.83°
补偿零偏 + 幅值 + 相位后：约 0.000041°
```

结论：

```text
只补偿零偏和幅值还不够，相位不正交仍然会造成周期性角度误差。
加入相位估计和补偿后，atan2 解角误差基本被消除。
```

---

## 8. 当前结果图怎么看

结果图：

```text
线性霍尔传感器/figures/dual_hall_ideal_sincos_validation.png
```

四张子图含义：

### 图 a：Hall 电压随时间变化

蓝色是 `H_s`，红色是 `H_c`。

当前同时加入：

```text
零偏误差：H_s 中点 1.70 V，H_c 中点 1.57 V
幅值不一致：A_s = 1.00 V，A_c = 0.85 V
相位误差：Δφ = 5°
```

### 图 b：Hall 电压随机械角变化

横轴从时间换成机械角 `θ_m`。  
因为磁钢是 6 对极，所以机械角一圈内 Hall 信号变化 6 个周期。

### 图 c：Lissajous 相图

横轴是归一化 `H_c`，纵轴是归一化 `H_s`。

```text
灰色：未补偿，受零偏、幅值、相位共同影响。
黑色：补偿零偏和幅值后，仍有相位不正交导致的倾斜/变形。
蓝色虚线：补偿相位后，接近标准圆。
```

### 图 d：atan2 解角误差

```text
红色：未补偿，误差最大。
蓝色：补偿零偏和幅值，误差下降但仍有周期波动。
绿色虚线：进一步补偿相位，误差接近 0。
```

汇报时可用一句话：

```text
双线性 Hall 高精度解角需要按“零偏补偿、幅值归一化、相位补偿”的顺序逐步校正，否则相位不正交会保留下明显的周期性角度误差。
```

---

## 9. Simulink 模型注意事项

模型文件：

```text
线性霍尔传感器/models/dual_hall_ideal_sincos.slx
```

构建脚本：

```text
线性霍尔传感器/build_dual_hall_ideal_sincos_model.m
```

当前模型已经插入 `Δφ` 模块：

```text
θ_mag -> Δφ -> cos -> A_c -> V₀c
```

默认运行：

```matlab
build_dual_hall_ideal_sincos_model()
```

行为：

```text
加载已有模型。
刷新参数到模型内存。
不强制保存 .slx。
不重排布局。
```

强制重建：

```matlab
build_dual_hall_ideal_sincos_model(true)
```

行为：

```text
先捕获当前 .slx 中已有模块 Position。
重建模型。
按模块名恢复 Position。
保存模型。
```

注意：

```text
如果用户手动调整了 Simulink 布局，应保存 .slx。
以后强制重建时，脚本会尽量恢复这些手动 Position。
新增模块第一次可能需要用户手动摆一下，保存后下次也会恢复。
```

---

## 10. VS Code 文件刷新问题

曾出现情况：

```text
磁盘文件已经被 Codex 修改，但 VS Code 左侧编辑器仍显示旧内容。
```

原因：

```text
VS Code 打开文件后会保留编辑器缓冲区。
如果外部工具修改磁盘文件，而 VS Code 没有刷新，就会看到旧版本。
```

重要提醒：

```text
不要在旧缓冲区里直接 Ctrl+S，否则可能把旧内容覆盖回磁盘。
```

处理方法：

```text
关闭文件标签页。
如果提示保存，选 Don't Save / 不保存。
重新从资源管理器打开文件。
```

或者：

```text
Ctrl + Shift + P
File: Revert File
```

---

## 11. 汇报讲述模板

### 11.1 理想 Hall 归一化

```text
这一页主要是在建立双线性 Hall 的理想信号模型。假设两个 Hall 传感器安装理想，输出分别是正弦和余弦信号，机械角经过磁钢极对数转换成磁场角后，就可以得到两路 Hall 电压信号。这里设置机械转速为 30 rpm，用 Simulink 生成两路理想正余弦信号。可以看到，两路信号相位相差 90°，归一化后的相图是一个标准圆，说明零偏、幅值和相位都是理想的。最后通过 atan2 对正余弦信号进行角度解算，解算角度和真实机械角基本重合，说明这个理想模型是正确的。后续的零偏、幅值不一致、谐波和噪声补偿，都是在这个理想模型基础上逐步加入的。
```

### 11.2 零偏和幅值不一致

```text
这一页主要说明双线性 Hall 信号中的两个基础误差：零偏误差和幅值不一致。理想情况下，两路 Hall 信号应该是标准的正弦和余弦，且中点电压相同、幅值相同。但实际安装中，由于 Hall 器件灵敏度、磁钢气隙、安装位置和模拟前端增益不同，两路信号可能出现中点偏移和幅值不一致。仿真结果表明，只存在零偏时，未补偿最大角度误差约 0.90°；进一步加入幅值不一致后，误差增加到约 1.75°。通过采样数据估计两路信号的中点和幅值，再分别归一化，可以将相图恢复到标准圆附近，角度误差也基本被消除。这一步是后续相位补偿和谐波补偿的基础。
```

### 11.3 相位不正交误差与补偿

```text
这一页是在零偏和幅值误差的基础上，进一步说明两路 Hall 相位不正交带来的影响。理想情况下，归一化后的两路信号应该是 sinθ 和 cosθ，也就是严格相差 90°。但实际安装中，由于 Hall 位置误差、磁钢偏心或装配误差，余弦通道可能变成 cos(θ+Δφ)，这里仿真中设置相位误差为 5°。从相图可以看到，只补偿零偏和幅值后，图形仍然不是标准圆，而是存在倾斜变形，说明相位误差还没有被消除。右下角的解角误差也能看到，未补偿时最大误差约 2.24°，补偿零偏和幅值后误差下降到约 0.83°，但仍有周期性角度误差。最后利用两路归一化信号的相关性估计 Δφ，并用相位补偿公式恢复理想余弦分量后，解角误差基本接近 0，说明相位不正交误差得到了有效补偿。
```

---

## 12. 用户下一步建议

当前建议下一步做的是“从仿真验证转向实测标定和工程闭环”，而不是继续只堆滤波算法。

优先级建议：

```text
1. 固定当前低速精度、噪声滤波、动态扰动和强扰动仿真的基准版本。
2. 整理 Git 工作区：决定哪些结果图/CSV 需要纳入版本管理，哪些只是临时输出。
3. 做实机标定流程设计：低速扫角采集 Hall 原始值，同步参考角度，估计零偏、幅值、相位和谐波参数。
4. 做查表/LUT 或分段补偿验证，用实测角度残差建立补偿表。
5. 加入温度维度：验证 Hall 零偏、幅值、相位和 gyro bias 随温度变化的影响。
6. 加入 IMU 实测或更真实的 IMU 噪声/零偏模型，继续验证 Hall + Gyro 互补滤波/Kalman。
7. 建立多轴耦合接口：用安装矩阵或耦合矩阵描述各级大角度转动下的轴间影响。
8. 将检测精度、动态误差和云台控制指标关联起来，例如像素抖动、姿态保持误差、抗冲击恢复时间。
```

已经完成但仍可作为历史基线保留的内容：

```text
理想正余弦、零偏、幅值不一致、相位不正交、谐波畸变、FFT 识别、谐波补偿、
ADC 量化、白噪声、电压 LPF、alpha-beta、离散 ESO、互补滤波、Kalman、
低速扫角、变速、冲击、负载扰动、强冲击、温漂、gyro 漂移和轴间耦合。
```

近期最值得汇报的工程结论：

```text
1. 低速准静态场景中，线性 Hall 方案具备达到 ≤±0.005° @1σ 的可能性，但依赖噪声、幅值、ADC、标定和机械装配质量。
2. 仅调滤波器不能解决结构冲击、温漂和轴间耦合带来的确定性误差。
3. Hall 负责轴/关节相对角度，Gyro 负责高频角速度和动态响应，两者不是重复传感器，而是互补信息源。
4. 下一阶段应把“离线标定参数 + 在线融合算法 + 多轴耦合补偿”组合起来验证。
```

---

## 13. 给新对话的启动提示词

如果用户换账号，可以把下面这段直接发给新 Codex：

```text
你现在接手我的“基于无刷电机和压电马达的多轴手机云台”项目。当前工作目录是：
D:\Desktop\基于无刷电机和压电马达的多轴手机云台\BO1808 NBH2B Simulation Modeling

请先阅读根目录的《项目交接说明_给新Codex.md》，然后继续协助我。我的主要协作习惯是：中文解释，讲清楚为什么，结果图按信号现象、相图形状、解角误差、工程结论来解释；代码可以逐行解释；Simulink 模型不要随便重建或打乱布局；线性霍尔传感器相关文件默认保存在“线性霍尔传感器”文件夹。

当前重点是双线性 Hall 位置检测仿真。已经完成理想正余弦、零偏误差、幅值不一致、相位不正交误差、采样估计零偏/幅值/相位、atan2 解角误差对比。关键文件是：
线性霍尔传感器/init_dual_hall_ideal_params.m
线性霍尔传感器/build_dual_hall_ideal_sincos_model.m
线性霍尔传感器/validate_dual_hall_ideal_sincos.m
线性霍尔传感器/models/dual_hall_ideal_sincos.slx

当前主验证脚本已经推进到：
线性霍尔传感器/validate_dual_hall_noise_filter.m
线性霍尔传感器/init_dual_hall_noise_filter_params.m
线性霍尔传感器/LPF_alpha-beta_ESO原理与Simulink实现说明.md

已完成内容包括：谐波畸变与 FFT/谐波补偿、ADC 量化、白噪声、电压 LPF、角度域 alpha-beta、离散 ESO、Hall+Gyro 互补滤波、Kalman、0.5/1/5 rpm 低速扫角精度验证、变速/冲击/负载扰动，以及强冲击、gyro 零偏漂移、温漂、轴间耦合等强扰动工况。下一步不是重新做谐波，而是做实测标定、查表/温度补偿、真实 IMU 融合和多轴耦合验证。请接着当前风格继续帮助我完成项目。
```

---

## 14. 常用命令

运行双线性 Hall 验证：

```matlab
validate_dual_hall_ideal_sincos
```

运行当前主线性 Hall 噪声、滤波、融合和强扰动验证：

```matlab
cd('D:/Desktop/基于无刷电机和压电马达的多轴手机云台/BO1808 NBH2B Simulation Modeling/线性霍尔传感器')
validate_dual_hall_noise_filter
```

如果 MATLAB 还在用旧函数缓存：

```matlab
clear build_dual_hall_ideal_sincos_model
validate_dual_hall_ideal_sincos
```

加载/刷新模型但不强制保存布局：

```matlab
build_dual_hall_ideal_sincos_model()
```

强制重建模型，并尽量恢复当前手动布局：

```matlab
build_dual_hall_ideal_sincos_model(true)
```

---

## 15. 工程注意事项

- 不要随便删除用户手动调整过的 `.slx`。
- 不要用 `git reset --hard` 或 `git checkout --` 还原用户文件，除非用户明确要求。
- MATLAB 运行后可能生成 `slprj`、`.slxc`、`.autosave`，这些是缓存文件，可清理，但不要误删源码、模型和结果图。
- 如果用户截图中显示文件没有更新，先检查磁盘文件，再考虑 VS Code 缓冲区问题。
- 如果用户问“这一页能不能说清楚”，优先给“可改点 + 一段式讲述文案”。
- 如果用户问“下一步做什么”，当前推荐是“实测标定、查表/温度补偿、真实 IMU 融合和多轴耦合验证”。
