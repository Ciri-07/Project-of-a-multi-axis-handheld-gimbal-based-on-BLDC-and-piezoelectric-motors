# BO1808NBH2B PMSM 建模与仿真工程说明

本工程用于 BO1808NBH2B01-144-11.4 精密伺服电机的 MATLAB/Simulink 建模、闭环控制仿真、SVPWM 验证和数据表一致性校验。模型形式采用同步旋转 d-q 坐标系，所有电机本体参数均按 BO1808NBH2B 11.4 V 版本整理，不直接套用示例电机参数。

## 文件

根目录保留可直接运行的 MATLAB 脚本：

- `init_bo1808_params.m`：BO1808NBH2B 参数初始化脚本。
- `build_bo1808_modular_model.m`：生成 `models/bo1808_fig1_9_modular.slx`，三模块 d-q 电机本体模型。
- `pmsm_bo1808.m`：BO1808NBH2B Level-1 S-Function，连续状态为 `[id; iq; omega_m]`。
- `build_bo1808_foc_id0_model.m`：生成 `models/bo1808_foc_id0.slx`，`id = 0` 的 d-q 闭环矢量控制模型。
- `init_bo1808_svpwm_params.m`：BO1808NBH2B 额定工况 SVPWM 仿真参数。
- `build_bo1808_svpwm_model.m`：生成 `models/bo1808_svpwm_sim.slx`，包含扇区判断、`T1/T2/T0`、`Tcm1/Tcm2/Tcm3`、三角载波比较和逆变器电压重构。
- `svpwm_bo1808_sfunc.m`：SVPWM Tcm 算法的 Level-1 MATLAB S-Function。
- `build_bo1808_svpwm_sfunc_model.m`：生成 `models/bo1808_svpwm_sfunc_sim.slx`，用 S-Function 实现 SVPWM 核心计算。
- `build_bo1808_foc_svpwm_model.m`：生成 `models/bo1808_foc_svpwm_bo1808.slx`，完整 FOC + 反 Park + SVPWM + 逆变器 + BO1808 闭环模型。
- `validate_bo1808_fig1_17.m`：闭环模型动态验证，输出转速、`iq`、`id` 曲线。
- `validate_bo1808_svpwm.m`：SVPWM 验证，输出扇区、调制波、相电压和 FFT。
- `validate_bo1808_svpwm_sfunc.m`：SVPWM S-Function 版本验证。
- `validate_bo1808_fig3_9.m`：完整 FOC+SVPWM 联合闭环验证，输出对应图3-9指标的转速、转矩、三相电流曲线。
- `validate_bo1808_model_against_datasheet.m`：验证模型参数与本地 `BO1808NBH2B.pdf` 数据表一致性，并生成特性曲线。
- `run_bo1808_setup.m`：一键初始化并生成全部 Simulink 模型。

整理后的目录用途：

```text
data/              BO1808NBH2B 电机数据表
references/        参考资料 PDF
models/            自动生成的 Simulink 模型
results/figures/   验证图片
results/reports/   验证报告
docs/              汇报和说明文档
cache/             MATLAB/Simulink 生成缓存
```

## 一键生成

在 MATLAB 当前目录切换到本工程文件夹后运行：

```matlab
run_bo1808_setup
```

会生成：

```text
models/bo1808_fig1_9_modular.slx
models/bo1808_foc_id0.slx
models/bo1808_svpwm_sim.slx
models/bo1808_svpwm_sfunc_sim.slx
models/bo1808_foc_svpwm_bo1808.slx
```

如果直接双击或从 `models/` 文件夹打开 `.slx` 后提示 `pmsm_bo1808` 或 `svpwm_bo1808_sfunc` 不存在，说明 MATLAB 路径没有包含工程根目录。可先运行：

```matlab
init_bo1808_params
```

新生成的 `bo1808_foc_svpwm_bo1808.slx` 已加入自动加工程根目录路径的模型回调，通常直接打开即可识别这两个 S-Function 文件。

## BO1808NBH2B 参数口径

默认认为数据表中的 `Terminal resistance` 和 `Terminal inductance` 是 Y 接线三相电机的线间测量值，因此：

```text
R_line = 14.4 ohm        -> Rs = 7.2 ohm
L_line = 0.74 mH         -> Ld = Lq = 0.37 mH
pn = 4
Kt = 0.0064 N*m/A
psi_f = Kt/(1.5*pn) = 0.0010667 Wb
J = 1.4e-7 kg*m^2
Vdc = 11.4 V
TL_nom = 0.0012 N*m
Nrated = 7790 rpm
N0 = 14300 rpm
I0 = 0.12 A
```

阻尼 `B` 已按数据表空载点进行等效标定：

```text
B = Kt*I0/omega0 = 5.13e-7 N*m*s
```

这个值把空载电流中的机械摩擦、铁耗和其他损耗等效到粘性阻尼项中，适合用于匹配数据表空载/稳态特性。若后续有实测空载损耗或转速衰减实验，可以继续重新标定。

## 模型正确性验证

验证数据表一致性：

```matlab
validate_bo1808_model_against_datasheet
```

输出：

```text
results/reports/bo1808_datasheet_validation_report.txt
results/figures/bo1808_datasheet_characteristic_curve.png
results/figures/bo1808_datasheet_characteristic_curve_overlay.png
```

该脚本检查端电阻/端电感、转矩常数、极对数、转子惯量、堵转电流、空载电压平衡、额定电流估算，并按数据表关系重构转速、电流、输出功率和效率曲线。

闭环动态验证：

```matlab
validate_bo1808_fig1_17
```

输出：

```text
results/figures/bo1808_fig1_17_validation.png
```

验证指标包括：转速能否跟踪并在负载扰动后恢复、`iq` 是否随负载增大、`id` 是否保持接近 0。负载扰动使用 BO1808 额定负载 `TL_nom = 0.0012 N*m`，不使用大功率示例电机的 `10 N*m`。

## SVPWM 仿真

运行：

```matlab
validate_bo1808_svpwm
```

输出：

```text
results/figures/bo1808_svpwm_validation.png
results/figures/bo1808_svpwm_alpha_beta_ref.png
results/figures/bo1808_svpwm_phase_voltage_fft.png
```

SVPWM 的图形不追求与示例图逐像素一致，但保留同类图例和验证指标：

- `u_alpha/u_beta` 参考电压时域波形和 α-β 电压矢量圆轨迹。
- 扇区编码 `N`，显示顺序为 `3-1-5-4-6-2`。
- `Tcm1/Tcm2/Tcm3` 马鞍形调制波。
- A 相对浮动中性点相电压 `Van`。
- 相电压 FFT，标出额定电角频率基波和 PWM 载波附近开关谐波峰值。

若要运行 SVPWM 的 S-Function 核心版本，执行：

```matlab
validate_bo1808_svpwm_sfunc
```

该版本的核心模块为 `svpwm_bo1808_sfunc.m`，接口为：

```text
输入：[Valpha, Vbeta, Udc, Tpwm]
输出：[Tcm1, Tcm2, Tcm3, sector, T1, T2, T0]
```

输出文件为：

```text
models/bo1808_svpwm_sfunc_sim.slx
results/figures/bo1808_svpwm_sfunc_validation.png
results/figures/bo1808_svpwm_sfunc_alpha_beta_ref.png
results/figures/bo1808_svpwm_sfunc_phase_voltage_fft.png
```

## 完整 FOC + SVPWM 联合闭环

运行：

```matlab
validate_bo1808_fig3_9
```

输出：

```text
models/bo1808_foc_svpwm_bo1808.slx
results/figures/bo1808_fig3_9_foc_svpwm_validation.png
results/figures/bo1808_fig3_9_foc_svpwm_diagnostics.png
```

该模型链路为：

```text
速度 PI -> iq*
id* = 0 -> d/q 电流 PI -> ud/uq
ud/uq -> 反 Park -> u_alpha/u_beta
u_alpha/u_beta -> SVPWM S-Function -> Sa/Sb/Sc
Sa/Sb/Sc -> 两电平逆变器电压重构 -> Park -> BO1808 d-q 电机本体
```

对应图3-9的验证指标保留为：转速响应、电磁转矩/负载转矩、三相电流。数值工况按 BO1808NBH2B 调整：速度参考默认 `1000 rpm`，负载扰动为 `TL_nom = 1.2 mN*m`，直流母线为 `11.4 V`，PWM 周期为 `20 us`。因此它验证的是 BO1808 驱动链路正确性，而不是复刻示例大功率电机的波形幅值。

负载输入由 `init_bo1808_params.m` 中的 `TL_load_mode` 控制：

```matlab
TL_load_mode = 'step';   % 默认：0.12 s 加入 TL_nom 阶跃负载
TL_load_mode = 'sine';   % 周期负载：TL_mean_sine + TL_amp_sine*sin(2*pi*f_load_sine*t)
```

正弦负载参数为：

```matlab
TL_mean_sine = TL_nom;
TL_amp_sine = 0.4e-3;
f_load_sine = 5;
TL_sine_start = 0.05;
```

若需要在负载上叠加白噪声扰动，可设置：

```matlab
TL_noise_enable = true;
TL_noise_std = 0.05e-3;
TL_noise_sample_time = 1e-4;
TL_noise_seed = 1808;
```

此时模型中的负载为基础负载加随机扰动，并经过下限 0 的限幅：

```text
TL = max(TL_base + TL_noise, 0)
```

注意：`validate_bo1808_fig3_9` 会重新生成 `bo1808_foc_svpwm_bo1808.slx`，因此手动替换模型中的 `TL_step` 会被覆盖。若要长期使用周期负载，应修改上述变量后再运行验证脚本。

也可以直接运行正弦负载专用验证脚本：

```matlab
validate_bo1808_fig3_9_sine_load
```

输出：

```text
results/figures/bo1808_fig3_9_foc_svpwm_sine_load.png
```

SVPWM 输入按 BO1808 额定工况推导：

```text
f_ref = pn*Nrated/60 = 4*7790/60 = 519.33 Hz
omega_e = 2*pi*f_ref = 3263.10 rad/s
iq_ref = TL_nom/Kt = 0.1875 A
ud_ref = -omega_e*Lq*iq_ref
uq_ref = Rs*iq_ref + omega_e*psi_f
Uref = sqrt(ud_ref^2 + uq_ref^2) ≈ 4.84 V
Vref_max = Vdc/sqrt(3) ≈ 6.58 V
m = Uref/Vref_max ≈ 0.735
```

`fs_pwm = 20 kHz` 是本仿真的控制/载波频率假设，因为 BO1808NBH2B 数据表没有规定驱动器 PWM 频率。该频率比 519.33 Hz 电角基波高约 38.5 倍，更适合微型高电阻电机的 SVPWM 观察和 FFT 分析。

## 与示例参数的差异

本工程保留 d-q 建模、扇区判断、`T1/T2/T0` 和 `Tcm` 计算等方法，但数值输入全部按 BO1808NBH2B 处理：

- 直流母线使用 `11.4 V`，不是示例中的高压母线。
- 额定负载使用 `1.2 mN*m`，不是大电机负载。
- SVPWM 基波频率使用 BO1808 额定转速推导得到的 `519.33 Hz`，不是为了复刻示例波形而固定为 `50 Hz`。
- SVPWM 参考电压幅值由 BO1808 额定 d-q 电压计算得到，不再使用 `200/700` 的比例缩放。
- 输出图可读性优先，图例和指标与验证目标一致，不要求版式完全复刻。

## d-q 模型方程

电流子系统：

```text
did/dt = (ud - Rs*id + omega_e*Lq*iq)/Ld
diq/dt = (uq - Rs*iq - omega_e*(Ld*id + psi_f))/Lq
```

表贴式 PMSM 转矩：

```text
Te = (3/2)*pn*psi_f*iq
```

机械方程：

```text
domega_m/dt = (Te - B*omega_m - TL)/J
theta_e = integral(pn*omega_m)
Nr = 60/(2*pi)*omega_m
```

## PI 整定建议

BO1808NBH2B 的相电阻高、相电感小：

```text
Rs = 7.2 ohm
Ld = Lq = 0.37 mH
L/R = 51.4 us
```

建议电流环先用保守带宽：

```matlab
wci = 2*pi*300;
Kp_id = Ld*wci;
Ki_id = Rs*wci;
Kp_iq = Lq*wci;
Ki_iq = Rs*wci;
```

默认约为：

```text
Kp_i = 0.697
Ki_i = 1.357e4
```

速度环比电流环慢一个数量级左右，默认：

```matlab
zeta_w = 0.90;
wn_w = 2*pi*18;
Kp_w = (2*zeta_w*wn_w*J - B)/Kt_q;
Ki_w = J*wn_w^2/Kt_q;
```

若出现 `ud/uq` 长时间限幅、`iq` 振荡或速度超调明显，应降低带宽或加入抗积分饱和。由于该电机额定转矩只有 `1.2 mN*m`，速度 PI 输出限幅建议先取 `Iq_max = 0.6 A`，后续再根据热限制和实机测试调整。

## 参数假设可切换

如果你认为数据表中的端电阻/端电感已经是相值，可在 `init_bo1808_params.m` 中改为：

```matlab
use_line_to_line_RL = false;
```

此时模型会使用：

```text
Rs = 14.4 ohm
Ld = Lq = 0.74 mH
```

`psi_f` 仍按 `Kt/(1.5*pn)` 计算，因为转矩常数通常已经对应相电流定义。
