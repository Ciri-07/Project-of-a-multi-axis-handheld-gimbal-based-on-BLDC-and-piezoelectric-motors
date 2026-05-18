%% BO1808NBH2B 模型与本地数据表一致性验证
% 数据来源：BO1808NBH2B.pdf，01-144-11.4 栏。
%
% 本脚本用于验证当前模型是否确实采用 BO1808NBH2B 参数，并按数据表
% 稳态关系重构电机特性曲线。

clear
clc

init_bo1808_params;

%% 数据表参数，11.4 V 绕组
DS = struct();
DS.Vdc = 11.4;                  % V
DS.R_line = 14.4;               % ohm, terminal resistance
DS.L_line = 0.74e-3;            % H, terminal inductance
DS.N0 = 14300;                  % rpm
DS.I0 = 0.12;                   % A
DS.T_stall = 4.3e-3;            % N*m
DS.I_stall = 0.8;               % A
DS.T_nom = 1.2e-3;              % N*m
DS.N_nom = 7790;                % rpm
DS.I_nom = 0.32;                % A
DS.P_max = 1.61;                % W
DS.eta_max = 0.371;             % -
DS.Ke_mV_per_rpm = 0.67;        % mV/rpm
DS.Kt = 6.4e-3;                 % N*m/A
DS.speed_torque_slope = 3320;   % rpm/mN*m
DS.J_gcm2 = 1.4;                % g*cm^2
DS.pn = 4;                      % pole pairs

%% 派生验证量
omega0 = DS.N0*2*pi/60;
Ke_V_per_rpm = DS.Ke_mV_per_rpm*1e-3;
Kt_q_model = 1.5*pn*psi_f;
R_line_model = 2*Rs;
L_line_model = 2*Ld;
J_gcm2_model = J/1e-7;

I_stall_from_R = DS.Vdc/DS.R_line;
T_stall_from_slope = DS.N0/DS.speed_torque_slope*1e-3;
V_no_load_balance = Ke_V_per_rpm*DS.N0 + DS.I0*DS.R_line;
N0_from_terminal_eq = (DS.Vdc - DS.I0*DS.R_line)/Ke_V_per_rpm;
I_nom_from_torque = DS.I0 + DS.T_nom/DS.Kt;
B_equiv_from_no_load = DS.Kt*DS.I0/omega0;
N0_dq_voltage_limit = Vdq_max/(pn*psi_f)*60/(2*pi);
Iq_nom_model = DS.T_nom/Kt_q_model;
tau_e = Ld/Rs;

%% 数据表同款特性曲线重构
T_mNm = linspace(0,DS.T_stall*1e3,400);
T_Nm = T_mNm*1e-3;
N_rpm_curve = max(DS.N0 - DS.speed_torque_slope*T_mNm,0);
I_A_curve = DS.I0 + T_Nm/DS.Kt;
P_W_curve = T_Nm.*(N_rpm_curve*2*pi/60);
eta_curve = P_W_curve./(DS.Vdc*I_A_curve);
eta_curve(1) = 0;

[P_peak_model,idx_pmax] = max(P_W_curve);
[eta_peak_model,idx_etamax] = max(eta_curve);

curve_file = fullfile(figures_dir,'bo1808_datasheet_characteristic_curve.png');
fig_curve = figure('Name','BO1808NBH2B Datasheet Characteristic Curve','Color','w');
tiledlayout(fig_curve,2,2,'TileSpacing','compact','Padding','compact');

nexttile
plot(T_mNm,N_rpm_curve,'b','LineWidth',1.6)
hold on
plot(DS.T_nom*1e3,DS.N_nom,'ko','MarkerFaceColor','k')
grid on
xlabel('转矩 T / mN·m')
ylabel('转速 N / rpm')
title('转速-转矩特性')
xlim([0 DS.T_stall*1e3])
ylim([0 1.08*DS.N0])

nexttile
plot(T_mNm,I_A_curve,'r','LineWidth',1.6)
hold on
plot(DS.T_nom*1e3,DS.I_nom,'ko','MarkerFaceColor','k')
plot(DS.T_stall*1e3,DS.I_stall,'ks','MarkerFaceColor',[0.8 0.8 0.8])
grid on
xlabel('转矩 T / mN·m')
ylabel('电流 I / A')
title('电流-转矩特性')
xlim([0 DS.T_stall*1e3])
ylim([0 1.08*DS.I_stall])

nexttile
plot(T_mNm,P_W_curve,'m','LineWidth',1.6)
hold on
plot(T_mNm(idx_pmax),P_peak_model,'ko','MarkerFaceColor','k')
grid on
xlabel('转矩 T / mN·m')
ylabel('输出功率 P / W')
title(sprintf('输出功率特性，峰值 %.3g W',P_peak_model))
xlim([0 DS.T_stall*1e3])
ylim([0 1.15*max(P_W_curve)])

nexttile
plot(T_mNm,100*eta_curve,'Color',[0 0.55 0],'LineWidth',1.6)
hold on
plot(T_mNm(idx_etamax),100*eta_peak_model,'ko','MarkerFaceColor','k')
yline(100*DS.eta_max,'k--','数据表最大效率','LabelHorizontalAlignment','left')
grid on
xlabel('转矩 T / mN·m')
ylabel('效率 \eta / %')
title(sprintf('效率特性，计算峰值 %.2f%%',100*eta_peak_model))
xlim([0 DS.T_stall*1e3])
ylim([0 45])

saveas(fig_curve,curve_file);

overlay_file = fullfile(figures_dir,'bo1808_datasheet_characteristic_curve_overlay.png');
fig_overlay = figure('Name','BO1808NBH2B PDF Style Characteristic Curve','Color','w', ...
    'Units','pixels','Position',[100 100 920 760]);

x_ticks = [0 0.4 0.9 1.3 1.7 2.2 2.6 3.0 3.5 3.9 4.3];
pos = [0.17 0.24 0.66 0.58];

axN = axes('Parent',fig_overlay,'Position',pos);
hold(axN,'on')
grid(axN,'on')
box(axN,'off')
hN = plot(axN,T_mNm,N_rpm_curve,'Color',[0.12 0.33 0.70],'LineWidth',1.45);
hP = plot(axN,T_mNm,P_W_curve/1.8*16000,'Color',[0.42 0.42 0.42],'LineWidth',1.45);
set(axN,'XLim',[0 DS.T_stall*1e3], ...
    'YLim',[0 16000], ...
    'XTick',x_ticks, ...
    'YTick',0:2000:16000, ...
    'YColor',[0.25 0.25 0.25], ...
    'FontName','Arial', ...
    'FontSize',10)
xlabel(axN,'T (mN.m)')
ylabel(axN,'N (rpm)')
title(axN,'BO1808NBH2B01-144-11.4','FontSize',17,'FontWeight','normal')

axP = axes('Parent',fig_overlay,'Position',[pos(1)-0.07 pos(2) pos(3) pos(4)], ...
    'Color','none','XAxisLocation','bottom','YAxisLocation','left', ...
    'XColor','none','YColor',[0.25 0.25 0.25], ...
    'XLim',[0 DS.T_stall*1e3],'YLim',[0 1.8], ...
    'YTick',0:0.2:1.8,'Box','off','FontName','Arial','FontSize',10);
ylabel(axP,'P(W)')

axI = axes('Parent',fig_overlay,'Position',pos, ...
    'Color','none','XAxisLocation','bottom','YAxisLocation','right', ...
    'XColor','none','YColor',[0.25 0.25 0.25], ...
    'XLim',[0 DS.T_stall*1e3],'YLim',[0 0.9], ...
    'YTick',0:0.1:0.9,'Box','off','FontName','Arial','FontSize',10);
hold(axI,'on')
hI = plot(axI,T_mNm,I_A_curve,'Color',[0.82 0.18 0.14],'LineWidth',1.45);
hEta = plot(axI,T_mNm,eta_curve/0.40*0.9,'k--','LineWidth',1.45);
ylabel(axI,'I(A)')

axEta = axes('Parent',fig_overlay,'Position',[pos(1)+0.07 pos(2) pos(3) pos(4)], ...
    'Color','none','XAxisLocation','bottom','YAxisLocation','right', ...
    'XColor','none','YColor',[0.25 0.25 0.25], ...
    'XLim',[0 DS.T_stall*1e3],'YLim',[0 40], ...
    'YTick',0:5:40,'YTickLabel',compose('%d%%',0:5:40), ...
    'Box','off','FontName','Arial','FontSize',10);
ylabel(axEta,'\eta')

text(axN,0.04,0.96,'P(W) N(rpm)','Units','normalized','FontWeight','bold','FontSize',10)
text(axN,0.96,0.96,'I(A)    \eta','Units','normalized','FontWeight','bold','FontSize',10, ...
    'HorizontalAlignment','right')
text(axN,0.06,0.73,'N','Color',[0.12 0.33 0.70],'FontSize',10,'Units','normalized')
text(axI,0.84,0.73,'I','Color',[0.82 0.18 0.14],'FontSize',10,'Units','normalized')
text(axN,0.57,0.84,'P','Color',[0.42 0.42 0.42],'FontSize',10,'Units','normalized')
text(axN,0.40,0.89,'\eta','Color','k','FontSize',10,'Units','normalized')

legend(axN,[hN hI hEta hP],{'Speed (N)','Current (I)','Efficiency (\eta)','Power (P)'}, ...
    'Location','southoutside','NumColumns',2,'Box','off','FontSize',13)
saveas(fig_overlay,overlay_file);

%% 输出并保存中文报告
report_file = fullfile(reports_dir,'bo1808_datasheet_validation_report.txt');
fid = fopen(report_file,'w');
cleanupObj = onCleanup(@() fclose(fid));

say(fid,'BO1808NBH2B 模型与本地数据表一致性验证报告');
say(fid,'================================================');
say(fid,'数据表文件：data/BO1808NBH2B.pdf');
say(fid,'验证对象：01-144-11.4，11.4 V 绕组');
say(fid,'生成时间：%s',datestr(now,'yyyy-mm-dd HH:MM:SS'));
say(fid,'');

say(fid,'一、模型参数与数据表直接对比');
check_value(fid,'极对数 pn',pn,DS.pn,'-',0);
check_value(fid,'端电阻 2*Rs',R_line_model,DS.R_line,'ohm',0.5);
check_value(fid,'端电感 2*Ld',L_line_model,DS.L_line,'H',0.5);
check_value(fid,'转矩常数 1.5*pn*psi_f',Kt_q_model,DS.Kt,'N*m/A',0.5);
check_value(fid,'转子惯量',J_gcm2_model,DS.J_gcm2,'g*cm^2',0.5);
check_value(fid,'直流母线电压',Vdc,DS.Vdc,'V',0);
check_value(fid,'额定负载转矩',TL_nom,DS.T_nom,'N*m',0.5);
say(fid,'');

say(fid,'二、数据表稳态关系校验');
check_value(fid,'由 V/R_line 估算堵转电流',I_stall_from_R,DS.I_stall,'A',2.0);
check_value(fid,'由转速/转矩斜率估算堵转转矩',T_stall_from_slope,DS.T_stall,'N*m',1.0);
check_value(fid,'空载电压平衡 Ke*N0 + I0*R_line',V_no_load_balance,DS.Vdc,'V',2.0);
check_value(fid,'由端电压方程反推空载转速',N0_from_terminal_eq,DS.N0,'rpm',2.0);
check_value(fid,'额定电流估算 I0 + TL/Kt',I_nom_from_torque,DS.I_nom,'A',5.0);
say(fid,'');

say(fid,'三、d-q 模型派生量检查');
say(fid,'永磁体磁链 psi_f = Kt/(1.5*pn) = %.9g Wb',psi_f);
say(fid,'额定 q 轴电流 TL_nom/Kt_q = %.6g A',Iq_nom_model);
say(fid,'电气时间常数 Ld/Rs = %.6g s，即 %.3f us',tau_e,tau_e*1e6);
say(fid,'SVPWM d-q 电压上限对应机械转速 Vdq_max/(pn*psi_f) = %.1f rpm',N0_dq_voltage_limit);
say(fid,'');

say(fid,'四、特性曲线重构结果');
say(fid,'特性曲线按数据表关系重构：');
say(fid,'N(T) = N0 - speed_torque_slope*T');
say(fid,'I(T) = I0 + T/Kt');
say(fid,'P(T) = T*omega');
say(fid,'eta(T) = P/(Vdc*I)');
say(fid,'计算最大输出功率 = %.4g W，数据表最大输出功率 = %.4g W',P_peak_model,DS.P_max);
say(fid,'计算最大效率 = %.3f%%，数据表最大效率 = %.3f%%',100*eta_peak_model,100*DS.eta_max);
say(fid,'已生成曲线文件：%s',curve_file);
say(fid,'已生成 PDF 风格归一化叠加曲线：%s',overlay_file);
say(fid,'');

say(fid,'五、阻尼系数检查');
say(fid,'当前模型 B = %.9g N*m*s',B);
say(fid,'由空载电流反推的等效粘性阻尼 B_equiv = Kt*I0/omega0 = %.9g N*m*s',B_equiv_from_no_load);
say(fid,'当前 B / 空载等效 B = %.3g',B/B_equiv_from_no_load);
if B > 3*B_equiv_from_no_load
    say(fid,'注意：当前 B 明显大于由数据表空载点反推的等效阻尼。');
    say(fid,'      若目标是严格匹配数据表空载/额定稳态点，建议使用 B ~= %.3g。',B_equiv_from_no_load);
    say(fid,'      数据表空载电流包含轴承、铁耗、驱动相关损耗等，该 B_equiv 是等效标定值，');
    say(fid,'      不是纯粹的物理粘性阻尼。');
else
    say(fid,'当前 B 已按数据表空载点的等效阻尼量级标定。');
end
say(fid,'');

say(fid,'六、建议的模型正确性验证流程');
say(fid,'A. 运行本脚本，确认参数和稳态关系大多显示为“通过”。');
say(fid,'B. 检查空载关系 Ke*N0 + I0*R_line 是否接近 Vdc。');
say(fid,'C. 检查额定电流估算 I0 + TL_nom/Kt 是否接近数据表额定电流。');
say(fid,'D. 运行 validate_bo1808_fig1_17，确认 id 接近 0、iq 随负载增加、速度可恢复。');
say(fid,'E. 运行 validate_bo1808_svpwm，确认扇区顺序为 3-1-5-4-6-2，且 FFT 在额定电角频率处的基波接近 Uref。');
say(fid,'');
say(fid,'报告已保存到：%s',report_file);

type(report_file);

function check_value(fid,name,model_value,datasheet_value,unit,tol_pct)
if datasheet_value == 0
    err_pct = 0;
else
    err_pct = abs(model_value - datasheet_value)/abs(datasheet_value)*100;
end
if err_pct <= tol_pct
    status = '通过';
else
    status = '需检查';
end
say(fid,'%-42s 模型=%-14.7g 数据表=%-14.7g %-8s 误差=%7.3f%%  %s', ...
    name,model_value,datasheet_value,unit,err_pct,status);
end

function say(fid,varargin)
msg = sprintf(varargin{:});
fprintf('%s\n',msg);
fprintf(fid,'%s\n',msg);
end
