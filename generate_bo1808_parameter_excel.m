%% Generate BO1808 parameter tables for report use
%
% Output:
%   results/reports/BO1808_parameter_tables.xlsx

clear
clc

init_bo1808_params;

output_file = fullfile(reports_dir,'BO1808_parameter_tables.xlsx');
if exist(output_file,'file')
    delete(output_file);
end

fe_rated = pn*Nrated/60;
tau_e = Ld/Rs;
T_B_rated = B*omega_rated;
T_total_rated = TL_nom + T_B_rated;
Iq_total_rated = T_total_rated/Kt_q;
E_rated = omegae_rated*psi_f;
R_drop_rated = Rs*Iq_rated;
P_out_rated = TL_nom*omega_rated;

sheet1 = {
    '类别','参数名称','符号','BO1808数值','单位','说明';
    '电机本体','极对数','pn',pn,'-','机械角速度与电角速度换算';
    '电机本体','额定电压/母线电压','Vdc',Vdc,'V','逆变器直流母线电压';
    '电气参数','端子线电阻','R_line',R_line,'ohm','PDF给出的端子电阻';
    '电气参数','相电阻','Rs',Rs,'ohm','星形连接假设下 R_line/2';
    '电气参数','端子线电感','L_line',L_line,'H','PDF给出的端子电感';
    '电气参数','d轴电感','Ld',Ld,'H','d-q电流动态';
    '电气参数','q轴电感','Lq',Lq,'H','d-q电流动态';
    '电磁参数','转矩常数','Kt',Kt,'N*m/A','电流到转矩的比例';
    '电磁参数','d-q转矩常数','Kt_q',Kt_q,'N*m/A','Kt_q = 1.5*pn*psi_f';
    '电磁参数','永磁体磁链','psi_f',psi_f,'Wb','由 Kt/(1.5*pn) 推导';
    '电磁参数','反电动势常数','Ke',Ke_mV_per_rpm,'mV/rpm','用于与磁链交叉验证';
    '机械参数','转子惯量','J',J,'kg*m^2','机械运动方程';
    '机械参数','粘性阻尼','B',B,'N*m*s','由空载电流和空载转速估算';
    '工况参数','额定负载转矩','TL_nom',TL_nom,'N*m','额定负载阶跃';
    '工况参数','额定转速','Nrated',Nrated,'rpm','PDF额定转速';
    '工况参数','空载转速','N0',N0,'rpm','PDF空载转速';
    '工况参数','空载电流','I0',I0,'A','PDF空载电流';
    '控制限制','额定q轴电流','Iq_rated',Iq_rated,'A','TL_nom/Kt_q';
    '控制限制','q轴电流限幅','Iq_max',Iq_max,'A','当前仿真速度环输出限幅';
    '控制限制','d轴电流限幅','Id_max',Id_max,'A','当前仿真d轴限制';
    '控制限制','SVPWM电压上限','Vdq_max',Vdq_max,'V','Vdc/sqrt(3)';
    };

sheet2 = {
    '推导量','符号/公式','BO1808数值','单位','含义';
    '电气时间常数','tau_e = Ld/Rs',tau_e,'s','电流自然响应速度';
    '额定机械角速度','omega_rated = Nrated*2*pi/60',omega_rated,'rad/s','额定机械速度';
    '额定电角速度','omegae_rated = pn*omega_rated',omegae_rated,'rad/s','额定电角速度';
    '额定三相电流频率','fe_rated = pn*Nrated/60',fe_rated,'Hz','额定转速对应的电频率';
    '额定负载电流','Iq_rated = TL_nom/Kt_q',Iq_rated,'A','只克服额定负载所需q轴电流';
    '额定阻尼转矩','T_B = B*omega_rated',T_B_rated,'N*m','高速时粘性阻尼消耗';
    '额定总需求转矩','T_total = TL_nom + B*omega_rated',T_total_rated,'N*m','负载加阻尼的稳态转矩';
    '额定总需求q轴电流','Iq_total = T_total/Kt_q',Iq_total_rated,'A','额定高速稳态所需q轴电流估计';
    '额定反电动势项','E = omegae_rated*psi_f',E_rated,'V','高速时q轴电压主要占用项';
    '额定电阻压降','Rs*Iq_rated',R_drop_rated,'V','高电阻小电机的电压损耗';
    '额定机械输出功率','P_out = TL_nom*omega_rated',P_out_rated,'W','额定负载下机械输出功率';
    };

sheet3 = {
    '参数名称','符号','必须程度','建议获取方式','说明';
    '极对数','pn','必须','结构设计或反电势测量','决定电角度、电频率、FOC变换';
    '相电阻','Rs','必须','四线法或低压直流测量','需明确线电阻/相电阻及星三角接法';
    'd轴电感','Ld','必须','LCR表或锁相测量','影响d轴电流环和解耦';
    'q轴电感','Lq','必须','LCR表或锁相测量','表贴电机通常接近Ld';
    '转矩常数','Kt','必须','测功机或力矩传感器','决定电流到转矩的精度';
    '反电动势常数','Ke','必须','拖动电机测线电压','与psi_f互相校验';
    '永磁体磁链','psi_f','必须','由Kt/(1.5*pn)或Ke换算','PMSM d-q模型核心参数';
    '转子惯量','J','必须','CAD估算、加速实验或厂家数据','速度环/位置环设计关键';
    '粘性阻尼','B','推荐实测','空载电流-转速曲线拟合','高速工况影响明显';
    '库仑摩擦','Tc','推荐实测','低速正反转实验','高精度低速定位重要';
    '齿槽转矩','Tcog(theta)','高精度必须','低速空载转矩扫描','造成位置纹波';
    '转矩纹波','Tripple(theta,i)','高精度必须','力矩传感器标定','影响微小角度控制';
    '额定电压','Vdc','必须','驱动器规格','决定电压裕量';
    '额定电流','Irated','必须','热设计或厂家数据','连续运行限制';
    '峰值电流','Ipeak','必须','驱动器和绕组热限制','加速与抗扰能力';
    '额定转速','Nrated','必须','测功机或厂家数据','额定工况验证';
    '空载转速','N0','推荐','空载测试','验证损耗模型';
    '空载电流','I0','推荐','空载测试','估算阻尼和铁损';
    };

sheet4 = {
    '模块','参数名称','符号','建议程度','用途';
    '逆变器','PWM频率','fs_pwm','必须','决定电流纹波和开关频谱';
    '逆变器','死区时间','t_dead','高精度推荐','影响低速电压误差';
    '逆变器','MOS导通电阻','Rds_on','高精度推荐','低压微电机电压损耗明显';
    '逆变器','母线电压纹波','Delta_Vdc','推荐','影响SVPWM输出电压';
    '采样系统','电流采样周期','Ts_i','必须','电流环设计';
    '采样系统','速度采样周期','Ts_w','必须','速度环设计';
    '采样系统','位置采样周期','Ts_pos','必须','位置环设计';
    '电流传感','电流采样噪声','sigma_i','高精度推荐','影响低电流控制精度';
    '位置传感','编码器分辨率','N_enc','必须','位置精度上限';
    '位置传感','电角度零位偏差','theta_offset','必须标定','影响Park变换和转矩输出';
    '位置传感','位置噪声/量化','sigma_theta','高精度推荐','低速抖动分析';
    '负载','负载惯量','J_load','必须','级联系统动态';
    '负载','负载阻尼','B_load','推荐','速度响应和抗扰';
    '传动机构','减速比','G','按结构需要','有齿轮/丝杆/柔性传动时使用';
    '传动机构','传动刚度','K_shaft','高精度推荐','耦合振动和弹性误差';
    '传动机构','传动阻尼','D_shaft','高精度推荐','抑制振荡';
    '多轴系统','惯量矩阵','M(q)','多自由度必须','多轴耦合动力学';
    '多轴系统','耦合刚度矩阵','K_c','多自由度推荐','轴间耦合误差';
    '多轴系统','耦合阻尼矩阵','D_c','多自由度推荐','轴间扰动传递';
    '控制系统','电流环PI参数','Kp_i/Ki_i','必须','电流跟踪';
    '控制系统','速度环PI参数','Kp_w/Ki_w','必须','速度闭环';
    '控制系统','位置环参数','Kp_pos','按需求','高精度角度控制';
    };

blank = cell(1,6);
combined = [
    {'一、BO1808 核心电机参数','','','','',''};
    sheet1;
    blank;
    {'二、由 BO1808 参数推导出的关键量','','','','',''};
    normalize_width(sheet2,6);
    blank;
    {'三、如果换成特制电机，最少要拿到这些参数','','','','',''};
    normalize_width(sheet3,6);
    blank;
    {'四、高精度控制还需要的非电机参数','','','','',''};
    normalize_width(sheet4,6)
    ];

writecell(combined,output_file,'Sheet','BO1808参数总表');

fprintf('Generated Excel file:\n%s\n', output_file);

function out = normalize_width(in, ncol)
out = cell(size(in,1), ncol);
out(:,1:size(in,2)) = in;
end
