function [sys,x0,str,ts] = svpwm_bo1808_sfunc(t,x,u,flag)
%SVPWM_BO1808_SFUNC Level-1 S-Function for the SVPWM Tcm algorithm.
%
% Input vector u:
%   u(1) = Valpha, alpha-axis reference voltage, V
%   u(2) = Vbeta,  beta-axis reference voltage, V
%   u(3) = Udc,    DC bus voltage, V
%   u(4) = Tpwm,   PWM period, s
%
% Output vector sys:
%   sys(1) = Tcm1, phase-A switching comparison time, s
%   sys(2) = Tcm2, phase-B switching comparison time, s
%   sys(3) = Tcm3, phase-C switching comparison time, s
%   sys(4) = sector, N-coded sector
%   sys(5) = T1, first active-vector time, s
%   sys(6) = T2, second active-vector time, s
%   sys(7) = T0, zero-vector time, s

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes();
    case 3
        sys = mdlOutputs(u);
    case {1,2,4,9}
        sys = [];
    otherwise
        error('svpwm_bo1808_sfunc:UnhandledFlag', ...
            'Unhandled S-function flag = %d',flag);
end

end

function [sys,x0,str,ts] = mdlInitializeSizes()
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 7;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];
end

function sys = mdlOutputs(u)
Valpha = u(1);
Vbeta = u(2);
Udc = u(3);
Tpwm = u(4);

if Udc <= 0 || Tpwm <= 0
    sys = zeros(7,1);
    return
end

Vref1 = Vbeta;
Vref2 = (sqrt(3)*Valpha - Vbeta)/2;
Vref3 = (-sqrt(3)*Valpha - Vbeta)/2;

zero_tol = max(1e-12,1e-10*Udc);
if abs(Valpha) + abs(Vbeta) <= zero_tol
    sector = 1;
else
    sector = 0;
    if Vref1 >= 0
        sector = sector + 1;
    end
    if Vref2 >= 0
        sector = sector + 2;
    end
    if Vref3 >= 0
        sector = sector + 4;
    end
    if sector < 1 || sector > 6
        sector = 1;
    end
end

X = sqrt(3)*Vbeta*Tpwm/Udc;
Y = Tpwm/Udc*(1.5*Valpha + sqrt(3)/2*Vbeta);
Z = Tpwm/Udc*(-1.5*Valpha + sqrt(3)/2*Vbeta);

switch sector
    case 1
        T1 = Z;
        T2 = Y;
    case 2
        T1 = Y;
        T2 = -X;
    case 3
        T1 = -Z;
        T2 = X;
    case 4
        T1 = -X;
        T2 = Z;
    case 5
        T1 = X;
        T2 = -Y;
    otherwise
        T1 = -Y;
        T2 = -Z;
end

T1 = max(T1,0);
T2 = max(T2,0);
if T1 + T2 > Tpwm
    scale = Tpwm/(T1 + T2);
    T1 = T1*scale;
    T2 = T2*scale;
end
T0 = Tpwm - T1 - T2;

Ta = T0/4;
Tb = Ta + T1/2;
Tc = Tb + T2/2;

switch sector
    case 1
        Tcm1 = Tb;
        Tcm2 = Ta;
        Tcm3 = Tc;
    case 2
        Tcm1 = Ta;
        Tcm2 = Tc;
        Tcm3 = Tb;
    case 3
        Tcm1 = Ta;
        Tcm2 = Tb;
        Tcm3 = Tc;
    case 4
        Tcm1 = Tc;
        Tcm2 = Tb;
        Tcm3 = Ta;
    case 5
        Tcm1 = Tc;
        Tcm2 = Ta;
        Tcm3 = Tb;
    otherwise
        Tcm1 = Tb;
        Tcm2 = Tc;
        Tcm3 = Ta;
end

Tcm1 = min(max(Tcm1,0),Tpwm/2);
Tcm2 = min(max(Tcm2,0),Tpwm/2);
Tcm3 = min(max(Tcm3,0),Tpwm/2);

sys = [Tcm1; Tcm2; Tcm3; sector; T1; T2; T0];
end
