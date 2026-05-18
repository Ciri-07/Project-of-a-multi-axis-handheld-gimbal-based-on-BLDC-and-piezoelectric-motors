function [sys,x0,str,ts] = pmsm_bo1808(t,x,u,flag)
%PMSM_BO1808 Level-1 S-Function for BO1808NBH2B PMSM d-q model.
%
% Continuous states:
%   x(1) = id       d-axis current, A
%   x(2) = iq       q-axis current, A
%   x(3) = omega_m  mechanical angular speed, rad/s
%
% Inputs:
%   u(1) = ud       d-axis voltage, V
%   u(2) = uq       q-axis voltage, V
%   u(3) = TL       load torque, N*m
%
% Outputs:
%   y(1) = id
%   y(2) = iq
%   y(3) = omega_m

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes();
    case 1
        sys = mdlDerivatives(t,x,u);
    case 3
        sys = mdlOutputs(t,x,u);
    case {2,4,9}
        sys = [];
    otherwise
        error('pmsm_bo1808:UnhandledFlag', 'Unhandled S-function flag = %d', flag);
end

end

function [sys,x0,str,ts] = mdlInitializeSizes()
sizes = simsizes;
sizes.NumContStates  = 3;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = [0; 0; 0];
str = [];
ts  = [0 0];
end

function sys = mdlDerivatives(~,x,u)
% BO1808NBH2B 11.4 V parameters, SI units.
pn = 4;
Rs = 7.2;
Ld = 0.37e-3;
Lq = 0.37e-3;
Kt = 6.4e-3;
psi_f = Kt/(1.5*pn);
J = 1.4e-7;
N0 = 14300;
I0 = 0.12;
omega0 = N0*2*pi/60;
B = Kt*I0/omega0;

sys = zeros(3,1);
sys(1) = (1/Ld)*u(1) - (Rs/Ld)*x(1) + (Lq/Ld)*pn*x(2)*x(3);
sys(2) = (1/Lq)*u(2) - (Rs/Lq)*x(2) - (Ld/Lq)*pn*x(1)*x(3) - (psi_f*pn/Lq)*x(3);
sys(3) = (1/J)*(1.5*pn*psi_f*x(2) - B*x(3) - u(3));
end

function sys = mdlOutputs(~,x,~)
sys = [x(1); x(2); x(3)];
end
