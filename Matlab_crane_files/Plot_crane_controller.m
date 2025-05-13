clc, clear, close all

s = tf('s');

% Define motor parameters - EMMT-ST-42-S-RMB
R = 2.1; % Ohm - from Festo
L = 0.3e-3; % Henry - from Festo
B = 1.09e-5; % Nm/s
J = 0.043e-6; % Nm/s^2 - from Festo
kt = 0.133e-3; % Nm/A - from Festo
ke = 0.133e-3;% Vs/rad - from Festo

K = kt/(B*R+kt*ke)
Tau = (R*J)/(B*R+kt*ke)

P_gain = 2.54663783566455
T_i = 1.44791962192646
T_d = 0.022556063524038
N = 5
Ts = 0.02

% P_gain = 4
% T_i = 0.8
% T_d = 0.02
% N = 100
% Ts = 0.02

P = K/(Tau*s^2+s)

C = pidstd(P_gain,T_i,T_d,N)
Cd = pidstd(P_gain,T_i,T_d,N,Ts,'DFormula', 'Trapezoidal')

%Cd = c2d(C,Ts)
Pd = c2d(P,Ts)

sysCl = (C*P)/(1+C*P)

sysW = P/(1-C*P)
sysWd = Pd/(1-Cd*Pd)

sysCld = (Cd*Pd)/(1+Cd*Pd)

%Plot step-response
% LineWidth = 2;

figure()
%step(sysCld)
pzplot(sysCld)
% margin(sysCl)
% 
% hold on
% 
% figure(2)
% margin(sysW)