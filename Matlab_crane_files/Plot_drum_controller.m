clc, clear, close all

s = tf('s');

% Define motor parameters - EMMT-ST-57-L-RMB
R = 0.26; % Ohm - from Festo
L = 0.95e-3; % Henry - from Festo
B = 1.09e-5; % Nm/s
J = 0.51e-6; % Nm/s^2 - from Festo
kt = 0.32e-3; % Nm/A - from Festo
ke = 0.32e-3;% Vs/rad - from Festo

K = kt/(B*R+kt*ke)
Tau = (R*J)/(B*R+kt*ke)

P_gain = 0.12116047045116
T_i = 1.29916833561059
T_d = 0.068625362231904
N = 5

% P_gain = 0.75
% T_i = 0.15
% T_d = 0.06
% N = 500

Ts = 0.02

P = K/(Tau*s^2+s)

C = pidstd(P_gain,T_i,T_d)

Cd = pidstd(P_gain,T_i,T_d,N,Ts,'DFormula', 'Trapezoidal')

%Cd = c2d(C,Ts)
Pd = c2d(P,Ts)
sysOL = P*C

sysCL = (C*P)/(1+C*P)

sysCld = (Cd*Pd)/(1+Cd*Pd)
sysW = P/(1-C*P)

%Plot step-response
% LineWidth = 2;

figure(1)
bode(sysCL)
%step(sysCld)
pzplot(sysCld)
% margin(sysOL)
% hold on
% 
% figure(2)
% margin(sysW)