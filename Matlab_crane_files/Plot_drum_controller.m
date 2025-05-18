clc, clear, close all

s = tf('s');

% Define motor parameters - EMMT-ST-57-L-RMB
R = 0.26; % Ohm - from Festo
L = 0.95e-3; % Henry - from Festo
B = 1.09e-5; % Nm/s
J = 0.51e-6; % Nm/s^2 - from Festo
kt = 0.32e-3; % Nm/A - from Festo
ke = 0.32e-3;% Vs/rad - from Festo

% Define transfer function constants for motor
K = kt/(B*R+kt*ke);
Tau = (R*J)/(B*R+kt*ke);

% PID values from Simulink model
P_gain = 0.12116047045116;
T_i = 1.29916833561059;
T_d = 0.068625362231904;
N = 5;

% PID values from AIM
% P_gain = 0.75;
% T_i = 0.15;
% T_d = 0.06;
% N = 500;

% Define sampling period (20 ms)
Ts = 0.02;

% Define plant function
P = K/(Tau*s^2+s);

% Define controller function (continuous)
C = pidstd(P_gain,T_i,T_d);

% Define controller function (discrete)
Cd = pidstd(P_gain,T_i,T_d,N,Ts,'DFormula', 'Trapezoidal');

% Discrete plant
Pd = c2d(P,Ts);

% Definer closed-loop transfer functions for system (negative unity
% feedback)
sysCl = (C*P)/(1+C*P);

% Closed loop discrete transfer function for system
sysCld = (Cd*Pd)/(1+Cd*Pd);

% Closed loop transfer function (continous) for disturbance input
sysW = P/(1-C*P);


% Plotting all relevant plots
figure(1)
bode(sysCl)
hold on

figure(2)
bode(sysW)
hold on

figure(3)
margin(sysCl)
hold on

figure(4)
margin(sysW)
hold on

figure(5)
step(sysCld)
hold on

figure(6)
pzplot(sysCld)
hold on

