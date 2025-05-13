clc, clear, close all

s = tf('s');

% % Define motor parameters - EMMT-ST-57-L-RMB
R = 0.26; % Ohm - from Festo
L = 0.95e-3; % Henry - from Festo
B = 1.09e-5; % Nm/s
J = 0.51e-6; % Nm/s^2 - from Festo
kt = 0.32e-3; % Nm/A - from Festo
ke = 0.32e-3;% Vs/rad - from Festo

K = kt/(B*R+kt*ke)
Tau = (R*J)/(B*R+kt*ke)
% % 
% % PID values from model
% P = 0.12116047045116
% T_i = 1.29916833561059
% T_d = 0.068625362231904

% Define motor parameters - EMMT-ST-42-S-RMB
% R = 2.1; % Ohm - from Festo
% L = 0.3e-3; % Henry - from Festo
% B = 1.09e-5; % Nm/s
% J = 0.043e-6; % Nm/s^2 - from Festo
% kt = 0.133e-3; % Nm/A - from Festo
% ke = 0.133e-3;% Vs/rad - from Festo

% PID values from model
% P = 2.54663783566455
% T_i = 1.44791962192646
% T_d = 0.022556063524038
% 
% K = kt/(B*R+kt*ke)
% Tau = (R*J)/(B*R+kt*ke)
