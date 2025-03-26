clc; clear; close all;

rb = 190; % Base radius
ra = 145; % Moving platform radius
h0 = 402; % Initial height of the platform
hf = 450; % Final height of the platform
t0 = 0; % Initial time
tf = 8; % Final time

t = linspace(t0, tf, 100); 

% Kane's transition function 
h = h0 + (hf-h0)*(t-t0)/(tf-t0)-((hf-h0)/(2*pi))*sin(2*pi*((t-t0)/(tf-t0)));

theta = 0; 
psi = 0; 

l1 = (cos(theta).*ra - rb).^2 + (-sin(theta).*ra + h).^2;

l2 = (-0.5.*cos(theta).*ra + (1/3).*sin(theta).*sin(psi).*sqrt(3).*ra + 0.5.*rb).^2 + ...
     (0.5.*cos(psi).*sqrt(3).*ra - 0.5.*sqrt(3).*rb).^2 + ...
     (0.5.*sin(theta).*ra + 0.5.*cos(theta).*sin(psi).*sqrt(3).*ra + h).^2;

l3 = (-0.5.*cos(theta).*ra - 0.5.*sin(theta).*sin(psi).*sqrt(3).*ra + 0.5.*rb).^2 + ...
     (-0.5.*cos(psi).*sqrt(3).*ra + 0.5.*sqrt(3).*rb).^2 + ...
     (0.5.*sin(theta).*ra - 0.5.*cos(theta).*sin(psi).*sqrt(3).*ra + h).^2;


L1 = sqrt(l1);
L2 = sqrt(l2);
L3 = sqrt(l3);

dt = t(2) - t(1);
vh = gradient(h, dt);
ah = gradient(vh, dt);


figure;
subplot(3,1,1);
plot(t, h, 'b');
title('Position of Platform in Z-axis');
xlabel('Time (s)'); ylabel('Position (mm)');
legend('Pos'); grid on;

subplot(3,1,2);
plot(t, vh, 'b');
title('Velocity of Platform in Z-axis');
xlabel('Time (s)'); ylabel('Velocity (mm/s)');
legend('velocity'); grid on;

subplot(3,1,3);
plot(t, ah, 'b');
title('Acceleration of Platform in Z-axis');
xlabel('Time (s)'); ylabel('Acceleration (mm/s^2)');
legend('acceleration'); grid on;

