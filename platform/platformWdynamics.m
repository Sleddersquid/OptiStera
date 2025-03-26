 clear; close all;
rb = 190;
rt = 125;
h = 414; % Height offset
phi = 0*pi/180; % Rotation angle around x-axis
theta = 30*pi/180; % Rotation angle around y-axis

% Define actuator connection points on bottom plate (B)
B1 = [rb; 0; 0];
B2 = [-rb/2; sqrt(3)*rb/2; 0];
B3 = [-rb/2; -sqrt(3)*rb/2; 0];

% Define actuator connection points on top plate (T) before transformation
T1_local = [rt; 0; 0]; 
T2_local = [-rt/2; sqrt(3)*rt/2; 0];
T3_local = [-rt/2; -sqrt(3)*rt/2; 0];

% Define circles for bottom and top plates
theta_circle = linspace(0, 2*pi, 100);
bottom_circle = rb * [cos(theta_circle); sin(theta_circle); zeros(1, 100)];
top_circle_local = rt * [cos(theta_circle); sin(theta_circle); zeros(1, 100)];

% Rotation matrices
Rx = [1 0 0;
      0 cos(phi) -sin(phi);
      0 sin(phi) cos(phi)];
  
Ry = [cos(theta) 0 sin(theta);
      0 1 0;
      -sin(theta) 0 cos(theta)];
  
R = Ry*Rx;
%R = Rx*Ry

% Platform position offset
P = [0; 0; h];

% Transform top plate connection points and circle
T1 = R * T1_local + P;
T2 = R * T2_local + P;
T3 = R * T3_local + P;
top_circle = R * top_circle_local + P;

% Compute leg vectors
l1 = T1 - B1;
l2 = T2 - B2;
l3 = T3 - B3;

% Compute leg lengths
L1 = norm(l1);
L2 = norm(l2);
L3 = norm(l3);

% Plot the structure
figure; hold on; axis equal;

% Plot bottom and top plate circles
plot3(bottom_circle(1, :), bottom_circle(2, :), bottom_circle(3, :), 'b', 'LineWidth', 1.5);
plot3(top_circle(1, :), top_circle(2, :), top_circle(3, :), 'r', 'LineWidth', 1.5);

% Plot bottom and top plate points
scatter3([B1(1), B2(1), B3(1)], [B1(2), B2(2), B3(2)], [B1(3), B2(3), B3(3)], 100, 'bo', 'filled');
scatter3([T1(1), T2(1), T3(1)], [T1(2), T2(2), T3(2)], [T1(3), T2(3), T3(3)], 100, 'ro', 'filled');

% Plot leg vectors
plot3([B1(1), T1(1)], [B1(2), T1(2)], [B1(3), T1(3)], 'k', 'LineWidth', 2);
plot3([B2(1), T2(1)], [B2(2), T2(2)], [B2(3), T2(3)], 'k', 'LineWidth', 2);
plot3([B3(1), T3(1)], [B3(2), T3(2)], [B3(3), T3(3)], 'k', 'LineWidth', 2);

% Display leg lengths
text(B1(1), 25, B1(3), sprintf('L1 = %.1f', L1), 'FontSize', 12, 'Color', 'k');
text(-60, 150, B2(3), sprintf('L2 = %.1f', L2), 'FontSize', 12, 'Color', 'k');
text(-60, -150, B3(3), sprintf('L3 = %.1f', L3), 'FontSize', 12, 'Color', 'k');

title('Leg Vectors and Top Plate Rotation');
xlabel('X-axis'); ylabel('Y-axis'); zlabel('Z-axis');
grid on;
