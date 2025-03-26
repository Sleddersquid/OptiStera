clear; close all;
% Parameters
rb = 190;
rt = 125;
h = 414;
phi = 0 * pi / 180;
theta = 0 * pi / 180;

% Bottom plate actuator points
B1 = [rb; 0; 0];
B2 = [-rb/2; sqrt(3)*rb/2; 0];
B3 = [-rb/2; -sqrt(3)*rb/2; 0];

% Top plate actuator points (local)
T1_local = [rt; 0; 0];
T2_local = [-rt/2; sqrt(3)*rt/2; 0];
T3_local = [-rt/2; -sqrt(3)*rt/2; 0];



% Rotation matrices
Rx = [1 0 0;
      0 cos(phi) -sin(phi);
      0 sin(phi) cos(phi)];

Ry = [cos(theta) 0 sin(theta);
      0 1 0;
      -sin(theta) 0 cos(theta)];

R = Ry * Rx;

P = [0; 0; h];

% Transform top plate
T1 = R * T1_local + P;
T2 = R * T2_local + P;
T3 = R * T3_local + P;

% Circle points for plates
theta_circle = linspace(0, 2*pi, 100);
bottom_circle = rb * [cos(theta_circle); sin(theta_circle); zeros(1, 100)];
top_circle_local = rt * [cos(theta_circle); sin(theta_circle); zeros(1, 100)];
top_circle = R * top_circle_local + P;

% Leg vectors and lengths
l1 = T1 - B1; L1 = norm(l1);
l2 = T2 - B2; L2 = norm(l2);
l3 = T3 - B3; L3 = norm(l3);

% Plotting
figure; hold on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z'); 
set(get(gca,'zlabel'),'rotation',0)
grid on;
title('3D Stewart Platform Representation');

% Plot bottom and top plates as filled disks
fill3(bottom_circle(1,:), bottom_circle(2,:), bottom_circle(3,:),'b', 'FaceAlpha', 0.3, 'EdgeColor', 'b', 'LineWidth', 1.5);
fill3(top_circle(1,:), top_circle(2,:), top_circle(3,:),'r', 'FaceAlpha', 0.3, 'EdgeColor', 'r', 'LineWidth', 1.5);

% Plot connection points
scatter3([B1(1), B2(1), B3(1)], [B1(2), B2(2), B3(2)], [B1(3), B2(3), B3(3)],80, 'bo', 'filled');
scatter3([T1(1), T2(1), T3(1)], [T1(2), T2(2), T3(2)], [T1(3), T2(3), T3(3)],80, 'ro', 'filled');

% Plot legs
plot3([B1(1), T1(1)], [B1(2), T1(2)], [B1(3), T1(3)], 'k-', 'LineWidth', 2);
plot3([B2(1), T2(1)], [B2(2), T2(2)], [B2(3), T2(3)], 'k-', 'LineWidth', 2);
plot3([B3(1), T3(1)], [B3(2), T3(2)], [B3(3), T3(3)], 'k-', 'LineWidth', 2);

% Display leg lengths
% text(B1(1), B1(2), B1(3), sprintf('L1 = %.1f', L1), 'FontSize', 10, 'Color', 'k');
% text(B2(1), B2(2), B2(3), sprintf('L2 = %.1f', L2), 'FontSize', 10, 'Color', 'k');
% text(B3(1), B3(2), B3(3), sprintf('L3 = %.1f', L3), 'FontSize', 10, 'Color', 'k');

%3D appearance
view(30, 25); % 3D viewing angle
camlight headlight; 
lighting gouraud;
axis vis3d;

% Reference coordinate system at bottom center
axis_length = 100; % length of the axis arrows

% X-axis (red)
quiver3(0, 0, 0, axis_length, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
text(axis_length, 0, 0, 'X', 'FontSize', 12, 'Color', 'k');

% Y-axis (green)
quiver3(0, 0, 0, 0, axis_length, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
text(0, axis_length, 0, 'Y', 'FontSize', 12, 'Color', 'k');

% Z-axis (blue)
quiver3(0, 0, 0, 0, 0, axis_length, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
text(0, 0, axis_length, 'Z', 'FontSize', 12, 'Color', 'k');


function drawCoordinateFrame(origin, axisLength)
    quiver3(origin(1), origin(2), origin(3), axisLength, 0, 0, 'r', 'LineWidth', 2);
    text(origin(1)+axisLength, origin(2), origin(3), 'X', 'FontSize', 12, 'Color', 'r');

    quiver3(origin(1), origin(2), origin(3), 0, axisLength, 0, 'g', 'LineWidth', 2);
    text(origin(1), origin(2)+axisLength, origin(3), 'Y', 'FontSize', 12, 'Color', 'g');

    quiver3(origin(1), origin(2), origin(3), 0, 0, axisLength, 'b', 'LineWidth', 2);
    text(origin(1), origin(2), origin(3)+axisLength, 'Z', 'FontSize', 12, 'Color', 'b');
end












