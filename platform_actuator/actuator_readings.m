clear
clc
close all

readings=readtable('11s.csv');

time = readings{:,1};
curr1 = readings{:,2};
curr2 = readings{:,3};
curr3 = readings{:,4};
desi1 = readings{:,5};
desi2 = readings{:,6};
desi3 = readings{:,7};

variation = mean(abs(diff(time)));
disp(['Average variation: ', num2str(variation)]);

disp(['Max (Actuator 1): ', num2str(max(curr1))]);
disp(['Max (Actuator 2): ', num2str(max(curr2))]);
disp(['Max (Actuator 3): ', num2str(max(curr3))]);

figure;
hold on;
plot(time, curr1, time, curr2, time, curr3, time, desi1);
xlabel('Time (ms)');
ylabel('Actuator Length (units)')
% xlim([0 6000]);
legend('Current Position Actuator 1', 'Current Position Actuator 2', 'Current Position Actuator 3', 'Desired Position');
title('Current Position vs Desired Position');

figure;
hold on;
plot(time, desi1 - curr1, time, desi2 - curr2, time, desi3 - curr3);
xlabel('Time (ms)');
ylabel('Actuator Speed (units)')
% xlim([0 6000]);
legend('Actuator 1', 'Actuator 2', 'Actuator 3');
title('Difference in Current and Desired Position');

% figure;
% hold on;
% plot(time, curr1, time, desi1);
% xlabel('Time (ms)')
% % ylim([0.6 1])
% legend('Current', 'Desired')
% title('Current vs Desired (Actuator 1)')
% 
% figure;
% hold on;
% plot(time, curr2, time, desi2);
% xlabel('Time (ms)')
% % ylim([0.6 1])
% legend('Current', 'Desired')
% title('Current vs Desired (Actuator 2)')
% 
% figure;
% hold on;
% plot(time, curr3, time, desi3);
% xlabel('Time (ms)')
% % ylim([0.6 1])
% legend('Current', 'Desired')
% title('Current vs Desired (Actuator 3)')

