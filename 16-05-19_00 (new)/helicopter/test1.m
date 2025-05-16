clear all;
close all;
clc
%%

% negative iput
fan1_voltage__ = -[0, 0.2, 0.4, 0.6, 0.8, 1];
fan1_omega__ = [0, -127, -204, -262, -312, -350];

figure;
hold on;
plot(fan1_voltage__, fan1_omega__, '-o');

title('Calibration for negative input \omega(voltage) in steady state');
hold off;

fan1_voltage_n = -[-1, -0.6, -0.2, 0];
fan1_omega_n = [352, 266, 128, 0];

figure;
hold on;
plot(fan1_voltage_n, fan1_omega_n, '-o');

title('Calibration for positive input \omega(voltage) in steady state');
hold off;


% omega:
% radians/s 


%%
fan1_voltage = -[-1, -0.6, -0.2, 0, 0.2, 0.4, 0.6, 0.8, 1];
fan1_omega = [352, 266, 128, 0, -127, -204, -262, -312, -350];
figure;
plot(fan1_voltage, fan1_omega, '-o');
title('calibration fan1: omega(voltage)');

% output1, 2 radians, no calibration, constant offset measured for input
% 0,0 is ~0.04, ~0.02
% gain -> -351 fan_1, 

fan2_voltage = -[-1, -0.6, -0.2, 0, 0.2, 0.4, 0.6, 0.8, 1];
fan2_omega = [342, 255, 123, 0, -122, -195, -253, -300, -342];
figure;
plot(fan2_voltage, fan2_omega, '-o');
title('calibration fan2: omega(voltage)');

figure;
hold on;
plot(fan1_voltage, fan1_omega, '-o');
plot(fan2_voltage, fan2_omega, '-o');
title('comparison');
hold off;
