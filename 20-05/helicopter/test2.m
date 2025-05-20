clear all;
close all;
clc
%%

% negative iput
fan1_voltage__ = [-1, -0.8, -0.6, -0.4, -0.2, 0];
fan1_omega__ = [-335.03, -301.68, -259.50, -203.99, -130.75, 0];

figure;
hold on;
plot(fan1_voltage__, fan1_omega__, '-o');

title('Calibration for negative input \omega(voltage) in steady state');
hold off;

fan1_voltage_n = [0, 0.2, 0.4, 0.6, 0.8, 1];
fan1_omega_n = [0, 128.82, 208.47, 263.25, 306.39, 337.07];

figure;
hold on;
plot(fan1_voltage_n, fan1_omega_n, '-o');

title('Calibration for positive input \omega(voltage) in steady state');
hold off;

% negative alpha
% fan1_alpha_negative = []
% fan1_alpha_negative = []

% omega:
% radians/s 

x = 1;
% negative:
negative = 272.9 * x * x + 593.4 * x - 8.529;

% positive:
positive = -284.3 * x * x + 609 * x + 7.086;



%%