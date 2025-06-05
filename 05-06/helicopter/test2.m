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

% negative iput
fan1_alpha__ = [-0.65, -0.52, -0.37, -0.22, -0.09, 0];
fan1_omega__ = [-335.03, -301.68, -259.50, -203.99, -130.75, 0];

figure;
hold on;
plot(fan1_alpha__, fan1_omega__, '-o');

title('Calibration for negative \omega from \alpha in steady state');
hold off;

fan1_alpha_n = [0, 0.04, 0.10, 0.16, 0.22, 0.29];
fan1_omega_n = [0, 128.82, 208.47, 263.25, 306.39, 337.07];

figure;
hold on;
plot(fan1_alpha_n, fan1_omega_n, '-o');

title('Calibration for positive \omega from \alpha in steady state');
hold off;

% negative:
%negative alpha = 6659*x^4 + 1.083*10000 *x^3 + 6343 * x * x +1888*x -
%1.157

%positive alpha = -1.905*100000*x^4 + 1.299*100000*x^3 - 3.225*10000 * x *
%x + 4233*x + 0.7921

% positive:
% positive alpha= -284.3 * x * x + 609 * x + 7.086;

%% From alpha to U

fan1_voltage__ = [-1, -0.8, -0.6, -0.4, -0.2, 0];
fan1_alpha__ = [-0.65, -0.52, -0.37, -0.22, -0.09, 0];

fan1_voltage_n = [0, 0.2, 0.4, 0.6, 0.8, 1];
fan1_alpha_n = [0, 0.04, 0.10, 0.16, 0.22, 0.29];


figure
plot(fan1_alpha__,fan1_voltage__)
title('\alpha to voltage')
xlabel('\alpha')
ylabel('voltage')


figure
plot(fan1_alpha_n,fan1_voltage_n)
title('\alpha to voltage')
xlabel('\alpha')
ylabel('voltage')





