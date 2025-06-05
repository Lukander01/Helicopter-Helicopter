clear all;
close all;
clc


x0 = [0;0;0]; % initial state

% T = 40;    % length of experiment
h = 0.01;   % sampling interval
T = 2;
t = 0:h:T;

t_offs = 0:h:1;
U_offs = zeros(size(t_offs));
timeser = timeseries(U_offs, t_offs);
sim('helicoptertemplate');
%y_offset = simOut.yout;
%size(y_offset)

y_offset = [alpha; omega];

T = 90;
t = 0:h:T;

% input:
U = 1*ones(1,length(t));

timeser = timeseries(U,t);
U_cost = [t', U'];
sim('helicoptertemplate');
y = [alpha - mean(y_offset(1,:)); omega - mean(y_offset(2,:))];

seconds_to_steady = 60;

alpha_steady = mean(y(1).Data((T-seconds_to_steady)/h:end));
omega_steady = mean(y(2).Data((T-seconds_to_steady)/h:end));

fprintf('\nSteady U: %.2f', U(1));
fprintf('\nSteady alpha: %.2f', alpha_steady);
fprintf('\nSteady omega: %.2f', omega_steady);

%%
figure;
subplot(4,1,1)
plot(y(1).Data((T-seconds_to_steady)/h:end));
title('From steady state on')
ylabel('\alpha [rad]')
subplot(4,1,2)
plot((y(1).Data((T-seconds_to_steady)/h:end)) * 360/(2 * pi));
ylabel('\alpha [deg]')
subplot(4,1,3)
plot(y(2).Data((T-seconds_to_steady)/h:end));
ylabel('\omega [rad/s]')
subplot(4,1,4);
plot(U((T-seconds_to_steady)/h:end));
ylabel('U [a.u.]')
xlabel('t [s]')

figure
subplot(4,1,1)
title('Reaching steady state')
plot(y(1).Data);
ylabel('\alpha [rad]')
subplot(4,1,2)
plot(y(1).Data * 360/(2 * pi));
ylabel('\alpha [deg]')
subplot(4,1,3)
plot(y(2).Data);
ylabel('\omega [rad/s]')
subplot(4,1,4);
plot(U);
ylabel('U [a.u.]')
xlabel('t [s]')




