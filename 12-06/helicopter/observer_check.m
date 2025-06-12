clc
clear all;
close all;

alpha_ref = -0.4;%30 * (2 * pi) / 360;

[A_lin,B_lin, C, L] = observer(alpha_ref);
% 
% A_lin = obs_matrix(1);
% B_lin = obs_matrix(2);
% C= obs_matrix(3);
% L = obs_matrix(4);
p = eig(A_lin);

x0 = [0;0;0]; % initial state
h = 0.01;   % sampling interval
T = 30;
t = 0:h:T;
U = 0.8*ones(1,length(t)) + 0.1*sin(1*2*pi*t);
timeser = timeseries(U, t);

% delmo = c2d(ss(A_lin,B_lin,C,[]),h);

% A_discrete = delmo.A;
% B_discrete = delmo.B;
% C_discrete = delmo.C;
% p = eig(A_discrete);

sim("helicoptertemplate.slx");


%%
    figure;
    hold on;
    subplot(2,1,1)
    plot(difference(1).Data(1, :));
    grid on;
    title('Observer error')
    xlabel('t[s]')
    ylabel('\alpha [rad]')
    subplot(2,1,2)
    plot(difference(1).Data(2, :));
    grid on;
    title('Observer error')
    xlabel('t[s]')
    ylabel('\omega [rad/s]')
    hold off;

%%
    figure;
    hold on;
    subplot(3,1,1)
    plot(x_hat(1).Data(1, :));
    grid on;
    title('x_{hat}')
    xlabel('t[s]')
    ylabel('\alpha [rad]')
    subplot(3,1,2)
    plot(x_hat(1).Data(2, :));
    grid on;
    title('x_{hat}')
    xlabel('t[s]');
    ylabel('d\alpha/dt [rad/s]');
    subplot(3,1,3)
    plot(x_hat(1).Data(3, :));
    grid on;
    title('x_{hat}')
    xlabel('t[s]')
    ylabel('\omega [rad/s]')
    hold off;

   %%
    figure;
    hold on;
    subplot(2,1,1)
    plot(alpha(1).Data);
    grid on;
    title('Real output');
    xlabel('t[s]')
    ylabel('\alpha [rad]')
    subplot(2,1,2)
    plot(omega(1).Data);
    title('Real output');
    xlabel('t[s]')
    ylabel('\omega [rad/s]')
    grid on;
    hold off;
    


