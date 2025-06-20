clc
clear all;
close all;


alpha_ref = -0.4; %30 * (2 * pi) / 360;
% 
% [A_lin,B_lin, C, L] = observer(alpha_ref);
% 
% A_lin = obs_matrix(1);
% B_lin = obs_matrix(2);
% C= obs_matrix(3);
% L = obs_matrix(4);
% p = eig(A_lin);
Q = [1,  0, 0; 0, 10, 0; 0, 0, 10];

R = [1, 0; 0, 10]; 


x0 = [0;0;0]; % initial state
h = 0.01;   % sampling interval
T = 30;
t = 0:h:T;
U = -0.4*ones(1,length(t)) + 0.1*sin(1*2*pi*t);
timeser = timeseries(U, t);

simout = sim("helicopter_observer_EKF.slx");

%%
    figure;
    hold on;
    subplot(2,1,1)
    plot(differenceEKF.Time,differenceEKF.Data(:, 1));
    grid on;
    title('EKF error')
    xlabel('t[s]')
    ylabel('\alpha [rad]')
    subplot(2,1,2)
    plot(differenceEKF.Time,differenceEKF.Data(:, 2));
    grid on;
    %title('Observer error')
    xlabel('t[s]')
    ylabel('\omega [rad/s]')
    hold off;

%%
    figure;
    subplot(3,1,1)
    plot(EKF_output.Time,EKF_output.Data(:,1));
    hold all
    plot(alpha.Time,alpha(1).Data);
    legend('EKF','Output','Location','best')
    grid on;
    title('EKF vs the output')
    xlabel('t[s]')
    ylabel('\alpha [rad]')
    subplot(3,1,2)
    plot(EKF_output.Time,EKF_output.Data(:, 2));
    grid on;
    %title('\alpha_{dot hat}')
    xlabel('t[s]');
    ylabel('d\alpha/dt [rad/s]');
    subplot(3,1,3)
    plot(EKF_output.Time,EKF_output.Data(:, 3));
    hold all
    plot(omega.Time,omega(1).Data)
    grid on;
    %title('\omega_{hat}')
    xlabel('t[s]')
    ylabel('\omega [rad/s]')
     legend('EKF','Output','Location','best')
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
    


