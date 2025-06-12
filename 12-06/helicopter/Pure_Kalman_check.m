clc
clear all;
close all;

alpha_ref = -0.37;%30 * (2 * pi) / 360;

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
U = -0.6*ones(1,length(t)); %+ 0.1*sin(1*2*pi*t);
timeser = timeseries(U, t);

%Q = [8*10^(3) 10 0; 0 1*10^(5) 10; 0 0 1*10^(10)];
%R = [1*10^(2) 0;0 1*10^(8)];

Q = [5*10^(1) 0 0; 0 2*5*10^(1) 0; 0 0 30*10^(1)];
R = [1*10^(1) 0;0 8*10^(1)];

% delmo = c2d(ss(A_lin,B_lin,C,[]),h);

% A_discrete = delmo.A;
% B_discrete = delmo.B;
% C_discrete = delmo.C;
% p = eig(A_discrete);

sim("helicopter_Kalman_pole.slx");