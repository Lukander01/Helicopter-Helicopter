clear all
close all
clc


% % Positive input
b = 0.025936940902745;
K1 = -10.577796958770156;
K3 = -2.433475804656021e-05;
K4 = 5.379206715771331;


% Negative input
% b = 0.239927543450809;
% K1 = -10.548800723683954;
% K3 = 5.498506290874219e-05;
% K4 = 4.594429729616914;


% Signal
T = 40;    % length of experiment
h = 0.01;   % sampling interval
A = 0.1;      % amplitude
t = 0:h:T;
freq = 1; 
%U = -0.7 step 
%U = -0.7 + sin(t*2*pi*freq)*A;
%U = -0.6 + chirp(t,0,T,1)*A;


% U = 0.7*ones(1,length(t)); % 73%
% U = 0.8 + sin(t*2*pi*freq)*A; 65%
% U = 0.6 + sin(t*2*pi*freq)*A; 69%
% U = 0.8 + chirp(t,0,T,1)*A; % 78%

U = 0.4 + sin(t*2*pi*freq)*A; %32 (shity close to 0)

U_cost = [t', U'];

% Offset aproxiamtion

T = 2;

t_offs = 0:h:T;
U_offs = zeros(size(t_offs));
timeser = timeseries(U_offs, t_offs);
sim('helicoptertemplate');
%y_offset = simOut.yout;
%size(y_offset)

y_offset = [alpha; omega];

T = 40;

timeser = timeseries(U,t);
%[tm_real,xm_real,ym_real]=sim('helicoptertemplate',U_cost(:,1),[],U_cost);
sim('helicoptertemplate');

y = [alpha - mean(y_offset(1,:)); omega - mean(y_offset(2,:))];
%%
y_alfa_real = y(1).Data;
y_omega_real = y(2).Data;

[tm_sim,xm_sim,ym_sim]=sim('helicopter_hat',U_cost(:,1),[],U_cost);

y_ = [y_alfa_real, y_omega_real];
ym = ym_sim;


% e = (y_-ym);


if (true)
    fit_alpha = 100 * (1 - norm(y_(:,1) - ym(:,1)) / norm(y_(:,1) - mean(y_(:,1))));
    fit_omega = 100 * (1 - norm(y_(:,2) - ym(:,2)) / norm(y_(:,2) - mean(y_(:,2))));
    fprintf('Fit: alpha = %.2f%%, omega = %.2f%%\n', fit_alpha, fit_omega);
end
if (true)
    figure(1)
    subplot(2,1,1)
    stairs(tm_sim,[y_(:,1) ym(:,1)])
    legend('Real','Simulation')
    subplot(2,1,2)
    stairs(tm_sim,[y_(:,2) ym(:,2)])
    legend('Real','Simulation')
end






