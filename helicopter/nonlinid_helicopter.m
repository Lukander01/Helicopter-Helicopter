% Estimate (fine-tune) a parameter in a nonlinear Simulink model using nonlinear least-squares optimization.
% NOTE: this method works very well in this simple example, but its performance quickly will deteriorate if 
% the complexity of the model increases. It is recommended to always start with linear system identification.

clear all; clc; 
close all

bi = 2.45;   % initial guesses
K1i = -0.3;
% K2i = -350;
K3i = 0.1;
K4i = 1;
% 
% bi = 1;   % initial guesses
% K1i = 1;
% K2i = 1;
% K3i = 1;
% K4i = 1;



thetai = [bi, K1i, K3i, K4i];

x0 = [0;0;0]; % initial state

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% design input excitation signal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T = 30;    % length of experiment
h = 0.02;   % sampling interval
%ts = 10;    % estimated settling time of the process
A = 0.05;      % amplitude of GBN
%U = [h*(0:T/h)' gbn(T,ts,A,h,1)];
t = 0:h:T;
freq = 3; 
U = sin(freq*t)*A + -0.6;
%U = chirp(t,0,T,0.1)*A;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data collection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Offset aproxiamtion

T = 2;

t_offs = 0:h:1;
U_offs = zeros(size(t_offs));
timeser = timeseries(U_offs, t_offs);
sim('helicoptertemplate');
%y_offset = simOut.yout;
%size(y_offset)

y_offset = [alpha; omega];

T = 30;


%sim('nlsysid');     % ouput data available in y
omega1_A = 0.1;
omega1_f = 1/200;
timeser = timeseries(U,t);
U_cost = [t', U'];
sim('helicoptertemplate');
y = [alpha - mean(y_offset(1,:)); omega - mean(y_offset(2,:))];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nonlinear optimization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
OPT = optimset('MaxIter',100);
[thetahat,fval]= lsqnonlin('costfun',thetai,[],[],OPT,U_cost,y);
thetahat, fval


function f_u = get_f_u(x)
f_u = 2.5*100*x*x - 5.8*100*x- 7.5;
end
