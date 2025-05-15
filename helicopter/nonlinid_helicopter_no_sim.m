% Estimate (fine-tune) a parameter in a nonlinear Simulink model using nonlinear least-squares optimization.
% NOTE: this method works very well in this simple example, but its performance quickly will deteriorate if 
% the complexity of the model increases. It is recommended to always start with linear system identification.
clear all; clc; close all;



load('U_1.mat');
load('y_moved_sin_1.mat');


T = 30;
h = 0.02;
t = 0:h:T;


% initial guesses

% bi = 2.45;
% K1i = -0.3;
% K3i = 0.1;
% K4i = 0.01;

bi_vals  = [3.0];
K1i_vals = [-10];
K3i_vals = [0.1];
K4i_vals = [1/0.7];

timeser = timeseries(U, t);

U_cost = [t', U'];


U_cost = U_cost(150:end, :); 
for i = 1:2
    y(i) = timeseries(y(i).Data(150:end), y(i).Time(150:end));
end

results = [];

OPT = optimset('MaxIter', 100);

idx = 1;
for bi = bi_vals
    for K1i = K1i_vals
        for K3i = K3i_vals
            for K4i = K4i_vals
                thetai = [bi, K1i, K3i, K4i];
                
                fprintf('\nTry:');
                fprintf('%.2f, %.2f, %.2f, %.2f', bi, K1i, K3i, K4i);

                [thetahat, fval] = lsqnonlin(@(theta) costfun(theta, U_cost, y), thetai, [], [], OPT);
                results(idx).initial = thetai;
                results(idx).estimated = thetahat;
                results(idx).fval = fval;
                idx = idx + 1;
            end
        end
    end
end

fprintf('\n--- Optimization Results ---\n');
for i = 1:length(results)
    fprintf('\nTry %d:\n', i);
    fprintf('Initial guess:    b=%.2f  K1=%.2f  K3=%.2f  K4=%.3f\n', results(i).initial);
    fprintf('Estimated params: b=%.4f  K1=%.4f  K3=%.4f  K4=%.4f\n', results(i).estimated);
    fprintf('Final cost (error): %.6f\n', results(i).fval);
end


function f_u = get_f_u(u)
f_u = 246.9*u*u - 584.4*u- 7.464;
end
