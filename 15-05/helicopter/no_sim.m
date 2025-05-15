% Estimate (fine-tune) a parameter in a nonlinear Simulink model using nonlinear least-squares optimization.
% NOTE: this method works very well in this simple example, but its performance quickly will deteriorate if 
% the complexity of the model increases. It is recommended to always start with linear system identification.
clear all; clc; close all;



% load('U_2.mat');
% load('y_moved_sin_2.mat');

load('chirp_signal1_UP.mat');
load('chirp_y1_UP.mat');
% U = timeser.Data(1,1,:);



T = 40;
h = 0.01;
t = 0:h:T;


% initial guesses

% bi = 2.45;
% K1i = -0.3;
% K3i = 0.1;
% K4i = 0.01;

% bi_vals  = [1.0, 10];
% K1i_vals = [-30, -10];
% K3i_vals = [-20, -8, 8, 20];
% K4i_vals = [1/0.7, 5];    %[1/0.7, 10];

bi_vals  = [2];
K1i_vals = [-10];
K3i_vals = [3, 5 ];
K4i_vals = [3, 20, 50];    %[1/0.7, 10];


%timeser = timeseries(U, t);

U_cost = [t', U'];

hnkl=hankel(U(1:20),U(20:end));
rank(hnkl)

% U_cost = U_cost(150:end, :); 
% for i = 1:2
%     y(i) = timeseries(y(i).Data(150:end), y(i).Time(150:end));
% end

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

                try
                    [thetahat, fval] = lsqnonlin(@(theta) costfun(theta, U_cost, y), thetai, [], [], OPT);
                    results(idx).initial = thetai;
                    results(idx).estimated = thetahat;
                    results(idx).fval = fval;
                catch ME
                    warning('\nSimulation failed for [%.2f %.2f %.2f %.2f]\n',thetai(1), thetai(2), thetai(3), thetai(4));
                    results(idx).initial = thetai;
                    results(idx).estimated = [NaN NaN NaN NaN];
                    results(idx).fval = Inf;
                end
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



[min_fval, best_idx] = min([results.fval]);

best_theta = results(best_idx).estimated;
assignin('base','b',best_theta(1));
assignin('base','K1',best_theta(2));
assignin('base','K3',best_theta(3));
assignin('base','K4',best_theta(4));

fprintf('\nBest init params: b=%.4f  K1=%.4f  K3=%.4f  K4=%.4f\n', results(best_idx).initial);
fprintf('\nBest params: b=%.4f  K1=%.4f  K3=%.4f  K4=%.4f\n', best_theta(1), best_theta(2), best_theta(3),best_theta(4));

[tm,~,ym] = sim('helicopter_hat',U_cost(:,1),[],U_cost);

y_alfa = y(1).Data;
y_omega = y(2).Data;
y_ = [y_alfa, y_omega];


fit_alpha = 100 * (1 - norm(y_(:,1) - ym(:,1)) / norm(y_(:,1) - mean(y_(:,1))));
fit_omega = 100 * (1 - norm(y_(:,2) - ym(:,2)) / norm(y_(:,2) - mean(y_(:,2))));

fprintf('Fit\n');
fprintf('Fit for alpha: %.2f %%\n', fit_alpha);
fprintf('Fit for omega: %.2f %%\n', fit_omega);


figure;
subplot(2,1,1)
stairs(tm, [y_(:,1), ym(:,1)])
title('Final fit: \alpha')
legend('Real','Simulation')
subplot(2,1,2)
stairs(tm, [y_(:,2), ym(:,2)])
title('Final fit: \omega')
legend('Real','Simulation')


% For negative input:
% f_u = 246.9*u*u + 584.4*u- 7.464;

% For positive input:
% f_u = -256.8 *u*u +600.1*u+6.35