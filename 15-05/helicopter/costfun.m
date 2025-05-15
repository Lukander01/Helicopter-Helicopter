function e = costfun(x,U_cost,y)
% cost function for nonlinear parameter tuning

% How many times error in x1 is more important
imp_factor = 1000;

assignin('base','b',x(1));% assign bhat in workspace
assignin('base','K1',x(2));
assignin('base','K3',x(3));
assignin('base','K4',x(4));

[tm,xm,ym]=sim('helicopter_hat',U_cost(:,1),[],U_cost);  % simulate nonlinear model

y_alfa = y(1).Data;
y_omega = y(2).Data;
y_ = [y_alfa, y_omega];

y_alfa_norm = [max(abs(y_(:, 1)))/imp_factor; max(abs(y_(:, 2))) ];

e = (y_-ym) ./ y_alfa_norm';                               % residual (error)



if (false)
    fit_alpha = 100 * (1 - norm(y_(:,1) - ym(:,1)) / norm(y_(:,1) - mean(y_(:,1))));
    fit_omega = 100 * (1 - norm(y_(:,2) - ym(:,2)) / norm(y_(:,2) - mean(y_(:,2))));
    fprintf('Fit: alpha = %.2f%%, omega = %.2f%%\n', fit_alpha, fit_omega);
end
if (false)
    figure(1)
    subplot(2,1,1)
    stairs(tm,[y_(:,1) ym(:,1)])
    legend('Real','Simulation')
    subplot(2,1,2)
    stairs(tm,[y_(:,2) ym(:,2)])
    legend('Real','Simulation')
end


%pause