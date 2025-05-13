function e = costfun(x,U_cost,y)
% cost function for nonlinear parameter tuning

assignin('base','b',x(1));% assign bhat in workspace
assignin('base','K1',x(2));
%assignin('base','K2',x(3));
assignin('base','K3',x(3));
assignin('base','K4',x(4));

[tm,xm,ym]=sim('helicopter_hat',U_cost(:,1),[],U_cost);  % simulate nonlinear model

size(ym)

%size(y)

y_alfa = y(1).Data;
y_omega = y(2).Data;
y_ = [y_alfa, y_omega];

size(y_)

y_alfa_norm = [max(abs(y_(:, 1)))/100; max(abs(y_(:, 2))) ];

size(y_alfa_norm)

e = (y_-ym) ./ y_alfa_norm';                               % residual (error)

% you can comment the below line to speed up
figure(1)           % intermediate fit
subplot(2,1,1)
stairs(tm,[y_(:,1) ym(:,1)])
legend('Real','Simulation')
subplot(2,1,2)
stairs(tm,[y_(:,2) ym(:,2)])
legend('Real','Simulation')

% error_alpha = sum((y_alfa - ym(:,1))./y_alfa) / (/h)
%pause