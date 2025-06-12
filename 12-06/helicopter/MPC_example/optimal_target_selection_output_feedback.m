function [x_ref_val, u_ref_val] = optimal_target_selection_output_feedback(A, B, Bd, C, Cd, R, y_ref, d_hat, u_lb, u_ub, x_min, x_max)

n_x = size(A, 1);
n_u = size(B, 2);

x_ref = sdpvar(n_x, 1);
u_ref = sdpvar(n_u, 1);

cost = u_ref' * R * u_ref;

% Steady-state equations (partial)
eq1 = (eye(n_x) - A) * x_ref - B * u_ref;
eq1 = eq1(1:3) == (Cd(1:3, :) * d_hat);

eq2 = C * x_ref + Cd * d_hat == y_ref;

constraints = [eq1; eq2; x_ref >= x_min; x_ref <= x_max];
% Optional: constraints = [constraints; u_ref >= u_lb; u_ref <= u_ub];

options = sdpsettings('solver', 'quadprog', 'verbose', 0);
sol = optimize(constraints, cost, options);

if sol.problem ~= 0
    warning('MPC problem infeasible (OTS)');
    x_ref_val = [];
    u_ref_val = [];
else
    x_ref_val = value(x_ref);
    u_ref_val = value(u_ref);
end

end
