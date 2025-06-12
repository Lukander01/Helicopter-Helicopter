function [x_bar, u_bar] = solve_offset_free_mpc_output_feedback(A, B, Bd, Q, R, P, x_hat, N, u_lb, u_ub, x_ref, u_ref, x_min, x_max, rho)

n_x = size(A, 1);
n_u = size(B, 2);

% Prediction matrices
[T, S] = gen_prediction_matrices(A, B, N);

u = sdpvar(N * n_u, 1);
x = T * x_hat + S * u;

% Cost
cost = compute_mpc_cost_terms(Q, R, P, T, S, x_hat, N, x_ref, u_ref, u);

% Constraints
[G_u, g_u] = gen_constraint_matrices(u_lb, u_ub, N);
constraints = [G_u * u <= g_u];
constraints = [constraints; gen_state_constraints(x, N, n_x, x_min, x_max)];

% Terminal constraint
x_N = x((N * n_x + 1):(N + 1) * n_x);
constraints = [constraints; (x_N - x_ref)' * P * (x_N - x_ref) <= rho];

% Solve
options = sdpsettings('solver', 'quadprog', 'verbose', 0);
sol = optimize(constraints, cost, options);

if sol.problem ~= 0
    warning('MPC problem infeasible (OFFSET)');
    u_bar = zeros(n_u, N);
    x_bar = zeros(n_x, N + 1);
else
    u_bar = reshape(value(u), n_u, N);
    x_bar = reshape(value(x), n_x, N + 1);
end

end
