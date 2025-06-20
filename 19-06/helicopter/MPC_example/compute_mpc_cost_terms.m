function total_cost = compute_mpc_cost_terms(Q, R, P, T, S, x0, N, x_ref, u_ref, u)

n_x = size(Q, 1);
n_u = size(R, 1);

% Create stacked reference vectors
x_ref_bar = repmat(x_ref, N + 1, 1);
u_ref_bar = repmat(u_ref, N, 1);

% Block-diagonal cost matrices
Q_bar = kron(eye(N), Q);
R_bar = kron(eye(N), R);

% Predict state trajectory
x_pred = T * x0 + S * u;

% Errors
x_err = x_pred - x_ref_bar;
xN_err = x_pred(end - n_x + 1:end) - x_ref;
u_err = u - u_ref_bar;

% Exclude terminal state from stage cost
x_err_stage = x_err(1:end - n_x);

% Cost terms
stage_cost = 0.5 * x_err_stage' * Q_bar * x_err_stage;
terminal_cost = 0.5 * xN_err' * P * xN_err;
control_cost = 0.5 * u_err' * R_bar * u_err;

total_cost = stage_cost + terminal_cost + control_cost;

end
