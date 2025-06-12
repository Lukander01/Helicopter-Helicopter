function [x_ref_opt, u_ref_opt] = optimal_target_selection_reference_tracking(A, B, C, y_ref, u_lb, u_ub, x_min, x_max)

    n_x = size(A, 1);
    n_u = size(B, 2);

    % Decision variables
    x_ref = sdpvar(n_x, 1);
    u_ref = sdpvar(n_u, 1);

    % Cost function: minimize control effort
    R_eye = eye(n_u);
    objective = u_ref' * R_eye * u_ref;

    % Constraints
    constraints = [];
    constraints = [constraints, (eye(n_x) - A) * x_ref == B * u_ref];   % steady-state
    constraints = [constraints, C * x_ref == y_ref];                    % output matching
    constraints = [constraints, u_lb <= u_ref <= u_ub];                 % input bounds
    constraints = [constraints, x_min <= x_ref <= x_max];              % state bounds

    % Solve
    options = sdpsettings('solver', 'quadprog', 'verbose', 0);
    optimize(constraints, objective, options);

    % Return result
    x_ref_opt = value(x_ref);
    u_ref_opt = value(u_ref);
end
