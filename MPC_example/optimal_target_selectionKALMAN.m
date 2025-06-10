function [x_ref_opt, u_ref_opt] = optimal_target_selectionKALMAN(A, B, C, R, y_ref, d_hat, u_lb, u_ub, x_min, x_max)
    n_x = size(A,1);
    n_u = size(B,2);

    x_ref = sdpvar(n_x, 1);
    u_ref = sdpvar(n_u, 1);

    constraints = [];
    constraints = [constraints, (eye(n_x) - A) * x_ref == B * u_ref];
    constraints = [constraints, C * x_ref + d_hat == y_ref];
    constraints = [constraints, u_lb <= u_ref <= u_ub];
    constraints = [constraints, x_min <= x_ref <= x_max];

    objective = u_ref' * eye(n_u) * u_ref;

    opts = sdpsettings('solver','quadprog','verbose',0);
    optimize(constraints, objective, opts);

    x_ref_opt = value(x_ref);
    u_ref_opt = value(u_ref);
end
