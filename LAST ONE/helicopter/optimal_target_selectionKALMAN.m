
function [x_ref_opt, u_ref_opt] = optimal_target_selectionKALMAN(A, B, C, R, y_ref, d_hat, u_lb, u_ub, x_min, x_max,Q_MPC)
    n_x = size(A, 1);
    n_u = size(B, 2);

    % Decision variable: z = [x_ref; u_ref]
    H = blkdiag(Q_MPC, R);  % Only penalize u_ref
    f = zeros(n_x + n_u, 1);     % Linear term in cost


    % (I - A)x_ref - B u_ref = 0  → [I - A, -B] * z = 0
    % C x_ref + d_hat = y_ref    → [C, 0] * z = y_ref - d_hat
    Aeq = [eye(n_x) - A, -B;
           C, zeros(size(C,1), n_u)];
    beq = [zeros(n_x,1);
           y_ref - d_hat];

    % Bounds
    lb = [];%[x_min; u_lb];
    ub = [];%[x_max; u_ub];

    % Solve QP
    options = optimoptions('quadprog','Display','off',Algorithm='active-set');%MAAAAAYBE
    [z_opt,~,exitflag] = quadprog(H, f, [], [], Aeq, beq, lb, ub, [0;0;0;0], options);

    % Extract solution
    if exitflag == 1
        x_ref_opt = z_opt(1:n_x);
        u_ref_opt = z_opt(n_x+1:end);
    else
        %warning('Optimal target selection (Kalman version) infeasible.');
        x_ref_opt = zeros(n_x, 1);
        u_ref_opt = zeros(n_u, 1);
    end
end