function u_bar  = MPC_step(x_est, y_ref_desired, y_meas, A_lin, B_lin, C, Q_MPC, R_MPC, P, N, u_lb, u_ub, x_min, x_max, rho)
    n_y = 2;
    n_x = 3;
    n_u =1;

    d_hat = zeros(n_y,1);
    d_hat(1:2) = y_meas(1:2) - C(1:2,:) * x_est;

    [x_ref, u_ref] = optimal_target_selectionKALMAN(A_lin, B_lin, C, [1], y_ref_desired, d_hat, u_lb, u_ub, x_min, x_max);

    [x_bar, u_bar] = solve_offset_free_mpc(A_lin, B_lin, Q_MPC, R_MPC, P, x_est, N, u_lb, u_ub, x_ref, u_ref, x_min, x_max, rho);

%     x_hist = zeros(N_sim+1, n_x); x_hist(1,:) = x0';
%     y_hist = zeros(N_sim+1, n_y);
%     u_hist = zeros(N_sim, n_u);
%     d_err = zeros(N_sim+1, 1);
%     hist_arr(3,t,:) = u_bar(1,:);
%     hist_arr(1,t+1,:) = (A_d * x_hist(t,:)' + B_d * u_hist(t,:)')';
%     hist_arr(2,t+1,:) = (C * x_hist(t+1,:)' + d_true)';
%     hist_arr(4,t) = norm(d_hat);

end

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

function [x_bar, u_bar] = solve_offset_free_mpc(A, B, Q, R, P, x0, N, u_lb, u_ub, x_ref, u_ref, x_min, x_max, rho)

    [T, S] = gen_prediction_matrices(A, B, N);
    n_x = size(A,1);
    n_u = size(B,2);

    % Cost matrices
    Q_bar = kron(eye(N), Q);
    Q_bar = blkdiag(Q_bar, P);
    R_bar = kron(eye(N), R);

    x_ref_bar = repmat(x_ref, N+1, 1);
    u_ref_bar = repmat(u_ref, N, 1);

    H = S' * Q_bar * S + R_bar;
    f = S' * Q_bar * (T * x0 - x_ref_bar) - R_bar * u_ref_bar;
    H = 0.5 * (H + H');  % Symmetry

    % Input constraints
    G_u = kron(eye(N), [eye(n_u); -eye(n_u)]);
    g_u = repmat([u_ub; -u_lb], N, 1);

    % State constraints
    A_x = [];
    b_x = [];
    for k = 1:N
        E_k = zeros(n_x, (N+1)*n_x);
        E_k(:, k*n_x+1:(k+1)*n_x) = eye(n_x);
        A_x = [A_x; E_k * S];
        b_x = [b_x; x_max - E_k * T * x0];
        A_x = [A_x; -E_k * S];
        b_x = [b_x; -(x_min - E_k * T * x0)];
    end

    % Combine constraints
    A_qp = [G_u; A_x];
    b_qp = [g_u; b_x];

    % Solve QP
    options = optimoptions('quadprog','Display','off');
    [u_opt, ~, exitflag] = quadprog(H, f, A_qp, b_qp, [], [], [], [], [], options);

    if exitflag ~= 1
        warning('QP infeasible');
        u_bar = zeros(N, n_u);
        x_bar = zeros(N+1, n_x);
    else
        u_bar = reshape(u_opt, n_u, N)';
        x_bar = reshape(T * x0 + S * u_opt, n_x, N+1)';
    end
end


function [T, S] = gen_prediction_matrices(A, B, N)
    [n_x, n_u] = size(B);
    T = zeros((N+1)*n_x, n_x);
    S = zeros((N+1)*n_x, N*n_u);

    Apowers = eye(n_x);
    for k = 0:N
        T(k*n_x+1:(k+1)*n_x, :) = Apowers;
        for j = 0:k-1
            S(k*n_x+1:(k+1)*n_x, j*n_u+1:(j+1)*n_u) = Apowers * B;
            Apowers = A * Apowers;
        end
    end
end

