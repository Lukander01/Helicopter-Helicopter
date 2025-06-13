function [x_bar, u_bar] = solve_mpc_quadprog(A, B, Q, R, P, x0, N, rho, x_ref, u_ref, x_min, x_max, u_lb, u_ub)

    [n_x, n_u] = size(B);
    [T, S] = gen_prediction_matrices(A, B, N);

    % Cost matrices
    Q_bar = kron(eye(N), Q);
    Q_bar_terminal = blkdiag(Q_bar, P);
    R_bar = kron(eye(N), R);

    % Reference trajectories
    x_ref_bar = repmat(x_ref, N+1, 1);
    u_ref_bar = repmat(u_ref, N, 1);

    % Build prediction
    H = S' * Q_bar_terminal * S + R_bar;
    f = S' * Q_bar_terminal * (T * x0 - x_ref_bar) - R_bar * u_ref_bar;
    f = f(:);

    % Enforce symmetry
    H = 0.5 * (H + H');

    % Constraints: input
    G_u = kron(eye(N), [eye(n_u); -eye(n_u)]);
    g_u = repmat([u_ub; -u_lb], N, 1);

    % Constraints: state
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

    % Terminal constraint
    E_N = zeros(n_x, (N+1)*n_x);
    E_N(:, N*n_x+1:(N+1)*n_x) = eye(n_x);
    x_N_shift = E_N * T * x0 - x_ref;

    % Terminal quad constraint: (x_N - x_ref)' * P * (x_N - x_ref) <= rho


    % Combine constraints
    A_qp = [G_u; A_x];
    b_qp = [g_u; b_x];

    % Solve QP
    options = optimoptions('quadprog','Display','none');
    [u_opt,~,exitflag] = quadprog(H, f, A_qp, b_qp, [], [], [], [], [], options);

    if exitflag ~= 1
        disp('MPC problem infeasible');
        u_bar = zeros(N, n_u);
        x_bar = zeros(N+1, n_x);
    else
        u_bar = reshape(u_opt, n_u, N)';
        x_bar = reshape(T * x0 + S * u_opt, n_x, N+1)';
    end
end
