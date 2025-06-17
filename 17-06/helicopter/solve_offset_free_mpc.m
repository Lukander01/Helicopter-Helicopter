
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
    H = 0.5 * (H + H');


    % Input constraints
    G_u = kron(eye(N), [eye(n_u); -eye(n_u)]);
    g_u = repmat([u_ub; -u_lb], N, 1);

    % State constraints
    A_x = zeros(2*N*n_x, N*n_u);
    b_x = zeros(2*N*n_x, 1);
    for k = 1:N
        E_k = zeros(n_x, (N+1)*n_x);
        E_k(:, k*n_x+1:(k+1)*n_x) = eye(n_x);

        row_start = (k-1)*2*n_x+1;
        row_mid = row_start + n_x;
        row_end = row_mid + n_x -1;

        A_x(row_start:row_start+n_x-1,:) = E_k * S;
        A_x(row_mid:row_end,:) = -E_k * S;
        b_x(row_start:row_start+n_x-1) = x_max - E_k * T * x0;
        b_x(row_mid:row_end) = -(x_min - E_k * T * x0);

%         A_x((2*k-1)*n_x+1:2*k*n_x, :)     =  E_k * S;
%         A_x((2*k)*n_x+1:(2*k+1)*n_x, :)   = -E_k * S;
%         b_x((2*k-1)*n_x+1:2*k*n_x, 1)     =  x_max - E_k * T * x0;
%         b_x((2*k)*n_x+1:(2*k+1)*n_x, 1)   = -(x_min - E_k * T * x0);
    end

    % Combine constraints
    A_qp = [G_u; A_x];
    b_qp = [g_u; b_x];
    
        options = optimoptions('quadprog','Display','off', Algorithm='active-set');
        [u_opt, ~, exitflag] = quadprog(H, f, A_qp, b_qp, [], [], [], [], zeros(1,N), options);

    %[u_opt, ~, exitflag] = quadprog(H, f, A_qp, b_qp, [], [], [], []);

    if exitflag ~= 1
        u_bar = zeros(N, n_u);
        x_bar = zeros(N+1, n_x);
    else
        u_bar = reshape(u_opt, n_u, N)';
        x_bar = reshape(T * x0 + S * u_opt', n_x, N+1)';
    end
end











% function [x_bar, u_bar] = solve_offset_free_mpc(A, B, Q, R, P, x0, N, u_lb, u_ub, x_ref, u_ref, x_min, x_max, rho)
% 
%     [T, S] = gen_prediction_matrices(A, B, N);
%     n_x = size(A,1);
%     n_u = size(B,2);
% 
%     % Cost matrices
%     Q_bar = kron(eye(N), Q);
%     Q_bar = blkdiag(Q_bar, P);
%     R_bar = kron(eye(N), R);
% 
%     x_ref_bar = repmat(x_ref, N+1, 1);
%     u_ref_bar = repmat(u_ref, N, 1);
% 
%     H = S' * Q_bar * S + R_bar;
%     f = S' * Q_bar * (T * x0 - x_ref_bar) - R_bar * u_ref_bar;
%     H = 0.5 * (H + H');  % Symmetry
% 
%     % Input constraints
%     G_u = kron(eye(N), [eye(n_u); -eye(n_u)]);
%     g_u = repmat([u_ub; -u_lb], N, 1);
% 
%     % State constraints
%     A_x = zeros(2*3*N,30);
%     b_x = zeros(2*3*N,1);
%     
%     for k = 1:6:6*N
%         E_k = zeros(n_x, (N+1)*n_x);
%         E_k(:, k*n_x+1:(k+1)*n_x) = eye(n_x);
%         A_x(k:k+2,:) = E_k * S;
%         b_x(k:k+2,:) = x_max - E_k * T * x0;
%         A_x(k+3:k+5,:) = -E_k * S;
%         b_x(k+3:k+5,:) = -(x_min - E_k * T * x0);
%     end
% 
%     % Combine constraints
%     A_qp = [G_u; A_x];
%     b_qp = [g_u; b_x];
%  %aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
%     % Solve QP
%     options = optimoptions('quadprog','Display','off', Algorithm='active-set');
%     [u_opt, ~, exitflag] = quadprog(H, f, A_qp, b_qp, [], [], [], [], zeros(1,30), options);
% 
%     if exitflag ~= 1
%        % warning('QP infeasible');
%         u_bar = zeros(N, n_u);
%         x_bar = zeros(N+1, n_x);
%     else
%         u_bar = reshape(u_opt, n_u, N)';
%         x_bar = reshape(T * x0 + S * (u_opt), n_x, N+1)';
%     end
% end
