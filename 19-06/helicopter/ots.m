function [x_ref_val, u_ref_val] = ots(A, B, C, R, y_ref, d_hat, u_lb, u_ub, x_min, x_max)
    % Dimensions
    [n_x, ~] = size(A);
    [~, n_u] = size(B);
    
    % Decision vector: z = [x_ref; u_ref]
    n = n_x + n_u;

    %% Cost function: 0.5 * z' * H * z
    H = blkdiag(zeros(n_x), R);  % Only u_ref has cost
    f = zeros(n, 1);

    %% Equality constraints
    % (I - A) * x_ref - B * u_ref == 0
    A_dyn = [(eye(n_x) - A), -B]
    b_dyn = zeros(n_x, 1)

    % C * x_ref + d_hat == y_ref  =>  C * x_ref == y_ref - d_hat
    A_out = [C, zeros(size(C, 1), n_u)]

    b_out = y_ref - [d_hat(1); d_hat(3)]
    

    Aeq = [A_dyn; A_out];
    beq = [b_dyn; b_out];
    
    A = [A_dyn, A_out];
    b = [10e-4*[1;1;1]];
    %% Inequality constraints (lower and upper bounds)
    % x_min <= x_ref <= x_max
    % u_lb  <= u_ref <= u_ub
    lb = [x_min; u_lb];
    ub = [x_max; u_ub];

    %% Solve QP
    options = optimoptions('quadprog', 'Display', 'final-detailed');
    [z_opt, ~, exitflag] = quadprog(H, f, A, b, [], [],lb, ub, [], options);

    
    %% Check result
    if exitflag ~= 1
        disp('MPC problem infeasible or solver failed');
        x_ref_val = [];
        u_ref_val = [];
        return;
    end

    %% Extract x_ref and u_ref
    x_ref_val = z_opt(1:n_x);
    u_ref_val = z_opt(n_x+1:end);
end