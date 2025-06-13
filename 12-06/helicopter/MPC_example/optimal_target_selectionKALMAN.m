function [x_ref_opt, u_ref_opt] = optimal_target_selectionKALMAN(A, B, C, R, y_ref, d_hat, u_lb, u_ub, x_min, x_max)
    n_x = size(A, 1);
    n_u = size(B, 2);

    % Decision variable: z = [x_ref; u_ref]
    H = blkdiag(zeros(n_x), R);  % Only penalize u_ref
    f = zeros(n_x + n_u, 1);     % Linear term in cost


    % (I - A)x_ref - B u_ref = 0  → [I - A, -B] * z = 0
    % C x_ref + d_hat = y_ref    → [C, 0] * z = y_ref - d_hat
    Aeq = [eye(n_x) - A, -B;
           C, zeros(size(C,1), n_u)];
    beq = [zeros(n_x,1);
           y_ref - d_hat];

    % Bounds
    lb = [x_min; u_lb];
    ub = [x_max; u_ub];

    % Solve QP
    options = optimoptions('quadprog','Display','off');
    [z_opt,~,exitflag] = quadprog(H, f, [], [], Aeq, beq, lb, ub, [], options);

    % Extract solution
    if exitflag == 1
        x_ref_opt = z_opt(1:n_x);
        u_ref_opt = z_opt(n_x+1:end);
    else
        warning('Optimal target selection (Kalman version) infeasible.');
        x_ref_opt = zeros(n_x, 1);
        u_ref_opt = zeros(n_u, 1);
    end
end




% function [x_ref_opt, u_ref_opt] = optimal_target_selectionKALMAN(A, B, C, R, y_ref, d_hat, u_lb, u_ub, x_min, x_max)
%     n_x = size(A,1);
%     n_u = size(B,2);
% 
%     x_ref = sdpvar(n_x, 1);
%     u_ref = sdpvar(n_u, 1);
% 
%     constraints = [];
%     constraints = [constraints, (eye(n_x) - A) * x_ref == B * u_ref];
%     constraints = [constraints, C * x_ref + d_hat == y_ref];
%     constraints = [constraints, u_lb <= u_ref <= u_ub];
%     constraints = [constraints, x_min <= x_ref <= x_max];
% 
%     objective = u_ref' * eye(n_u) * u_ref;
% 
%     opts = sdpsettings('solver','quadprog','verbose',0);
%     optimize(constraints, objective, opts);
% 
%     x_ref_opt = value(x_ref);
%     u_ref_opt = value(u_ref);
% end
