function [x_ref_opt, u_ref_opt] = optimal_target_selection_reference_tracking(A, B, C, y_ref, u_lb, u_ub, x_min, x_max)

    n_x = size(A, 1);
    n_u = size(B, 2);

    % Decision variable: z = [x_ref; u_ref]
    % Cost: (1/2) * z' * H * z + f' * z
    H = blkdiag(zeros(n_x), eye(n_u));  % only penalize u_ref
    f = zeros(n_x + n_u, 1);

    % Equality constraints:
    % (I - A)x_ref - Bu_ref = 0   → [I - A, -B] * z = 0
    % Cx_ref = y_ref              → [C, 0] * z = y_ref
    Aeq = [eye(n_x) - A, -B;
           C, zeros(size(C,1), n_u)];
    beq = [zeros(n_x,1); y_ref];

    % Inequality constraints (bounds)
    lb = [x_min; u_lb];
    ub = [x_max; u_ub];

    % Solve QP
    options = optimoptions('quadprog','Display','off');
    [z_opt,~,exitflag] = quadprog(H, f, [], [], Aeq, beq, lb, ub, [], options);

    if exitflag ~= 1
        warning('Optimal target selection infeasible.');
        x_ref_opt = zeros(n_x, 1);
        u_ref_opt = zeros(n_u, 1);
    else
        x_ref_opt = z_opt(1:n_x);
        u_ref_opt = z_opt(n_x+1:end);
    end
end



% function [x_ref_opt, u_ref_opt] = optimal_target_selection_reference_tracking(A, B, C, y_ref, u_lb, u_ub, x_min, x_max)
% 
%     n_x = size(A, 1);
%     n_u = size(B, 2);
% 
%     % Decision variables
%     x_ref = sdpvar(n_x, 1);
%     u_ref = sdpvar(n_u, 1);
% 
%     % Cost function: minimize control effort
%     R_eye = eye(n_u);
%     objective = u_ref' * R_eye * u_ref;
% 
%     % Constraints
%     constraints = [];
%     constraints = [constraints, (eye(n_x) - A) * x_ref == B * u_ref];   % steady-state
%     constraints = [constraints, C * x_ref == y_ref];                    % output matching
%     constraints = [constraints, u_lb <= u_ref <= u_ub];                 % input bounds
%     constraints = [constraints, x_min <= x_ref <= x_max];              % state bounds
% 
%     % Solve
%     options = sdpsettings('solver', 'quadprog', 'verbose', 0);
%     optimize(constraints, objective, options);
% 
%     % Return result
%     x_ref_opt = value(x_ref);
%     u_ref_opt = value(u_ref);
% end
