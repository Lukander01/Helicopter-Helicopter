function [x_hist, u_hist] = run_mpc_simulation(A, B, Q, R, P, ...
    x0, N_sim, N, rho, x_ref, u_ref, x_min, x_max, u_lb, u_ub)

    n_x = size(A,1);
    n_u = size(B,2);

    x_hist = zeros(N_sim+1, n_x);
    u_hist = zeros(N_sim, n_u);
    x_hist(1,:) = x0';

    for t = 1:N_sim
        [x_bar, u_bar] = solve_mpc_quadprog(A, B, Q, R, P, ...
            x_hist(t,:)', N, rho, x_ref, u_ref, x_min, x_max, u_lb, u_ub);

        u0 = u_bar(1,:)';  % first control input
        u_hist(t,:) = u0';

        x_next = A * x_hist(t,:)' + B * u0;
        x_hist(t+1,:) = x_next';
    end
end
