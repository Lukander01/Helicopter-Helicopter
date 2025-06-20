function u_bar  = MPC_step(x_est, y_ref_desired, y_meas, A_lin, B_lin, P)
    n_y = 2;
    n_x = 3;
    n_u =1;

Q_MPC = [5000, 0, 0; 0, 20, 0; 0, 0, 0.1];
R_MPC = [80];
    
    C = [1, 0, 0; 0, 0, 1];
    
    % Constraints
    x_min = [-pi/4; -1000; -335];
    x_max = [ pi/4;  1000; 337];
    u_lb = [-1];
    u_ub = [1];
    
    rho = 1000;
    N = 5; % 3sec

    d_hat = zeros(n_y,1);
    d_hat(1:2) = y_meas(1:2) - C(1:2,:) * x_est;

    [x_ref, u_ref] = optimal_target_selectionKALMAN(A_lin, B_lin, C, [1], y_ref_desired, d_hat, u_lb, u_ub, x_min, x_max);

    [x_bar, u_bar] = solve_offset_free_mpc(A_lin, B_lin, Q_MPC, R_MPC, P, x_est, N, u_lb, u_ub, x_ref, u_ref, x_min, x_max, rho);


end