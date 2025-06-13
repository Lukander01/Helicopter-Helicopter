clear; clc;

%% Setup
dt = 2.0;
[A_c, B_c, A_d, B_d, C] = get_satellite_dynamics(dt);
n_x = size(A_d,1); n_u = size(B_d,2); n_y = n_x;


addpath(genpath('C:\TUDelft\Q4\YALMIP-master'));
addpath(genpath('C:\TUDelft\Q4\sedumi-master'));


Q = diag([10,10,10, 1,1,1]);
R = 0.1 * eye(3);
[P_inf, ~, K_inf] = dare(A_c, B_c, Q, R);
P = P_inf;

% Constraints
x_min = [-10; -10; -10; -0.3; -0.3; -0.3];
x_max = [ 10;  10;  10;  0.3;  0.3;  0.3];
u_lb = [-0.5; -0.5; -0.5];
u_ub = [ 0.5;  0.5;  0.5];

x0 = [0.1; 2; 0.5; 0; 0; 0];
y_ref_desired = [1; 0; -0.5; 0; 0; 0];
d_true = [0.1; -0.2; 0.5; 0; 0; 0];

gain = 0.1;
L_d = gain * eye(n_y);
d_hat = zeros(n_y,1);


rho = 3.5;
K_inf = -K_inf;
% Automatically tune rho for terminal constraint
[rho, ~] = safe_tune_rho_for_proof(P, K_inf, x_min, x_max, u_lb, u_ub);

N_sim = 100; N = 25;

%% Kalman init
Q_kf = 1e-4 * eye(n_x);
R_kf = 1e-4 * eye(n_y);
x_est = x0;
P_est = 1e-4 * eye(n_x);

x_hist = zeros(N_sim+1, n_x); x_hist(1,:) = x0';
y_hist = zeros(N_sim+1, n_y);
u_hist = zeros(N_sim, n_u);
d_err = zeros(N_sim+1, 1);
y_hist(1,:) = (C * x0 + d_true)';

%% Loop
for t = 1:N_sim
    y_meas = C * x_hist(t,:)' + d_true;

    if t > 1
        [x_est, P_est] = kalman_filter_update(x_est, P_est, y_meas, u_hist(t-1,:)', A_d, B_d, C, Q_kf, R_kf);
    end

    d_hat = zeros(n_y,1);
    d_hat(1:3) = y_meas(1:3) - C(1:3,:) * x_est;

    [x_ref, u_ref] = optimal_target_selectionKALMAN(A_d, B_d, C, R, y_ref_desired, d_hat, u_lb, u_ub, x_min, x_max);

    [x_bar, u_bar] = solve_offset_free_mpc(A_d, B_d, Q, R, P, x_est, N, u_lb, u_ub, x_ref, u_ref, x_min, x_max, rho);

    u_hist(t,:) = u_bar(1,:);
    x_hist(t+1,:) = (A_d * x_hist(t,:)' + B_d * u_hist(t,:)')';
    y_hist(t+1,:) = (C * x_hist(t+1,:)' + d_true)';

    d_err(t) = norm(d_hat);
end

%% Plotting (can be separated)
time_vec = (0:N_sim) * dt;
time_vec_u = (0:N_sim-1) * dt;

figure;
subplot(2,1,1);
stairs(time_vec, x_hist(:,1:3)); hold on;
yline(x_ref(1),'--'), yline(x_min(1),'r--'), yline(x_max(1),'r--');
title('Angular Positions'); ylabel('Angle [rad]'); grid on;

subplot(2,1,2);
stairs(time_vec, x_hist(:,4:6)); hold on;
yline(x_ref(4),'--'), yline(x_min(4),'r--'), yline(x_max(4),'r--');
title('Angular Velocities'); ylabel('Rad/s'); xlabel('Time [s]'); grid on;

figure;
stairs(time_vec_u, u_hist); hold on;
yline(u_lb,'r--'); yline(u_ub,'r--');
title('Control Inputs'); ylabel('Input'); xlabel('Time [s]'); grid on;

figure;
stairs(time_vec, y_hist); hold on;
yline(x_min(1),'r--'); yline(x_max(1),'r--');
title('Measured Outputs (with disturbance)'); grid on;

figure;
stairs(time_vec_u, d_err(1:end-1));
title('Disturbance Estimation Error'); xlabel('Time [s]'); ylabel('||d_{hat}||'); grid on;
