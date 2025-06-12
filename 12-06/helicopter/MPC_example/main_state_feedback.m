% Main MPC Output Feedback Script (MATLAB)

clear; clc;

% Load system matrices
[A_c, B_c, A_d, B_d, C] = get_satellite_dynamics(2.0);


addpath(genpath('C:\TUDelft\Q4\YALMIP-master'));
addpath(genpath('C:\TUDelft\Q4\sedumi-master'));

dim_x = size(A_c, 1);
dim_u = size(B_c, 2);
dim_y = dim_x;

% Disturbance matrices
Bd = zeros(6);
Bd(1,1) = 0.1;
Bd(2,2) = 0.1;
Bd(3,3) = 0.1;

Cd = zeros(6);
Cd(1,1) = 1;
Cd(2,2) = 1;
Cd(3,3) = 1;

% Cost
Q = diag([10, 10, 10, 1, 1, 1]);
R = 0.1 * eye(3);

% Constraints
u_lb = [-0.5; -0.5; -0.5];
u_ub = [0.5; 0.5; 0.5];
x_min = [-10; -10; -10; -0.3; -0.3; -0.3];
x_max = [10; 10; 10; 0.3; 0.3; 0.3];

% Disturbance (true)
d_true = [0.1; -0.2; 0.5; 0.1; -0.1; 0.1];

% Observer gain
gain1 = 0.5;
gain2 = 0.5;
L1 = gain1 * eye(dim_y);
L2 = gain2 * eye(dim_y);

% Reference
x0 = [0.1; 2; 0.5; 0; 0; 0];
y_ref_desired = [1.0; 0.0; -0.5; 0.0; 0.0; 0.0];

[P, ~, K_inf] = dare(A_c, B_c, Q, R);

%rho = 3.5;
K_inf = -K_inf;
% Automatically tune rho for terminal constraint
[rho, ~] = safe_tune_rho_for_proof(P, K_inf, x_min, x_max, u_lb, u_ub, 50, 10^-3, y_ref_desired);


% Simulation parameters
N_sim = 100;
N = 35;

x_aug_hat = zeros(2 * dim_x, 1);
x_hist = zeros(dim_x, N_sim + 1);
u_hist = zeros(dim_u, N_sim);
y_hist = zeros(dim_y, N_sim + 1);
d_err = zeros(1, N_sim + 1);
output_err = zeros(1, N_sim + 1);

x_hist(:,1) = x0 + Bd * d_true;
y_hist(:,1) = C * x0 + Cd * d_true;
x_aug_hat(1:dim_x) = x_hist(:,1);

for t = 1:N_sim
    disp("Iteration " + t);
    
    y_meas = C * x_hist(:,t) + Cd * d_true;
    
    x_hat = x_aug_hat(1:dim_x);
    d_hat = x_aug_hat(dim_x+1:end);

    y_tilde = y_meas - (C * x_hat + Cd * d_hat);
    
    if t > 1
        u_prev = u_hist(:, t-1);
    else
        u_prev = zeros(dim_u,1);
    end

    x_aug_hat(1:dim_x) = A_d * x_hat + B_d * u_prev + Bd * d_hat + L1 * y_tilde;
    x_aug_hat(dim_x+1:end) = d_hat + L2 * y_tilde;
    
    x_hat = x_aug_hat(1:dim_x);
    d_hat = x_aug_hat(dim_x+1:end);

    [x_ref, u_ref] = optimal_target_selection_output_feedback(A_d, B_d, Bd, C, Cd, R, y_ref_desired, d_hat, u_lb, u_ub, x_min, x_max);
    
    [x_bar, u_bar] = solve_offset_free_mpc_output_feedback(A_d, B_d, Bd, Q, R, P, x_hat, N, u_lb, u_ub, x_ref, u_ref, x_min, x_max, rho);

    u0 = u_bar(:,1);
    u_hist(:,t) = u0;

    x_hist(:,t+1) = A_d * x_hist(:,t) + Bd * d_true + B_d * u0;
    y_hist(:,t+1) = C * x_hist(:,t+1) + Cd * d_true;

    d_err(t) = norm(Bd * d_hat - Bd * d_true);
    y_est = C * x_hat + Cd * d_hat;
    output_err(t) = norm(y_est - y_meas);
end

% --- PLOTS ---
figure;
subplot(2,1,1);
for i = 1:3
    stairs(0:N_sim, x_hist(i,:), 'DisplayName', ['\theta_', num2str(i)]);
    hold on;
    yline(x_ref(i), '--');
end
ylabel('Angle [rad]');
title('Angular Positions');
legend(); grid on;

subplot(2,1,2);
for i = 4:6
    stairs(0:N_sim, x_hist(i,:), 'DisplayName', ['\omega_', num2str(i-3)]);
    hold on;
    yline(x_ref(i), '--');
end
xlabel('Time step');
ylabel('Angular Velocity [rad/s]');
title('Angular Velocities');
legend(); grid on;

figure;
for i = 1:dim_u
    stairs(0:N_sim-1, u_hist(i,:), 'DisplayName', ['u_', num2str(i)]);
    hold on;
end
xlabel('Time step'); ylabel('Control Input'); title('Control Inputs'); legend(); grid on;

figure;
for i = 1:dim_y
    stairs(0:N_sim, y_hist(i,:), 'DisplayName', ['y_', num2str(i)]);
    hold on;
end
xlabel('Time step'); ylabel('Output'); title('Measured Outputs'); legend(); grid on;

figure;
stairs(0:N_sim-1, d_err(1:end-1));
xlabel('Time step'); ylabel('||Bd(\hat{d} - d)||'); title('State Disturbance Estimation Error'); grid on;

figure;
stairs(0:N_sim-1, output_err(1:end-1));
xlabel('Time step'); ylabel('||y_{est} - y||'); title('Output Estimation Error'); grid on;
