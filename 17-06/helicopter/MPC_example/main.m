clear; clc;

%% Parameters
dt = 1.0;
N = 50;         % Horizon
N_sim = 100;    % Simulation steps

% Get system dynamics
[A_c, B_c, A_d, B_d, C] = get_satellite_dynamics(dt);

% State/input dims
n_x = size(A_c, 1);
n_u = size(B_c, 2);

% Cost weights
Q = diag([10, 10, 10, 1, 1, 1]);
R = 0.1 * eye(3);
[P, ~, K_inf] = dare(A_c, B_c, Q, R);
K_inf = -K_inf; % for feedback

%% Constraints
x_min = [-5; -5; -5; -0.3; -0.3; -0.3];
x_max = [ 5;  5;  5;  0.3;  0.3;  0.3];
u_lb  = [-10; -10; -10];
u_ub  = [ 10;  10;  10];

%% Initial and reference states
x0 = [2; 5; 4; 0; 0; 0];
y_ref = [0; -4; 3; 0; 0; 0];

%% Compute steady-state target
[x_ref, u_ref] = optimal_target_selection_reference_tracking(A_d, B_d, C, y_ref, u_lb, u_ub, x_min, x_max);

%% Terminal constraint tuning (rho)
rho = 3.5;
K_inf = -K_inf;
% Automatically tune rho for terminal constraint
[rho, ~] = safe_tune_rho_for_proof(P, K_inf, x_min, x_max, u_lb, u_ub, 50, 10^-3, y_ref);

%% Run MPC simulation
[x_hist, u_hist] = run_mpc_simulation(A_d, B_d, Q, R, P, ...
    x0, N_sim, N, rho, x_ref, u_ref, x_min, x_max, u_lb, u_ub);

time_vec = (0:N_sim) * dt;
time_vec_u = (0:N_sim-1) * dt;

%% Plot Angular Position
figure;
subplot(2,1,1)
for i = 1:3
    stairs(time_vec, x_hist(:,i), 'LineWidth', 1.0); hold on;
    yline(x_min(i), 'r--');
    yline(x_max(i), 'r--');
end
title('Angular Position');
ylabel('Angle [rad]');
xlabel('Time [s]');
legend('\theta_1','\theta_2','\theta_3');
grid on;

%% Plot Angular Velocity
subplot(2,1,2)
for i = 4:6
    stairs(time_vec, x_hist(:,i), 'LineWidth', 1.0); hold on;
    yline(x_min(i), 'r--');
    yline(x_max(i), 'r--');
end
title('Angular Velocity');
ylabel('\omega [rad/s]');
xlabel('Time [s]');
legend('\omega_1','\omega_2','\omega_3');
grid on;

%% Plot Control Inputs
figure;
for i = 1:n_u
    stairs(time_vec_u, u_hist(:,i), 'LineWidth', 1.0); hold on;
    yline(u_lb(i), 'r--');
    yline(u_ub(i), 'r--');
end
title('Control Inputs');
ylabel('Torque [Nm]');
xlabel('Time [s]');
legend('u_1','u_2','u_3');
grid on;
