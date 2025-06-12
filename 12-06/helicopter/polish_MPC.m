clear all;
close all;
clc

addpath(genpath('C:\Users\lcaric\helicopter\yalmip\YALMIP-master'));
% addpath(genpath('C:\TUDelft\Q4\sedumi-master'));
addpath(genpath('MPC_exaple'));

alpha_ref = -0.25; %30 * (2 * pi) / 360;

x = alpha_ref;

alpha_dot_ref = 0;


K1 = 0;
K3 = 0;
K4 = 0;
b = 0;
C = [1, 0, 0; 0, 0, 1];
omega_ref = 0;

% choose alpha -> calc omega
if x < 0
    omega_ref = 6659*x^4 + 1.083*10000 *x^3 + 6343 * x * x +1888*x - 1.157;
    b = 0.239927543450809;
    K1 = -10.548800723683954;
    K3 = 5.498506290874219e-05;
    K4 = 4.594429729616914;
%     f_u = 272.9 * u * u + 593.4 * u - 8.529;
else
    omega_ref = -1.905*100000*x^4 + 1.299*100000*x^3 - 3.225*10000 * x *x + 4233*x + 0.7921;
    b = 0.025936940902745;
    K1 = -10.577796958770156;
    K3 = -2.433475804656021e-05;
    K4 = 5.379206715771331;
%     f_u = -284.3 * u * u + 609 * u + 7.086;
end

h = 0.01;   % sampling interval
T = 30;
t = 0:h:T;
U = -0*ones(1,length(t));
timeser = timeseries(U, t);

[A_lin,B_lin, C, L, p_obs] = observer(alpha_ref);

n_x = size(A_lin,1);
n_u = size(B_lin,2); 
n_y = 2;


Q_kalman = [1,  0, 0; 0, 1, 0; 0, 0, 10];
R_kalman = [1 0; 0 1];

Q_MPC = [1000, 0, 0; 0, 20, 0; 0, 0, 0.1];
R_MPC = [80];

[P_inf, ~, K_inf] = dare(A_lin, B_lin, Q_MPC, R_MPC);
P = P_inf;

% Constraints
x_min = [-pi/4; -1000; -335];
x_max = [ pi/4;  1000; 337];
u_lb = [-1];
u_ub = [1];

x0 = [0; 0; 0];

y_ref_desired = [alpha_ref; omega_ref];

x_ref = [alpha_ref; alpha_dot_ref ; omega_ref];

d_true = [0; 0];

% gain = 0.1;
% L_d = gain * eye(n_y);

d_hat = zeros(n_y,1);

rho = 100;
K_inf = -K_inf;
% Automatically tune rho for terminal constraint
%[rho, ~] = safe_tune_rho_for_proof(P, K_inf, x_min, x_max, u_lb, u_ub);

t_mpc = 0.2;
N_sim = T/t_mpc;

N = 25;

%% Kalman init

%Q_kf = 1e-4 * eye(n_x);
%R_kf = 1e-4 * eye(n_y);
x_est = x0;
%P_est = 1e-4 * eye(n_x);


x_hist = zeros(N_sim+1, n_x); x_hist(1,:) = x0';
y_hist = zeros(N_sim+1, n_y);
u_hist = zeros(N_sim, n_u);
d_err = zeros(N_sim+1, 1);

hist_arr= {x_hist,y_hist,u_hist,d_err};
y_hist(1,:) = (C * x0 + d_true);

sys = ss(A_lin, B_lin, C, []);
Nc= 50;
Np = 500;
mpcobj = mpc(sys, h, Np, Nc);

mpcobj.Weights.ManipulatedVariables = R_MPC;      
% mpcobj1.Weights.ManipulatedVariablesRate = [0.01];
mpcobj.Weights.OutputVariables = [Q_MPC(1,1), Q_MPC(3,3)];


% mpcobj.Weight.ManipulatedVariablesRate = 0.0;
mpcobj.MV(1).Min = -1;
mpcobj.MV(1).Max = 1;


mpcobj.OV(1).Min = -pi/4;
mpcobj.OV(1).Max = pi/4;
mpcobj1.OV(2).Min = -335;
mpcobj1.OV(2).Max = 337;

% [Kest, L] = kalman(sys_d, Q, R);
setEstimator(mpcobj, 'custom');
% setmpcsignals(mpcobj1, 'MV', 1, 'MO', 1:4);

% mpcobj.Model.Nominal.X = [0; 0; 0; 0];
% mpcobj.Model.Nominal.U = 0;
% mpcobj.Model.Nominal.Y = [0; 0; 0; 0];
state = mpcstate(mpcobj);

%% Loop
% assignin("base", "u_matlab", [now, 0]);
% 
% 
% simFuture = parfeval(pool, @sim, 1, "helicopter_MPC_german")
% simOut = sim("helicopter_MPC_german");
% 
% while strcmp(get_param('helicopter_MPC_german', 'SimulationStatus'), 'running')
% 
%     y = get(simOut.logsout, 'measuremnt').Values.Data(end);
%     x = get(simOut.logsout, 'x_hat_MPC').Values.Data(end);
%     y_meas = y;
%     x_est = x;
%     u_bar  = MPC_step(x_est, y_ref_desired, y_meas, A_lin, B_lin, C, Q_MPC, R_MPC, P, N, u_lb, u_ub, x_min, x_max, rho);
%     u_cmd = [now, u_bar];
%     assignin("base", "u_matlab", u_cmd);
% 
%     pause(0.2);
% end

sim("helicopter_MPC_german.slx");



% for t = 1:N_sim:
%     y_meas = C * x_hist(t,:)' + d_true;
% 
%     if t > 1
%         [x_est, P_est] = kalman_filter_update(x_est, P_est, y_meas, u_hist(t-1,:)', A_d, B_d, C, Q_kf, R_kf);
%     end
% 
%     d_hat = zeros(n_y,1);
%     d_hat(1:2) = y_meas(1:2) - C(1:2,:) * x_est;
% 
%     [x_ref, u_ref] = optimal_target_selectionKALMAN(A_d, B_d, C, R, y_ref_desired, d_hat, u_lb, u_ub, x_min, x_max);
% 
%     [x_bar, u_bar] = solve_offset_free_mpc(A_d, B_d, Q, R, P, x_est, N, u_lb, u_ub, x_ref, u_ref, x_min, x_max, rho);
% 
%     u_hist(t,:) = u_bar(1,:);
%     x_hist(t+1,:) = (A_d * x_hist(t,:)' + B_d * u_hist(t,:)')';
%     y_hist(t+1,:) = (C * x_hist(t+1,:)' + d_true)';
% 
%     d_err(t) = norm(d_hat);
% end

%% Plotting (can be separated)

dt = h;
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
