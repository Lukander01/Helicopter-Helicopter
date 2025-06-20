
clc
clear all;
close all;

%alpha_ref = -0.3*t; %30 * (2 * pi) / 360;

% 
% K1 = 0;
% K3 = 0;
% K4 = 0;
% b = 0;
% C = [1, 0, 0; 0, 0, 1];
%omega_ref = 0;

Q = [1,  0, 0; 0, 1, 0; 0, 0, 10];
R = [1, 0; 0, 10]; 

h = 0.01;   % sampling interval
T = 30;
t = 0:h:T;
omega_ref = zeros(length(t),1);
alpha_ref = -sin(2*pi*0.5*t)*0.1-0.3;
alpha_dot_ref = [diff(alpha_ref)/h 0];

timeser_alpha = timeseries(alpha_ref,t);
timeser_dot = timeseries(alpha_dot_ref,t);
for i =1:length(alpha_ref)
    if alpha_ref(i) < 0
        x=alpha_ref(i);
    omega_ref(i) = 6659*x^4 + 1.083*10000 *x^3 + 6343 * x * x +1888*x - 1.157;
%     b = 0.239927543450809;
%     K1 = -10.548800723683954;
%     K3 = 5.498506290874219e-05;
%     K4 = 4.594429729616914;
%     f_u = 272.9 * u * u + 593.4 * u - 8.529;
    else
        x=alpha_ref(i);
    omega_ref(i) = -1.905*100000*x^4 + 1.299*100000*x^3 - 3.225*10000 * x *x + 4233*x + 0.7921;
%     b = 0.025936940902745;
%     K1 = -10.577796958770156;
%     K3 = -2.433475804656021e-05;
%     K4 = 5.379206715771331;
%     f_u = -284.3 * u * u + 609 * u + 7.086;
end
    
end


x_ref = [alpha_ref;alpha_dot_ref;omega_ref'];
timeres_xref = timeseries(x_ref,t);

disturbance = zeros(1,length(t));
disturbance(10/h:12/h) = 0.5;
disturbance(20/h:22/h) = -0.5;
U =  -0*ones(1,length(t)); %disturbance;
timeser = timeseries(U, t);

%u_start = alpha_to_u(alpha_ref);


%%

sim("helicopter_LQR_tracking.slx")

%% savings
% str1 = 'up_disturbance';
% save(['LQR_timeseries\u_LQR_' str1], 'u_actual');
% save(['LQR_timeseries\alpha_LQR_' str1], 'alpha');
% save(['LQR_timeseries\omega_LQR_' str1], 'omega');
% save(['LQR_timeseries\xhat_LQR_' str1], 'x_hat');
%%
% %%
%     figure;
%     hold on;
%     subplot(2,1,1)
%     plot(differenceEKF(1).Data(1, :));
%     grid on;
%     title('Observer error')
%     xlabel('t[s]')
%     ylabel('\alpha [rad]')
%     subplot(2,1,2)
%     plot(differenceEKF(1).Data(2, :));
%     grid on;
%     title('Observer error')
%     xlabel('t[s]')
%     ylabel('\omega [rad/s]')
%     hold off;
% 
% %%
%     figure;
%     subplot(3,1,1)
%     plot(x_hat(1).Data(1, :))
%     hold all
%     plot(alpha(1).Data)
%     grid on;
%     title('x_{hat}')
%     xlabel('t[s]')
%     ylabel('\alpha [rad]')
%     legend('Observer', 'Real')
%     subplot(3,1,2)
%     plot(x_hat(1).Data(2, :))
%     hold on
%     plot((0:10:length(alpha(1).Data)-3),diff(alpha(1).Data(1:10:end))/10/h);
%     grid on;
%     title('x_{hat}')
%     xlabel('t[s]');
%     ylabel('d\alpha/dt [rad/s]')
%     subplot(3,1,3)
%     plot(x_hat(1).Data(3, :))
%     hold all
%     plot(omega(1).Data)
%     grid on;
%     title('x_{hat}')
%     xlabel('t[s]')
%     ylabel('\omega [rad/s]')
%     legend('Observer', 'Real')
%     
%    %%
%     figure;
%     hold on;
%     subplot(2,1,1)
%     plot(alpha(1).Data);
%     grid on;
%     title('Real output');
%     xlabel('t[s]')
%     ylabel('\alpha [rad]')
%     subplot(2,1,2)
%     plot(omega(1).Data);
%     title('Real output');
%     xlabel('t[s]')
%     ylabel('\omega [rad/s]')
%     grid on;
%     hold off;
   %%

%    scaled_data = alpha(1).Data(:) / alpha_ref; 
%     clc
%     info = stepinfo(scaled_data)
%     
% 
% figure
% hold on
% plot(real(p_obs),imag(p_obs), 'x')
% plot(real(p_control),imag(p_control), 'x')
% plot(real(p),imag(p), 'x')
% legend('observe', 'control', 'original')


% calc rise time, ss error
% (logic for push to linearized position)
% :)
