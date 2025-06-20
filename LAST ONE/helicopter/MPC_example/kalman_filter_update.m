function [x_new, P_new] = kalman_filter_update(x_est, P_est, y_meas, u_prev, A, B, C, Q_kf, R_kf)
    % Prediction
    x_pred = A * x_est + B * u_prev;
    P_pred = A * P_est * A' + Q_kf;

    % Correction
    K = P_pred * C' / (C * P_pred * C' + R_kf);
    x_new = x_pred + K * (y_meas - C * x_pred);
    P_new = (eye(size(K,1)) - K * C) * P_pred;
end
