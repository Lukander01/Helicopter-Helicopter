function [A_c, B_c, A_d, B_d, C] = get_satellite_dynamics(dt)
    I_11 = 11.516;
    I_22 = 11.084;
    I_33 = 11.084;
    I_omega = 0.025;

    A_c = [ 0, 0, 0, 0, 0, 1;
            0, 0, 0, 0, 1, 0;
            0, 0, 0, 1, 0, 0;
            0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, 0, 0];

    B_c = [ 0, 0, 0;
            0, 0, 0;
            0, 0, 0;
           -I_omega/(I_11 + I_omega), 0, 0;
            0, -I_omega/(I_22 + I_omega), 0;
            0, 0, -I_omega/(I_33 + I_omega)];

    % Discretization
    AB_c = [A_c, B_c; zeros(3, 6 + 3)];
    expm_ABc = expm(AB_c * dt);
    A_d = expm_ABc(1:6, 1:6);
    B_d = expm_ABc(1:6, 7:end);

    C = eye(6);
end
