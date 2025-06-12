function [rho, corners, eigvecs, axes_lengths] = tune_rho_highdim(P, K, u_lb, u_ub, x_min, x_max, rho_initial, tol, x_ref)
    if nargin < 8, tol = 1e-3; end
    if nargin < 7, rho_initial = 50.0; end
    if nargin < 9, x_ref = zeros(size(P,1),1); end

    n = size(P,1);
    rho = rho_initial;
    sign_patterns = dec2bin(0:2^n-1) - '0';
    sign_patterns = 2*sign_patterns - 1;

    while rho > tol
        ellipsoid_matrix = inv(P) * rho;
        [eigvecs, D] = eig(ellipsoid_matrix);
        axes_lengths = sqrt(diag(D));
        corners = (sign_patterns .* axes_lengths') * eigvecs' + x_ref';

        % Check state constraints
        if any(any(corners' < x_min)) || any(any(corners' > x_max))
            rho = rho * 0.98;
            continue;
        end

        % Check input constraints
        u_vals = (K * corners')';
        if any(any(u_vals < u_lb')) || any(any(u_vals > u_ub'))
            rho = rho * 0.98;
            continue;
        end

        fprintf('Tuned rho: %.4f\n', rho);
        return;
    end

    warning('Could not find suitable rho within tolerance.');
end
