function [rho, result] = safe_tune_rho_for_proof(P, K, x_min, x_max, u_lb, u_ub, rho_initial, tol, x_ref)
    if nargin < 7, rho_initial = 50.0; end
    if nargin < 8, tol = 1e-3; end
    if nargin < 9, x_ref = zeros(size(P,1),1); end

    rho = rho_initial;
    while rho > tol
        result = diagonalize_and_check(P, K, rho, x_min, x_max, u_lb, u_ub, x_ref);
        if result.state_ok && result.input_ok
            fprintf('\n✅ Safe rho found: %.4f\n', rho);
            return;
        end
        rho = rho * 0.95;
    end

    warning('❌ No safe rho found.');
end
