function result = diagonalize_and_check(P, K, rho, x_min, x_max, u_lb, u_ub, x_ref)
    if nargin < 8, x_ref = zeros(size(P,1),1); end

    [S, D] = eig(P);
    n = size(P,1);

    % Generate vertices in y-space (rhombus)
    y_vertices = [];
    for i = 1:n
        y_vec = zeros(n,1);
        y_vec(i) = 2 * sqrt(rho / D(i,i));
        y_vertices = [y_vertices, y_vec, -y_vec];
    end
    y_vertices = y_vertices';  % Each row is a vertex

    % Transform to x-space and shift
    x_vertices = (S * y_vertices')' + x_ref';

    x_state_ok = all((x_vertices >= x_min') & (x_vertices <= x_max'), 2);
    u_vertices = (K * x_vertices')';
    u_input_ok = all((u_vertices >= u_lb') & (u_vertices <= u_ub'), 2);

    result = struct();
    result.S = S;
    result.D = D;
    result.y_vertices = y_vertices;
    result.x_vertices = x_vertices;
    result.u_vertices = u_vertices;
    result.state_ok = all(x_state_ok);
    result.input_ok = all(u_input_ok);

    fprintf("State constraints satisfied: %d\n", result.state_ok);
    fprintf("Input constraints satisfied: %d\n", result.input_ok);
end
