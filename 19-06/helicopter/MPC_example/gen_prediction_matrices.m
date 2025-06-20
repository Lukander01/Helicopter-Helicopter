function [T, S] = gen_prediction_matrices(A, B, N)
    [n_x, n_u] = size(B);
    T = zeros((N+1)*n_x, n_x);
    S = zeros((N+1)*n_x, N*n_u);

    Apowers = eye(n_x);
    for k = 0:N
        T(k*n_x+1:(k+1)*n_x, :) = Apowers;
        for j = 0:k-1
            S(k*n_x+1:(k+1)*n_x, j*n_u+1:(j+1)*n_u) = Apowers * B;
            Apowers = A * Apowers;
        end
    end
end
