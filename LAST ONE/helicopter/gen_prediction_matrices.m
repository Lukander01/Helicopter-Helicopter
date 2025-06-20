function [T, S] = gen_prediction_matrices(A, B, N)
    

%     n_x = size(A,1);
%     n_u = size(B,2);
% 
%     T = zeros((N)*n_x, n_x);
%     S = zeros((N)*n_x, N*n_u);
% 
%     %Apowers = eye(n_x);
%     for k = 1:N
%         T((k-1)*n_x+1:(k)*n_x, :) = A^k;
%         for j = 0:k-1
%             S((k-1)*n_x+1:(k)*n_x, j*n_u+1:(j+1)*n_u) = A^(k-1-j)*B;
%         end
%     end
% end

    nx = size(A, 1);
    nu = size(B, 2);
    Phi = zeros(N*nx, nx);
    Gamma = zeros(N*nx, N*nu);
    for i = 1:N
        rows = (i-1)*nx+1:i*nx;
        Phi(rows, :) = A^i;
        for j = 1:i
            cols = (j-1)*nu+1:j*nu;
            Gamma(rows, cols) = A^(i-j)*B;
        end
    end
    T = Phi;
    S = Gamma; 
end


