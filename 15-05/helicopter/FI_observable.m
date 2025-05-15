K = 0;
K_2 = 0;
K_3 = 0;
b = 0;
Tau = 0;
% init_params
x1 = 0;
x2 = 0;
x3 = 500;
params = [];

A_lin = [0, 1, 0; K*cos(x1), -b, -2*K_3*x3; 0, 0, -1/Tau];

 csys = compreal(A_lin,"o");
 csys