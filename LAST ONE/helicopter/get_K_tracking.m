function K = get_K_tracking(A_lin,B_lin,Q_lqr,R_lqr)

N=0;
system = ss(A_lin, B_lin, [], []);

[K,S,e]  = lqi(system, Q_lqr, R_lqr, N);
end