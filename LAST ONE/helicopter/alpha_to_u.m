function u = alpha_to_u(alpha)
if(alpha<0)
    u = 1.485*alpha -0.04221;
else
    u = 3.397*alpha +0.04143;
end
end