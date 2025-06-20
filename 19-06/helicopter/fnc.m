function x = fcn(x,u)

dt = 
x = xu(1:3);
u = xu(4:6);

x_next = nonlin_evolution(x,u);

end
