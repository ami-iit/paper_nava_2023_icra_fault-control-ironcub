function cost = computeCostQP(H, g, u_star)

% compute the quatratic cost for the selected QP task given the optim. u
cost = (transpose(u_star)* H * u_star)/2 + transpose(u_star) * g;

end
