function scaling = computeScalingCostQP(H)

tol = 0.1;

if norm(H,'fro') > tol
    
    scaling = norm(H,'fro');
else
    scaling = 1;
end
end