function [H_symmetry, g_symmetry] = computeHessianGradientThrustSymmetryTask(njets, ndof, Config)

% WARNING: hard-coded thrusts configuration: [lArm, rArm, lChest, rChest]
if Config.tasksQP.symLeftAndRightThrusts
    
    Ht_symmetry = [1    0   -1    0
                   0    1    0   -1
                  -1    0    1    0
                   0   -1    0    1];
    
elseif Config.tasksQP.symArmsAndChestThrusts
    
    Ht_symmetry = [1   -1    0    0
                  -1    1    0    0
                   0    0    1   -1
                   0    0   -1    1];
    
elseif Config.tasksQP.symChestThrusts
    
    Ht_symmetry = [0    0    0    0
                   0    0    0    0
                   0    0    1   -1
                   0    0   -1    1];
    
elseif Config.tasksQP.symArmsThrusts
    
    Ht_symmetry = [1   -1    0    0
                  -1    1    0    0
                   0    0    0    0
                   0    0    0    0];
    
elseif Config.tasksQP.symAllThrusts
    
    Ht_symmetry = [1   -1    0    0
                  -1    2   -1    0
                   0   -1    2   -1
                   0    0   -1    1];
else
    Ht_symmetry = zeros(4);
end

H_symmetry      = blkdiag(Ht_symmetry, zeros(ndof));
g_symmetry      = zeros(njets + ndof, 1);

end