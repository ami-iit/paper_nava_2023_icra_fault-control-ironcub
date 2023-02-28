function cost = computeCostFunction(u, KinDynModel, Config)

    % COMPUTECOSTFUNCTION computes the cost function for jet failure optimization.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jun. 2022
    %

    % update robot state for all costs
    [basePos, baseRot, jointPos, ~] = wbc.vectorDemux(u,[3; 3; Config.ndof; Config.turbinesData.njets]);

    w_R_b = wbc.rotationFromRollPitchYaw(baseRot);
    w_H_b = [w_R_b, basePos;
             0   0   0  1];

    % set the current model state
    iDynTreeWrappers.setRobotState(KinDynModel, w_H_b, jointPos, zeros(6,1), zeros(Config.ndof,1), Config.gravityAcc);

    %---------------------------------------------------------------------%
    
    % cost on det(controllability matrix) > 0
    cost_controllability = Config.opti.WeightControllability*computeControllabilityCost(u, KinDynModel, Config);

    %---------------------------------------------------------------------%
    % sum up all costs
    cost = cost_controllability;
end