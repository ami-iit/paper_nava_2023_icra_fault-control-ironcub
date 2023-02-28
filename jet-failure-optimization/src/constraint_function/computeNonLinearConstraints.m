function [c, ceq, grad_c, grad_ceq] = computeNonLinearConstraints(u, KinDynModel, Config)

    % COMPUTENONLINEARCONSTRAINTS computes the nonlinear constraints for
    %                             jet failure optimization.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jun. 2022
    %

    % update robot state for all constraints
    [basePos, baseRot, jointPos, ~] = wbc.vectorDemux(u,[3; 3; Config.ndof; Config.turbinesData.njets]);

    w_R_b = wbc.rotationFromRollPitchYaw(baseRot);
    w_H_b = [w_R_b, basePos;
             0   0   0  1];

    % set the current model state
    iDynTreeWrappers.setRobotState(KinDynModel, w_H_b, jointPos, zeros(6,1), zeros(Config.ndof,1), Config.gravityAcc);

    %---------------------------------------------------------------------%
    % nonlinear equality constraints
    ceq      = [];
    grad_ceq = [];

    % nonlinear inequality constraints

    % square norm of momentum derivative < tolerance
    [LDot, grad_LDot] = computeMomentumDerivativeAndGradient(u, KinDynModel, Config);

    c_momentum      = transpose(LDot)*LDot - Config.opti.momentumNormTol;
    grad_c_momentum = 2*transpose(LDot)*grad_LDot;

    % add self-collision detection constraint
    [c_collision, grad_c_collision] = computeSelfCollisionsAndGradient(u, KinDynModel, Config);

    % add collision with jet cones constraints
    [c_collisionCones, grad_c_collisionCones] = computeConesCollisionsAndGradient(u, KinDynModel, Config);

    %---------------------------------------------------------------------%
    % stack all constraints
    if Config.FAILURE_TYPE == 1 && ~Config.RERUN_WITH_BETTER_INIT_COND
    
        c      = [c_momentum; c_collision];
        grad_c = transpose([grad_c_momentum; grad_c_collision]);
    else
        c      = [c_momentum; c_collision; c_collisionCones];
        grad_c = transpose([grad_c_momentum; grad_c_collision; grad_c_collisionCones]);
    end
end