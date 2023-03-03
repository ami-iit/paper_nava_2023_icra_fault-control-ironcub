function [LDot, grad_LDot] = computeMomentumDerivativeAndGradient(u, KinDynModel, Config)

    % COMPUTEMOMENTUMDERIVATIVEANDGRADIENT calculates the momentum rate of
    %                                      change and its gradient w.r.t. 
    %                                      the optimization variables.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jun. 2022
    %

    % turbines and joints data
    njets           = Config.turbinesData.njets;
    turbineList     = Config.turbinesData.turbineList;
    turbineAxis     = Config.turbinesData.turbineAxis;
    ndof            = Config.ndof;
    jetsIntensities = u(end-njets+1:end);

    % total mass of the system and gravity forces
    M       = iDynTreeWrappers.getFreeFloatingMassMatrix(KinDynModel);
    m       = M(1,1);
    mg      = m*Config.gravityAcc;

    % CoM quantities
    w_o_CoM = iDynTreeWrappers.getCenterOfMassPosition(KinDynModel);
    J_CoM   = iDynTreeWrappers.getCenterOfMassJacobian(KinDynModel);

    % initialize matrices
    A      = zeros(6,njets);
    Lambda = zeros(6,ndof+6);

    % iterate on njets to calculate jets quantities
    for i = 1:njets

        % i-th jet Jacobian
        J_jet_i = iDynTreeWrappers.getFrameFreeFloatingJacobian(KinDynModel,turbineList{i});
        J_r_i   = J_jet_i - [J_CoM; zeros(3,ndof+6)];

        % forward kinematics
        w_H_j_i = iDynTreeWrappers.getWorldTransform(KinDynModel,turbineList{i});
        w_R_j_i = w_H_j_i(1:3,1:3);
        w_o_j_i = w_H_j_i(1:3,4);
        r_jet_i = w_o_j_i - w_o_CoM;
        l_jet_i = sign(turbineAxis(i))*w_R_j_i(1:3,abs(turbineAxis(i)));

        % compute i-th column of matrix A
        A(:,i)  = iRonCubLib_v1.skewBar(r_jet_i)*l_jet_i;

        % compute  i-th column of matrix Lambda
        Lambda = Lambda - [jetsIntensities(i) * iRonCubLib_v1.skewZeroBar(l_jet_i), jetsIntensities(i) * iRonCubLib_v1.skewBar(r_jet_i) * wbc.skew(l_jet_i)] * J_r_i;
    end

    % from angular velocity to Euler angles (rpy)
    % see also http://www.diag.uniroma1.it/~deluca/rob1_en/11_DifferentialKinematics.pdf
    roll  = u(5);
    yaw   = u(6);
    E_rpy = [ cos(roll)*cos(yaw)  -sin(yaw)   0
              cos(roll)*sin(yaw)   cos(yaw)   0
             -sin(roll)            0          1];

    % isolate Lambda_b and Lambda_s
    Lambda_b  = Lambda(:,1:6)*blkdiag(eye(3), E_rpy);
    Lambda_s  = Lambda(:,7:end);

    % momentum derivative and gradient
    grad_LDot = [Lambda_b, Lambda_s, A];
    LDot      = A*jetsIntensities + [mg; zeros(3,1)];
end