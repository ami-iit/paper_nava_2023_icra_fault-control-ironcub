function [HessianMatrixQP, gVectorQP, lowerBoundQP, upperBoundQP, L_des, LDot_estimated, verifyAngMomAndInertia, velCoM] = ...
             flyingMomentumControl(jointPos, w_baseTwist, jetsIntensities, J_jets, J_CoM, J_LFoot, J_RFoot, posCoM, w_R_b, w_H_LFoot, ...
                                   w_H_RFoot, matrixOfJetsAxes, matrixOfJetsArms, w_I_c, L, M, CMM, pos_vel_acc_jerk_CoM_des, ....
                                   rot_vel_acc_jerk_base_des, jointPos_des, KP_momentum, KD_momentum, KP_postural, feetContactIsActive, ...
                                   decreaseMaxFootVerticalForce, increaseMinFootVerticalForce, robotIsLanded, turbinesStatus, contactForces_hat, Config)
                               
    % FLYINGMOMENTUMCONTROL implements a momentum-based flying controller.
    %                       Two different control algorithms are implemented.
    %
    % REFERENCES: - IEEE-RAL 2018, "Momentum Control of an Underactuated Flying Humanoid Robot"; 
    %
    %             - IEEE-HUMANOIDS 2018, "Positon and Attitude Control of an Underactuated Flying Humanoid Robot".
    %
    %% ------------Initialization----------------
    
    persistent eps_increase_forces eps_decrease_forces eps_constr_momentum time

    if isempty(time)

        time = 0;
    end
    if isempty(eps_increase_forces)
    
        eps_increase_forces = 0;     
    end
    if isempty(eps_decrease_forces)
    
        eps_decrease_forces = 0;
    end
     if isempty(eps_constr_momentum)
    
        eps_constr_momentum = 0;
    end

    % compute the number of degrees of freedom, number of jets, robot total 
    % mass and gravity, estimated contact forces  
    ndof                   = size(Config.N_DOF_MATRIX, 1);
    m                      = M(1,1);
    f_grav                 = m*Config.GRAVITY_ACC*[0; 0; 1; zeros(3,1)];
    
    % calculate CoM velocities for visualization [km/h]
    rpy_base               = wbc.rollPitchYawFromRotation(w_R_b);
    velCoM                 = transpose(wbc.rotationFromRollPitchYaw([0; 0; rpy_base(3)]))*L(1:3)/m*3.6;
    
    % feet positions and intialize output for visualization and debug
    posLeftFoot            = w_H_LFoot(1:3,4); 
    posRightFoot           = w_H_RFoot(1:3,4);
    verifyAngMomAndInertia = zeros(4,2);
        
    % compute momentum references and demux jet axes and arms
    [LDDot_des, LDot_des, L_des, intL_des]               = iRonCubLib_v1.computeMomentumReferences(pos_vel_acc_jerk_CoM_des, m);
    [r_J1, r_J2, r_J3, r_J4, ax_J1, ax_J2, ax_J3, ax_J4] = iRonCubLib_v1.demuxJetAxesAndArms(matrixOfJetsAxes, matrixOfJetsArms);

    %% %%%%%%%%%%%%%%% COMPUTE THE MOMENTUM ACCELERATION %%%%%%%%%%%%%%% %%
    %
    % WHILE FLYING: the momentum acceleration is of the following form: 
    %
    %   LDDot = Aj * jetsIntensitiesDot + Lambda_js * sDot + Lambda_jb * w_baseTwist (1a)
    %
    % WHILE BALANCING: the momentum acceleration is different:
    %
    %   LDDot = Aj * jetsIntensitiesDot + Lambda_js * sDot + Lambda_jb * w_baseTwist + ...
    %           Ac * contactForcesDot + Lambda_cs * sDot + Lambda_cb * w_baseTwist (1b)
    %
    % it is therefore necessary to compute Aj, Ac, Lambda_js, Lambda_jb,
    % Lambda_cs and Lambda_cb. Read the references for more details.

    % multiplier of jetsIntensitiesDot in the angular momentum equations 
    % (namely, the last 3 rows of Aj)
    Aj_angular      = [wbc.skew(r_J1) * ax_J1, ...
                       wbc.skew(r_J2) * ax_J2, ...
                       wbc.skew(r_J3) * ax_J3, ...
                       wbc.skew(r_J4) * ax_J4];
              
    % multiplier of jetsIntensitiesDot in the linear momentum equations 
    % (namely, the first 3 rows of Aj)                
    Aj_linear       = matrixOfJetsAxes;
    
    % compute matrix Aj
    Aj              = [Aj_linear; Aj_angular];
       
    % compute matrix Ac
    r_left          = posLeftFoot  - posCoM;
    r_right         = posRightFoot - posCoM;
    
    Ac_leftFoot     = [eye(3),           zeros(3);
                       wbc.skew(r_left), eye(3)];
               
    Ac_rightFoot    = [eye(3),            zeros(3);
                       wbc.skew(r_right), eye(3)];
    
    Ac              = [Ac_leftFoot, Ac_rightFoot];
    
    % J_jr = J_j - J_ext is the map from the derivative of "r_Ji" and the 
    % angular velocity of the turbine frame "omega_Ji" to the state velocities.
    J_ext           = [J_CoM; zeros(3,ndof+6); J_CoM; zeros(3,ndof+6); J_CoM; zeros(3,ndof+6); J_CoM; zeros(3,ndof+6)];
    J_jr            = J_jets - J_ext;
    
    % Jr_left/rightFoot is the map from the derivative of "r_left" and 
    % "r_right" to the state velocities.
    Jr_leftFoot     = J_LFoot(1:3,:) - J_CoM(1:3,:);
    Jr_rightFoot    = J_RFoot(1:3,:) - J_CoM(1:3,:); 
    
    % multipliers of the state velocities in the momentum acceleration equations
    
    % terms related to the contact forces
    wrench_LFoot    = contactForces_hat(1:6);
    wrench_RFoot    = contactForces_hat(7:end);
    AcDot_leftFoot  = [zeros(3,ndof+6); - wbc.skew(wrench_LFoot(1:3)) * Jr_leftFoot];
    AcDot_rightFoot = [zeros(3,ndof+6); - wbc.skew(wrench_RFoot(1:3)) * Jr_rightFoot];
    Lambda_c        = AcDot_leftFoot + AcDot_rightFoot;
    
    % terms related to the thrust forces
    Lambda_j        = -[jetsIntensities(1) * iRonCubLib_v1.skewZeroBar(ax_J1),...
                        jetsIntensities(1) * iRonCubLib_v1.skewBar(r_J1) * wbc.skew(ax_J1),...
                        jetsIntensities(2) * iRonCubLib_v1.skewZeroBar(ax_J2),...
                        jetsIntensities(2) * iRonCubLib_v1.skewBar(r_J2) * wbc.skew(ax_J2),...
                        jetsIntensities(3) * iRonCubLib_v1.skewZeroBar(ax_J3),...
                        jetsIntensities(3) * iRonCubLib_v1.skewBar(r_J3) * wbc.skew(ax_J3),...
                        jetsIntensities(4) * iRonCubLib_v1.skewZeroBar(ax_J4),...
                        jetsIntensities(4) * iRonCubLib_v1.skewBar(r_J4) * wbc.skew(ax_J4)] * J_jr;
   
    % isolate Lambda_jb and Lambda_js, Lambda_cb and Lambda_cs
    Lambda_jb       = Lambda_j(:,1:6);
    Lambda_js       = Lambda_j(:,7:end);
    Lambda_cb       = Lambda_c(:,1:6);
    Lambda_cs       = Lambda_c(:,7:end);
    
    %% %%%%%%%%%%%%%%%%%%  CONTROLLERS DEFINITION  %%%%%%%%%%%%%%%%%%%%% %%
    %
    % The control inputs of Eq. (1a)-(1b) are: 
    %
    %  u1 = [jetsIntensitiesDot; contactForcesDot]
    %  u2 = sDot
    %
    % The input contactForcesDot is deactivated when the robot is flying.
    % For the purpose of this documentation, it suffices to know that the
    % robot momentum converges to the desired values if the following 
    % equation is verified:
    %
    %  ATilde * u1 + BTilde * u2 + deltaTilde = 0 (2)
    %
    % where: ATilde, BTilde and deltaTilde must be properly calculated and 
    % are different in case the robot is in contact with the environment.       
    % For details on the controller implementation, see the RAL 2018 paper
    % at: https://ieeexplore.ieee.org/document/7997895. 
    %
    % A modified version of the RAL controller that also allows to perform
    % attitude tracking has also been developed. Check the corresponding
    % matlab function (attitudeController.m) or read HUMANOIDS 2018 paper 
    % for more details.
    %
    % Eq. (2) will be used inside a QP optimization procedure.
   
    % split the centroidal momentum matrix into the relation between the 
    % momentum and the base and joint velocities, respectively
    JL_b            = CMM(:,1:6); 
    JL_s            = CMM(:,7:ndof+6); 
    
    % part of the cost function related to the constraints on the angular
    % momentum (see IEEE-HUMANOIDS 2018 controller; in the IEEE-RAL
    % controller these variables are zero)
    H_angMomentum   = blkdiag(zeros(4), zeros(12), zeros(ndof));
    g_angMomentum   = [zeros(4,1); zeros(12,1); zeros(ndof,1)];  
    
    % compute the momentum error derivative/integral
    LDot_estimated  = Aj * jetsIntensities + Ac * contactForces_hat .* feetContactIsActive - f_grav;
    LDot_tilde      = LDot_estimated - LDot_des;
    intL_tilde      = [(m * posCoM - intL_des(1:3)); zeros(3,1)];
    
    % substitute the angular momentum integral with the base orientation
    % control in SO(3)(only for RAL control, overwritten for HUMANOIDS control)
    intL_tilde(4:6) = wbc.skewVee(w_R_b * rot_vel_acc_jerk_base_des(1:3,1:3)');
    
    % CONTROL # 1: momentum-based control with Lyapunov stability (IEEE-RAL 2018)
    KTilde     = KP_momentum + inv(Config.gains.momentum.KO) + KD_momentum;    
    ATilde     = [Aj, Ac .* feetContactIsActive];
    BTilde     = Lambda_js + KTilde * JL_s + Lambda_cs .* feetContactIsActive;
    deltaTilde = (Lambda_jb + Lambda_cb .* feetContactIsActive + KTilde * JL_b) * w_baseTwist - KTilde * L_des - LDDot_des + ...
                 (KD_momentum + eye(6)) * LDot_tilde + KP_momentum * intL_tilde; 
    
    % CONTROL # 2: linear momentum and attitude control (IEEE-HUMANOIDS 2018)
    if Config.USE_ATTITUDE_CONTROL
        
        % The outcome of this controller is still an equation of the form
        % of Eq (2) as for controller 1. The part of Eq. (2) related to the
        % linear momentum is calculated as for the IEEE-RAL control. The part 
        % of Eq (2) related to the angular momentum is instead different: it 
        % is used to design a whole-body attitude controller.
        
        % Compute the linear momentum part
        %
        % The linear momentum control is achieved by imposing:
        %
        %  ATilde_linear * u1 + BTilde_linear * u2 + deltaTilde_linear = 0
        %
        % where ATilde_linear, BTilde_linear, deltaTilde_linear correspond to the 
        % first 3 rows ATilde, BTilde, deltaTilde.
        %
        ATilde_linear     = ATilde(1:3,:);                
        BTilde_linear     = BTilde(1:3,:);
        deltaTilde_linear = deltaTilde(1:3);
        
        % Compute the linear momentum part
        %
        % The linear momentum control is achieved by imposing:
        %
        %  ATilde_angular * u1 + BTilde_angular * u2 + deltaTilde_angular = 0
        %
        % where ATilde_angular, BTilde_angular, deltaTilde_angular are
        % achieved using an attitude controller.
        
        % attitude control gains
        c0                = KP_momentum(4:6,4:6);
        c1                = KD_momentum(4:6,4:6);
       
        [H_angMomentum, g_angMomentum, ATilde_angular, BTilde_angular, deltaTilde_angular, verifyAngMomAndInertia, w_L_angMom_des] = ...
            iRonCubLib_v1.flyingAttitudeControl(w_R_b, w_I_c, w_baseTwist, CMM, rot_vel_acc_jerk_base_des, Ac, Aj, Lambda_cb, Lambda_cs, Lambda_jb, ...
                                             Lambda_js, L, LDot_estimated, feetContactIsActive, c0, c1);
        % for visualization
        L_des(4:6)        = w_L_angMom_des;
        
        % combine linear and angular part
        ATilde            = [ATilde_linear; ATilde_angular];
        BTilde            = [BTilde_linear; BTilde_angular];
        deltaTilde        = [deltaTilde_linear; deltaTilde_angular];
    end
         
    %% %%%%%%%%%%%%%%%%%%  QP PROBLEM CONSTRUCTION  %%%%%%%%%%%%%%%%%%%% %%
    %
    % Eq. (2) is considered to be a task of the following optimization:
    %
    %   min_u (0.5 * |H * u - g|^2)
    %
    %      s.t. u_min < u < u_max
    %
    % where H is an Hessian matrix accounting for the main control task 
    % Eq. (2) and for other control tasks with lower priority, such as to 
    % control the robot posture. The vector g is the associated gradient,
    % while u_min/max are boundaries on the control inputs.
    %
    % In case the robot is in contact with the environment, the additional
    % constraint 
    %
    %  Jc * nu = 0
    % 
    % is added to the problem. It represents the constraint that the feet
    % shouldn't move while balancing.
    %
    % However, note that there are no equality constraints in our 
    % QP formulation. This because we would like to avoid the QP to fail
    % because of infeasibility as it may be dangerous if it happens on the
    % real robot. Therefore all tasks, even feet constraints, are added to
    % the cost function.
        
    % Primary tasks to be added to the cost function: angular momentum
    %
    % IEEE-RAL       = no task;
    %
    % IEEE-HUMANOIDS = already calculated: H_angMomentum, g_angMomentum 
    
    % Primary tasks to be added to the cost function: momentum control
    H_momentum         = transpose([ATilde, BTilde]) * [ATilde, BTilde];
    g_momentum         = transpose([ATilde, BTilde]) * deltaTilde;
   
    % Primary tasks to be added to the cost function: feet constraints
    J_feet             = [J_LFoot; J_RFoot];
    J_feet_b           = J_feet(:,1:6);
    J_feet_s           = J_feet(:,7:end); 
    H_feet             = blkdiag(zeros(4), zeros(12), (transpose(J_feet_s) * J_feet_s) .* feetContactIsActive);
    g_feet             = [zeros(4,1); zeros(12,1); (transpose(J_feet_s) * J_feet_b * w_baseTwist) .* feetContactIsActive];
                         
    % Secondary tasks to be added to the cost function: postural task
    H_postural         = blkdiag(zeros(4), zeros(12), eye(ndof));
    g_postural         = [zeros(4,1); zeros(12,1); KP_postural * (jointPos - jointPos_des)];  
    
    % Secondary tasks to be added to the cost function: minimize the arms
    % jets thrust variations
    H_minArmJet        = blkdiag(eye(2), zeros(2), zeros(12), zeros(ndof));
    g_minArmJet        = zeros(4 + 12 + ndof, 1); 
    
    % Secondary tasks to be added to the cost function: minimize the chest
    % jets thrust variations
    H_minChestJet      = blkdiag(zeros(2), eye(2), zeros(12), zeros(ndof));
    g_minChestJet      = zeros(4 + 12 + ndof, 1); 
    
    % Secondary tasks to be added to the cost function: minimize the
    % contact forces variations
    H_minContactForces = blkdiag(zeros(2), zeros(2), eye(12), zeros(ndof));
    g_minContactForces = zeros(4 + 12 + ndof, 1); 
    
    % Secondary tasks to be added to the cost function: minimize the joint
    % velocities
    H_minJointVel      = blkdiag(zeros(2), zeros(2), zeros(12), eye(ndof));
    g_minJointVel      = zeros(4 + 12 + ndof, 1); 
    
    % Secondary tasks to be added to the cost function: symmetry of the
    % jets thrusts.
    % 
    % WARNING: hard-coded thrusts configuration: [lArm, rArm, lChest,rChest]
    if Config.weights.symLeftAndRightThrusts
        
        Ht_symmetry = [1    0   -1    0
                       0    1    0   -1
                      -1    0    1    0
                       0   -1    0    1];
    
    elseif Config.weights.symArmsAndChestThrusts
    
        Ht_symmetry = [1   -1    0    0
                      -1    1    0    0
                       0    0    1   -1
                       0    0   -1    1];
    
    elseif Config.weights.symChestThrusts
    
        Ht_symmetry = [0    0    0    0
                       0    0    0    0
                       0    0    1   -1
                       0    0   -1    1];
    
    elseif Config.weights.symArmsThrusts  
    
        Ht_symmetry = [1   -1    0    0
                      -1    1    0    0
                       0    0    0    0
                       0    0    0    0];
    
    elseif Config.weights.symAllThrusts 
    
        Ht_symmetry = [1   -1    0    0
                      -1    2   -1    0
                       0   -1    2   -1
                       0    0   -1    1]; 
    else      
        Ht_symmetry = zeros(4);
    end
    
    H_symmetry      = blkdiag(Ht_symmetry, zeros(12), zeros(ndof));
    g_symmetry      = zeros(4 + 12 + ndof, 1);
    
    % while landing, increase progressively the weights on equality
    % constraints to have a smoother transition from flying to landing   
    if robotIsLanded
        
        weights_equalityConstraints = min((eps_constr_momentum + Config.deltas.delta_eps_constr_momentum * increaseMinFootVerticalForce), Config.weights.eqConstraints_momentumControl);
        eps_constr_momentum         = eps_constr_momentum + Config.deltas.delta_eps_constr_momentum * increaseMinFootVerticalForce;
    else
        weights_equalityConstraints = Config.weights.eqConstraints_momentumControl;
        eps_constr_momentum         = 0;
    end    
 
    %%%%%%%%%%%%%%%%%%%%%
    % FAILURE DETECTION %
    %%%%%%%%%%%%%%%%%%%%%  
    if sum(turbinesStatus) > 0.5
        
        if time < Config.t_updateWeightsDuringFailure
      
            alpha                   = min(2*time, 1);
            Config.weights.momentum = (1-alpha)*Config.weights.momentum + alpha*Config.weights.momentum/10;
        else
            alpha                   = min(2*(time-Config.t_updateWeightsDuringFailure), 1);
            Config.weights.momentum = (1-alpha)*Config.weights.momentum/10 + alpha*Config.weights.momentum;
        end
        time = time + Config.tStep;
    end

    % build the QP cost function (Hessian matrix and gradient)
    HessianMatrixQP = Config.weights.postural * H_postural + ...
                      Config.weights.minArmsThrustDot * H_minArmJet + ...
                      Config.weights.minChestThrustDot * H_minChestJet + ...
                      Config.weights.minJointVel * H_minJointVel + ...
                      Config.weights.symmetryThrust * H_symmetry + ...
                      Config.weights.minContactForcesDot * H_minContactForces + ...
                      Config.weights.angMomentumConstraint * H_angMomentum + ...
                      Config.weights.momentum * H_momentum + ...
                      weights_equalityConstraints * H_feet *feetContactIsActive;
    
    % regularization of the Hessian. Enforce symmetry and positive definiteness 
    HessianMatrixQP = 0.5 * (HessianMatrixQP + transpose(HessianMatrixQP)) + eye(size(HessianMatrixQP,1)) * Config.reg.hessianQp;
    
    % compute the gradient 
    gVectorQP       = Config.weights.postural * g_postural + ...
                      Config.weights.minArmsThrustDot * g_minArmJet + ...
                      Config.weights.minChestThrustDot * g_minChestJet + ...
                      Config.weights.minJointVel * g_minJointVel + ...
                      Config.weights.symmetryThrust * g_symmetry + ...
                      Config.weights.minContactForcesDot * g_minContactForces + ...
                      Config.weights.angMomentumConstraint * g_angMomentum + ...
                      Config.weights.momentum * g_momentum + ...
                      weights_equalityConstraints * g_feet * feetContactIsActive;
    
    % upper and lower bounds of the control input u. In particular, one has
    % that lowerBoundQP < u < upperBoudQP 
    maxJointPos         =  Config.sat.jointPositionLimits(:,2);
    minJointPos         =  Config.sat.jointPositionLimits(:,1);
    minJetsInt          =  zeros(4,1);
    maxJetsInt          =  Config.sat.maxJetsInt;
    maxJetsIntVar       =  Config.sat.maxJetsIntVar;
    minJetsIntVar       = -Config.sat.maxJetsIntVar;
    maxJointVelDes      =  Config.sat.maxJointVelDes;
    minJointVelDes      = -Config.sat.maxJointVelDes;
    maxContactForcesVar =  Config.sat.maxContactForcesVar;
    minContactForcesVar = -Config.sat.maxContactForcesVar;
  
    %%%%%%%%%%%%%%%%%%%%%
    % FAILURE DETECTION %
    %%%%%%%%%%%%%%%%%%%%%
    if turbinesStatus(2) > 0
    
        maxJetsInt(2) = maxJetsInt(2)*exp(-Config.failureDetection.eps_thrustBound*time);         
    end
    if turbinesStatus(3) > 0
    
        maxJetsInt(3) = maxJetsInt(3)*exp(-Config.failureDetection.eps_thrustBound*time);         
    end

    % include thrusts limits in the controller. Define the following quantities:
    %
    %  T    = jetsIntensities;
    %  TDot = jetsIntensitiesDot;
    %
    % Basically, we tried to update online the boundaries of TDot in such a 
    % way that also the boundaries on T are respected. In particular, the
    % thrust and its rate of change always fall in one of the following cases:
    %
    %  if Tmin < T < Tmax, then TDotMin <= TDot <= TDotMax;
    %  if T = Tmin,        then TDotMin = 0;
    %  if T = Tmax,        then TDotMax = 0;
    %  if T < Tmin,        then TDotmin > 0;
    %  if T > Tmax,        then TDotMax < 0;
    %
    % In order to implement this logic, we parametrized the boundaries
    % of TDot (TDotMin/TDotMax) as follows:
    %
    %    TDotMin = tanh((T - Tmin) * eps) * TDotMin;
    %    TDotMax = tanh((Tmax - T) * eps) * TDotMax.
    %
    % with eps a positive value (the sharpness of the function).
    if Config.INCLUDE_THRUST_LIMITS
        
        for jj = 1:4
            
            eps               = Config.eps_thrust_limit;
            maxJetsIntVar(jj) = tanh((maxJetsInt(jj) - jetsIntensities(jj)) * eps) * maxJetsIntVar(jj);
            minJetsIntVar(jj) = tanh((jetsIntensities(jj) -minJetsInt(jj)) * eps) * minJetsIntVar(jj);
        end
    end
    
    % include joints limits in the controller. Same as the the thrust
    % limits but for the joints positions and velocities.
    if Config.INCLUDE_JOINTS_LIMITS

        for jj = 1:ndof
             
            eps                = Config.eps_joint_limit;
            maxJointVelDes(jj) = tanh((maxJointPos(jj) - jointPos(jj)) * eps) * maxJointVelDes(jj);
            minJointVelDes(jj) = tanh((jointPos(jj) - minJointPos(jj)) * eps) * minJointVelDes(jj);
        end
    end
    
    %% TAKE OFF AND LANDING MANEUVERS

    % TAKE OFF: constrain the contact forces rate of change of the feet
    %           vertical forces to be negative. This will force the
    %           controller to balance using the jets rather than using the
    %           contact forces at feet.
    
    % reset ratio for increasing/decreasing forces when the robot is flying
    if ~feetContactIsActive
        
        eps_decrease_forces = 0;
        eps_increase_forces = 0;
    end
    
    if decreaseMaxFootVerticalForce && wrench_LFoot(3) > Config.residualVerticalForceTakeOff && wrench_RFoot(3) > Config.residualVerticalForceTakeOff
          
        maxContactForcesVar(3) = (1 - 2 * tanh(eps_decrease_forces + Config.deltas.delta_eps_decreaseForces * decreaseMaxFootVerticalForce)) * maxContactForcesVar(3);
        maxContactForcesVar(9) = (1 - 2 * tanh(eps_decrease_forces + Config.deltas.delta_eps_decreaseForces * decreaseMaxFootVerticalForce)) * maxContactForcesVar(9);
        
        eps_decrease_forces    = eps_decrease_forces + Config.deltas.delta_eps_decreaseForces * decreaseMaxFootVerticalForce;
    end
    
    % LANDING: constrain the contact forces rate of change along the feet
    %          vertical forces to be positive. This will force the
    %          controller to balance using the forces rather than using the
    %          jets thrusters.
    if increaseMinFootVerticalForce && wrench_LFoot(3) < Config.residualVerticalForceLanding && wrench_RFoot(3) < Config.residualVerticalForceLanding
        
        minContactForcesVar(3) = (1 - 2 * tanh(eps_increase_forces + Config.deltas.delta_eps_increaseForces * increaseMinFootVerticalForce)) * minContactForcesVar(3);
        minContactForcesVar(9) = (1 - 2 * tanh(eps_increase_forces + Config.deltas.delta_eps_increaseForces * increaseMinFootVerticalForce)) * minContactForcesVar(9);
        
        eps_increase_forces    = eps_increase_forces + Config.deltas.delta_eps_increaseForces * increaseMinFootVerticalForce;
    end
     
    upperBoundQP = [maxJetsIntVar; maxContactForcesVar; maxJointVelDes];
    lowerBoundQP = [minJetsIntVar; minContactForcesVar; minJointVelDes]; 
end
