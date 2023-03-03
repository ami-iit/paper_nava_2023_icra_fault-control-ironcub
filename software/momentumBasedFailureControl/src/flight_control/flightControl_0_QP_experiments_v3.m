function [HessianQP, gradientQP, lb, ub, A_equality, lbA, ubA, costVectorQP, LDot_estimated, LDot_tilde, L_tilde, intL_tilde, jetsIntensitiesDot_postural] = ...
         flightControl_0_QP_experiments_v3(posCoM_error, jointPos, w_baseTwist, jetsIntensities, J_jets, J_CoM, w_R_b, matrixOfJetsAxes, matrixOfJetsArms, M, L, CMM, ...
                                           w_I_c, pos_vel_acc_jerk_CoM_des, rot_vel_acc_jerk_base_des, jointPos_des, jetsIntensities_des, KP_momentum, KD_momentum, KP_postural, u_star, Config)

% flightControl_0_QP_experiments_v3 implements the third version of the
%                                   take off-hovering-landing control that
%                                   is used with the real iRonCub-MK1_1.

% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %
persistent H_TDotPost_0 H_linMom_xy_0 H_angMom_0 H_linMom_z_0

% compute useful parameters and references
ndof   = size(Config.N_DOF_MATRIX, 1);
njets  = size(Config.N_JETS_MATRIX, 1);
ntorso = size(Config.N_TORSO_MATRIX, 1);
narms  = size(Config.N_ARMS_MATRIX, 1);
nlegs  = size(Config.N_LEGS_MATRIX, 1);
m      = M(1,1);
f_grav = m*Config.GRAVITY_ACC*[0; 0; 1; zeros(3,1)];
JL_b   = CMM(:,1:6);
JL_s   = CMM(:,7:ndof+6);

% compute momentum references and demux jet axes and arms
[LDDot_des, LDot_des, L_des, ~]                      = iRonCubLib.computeMomentumReferences(pos_vel_acc_jerk_CoM_des, m);
[r_J1, r_J2, r_J3, r_J4, ax_J1, ax_J2, ax_J3, ax_J4] = iRonCubLib.demuxJetAxesAndArms(matrixOfJetsAxes, matrixOfJetsArms);

%% MOMENTUM FIRST AND SECOND DERIVATIVE

% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %
% The momentum first and second derivatives are of the following form:
%
%   LDot  = Aj*jetsIntensities - f_grav
%   LDDot = Aj*jetsIntensitiesDot + Lambda_js*sDot + Lambda_jb*w_baseTwist
%
% it is therefore necessary to compute Aj, Lambda_js, Lambda_jb.

% multiplier of jetsIntensitiesDot in the linear momentum equations
% (namely, the first 3 rows of Aj)
Aj_linear      = matrixOfJetsAxes;

% multiplier of jetsIntensitiesDot in the angular momentum equations
% (namely, the last 3 rows of Aj)
Aj_angular     = [wbc.skew(r_J1) * ax_J1, ...
                  wbc.skew(r_J2) * ax_J2, ...
                  wbc.skew(r_J3) * ax_J3, ...
                  wbc.skew(r_J4) * ax_J4];
               
% compute matrix Aj
Aj             = [Aj_linear; Aj_angular];

% compute the momentum first derivative
LDot_estimated = Aj * jetsIntensities - f_grav;

% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %
% Multipliers of the state velocities in the momentum acceleration equations

% J_jr = J_j - J_ext is the map from the derivative of "r_Ji" and the
% angular velocity of the turbine frame "omega_Ji" to the state velocities.
J_ext          = [J_CoM; zeros(3,ndof+6); J_CoM; zeros(3,ndof+6); J_CoM; zeros(3,ndof+6); J_CoM; zeros(3,ndof+6)];
J_jr           = J_jets - J_ext;

% compute matrix Lambda_j
Lambda_j       = -[jetsIntensities(1) * iRonCubLib.skewZeroBar(ax_J1),...
                   jetsIntensities(1) * iRonCubLib.skewBar(r_J1) * wbc.skew(ax_J1),...
                   jetsIntensities(2) * iRonCubLib.skewZeroBar(ax_J2),...
                   jetsIntensities(2) * iRonCubLib.skewBar(r_J2) * wbc.skew(ax_J2),...
                   jetsIntensities(3) * iRonCubLib.skewZeroBar(ax_J3),...
                   jetsIntensities(3) * iRonCubLib.skewBar(r_J3) * wbc.skew(ax_J3),...
                   jetsIntensities(4) * iRonCubLib.skewZeroBar(ax_J4),...
                   jetsIntensities(4) * iRonCubLib.skewBar(r_J4) * wbc.skew(ax_J4)] * J_jr;

% isolate Lambda_jb and Lambda_js
Lambda_jb      = Lambda_j(:,1:6);
Lambda_js      = Lambda_j(:,7:end);

%% FLIGHT CONTROLLER

% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %
% The robot momentum accelerations converge to the desired values if the 
% following equation is verified:
%
%  ATilde * u1 + BTilde * u2 + deltaTilde = 0
%
% where: ATilde, BTilde and deltaTilde must be properly calculated.
%
% see also https://ieeexplore.ieee.org/document/7997895.

% postural task jets intensities derivative
jetsIntensitiesDot_postural  = Config.gains.postural.KP_jets * (jetsIntensities - jetsIntensities_des);

% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %
% compute momentum errors. Substitute the angular momentum integral error 
% with the base orientation error in SO(3)
L_tilde         = L - L_des;
LDot_tilde      = LDot_estimated - LDot_des;
intL_tilde      = [(m * posCoM_error); zeros(3,1)];
intL_tilde(4:6) = w_I_c * wbc.skewVee(w_R_b * rot_vel_acc_jerk_base_des(1:3,1:3)');

% momentum-based control with Lyapunov stability (IEEE-RAL 2018)
KTilde          = (KP_momentum + inv(Config.gains.momentum.KO) + KD_momentum);
ATilde          = Aj;
BTilde          = Lambda_js + KTilde * JL_s;
deltaTilde      = (Lambda_jb + KTilde * JL_b) * w_baseTwist- KTilde * L_des - LDDot_des + ...
                  (KD_momentum + eye(6)) * LDot_tilde + KP_momentum * intL_tilde;

% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %
% overall robot attitude control

% isolate the linear momentum control first
ATilde_linear     = ATilde(1:3,:);                
BTilde_linear     = BTilde(1:3,:);
deltaTilde_linear = deltaTilde(1:3);
   
% get the attitude gains
c0                = KP_momentum(4:6,4:6);
c1                = KD_momentum(4:6,4:6);

[b_JH_angMom, bVector_angMom, ATilde_angular, BTilde_angular, deltaTilde_angular, ~, ~] = ...
    flightAttitudeControl_experiments(w_R_b, w_I_c, w_baseTwist, CMM, rot_vel_acc_jerk_base_des, Aj, Lambda_jb, Lambda_js, L, LDot_estimated, c0, c1);
                                                                                                                        
% combine linear and angular part
ATilde            = [ATilde_linear; ATilde_angular];
BTilde            = [BTilde_linear; BTilde_angular];
deltaTilde        = [deltaTilde_linear; deltaTilde_angular];             
               
%% QP TASKS

% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %
% Tasks to be added to the QP cost function

% LINEAR MOMENTUM TASK - X Y
H_linMom_xy = transpose([ATilde(1:2,:), BTilde(1:2,:)]) * [ATilde(1:2,:), BTilde(1:2,:)];
g_linMom_xy = transpose([ATilde(1:2,:), BTilde(1:2,:)]) * deltaTilde(1:2);

% add weights and initialize Hessian scaling
if isempty(H_linMom_xy_0)

    H_linMom_xy_0 = H_linMom_xy;
end

H_linMom_xy = Config.weightsQP.linMom_xy * H_linMom_xy/computeScalingCostQP(H_linMom_xy_0);
g_linMom_xy = Config.weightsQP.linMom_xy * g_linMom_xy/computeScalingCostQP(H_linMom_xy_0);

% ----------------------------------------------------------------------- %

% LINEAR MOMENTUM TASK - Z
H_linMom_z = transpose([ATilde(3,:), BTilde(3,:)]) * [ATilde(3,:), BTilde(3,:)];
g_linMom_z = transpose([ATilde(3,:), BTilde(3,:)]) * deltaTilde(3);

% add weights and initialize Hessian scaling
if isempty(H_linMom_z_0)

    H_linMom_z_0 = H_linMom_z;
end

H_linMom_z = Config.weightsQP.linMom_z * H_linMom_z/computeScalingCostQP(H_linMom_z_0);
g_linMom_z = Config.weightsQP.linMom_z * g_linMom_z/computeScalingCostQP(H_linMom_z_0);

% ----------------------------------------------------------------------- %

% ANGULAR MOMENTUM TASK
H_angMom = transpose([ATilde(4:6,:), BTilde(4:6,:)]) * [ATilde(4:6,:), BTilde(4:6,:)];
g_angMom = transpose([ATilde(4:6,:), BTilde(4:6,:)]) * deltaTilde(4:6);

% add weights and initialize Hessian scaling
if isempty(H_angMom_0)
    
    H_angMom_0 = H_angMom;
end

H_angMom = Config.weightsQP.angMom * H_angMom/computeScalingCostQP(H_angMom_0);
g_angMom = Config.weightsQP.angMom * g_angMom/computeScalingCostQP(H_angMom_0);

% ----------------------------------------------------------------------- %

% POSTURAL TASK TORSO
H_torsoPost = blkdiag(zeros(njets), eye(ntorso), zeros(narms + nlegs));
g_torsoPost = [zeros(njets,1); KP_postural(1:ntorso,1:ntorso) * (jointPos(1:ntorso) - jointPos_des(1:ntorso)); zeros(narms + nlegs,1)];

% add weights
H_torsoPost = Config.weightsQP.postural_torso * H_torsoPost;
g_torsoPost = Config.weightsQP.postural_torso * g_torsoPost;
  
% ----------------------------------------------------------------------- %

% POSTURAL TASK ARMS
H_armsPost = blkdiag(zeros(njets + ntorso), eye(narms), zeros(nlegs));
g_armsPost = [zeros(njets + ntorso,1); KP_postural(ntorso+1:ntorso+narms, ntorso+1:ntorso+narms) * (jointPos(ntorso+1:ntorso+narms) - jointPos_des(ntorso+1:ntorso+narms)); zeros(nlegs,1)];

% add weights
H_armsPost = Config.weightsQP.postural_arms * H_armsPost;
g_armsPost = Config.weightsQP.postural_arms * g_armsPost;
 
% ----------------------------------------------------------------------- %

% POSTURAL TASK LEGS
H_legsPost = blkdiag(zeros(njets + ntorso + narms), eye(nlegs));
g_legsPost = [zeros(njets + ntorso + narms,1); KP_postural(nlegs:end, nlegs:end) * (jointPos(nlegs:end) - jointPos_des(nlegs:end))];

% add weights 
H_legsPost = Config.weightsQP.postural_legs * H_legsPost;
g_legsPost = Config.weightsQP.postural_legs * g_legsPost;
  
% ----------------------------------------------------------------------- %

% POSTURAL TASK THRUST DERIVATIVE
H_TDotPost = blkdiag(eye(njets), zeros(ndof));
g_TDotPost = [jetsIntensitiesDot_postural; zeros(ndof,1)];

% add weights and initialize Hessian scaling
if isempty(H_TDotPost_0)

    H_TDotPost_0 = H_TDotPost;
end

H_TDotPost = Config.weightsQP.postural_TDot * H_TDotPost/computeScalingCostQP(H_TDotPost_0);
g_TDotPost = Config.weightsQP.postural_TDot * g_TDotPost/computeScalingCostQP(H_TDotPost_0);
         
% ----------------------------------------------------------------------- %

% THRUST SYMMETRY
[H_symmThrust, g_symmThrust] = computeHessianGradientThrustSymmetryTask(njets, ndof, Config);
   
% add weights
H_symmThrust = Config.weightsQP.symmThrust * H_symmThrust;
g_symmThrust = Config.weightsQP.symmThrust * g_symmThrust;
         
% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %
% Build the QP cost function (Hessian matrix and gradient)
HessianQP  = H_torsoPost + ...
             H_armsPost + ...
             H_legsPost + ...
             H_TDotPost + ...
             H_linMom_xy + ...
             H_linMom_z + ...
             H_angMom + ...
             H_symmThrust;
         
% regularization of the Hessian. Enforce symmetry and positive definiteness
HessianQP  = 0.5 * (HessianQP + transpose(HessianQP)) + eye(size(HessianQP,1)) * Config.tasksQP.reg_HessianQP;

% compute the gradient
gradientQP = g_torsoPost + ...
             g_armsPost + ...
             g_legsPost + ...
             g_TDotPost + ...
             g_linMom_xy + ...
             g_linMom_z + ...
             g_angMom + ...
             g_symmThrust;
         
% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %
% Compute costs for visualization
costVectorQP = [computeCostQP(H_linMom_xy, g_linMom_xy, u_star)
                computeCostQP(H_linMom_z, g_linMom_z, u_star)
                computeCostQP(H_angMom, g_angMom, u_star)
                computeCostQP(H_torsoPost, g_torsoPost, u_star)
                computeCostQP(H_armsPost, g_armsPost, u_star)
                computeCostQP(H_legsPost, g_legsPost, u_star)
                computeCostQP(H_TDotPost, g_TDotPost, u_star)
                computeCostQP(H_symmThrust, g_symmThrust, u_star)];
         
%% QP BOUNDARIES

% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %
% QP input boundaries. They are of the form:
%
% lb <= u <= ub
%
maxJointPos    =  Config.inequalitiesQP.jointPositionLimits(:,2);
minJointPos    =  Config.inequalitiesQP.jointPositionLimits(:,1);
maxJetsInt     =  Config.inequalitiesQP.maxJetsInt;
idleJetsInt    =  Config.inequalitiesQP.idleJetsInt;
maxJetsIntVar  =  Config.inequalitiesQP.maxJetsIntVar;
minJetsIntVar  = -Config.inequalitiesQP.maxJetsIntVar;
maxJointVelDes =  Config.inequalitiesQP.maxJointVelDes;
minJointVelDes = -Config.inequalitiesQP.maxJointVelDes;

% INCLUDE THRUST LIMITS in the QP boundaries
%
% we parametrize the boundaries of TDot (TDotMin/TDotMax) as follows:
%
%    TDotMin = tanh((T - Tmin) * eps) * TDotMin;
%    TDotMax = tanh((Tmax - T) * eps) * TDotMax.
%
% with eps a positive value (the sharpness of the function).
if Config.inequalitiesQP.INCLUDE_THRUST_LIMITS
    
    for jj = 1:njets
        
        eps_thrust_limit  = Config.inequalitiesQP.eps_thrust_limit;
        maxJetsIntVar(jj) = tanh((maxJetsInt(jj) - jetsIntensities(jj)) * eps_thrust_limit) * maxJetsIntVar(jj);
        minJetsIntVar(jj) = tanh((jetsIntensities(jj) - idleJetsInt(jj)) * eps_thrust_limit) * minJetsIntVar(jj);
    end
end

% INCLUDE JOINT LIMITS in the QP boundaries. Same procedure as before
if Config.inequalitiesQP.INCLUDE_JOINTS_LIMITS
    
    for jj = 1:ndof
        
        eps_joint_limit    = Config.inequalitiesQP.eps_joint_limit;
        maxJointVelDes(jj) = tanh((maxJointPos(jj) - jointPos(jj)) * eps_joint_limit) * maxJointVelDes(jj);
        minJointVelDes(jj) = tanh((jointPos(jj) - minJointPos(jj)) * eps_joint_limit) * minJointVelDes(jj);
    end
end

% Combine QP bounds
ub = [maxJetsIntVar; maxJointVelDes];
lb = [minJetsIntVar; minJointVelDes];

%% QP CONSTRAINTS

% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %
% (in)equality constraints. Basically, all the QP constraints of the form:
%
% A*u = b
%
% which are relaxed as:
%
% b-eps <= A*u <= b+eps
%
% with eps a small positive number

% POSTURAL TASK
eps_legs          =  Config.equalitiesQP.eps_legs;
A_legsPost        =  [zeros(nlegs, njets), zeros(nlegs, ntorso+narms) eye(nlegs)];
lbA_legsPost      = -KP_postural(nlegs:end, nlegs:end)*(jointPos(nlegs:end) - jointPos_des(nlegs:end)) - eps_legs;
ubA_legsPost      = -KP_postural(nlegs:end, nlegs:end)*(jointPos(nlegs:end) - jointPos_des(nlegs:end)) + eps_legs;

% ANGULAR MOMENTUM TASK
eps_angMom        =  Config.equalitiesQP.eps_angMom;
A_angMom          =  [ATilde(4:6,:), BTilde(4:6,:)];
lbA_angMom        = -deltaTilde(4:6) - eps_angMom;
ubA_angMom        = -deltaTilde(4:6) + eps_angMom;

% LINEAR MOMENTUM TASK ON X-Y
eps_linMom_xy     =  Config.equalitiesQP.eps_linMom_xy;
A_linMom_xy       =  [ATilde(1:2,:), BTilde(1:2,:)];
lbA_linMom_xy     = -deltaTilde(1:2) - eps_linMom_xy;
ubA_linMom_xy     = -deltaTilde(1:2) + eps_linMom_xy;
  
% CONSTAINT ON THE ROBOT ANGULAR MOMENTUM
eps_angMom_constr =  Config.equalitiesQP.eps_angMom_constr;
A_angMom_constr   =  [zeros(3, njets), b_JH_angMom];
lbA_angMom_constr = -bVector_angMom - eps_angMom_constr;
ubA_angMom_constr = -bVector_angMom + eps_angMom_constr;

% Combine QP equality constraints
A_equality        = [A_legsPost; A_angMom; A_linMom_xy; A_angMom_constr];
lbA               = [lbA_legsPost; lbA_angMom; lbA_linMom_xy; lbA_angMom_constr];
ubA               = [ubA_legsPost; ubA_angMom; ubA_linMom_xy; ubA_angMom_constr];

end
