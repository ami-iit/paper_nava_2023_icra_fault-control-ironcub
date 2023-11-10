%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %  
%                 PARAMETERS FOR THE FLYING CONTROLLER                    %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                      
%% REFERENCE GENERATOR PARAMETERS

% use smoothed references for the CoM trajectory
Config.references.SMOOTH_COM_POS_REFERENCES    = true; 

% use smoothed references for the base orientation
Config.references.SMOOTH_BASE_ROT_REFERENCES   = true;

% delay introduced by the minjerk Simulink block when computing the time
% derivative of the reference base angular velocity and CoM acceleration, 
% respectively. They are continuos signals: keep these values small
Config.references.orientation.tSmoothingAngVel = 0.05;
Config.references.com.tSmoothingCoMAcc         = 0.005;

% smoothing time for the CoM and joints position reference
Config.references.smoothingTimeCoM             = 1.5;
Config.references.smoothingTimeJoints          = 0.25;

% we cannot apply a normal minjerk generator to generate a smooth
% rotation in SO(3), therefore a rotational controller is used. The gain 
% is the value that multiplies the rotation error. The higher, the faster, 
% but high values may lead to instability and overshoots. 
Config.references.gainSmoothingRotation        = 1;

% rate of change of the references w.r.t. time. CoM position step length
% must be interpreted as: [meters/timeStep]; base rotation as [deg/timeStep] 
Config.references.stepLength_CoM               = [0.0075; 0.0025; 0.0075];
Config.references.stepLength_rpyBase           = [0.15; 0.35; 0.35] .* pi/180;

% limit the rate of change along z CoM direction while landing (button Y pressed)
Config.references.zCoMlimitStepLengthLanding   = 0.001;

% limits on the reference roll and picth while flying
Config.references.minBaseRP_flying             = [-5; -30] .* pi/180; 
Config.references.maxBaseRP_flying             = [ 5;  0] .* pi/180;

% limits on the reference roll and picth while balancing
Config.references.minBaseRP_balancing          = [0; 0] .* pi/180; 
Config.references.maxBaseRP_balancing          = [0; 0] .* pi/180;

% limit on the CoM position while balancing
Config.references.minCoMPos_balancing          = [-0.02; -0.02; -0.01];
Config.references.maxCoMPos_balancing          = [ 0.02;  0.02;  0.25];

% turbo boost. The velocity of the robot front movement will progressively
% become (Config.turbo.speedBoost + 1)*Config.references.stepLength_CoM(1)
% when the turbo button is pressed
Config.turbo.speedBoost                        = 9;
 
%% TAKE OFF AND LANDING PARAMETERS

% rate of change of the weight on the equality constraints at feet during
% landing maneuver. Keeping the buttons L1 and R1 will increase the weight
% of the feet eq. constraints by [delta_eps/timeStep] (up to the maximum
% user defined value)
Config.deltas.delta_eps_constr_torque   = 0.005;
Config.deltas.delta_eps_constr_momentum = 0.5;

% this limits avoid the "decrease/increase forces" button to keep
% decreasing/increasing the forces ratio when the vertical forces reach a
% limit user defined value
Config.residualVerticalForceTakeOff     = 2.5; % [N]
Config.residualVerticalForceLanding     = 180; % [N]

% ratio for reducing/increasing the contact forces rate of change
% boundaries. The higher, the faster the robot will reduce/increase the
% contact forces during take off/landing
if Config.USE_JET_DYNAMICS
    
    % slow down take off/landing phase when jet dynamics is simulated
    Config.deltas.delta_eps_decreaseForces  = 0.001;
    Config.deltas.delta_eps_increaseForces  = 0.125;
else
    Config.deltas.delta_eps_decreaseForces  = 0.05;
    Config.deltas.delta_eps_increaseForces  = 0.125;
end

% ratio for reacting when reaching the thrusts or joints limits (to stay
% inside the limits)
Config.eps_thrust_limit                 = 2;
Config.eps_joint_limit                  = 5;

% minimal vertical force at each foot to consider the robot in contact
Config.minVerticalForces                = 3.5;

%% CONTROLLER GAINS

% Linear momentum gains 
Config.gains.momentum.KP_linear = [15, 15, 15];                               
Config.gains.momentum.KD_linear = 2 * sqrt(Config.gains.momentum.KP_linear);
Config.gains.momentum.KO        = 10 .* eye(6); 

% Attitude controller gains
Config.gains.momentum.KP_angular = 50 .* ones(1,3);                            
Config.gains.momentum.KD_angular = 125 .* ones(1,3); 

% Postural task gains              % torso    % left arm      % right arm     % left leg           % right leg
Config.gains.postural.KP         = [30 30 30  20 20 20 20     20 20 20 20     30 30 30 30 30 30    30 30 30 30 30 30]/15;
                             
% Torque control gains
Config.gains.torqueControl.KI    = 250;                            
Config.gains.torqueControl.KP    = 2 * sqrt(Config.gains.torqueControl.KI);

%% QP WEIGHTS AND THRESHOLDS

% QP FLYING CONTROLLER

% thrusts symmetry tasks
Config.weights.symChestThrusts               = false;
Config.weights.symArmsThrusts                = false;
Config.weights.symArmsAndChestThrusts        = false;
Config.weights.symLeftAndRightThrusts        = false;
Config.weights.symAllThrusts                 = false;

% cost function weights
Config.weights.minArmsThrustDot              = 0.01;
Config.weights.minChestThrustDot             = 0.01;
Config.weights.minContactForcesDot           = 0.001;
Config.weights.minJointVel                   = 10;
Config.weights.symmetryThrust                = 0.0;
Config.weights.postural                      = 5;
Config.weights.eqConstraints_momentumControl = 150;
Config.weights.momentum                      = 10;
Config.weights.angMomentumConstraint         = 150;

% QP TORQUE CONTROLLER

% WARNING! Equality constraint is activated with a separate joysytick
% button. This bacause the addition of equality constraints generate a
% peak in the legs torques that destabilizes the robot after landing. This
% is still to be investigated.
Config.weights.eqConstraints_torqueControl   = 50;
Config.weights.minForces                     = 1;
Config.weights.minJointAcc                   = 1;
Config.weights.minTorques                    = 0.001;

% HESSIAN REGULARIZATION
Config.reg.hessianQp                         = 1e-4;

% saturation of the joint torques derivative
Config.sat.jointTorquesDotMax                = 300;

%% INPUTS SATURATIONS 

% DeadZone threshold for numerical integration
Config.sat.deadZoneThreshold                 = 0.00001;

% Joint Position error integral saturation
Config.sat.jointPosIntegralMaxError          = 50; %[deg]

% QP boundaries
Config.sat.maxJetsIntVar                     = [100; 100; 100; 100];
Config.sat.maxJetsInt                        = [160; 160; 220; 220];
Config.sat.maxJointVelDes                    = 45 .* pi/180 .* ones(Config.N_DOF,1);
Config.sat.maxContactForcesVar               = 100 .* ones(12,1);
Config.sat.jointTorquesSaturation            = 50 .* ones(Config.N_DOF,1);

%% CONTACT FORCES INEQUALITY CONSTRAINTS

% feet inequality constraints
numberOfPoints                               = 4;  
forceFrictionCoefficient                     = 1/10;    
torsionalFrictionCoefficient                 = 1/75;
fZmin                                        = 1;

% physical size of the foot                             
feet_size                                    = [-0.07  0.12;  % xMin, xMax
                                                -0.045 0.05]; % yMin, yMax  
                                
% compute contact constraints (friction cone, unilateral constraints)
[ConstraintMatrix_feet, biasVectorConstraint_feet] = wbc.computeRigidContactConstraints(forceFrictionCoefficient, numberOfPoints, ...
                                                                                        torsionalFrictionCoefficient, feet_size, fZmin);

%% FAILURE DETECTION 

% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %

Config.failureDetection.maxRPMError         = 3750; % [N]
Config.failureDetection.timeThr_partFail    = 0.1; % [s]  
Config.failureDetection.size_window_rpm_err = 25;   % [steps]
      
% references from offline optimization
%
% For now we focus on: right arm turbine failure; 
%                      left jetpack turbine failure.
%
Config.USE_OPTIMIZATION_REF_FOR_FAULT_RESPONSE = true;
Config.t_stopRobotDuringFailure                = 7.5;
Config.failureDetection.eps_com_ref            = 0.75;

load('failure_rightArm_optim.mat')
load('failure_leftBack_optim.mat')

Config.jointPos_rightArm_fail     = jointPos_opt_rightArm';
Config.attitude_upd_rightArm_fail = attitude_opt_rightArm';

Config.jointPos_leftBack_fail     = jointPos_opt_leftBack';
Config.attitude_upd_leftBack_fail = attitude_opt_leftBack';

% fault tolerant control parameters
Config.t_updateWeightsDuringFailure     = 5;
Config.failureDetection.eps_thrustBound = 10;

%% Autopilot
Config.AUTOPILOT_ON          = true;
Config.autopilot.timeFailure = 15;
