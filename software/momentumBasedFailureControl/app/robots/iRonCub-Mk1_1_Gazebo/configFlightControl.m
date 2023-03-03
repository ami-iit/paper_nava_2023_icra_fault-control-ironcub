%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %  
%                 PARAMETERS FOR THE FLIGHT CONTROLLER                    %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% UTILITY VARIABLES

% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %

% degrees of freedom distribution and gravity (hard-coded for now)
Config.N_DOF          = 23;
Config.N_JETS         = 4;
Config.N_TORSO        = 3; 
Config.N_ARMS         = 8;
Config.N_LEGS         = 12; 
Config.N_DOF_MATRIX   = eye(Config.N_DOF);
Config.N_JETS_MATRIX  = eye(Config.N_JETS);
Config.N_TORSO_MATRIX = eye(Config.N_TORSO);
Config.N_ARMS_MATRIX  = eye(Config.N_ARMS);
Config.N_LEGS_MATRIX  = eye(Config.N_LEGS);
Config.GRAVITY_ACC    = 9.81;

%% ONLINE PLANNER

% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %

% take off, hovering and landing timings [s]
Config.planner.timeStartTakeOff    = 5;
Config.planner.timeToReachHovering = 20;
Config.planner.timeStartLanding    = Config.planner.timeToReachHovering + 20;

% distance from the ground to reach during the hovering phase
Config.planner.z_CoM_des           = 0.2; % [m]
Config.planner.smoothingTimeCoMRef = 5;   % [s]

% minimal vertical force at each foot to consider the robot in contact [N]
Config.minVerticalForces           = 3.5;
Config.planner.alpha0              = 0;

% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %

% COMPUTATION OF COM POSITION ERROR

% Default: all options FALSE. CoM position error estimated via kinematics.

% If this option is TRUE, CoM position error estimated via integration of
% the CoM velocity error
Config.COM_POS_ERROR_FROM_VELOCITY_INTEGRATION = false;

% If this option is TRUE (or both options are TRUE), CoM position error 
% estimated via integration of the CoM velocity error but only when feet
% are not in contact
Config.COM_POS_ERROR_FROM_VELOCITY_INTEGRATION_ONLY_HOVERING = false;

%% CONTROLLER GAINS

% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %

% Linear momentum gains 
Config.gains.momentum.KP_linear  = [15, 15, 15];                             
Config.gains.momentum.KD_linear  = 2 * sqrt(Config.gains.momentum.KP_linear)/10;
Config.gains.momentum.KO         = 10 .* eye(6); 

% Angular momentum gains
Config.gains.momentum.KP_angular = 50 .* ones(1,3);                            
Config.gains.momentum.KD_angular = 125 .* ones(1,3)/2;

% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %

% Postural task gains: joints                               
Config.gains.postural.KP         = [0.75 1.25 1.25 ...                % torso
                                    1 1 1 1 ...                       % left arm
                                    1 1 1 1 ...                       % right arm
                                    0.75 0.75 0.75 0.75 0.75 0.75 ... % left leg
                                    0.75 0.75 0.75 0.75 0.75 0.75];   % right leg  

% Postural task gains: jets intensities
Config.gains.postural.KP_jets    = 0.1;

%% QP WEIGHTS AND THRESHOLDS

% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %

% QP TASKS

% weights for the QP tasks
Config.weightsQP.postural_torso = 1;
Config.weightsQP.postural_arms  = 1;
Config.weightsQP.postural_legs  = 1;
Config.weightsQP.linMom_xy      = 1;
Config.weightsQP.linMom_z       = 1;
Config.weightsQP.angMom         = 1;
Config.weightsQP.symmThrust     = 0.0;
Config.weightsQP.postural_TDot  = 0.0;

% thrusts symmetry task
Config.tasksQP.symChestThrusts        = false;
Config.tasksQP.symArmsThrusts         = false;
Config.tasksQP.symArmsAndChestThrusts = true;
Config.tasksQP.symLeftAndRightThrusts = false;
Config.tasksQP.symAllThrusts          = false;

% QP Hessian regularization
Config.tasksQP.reg_HessianQP          = 1e-4;  

% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %

% QP BOUNDARIES

Config.inequalitiesQP.INCLUDE_THRUST_LIMITS = true;
Config.inequalitiesQP.INCLUDE_JOINTS_LIMITS = true;
Config.inequalitiesQP.eps_thrust_limit      = 0.1;
Config.inequalitiesQP.eps_joint_limit       = 5;

% limits on T, TDot, jointPos, jointVel
Config.inequalitiesQP.maxJetsIntVar         = [10; 10; 10; 10];
Config.inequalitiesQP.maxJetsInt            = [160; 160; 220; 220];
Config.inequalitiesQP.maxJointVelDes        = 20 .* pi/180 .* ones(Config.N_DOF,1);
Config.inequalitiesQP.idleJetsInt           = [0; 0; 0; 0];

% joints limits list. Limits are scaled by a safety range (smaller than the
% normal joints limit range)
scaleTorsoJointsLimits = 0.7;
scaleArmsJointsLimits  = 1;
scaleLegsJointsLimits  = 0.75;

torsoJointsLimit       = scaleTorsoJointsLimits * [15, 45; -20, 20; -20, 20];
armsJointsLimit        = scaleArmsJointsLimits * [-90, 10; 0, 160; -30, 80; 15-0.1, 100];
legsJointsLimit        = scaleLegsJointsLimits * [-35, 80; 5, 90; -70, 70; -35, 0+0.5; -30, 30; -20, 20];

Config.inequalitiesQP.jointPositionLimits = [torsoJointsLimit;          % torso
                                             armsJointsLimit;           % larm
                                             armsJointsLimit;           % rarm
                                             legsJointsLimit;           % lleg
                                             legsJointsLimit] * pi/180; % rleg
                              
% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %

% QP EQUALITY CONSTRAINTS

% tolerances on equality constraints
Config.equalitiesQP.eps_legs          = 0.1*pi/180;
Config.equalitiesQP.eps_angMom        = 0.1;
Config.equalitiesQP.eps_linMom_xy     = 0.1;
Config.equalitiesQP.eps_angMom_constr = 0.01;

%% SATURATIONS OUTSIDE THE QP 

% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %

% if TRUE, QP output is saturated
Config.FILTER_SDOT_AFTER_QP                 = false;
Config.FILTER_TDOT_AFTER_QP                 = false;

% LowPass filter cutoff frequency [Hz]
Config.filter.cutOffFrequency               = 30;

% DeadZone threshold for numerical integration
Config.deadzone.jetsIntensitiesDot_deadzone = 0.1;
Config.deadzone.jointVel_deadzone           = 0.1*pi/180;
Config.deadzone.velCoM_deadzone             = 1e-3;

% Saturations
Config.saturation.jointPositionLimits       = Config.inequalitiesQP.jointPositionLimits;
Config.saturation.maxJetsIntVar             = Config.inequalitiesQP.maxJetsIntVar;
Config.saturation.maxJetsInt                = Config.inequalitiesQP.maxJetsInt;        
Config.saturation.maxJointVelDes            = Config.inequalitiesQP.maxJointVelDes;   
Config.saturation.idleJetsInt               = Config.inequalitiesQP.idleJetsInt;   

%% FAILURE DETECTION 

% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %

Config.failureDetection.maxThrustError    = 5;   % [N]
Config.failureDetection.timeThr_drop      = 0.5; % [s] 
Config.failureDetection.timeThr_partFail  = 2.5; % [s]  
Config.failureDetection.size_window_T_err = 100; % [steps]
