% INITJETFAILUREOPTIMIZATION_MK1_1 initialization for the jet failure 
%                                  optimization. This file is for the robot 
%                                  iRonCub-Mk1_1.
%
% Author: Gabriele Nava (gabriele.nava@iit.it)
% Genova: Jun. 2022
%

% FAILURE TYPE:
%
% 0 - no failure
% 1 - failure on the right arm turbine
% 2 - failure on the left back turbine
%
Config.FAILURE_TYPE = 1;

% re-run simulation with optimized initial conditions. The new initial 
% conditions are saved from previous simulations
Config.RERUN_WITH_BETTER_INIT_COND = true;

% specify the list of joints that are going to be considered in the reduced model
Config.model.jointList = {'torso_pitch','torso_roll','torso_yaw', ...
                          'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
                          'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
                          'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
                          'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};
        
% select the link that will be used as base link
Config.model.baseLinkName = 'root_link';

% model name and path (hard-coded for the moment)
ironcub_software_dir   = getenv('IRONCUB_SOFTWARE_SOURCE_DIR');
Config.meshesPath      = [ironcub_software_dir, '/models/'];
Config.model.modelName = 'model_stl.urdf';
Config.model.modelPath = [ironcub_software_dir, '/models/iRonCub-Mk1_1/iRonCub/robots/iRonCub-Mk1_1/'];
DEBUG                  = false;

% generate the TurbinesData structure
Config.turbinesData.turbineList = {'l_arm_jet_turbine','r_arm_jet_turbine','chest_l_jet_turbine','chest_r_jet_turbine'};
Config.turbinesData.turbineAxis = [-3; -3; -3; -3];
Config.turbinesData.njets       = length(Config.turbinesData.turbineList);

% set the initial robot position and velocity [deg] and gravity vector
torso_Position     = [ 10  0  0];                 
left_arm_Position  = [-10 25 15 18];           
right_arm_Position = [-10 25 15 18];                
left_leg_Position  = [  5  0  0  0  0  0];
right_leg_Position = [  5  0  0  0  0  0]; 

Config.initCond.jointPos_init = [torso_Position';left_arm_Position';right_arm_Position';left_leg_Position';right_leg_Position']*pi/180;
Config.gravityAcc             = [0;0;-9.81];
Config.ndof                   = length(Config.initCond.jointPos_init);

% initial jets intensities
switch Config.FAILURE_TYPE

    case 1

        Config.initCond.jetIntensities_init = [95; 0; 120; 120]; % [N]
    case 2

        Config.initCond.jetIntensities_init = [95; 95; 0; 120]; % [N]
    otherwise

        Config.initCond.jetIntensities_init = [95; 95; 120; 120]; % [N]
end

% initial base pose w.r.t. the world frame
Config.initCond.w_H_b_init = eye(4); 

% load the reduced model
KinDynModel = iDynTreeWrappers.loadReducedModel(Config.model.jointList, Config.model.baseLinkName, ...
                                                Config.model.modelPath, Config.model.modelName, DEBUG); 

% set the initial robot position (no w_H_b set yet)
iDynTreeWrappers.setRobotState(KinDynModel, Config.initCond.w_H_b_init, Config.initCond.jointPos_init, ...
                               zeros(6,1), zeros(Config.ndof,1), Config.gravityAcc);

% set the w_H_b transformation, assuming that the world frame is attached to the robot left foot                           
Config.initCond.w_H_b_init = iDynTreeWrappers.getRelativeTransform(KinDynModel,'l_sole','root_link');

% set the initial robot state (w_H_b is now correct)
iDynTreeWrappers.setRobotState(KinDynModel, Config.initCond.w_H_b_init, Config.initCond.jointPos_init, ...
                               zeros(6,1), zeros(Config.ndof,1), Config.gravityAcc);

%-------------------------------------------------------------------------%

% weights on controllability task
Config.opti.WeightControllability = 1;

% tolerances on inequality constraints
Config.opti.momentumNormTol = 0.01;

% joints limits. NOTE: the joints order is hard-coded.
jointPositionLimits = [-20,  70; -30,  30;  -50,  50; ...                                %torso
                       -90,  10;  0,   160; -35,  80;  17,  105; ...                     %larm
                       -90,  10;  0,   160; -35,  80;  17,  105; ...                     %rarm
                       -35,  80; -15,  90;  -70,  70; -100, 0;  -30,  30; -20,  20; ...  %rleg  
                       -35,  80; -15,  90;  -70,  70; -100, 0;  -30,  30; -20,  20];     %lleg

lowerBoundJointPos  = jointPositionLimits(:,1)*pi/180;
upperBoundJointPos  = jointPositionLimits(:,2)*pi/180;

% scaling joints limits
[lowerBoundJointPos, upperBoundJointPos] = scaleJointsPositionLimits(lowerBoundJointPos, upperBoundJointPos, Config);

% limits for the base position and rotation
basePosDelta = 0.1;       %[m]
baseRotDelta = 50*pi/180; %[rad]

upperBoundBasePos = Config.initCond.w_H_b_init(1:3,4) + basePosDelta.*ones(3,1); %[m]
lowerBoundBasePos = Config.initCond.w_H_b_init(1:3,4) - basePosDelta.*ones(3,1); %[m]
upperBoundBaseRot = wbc.rollPitchYawFromRotation(Config.initCond.w_H_b_init(1:3,1:3)) + baseRotDelta.*ones(3,1); %[rad]
lowerBoundBaseRot = wbc.rollPitchYawFromRotation(Config.initCond.w_H_b_init(1:3,1:3)) - baseRotDelta.*ones(3,1); %[rad]

% enlarge bound on base yaw
upperBoundBaseRot(3) =  pi + pi/10;
lowerBoundBaseRot(3) =  pi - pi/10;

% turbines bounds
switch Config.FAILURE_TYPE

    case 1

        upperBoundTurbines = [160; 0; 220; 220]; 
    case 2

        upperBoundTurbines = [160; 160; 0; 220]; 
    otherwise

        upperBoundTurbines = [160; 160; 220; 220]; 
end

lowerBoundTurbines = zeros(Config.turbinesData.njets,1);

Config.opti.upperBound = [upperBoundBasePos; upperBoundBaseRot; upperBoundJointPos; upperBoundTurbines];
Config.opti.lowerBound = [lowerBoundBasePos; lowerBoundBaseRot; lowerBoundJointPos; lowerBoundTurbines];

%-------------------------------------------------------------------------%
% overwrite initial conditions with whose of previous step
if Config.RERUN_WITH_BETTER_INIT_COND

    switch Config.FAILURE_TYPE

        case 1
            load('uStar_prev_arm_fail')
            Config.initCond.w_H_b_init          = wbc.fromPosRpyToTransfMatrix(uStar(1:6));
            Config.initCond.jointPos_init       = uStar(7:23+6);        
            Config.initCond.jetIntensities_init = uStar(23+7:end);    
        case 2
            load('uStar_prev_back_fail')
            Config.initCond.w_H_b_init          = wbc.fromPosRpyToTransfMatrix(uStar(1:6));
            Config.initCond.jointPos_init       = uStar(7:23+6);        
            Config.initCond.jetIntensities_init = uStar(23+7:end);
    end
end

%-------------------------------------------------------------------------%
% set options of the nonlinear optimization

% fmincon properties:
%                     Algorithm: 'interior-point'
%            BarrierParamUpdate: 'monotone'
%                CheckGradients: 0
%           ConstraintTolerance: 1.0000e-06
%                       Display: 'final'
%         EnableFeasibilityMode: 0
%      FiniteDifferenceStepSize: 'sqrt(eps)'
%          FiniteDifferenceType: 'forward'
%          HessianApproximation: 'bfgs'
%                    HessianFcn: []
%            HessianMultiplyFcn: []
%                   HonorBounds: 1
%        MaxFunctionEvaluations: 3000
%                 MaxIterations: 1000
%                ObjectiveLimit: -1.0000e+20
%           OptimalityTolerance:  1.0000e-06
%                     OutputFcn: []
%                       PlotFcn: []
%                  ScaleProblem: 0
%     SpecifyConstraintGradient: 0
%      SpecifyObjectiveGradient: 0
%                 StepTolerance: 1.0000e-10
%           SubproblemAlgorithm: 'factorization'
%                      TypicalX: 'ones(numberOfVariables,1)'
%                   UseParallel: 0
%
Config.fminconOptions = optimoptions('fmincon', 'algorithm', 'interior-point','MaxFunEvals',7000,'MaxIter',7000,'SpecifyConstraintGradient',true, ...
                                     'ConstraintTolerance', 1e-10, 'OptimalityTolerance', 1e-10, 'StepTolerance', 1e-10, ...
                                     'Display', 'final-detailed', 'PlotFcn', {'optimplotfvalconstr','optimplotconstrviolation'}, ...
                                     'CheckGradients', false, 'ScaleProblem', false, 'FiniteDifferenceType', 'forward', 'SpecifyObjectiveGradient', false);
