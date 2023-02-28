% RUNJETFAILUREOPTIMIZATION runs an algorithm to optimize joints position
%                           and jets intensities in case of a turbine failure.
%
% The optimal configuration is calculated by solving a nonlinear
% minimization problem with fmincon.
%
% Author: Gabriele Nava (gabriele.nava@iit.it)
% Genova, Jun 2022
%

clear variables
close all
clc

% add path to local functions
addpath(genpath('./src'))

% run the scripts for initializing the robot and load collisions params.
run('./init/initJetFailureOptimization_MK1_1.m');
run('./init/loadCollisionParams_MK1_1.m');

%-------------------------------------------------------------------------%

% the variables to be optimized are collected in a vector as follows:
%
%   u = [basePos; baseRot; jointPos; jetIntensities]
%
% baseRot is parametrized using Euler angles (roll-pitch-yaw)

% initial conditions for the optimization problem
basePos_init = Config.initCond.w_H_b_init(1:3,4);
w_R_b_init   = Config.initCond.w_H_b_init(1:3,1:3);
rpy_init     = wbc.rollPitchYawFromRotation(w_R_b_init);
Config.uInit = [basePos_init; rpy_init; Config.initCond.jointPos_init; Config.initCond.jetIntensities_init];

% compute the nonlinear constraints function
nonLinearConstraints = @(u) computeNonLinearConstraints(u, KinDynModel, Config);

% compute the cost function
costFunction = @(u) computeCostFunction(u, KinDynModel, Config);

%-------------------------------------------------------------------------%
% run nonlinear optimization
disp('[runJetFailureOptimization]: running optimization...')

tic;
[uStar, fval, exitflag, output] = fmincon(costFunction, Config.uInit, [], [], [], [], Config.opti.lowerBound, ...
                                          Config.opti.upperBound, nonLinearConstraints, Config.fminconOptions);

timeSim = toc;
%-------------------------------------------------------------------------%

% Dislpay results

% check 1: compare the momentum derivatives (initial, final)
initialMomentumDerivative = computeMomentumDerivativeAndGradient(Config.uInit, KinDynModel, Config);
finalMomentumDerivative   = computeMomentumDerivativeAndGradient(uStar, KinDynModel, Config);

disp('Momentum derivative: [initial, final]')
disp(num2str([initialMomentumDerivative, finalMomentumDerivative]))

% check 2: display the initial and optimized joints position
disp(' ')
disp('Joints position: [initial, optimized]')
disp(num2str([Config.uInit(7:6+length(Config.initCond.jointPos_init)), uStar(7:6+length(Config.initCond.jointPos_init))]*180/pi))

% check 3: display the initial and optimized thrusts intensities
disp(' ')
disp('Thrust intensities: [initial, optimized]')
disp(num2str([Config.uInit(end-Config.turbinesData.njets+1:end), uStar(end-Config.turbinesData.njets+1:end)]))

% check 4: display the initial and optimized base pose
disp(' ')
disp('Base Pose: [initial, optimized]')
disp(num2str([[Config.uInit(1:3); Config.uInit(4:6)*180/pi], [uStar(1:3); uStar(4:6)*180/pi]]))

% optimal pose visualization
disp('[runJetFailureOptimization]: visualize the robot pose...')
visualizeRobot(uStar, KinDynModel, Config)

% remove local paths
rmpath(genpath('./src'))
disp('[runJetFailureOptimization]: optimization exited correctly.')
