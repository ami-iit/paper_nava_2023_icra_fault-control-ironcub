%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2018 
%  * @author: Daniele Pucci & Gabriele Nava
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  */
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear variables
close all
clc

%% GENERAL SIMULATION INFO

% Set path to the utility functions and to WBC library
import wbc.*
addpath(genpath('./src/'));

% Simulation time and delta_t [s]
Config.simulationTime                    = inf;
Config.tStep                             = 0.01;

%% SIMULATION SETTINGS

% If TRUE, jet dynamics is included in the controller. WARNING: if the
% controller is interfaced with Gazebo simulator, also Gazebo needs 
% to model the jet dynamics. Set also the cmake option 
% USE_NONLINEAR_JET_DYNAMICS_PLUGIN to TRUE in `component_ironcub` 
Config.USE_JET_DYNAMICS                 = false;

% If TRUE, the thrusts used by the controller are estimated via Extended
% Kalman Filter. WARNING: it requires Config.USE_JET_DYNAMICS to be TRUE
Config.USE_THRUST_ESTIMATOR             = false;

% ----------------------------------------------------------------------- %
% Compatibility with real robot controller

% if true, the controller is activated with jets OFF (leave it false)
Config.BALANCING_NO_JETS                = false;
% ----------------------------------------------------------------------- %

% Update thrust estimation option in case jets dynamics is FALSE
if ~Config.USE_JET_DYNAMICS && Config.USE_THRUST_ESTIMATOR
    
    Config.USE_THRUST_ESTIMATOR = false;
    warning('USE_THRUST_ESTIMATOR is TRUE but USE_JET_DYNAMICS is FALSE. Setting USE_THRUST_ESTIMATOR to FALSE.')   
end

% Save data on the workspace after the simulation
Config.SAVE_WORKSPACE                  = false;

%% ADD CONFIGURATION FILES

% Run robot-specific and controller-specific configuration parameters
run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/configRobotAndJets.m')); 
run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/configFlightControl.m'));
run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/configJetControl.m'));
