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
addpath('../controlAndDataGui/');

% Simulation time and delta_t [s]
Config.simulationTime                   = 45; %inf;
Config.tStep                            = 0.01;

%% SIMULATION SETTINGS

% If TRUE, jet dynamics is included in the controller. WARNING: if the
% controller is interfaced with Gazebo simulator, also Gazebo needs 
% to model the jet dynamics. Set also the cmake option 
% USE_NONLINEAR_JET_DYNAMICS_PLUGIN to TRUE in `component_ironcub` 
Config.USE_JET_DYNAMICS                 = false;

% If TRUE, the thrusts used by the controller are estimated via Extended
% Kalman Filter. WARNING: it requires Config.USE_JET_DYNAMICS to be TRUE
Config.USE_THRUST_ESTIMATOR             = false;

% Update thrust estimation option in case jets dynamics is FALSE
if ~Config.USE_JET_DYNAMICS && Config.USE_THRUST_ESTIMATOR
    
    Config.USE_THRUST_ESTIMATOR = false;
    warning('USE_THRUST_ESTIMATOR is TRUE but USE_JET_DYNAMICS is FALSE. Setting USE_THRUST_ESTIMATOR to FALSE.')   
end

% Controller type: native GUI or joystick
Config.USE_NATIVE_GUI                   = false;
Config.USE_FLIGHT_DATA_GUI              = false;

% Control type:
%
% Default controller => MOMENTUM BASED CONTROL WITH LYAPUNOV STABILITY (IEEE-RAL)
% If USE_ATTITUDE_CONTROL = true => LINEAR MOMENTUM AND ATTITUDE CONTROL (IEEE-HUMANOIDS)
%
Config.USE_ATTITUDE_CONTROL             = true;

% If Config.INCLUDE_THRUST_LIMITS and/or Config.INCLUDE_JOINTS_LIMITS are
% set to true, the thrusts limits and/or the joints limits are included in
% the control algorithm (as QP constraints)
Config.INCLUDE_THRUST_LIMITS            = true;
Config.INCLUDE_JOINTS_LIMITS            = true;

% Activate visualization and data collection
Config.SCOPE_JOINTS                     = true;
Config.SCOPE_QP                         = true;
Config.SCOPE_COM                        = true;
Config.SCOPE_BASE                       = true;
Config.SCOPE_MOMENTUM                   = true;
Config.SCOPE_JETS                       = true;
Config.SCOPES_WRENCHES                  = true;
Config.SCOPE_GAINS_AND_STATE_MACHINE    = true;

% Save data on the workspace after the simulation
Config.SAVE_WORKSPACE                   = true;

%% ADD CONFIGURATION FILES

% Run robot-specific and controller-specific configuration parameters
run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/configRobot.m')); 
run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/configJets.m')); 
run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/gainsAndParameters.m'));

% open the native GUI for control (if no joystick is present)
if Config.USE_NATIVE_GUI
    
    ironcubControlGui;
end

% Open flight data GUI
if Config.USE_FLIGHT_DATA_GUI
    
    flightGui = flightDataGui;
end
