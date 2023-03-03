% LOADCOLLISIONPARAMS_MK1_1 loads the parameters for collision detection
%                           between the iRonCub-Mk1_1 links.
%
% Author: Gabriele Nava (gabriele.nava@iit.it)
% Genova: Jul. 2022
%

% list of links with collisions
Config.collisions.framesList = {'l_arm_jet_turbine', 'r_arm_jet_turbine', ...
                                'chest_l_jet_turbine', 'chest_r_jet_turbine' , ...
                                'l_upper_arm', 'r_upper_arm', ...
                                'l_upper_leg', 'r_upper_leg', ...
                                'l_lower_leg', 'r_lower_leg', ...
                                'root_link', 'chest'};

%---------------------------- arm turbines -------------------------------%
Config.collisions.(Config.collisions.framesList{1}).centers  = [0.0  0.0  -0.05;
                                                                0.0  0.0  -0.1;
                                                                0.0  0.0  -0.15;
                                                                0.0  0.0  -0.2; ...
                                                                % jet cones
                                                                0.0  0.0   0.05;
                                                                0.0  0.0   0.1;
                                                                0.0  0.0   0.15;
                                                                0.0  0.0   0.20;
                                                                0.0  0.0   0.25];

Config.collisions.(Config.collisions.framesList{2}).centers  = Config.collisions.(Config.collisions.framesList{1}).centers;

Config.collisions.(Config.collisions.framesList{1}).radiuses = [0.05; 0.055; 0.055; 0.055; ...
                                                                % jet cones
                                                                0.05; 0.06; 0.0725; 0.085; 0.1];

Config.collisions.(Config.collisions.framesList{2}).radiuses = Config.collisions.(Config.collisions.framesList{1}).radiuses;

%---------------------------- jetpack turbines ----------------------------%
Config.collisions.(Config.collisions.framesList{3}).centers  = [0.0  0.0  -0.06;
                                                                0.0  0.0  -0.12;
                                                                0.0  0.0  -0.16;
                                                                0.0  0.0  -0.225; ...
                                                                % jet cones
                                                                0.0  0.0   0.05;
                                                                0.0  0.0   0.1;
                                                                0.0  0.0   0.15;
                                                                0.0  0.0   0.20;
                                                                0.0  0.0   0.25];
 
Config.collisions.(Config.collisions.framesList{4}).centers  = Config.collisions.(Config.collisions.framesList{3}).centers;

Config.collisions.(Config.collisions.framesList{3}).radiuses = [0.05; 0.06; 0.06; 0.06; ...
                                                                % jet cones
                                                                0.05; 0.06; 0.0725; 0.085; 0.1];

Config.collisions.(Config.collisions.framesList{4}).radiuses = Config.collisions.(Config.collisions.framesList{3}).radiuses;

%------------------------------ upper arms -------------------------------%
Config.collisions.(Config.collisions.framesList{5}).centers  = [-0.015  0.0  0.005;
                                                                -0.015  0.0  0.05];

Config.collisions.(Config.collisions.framesList{6}).centers  = Config.collisions.(Config.collisions.framesList{5}).centers;

Config.collisions.(Config.collisions.framesList{5}).radiuses = [0.0475; 0.0475];

Config.collisions.(Config.collisions.framesList{6}).radiuses = Config.collisions.(Config.collisions.framesList{5}).radiuses;

%------------------------------ upper legs -------------------------------%
Config.collisions.(Config.collisions.framesList{7}).centers  = [0.005  0.0   0.01;
                                                                0.01   0.0  -0.06];

Config.collisions.(Config.collisions.framesList{8}).centers  = Config.collisions.(Config.collisions.framesList{7}).centers;

Config.collisions.(Config.collisions.framesList{7}).radiuses = [0.06; 0.06];

Config.collisions.(Config.collisions.framesList{8}).radiuses = Config.collisions.(Config.collisions.framesList{7}).radiuses;

%------------------------------ lower legs -------------------------------%
Config.collisions.(Config.collisions.framesList{9}).centers   = [0.01  0.0 -0.04;
                                                                 0.0   0.0 -0.1;
                                                                 0.0   0.0 -0.14];

Config.collisions.(Config.collisions.framesList{10}).centers  = Config.collisions.(Config.collisions.framesList{9}).centers;

Config.collisions.(Config.collisions.framesList{9}).radiuses  = [0.06; 0.06; 0.06];

Config.collisions.(Config.collisions.framesList{10}).radiuses = Config.collisions.(Config.collisions.framesList{9}).radiuses;

%------------------------------ base link --------------------------------%
Config.collisions.(Config.collisions.framesList{11}).centers  = [0.02   0.05 -0.02;
                                                                 0.02  -0.05 -0.02;
                                                                 0.02   0.0  -0.09];

Config.collisions.(Config.collisions.framesList{11}).radiuses = [0.06; 0.06; 0.065];

%-------------------------------- chest ----------------------------------%
Config.collisions.(Config.collisions.framesList{12}).centers  = [ 0.0    0.04  0.01;
                                                                 -0.045  0.02  -0.1;
                                                                  0.045  0.02  -0.1;
                                                                  0.0    0.1   -0.1];

Config.collisions.(Config.collisions.framesList{12}).radiuses = [0.08; 0.075; 0.075; 0.1];

%-------------------------------------------------------------------------%
% remove jet cone for faulty turbine
switch Config.FAILURE_TYPE

    case 1
        
        Config.collisions.(Config.collisions.framesList{2}).centers  = Config.collisions.(Config.collisions.framesList{2}).centers(1:4,:);
        Config.collisions.(Config.collisions.framesList{2}).radiuses = Config.collisions.(Config.collisions.framesList{2}).radiuses(1:4);
    case 2

        Config.collisions.(Config.collisions.framesList{3}).centers  = Config.collisions.(Config.collisions.framesList{3}).centers(1:4,:);
        Config.collisions.(Config.collisions.framesList{3}).radiuses = Config.collisions.(Config.collisions.framesList{3}).radiuses(1:4);
    otherwise
end
