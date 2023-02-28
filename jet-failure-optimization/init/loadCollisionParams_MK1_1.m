% LOADCOLLISIONPARAMS_MK1_1 loads the parameters for collision detection
%                           between the iRonCub-Mk1_1 links and jet cones.
%
% Author: Gabriele Nava (gabriele.nava@iit.it)
% Genova: Jul. 2022
%

% turn on to verify that the current robot configuration does not contain
% self collisions or collisions with jet cones. WARNING: the solver may
% find intermediate solutions which do not respect constraints, it is fine
Config.DEBUG_COLLISIONS = false;

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
                                                                0.0  0.0  -0.2];

Config.collisions.(Config.collisions.framesList{2}).centers  = Config.collisions.(Config.collisions.framesList{1}).centers;

Config.collisions.(Config.collisions.framesList{1}).radiuses = [0.05; 0.055; 0.055; 0.055];

Config.collisions.(Config.collisions.framesList{2}).radiuses = Config.collisions.(Config.collisions.framesList{1}).radiuses;

%---------------------------- jetpack turbines ----------------------------%
Config.collisions.(Config.collisions.framesList{3}).centers  = [0.0  0.0  -0.06;
                                                                0.0  0.0  -0.12;
                                                                0.0  0.0  -0.16;
                                                                0.0  0.0  -0.225];
 
Config.collisions.(Config.collisions.framesList{4}).centers  = Config.collisions.(Config.collisions.framesList{3}).centers;

Config.collisions.(Config.collisions.framesList{3}).radiuses = [0.05; 0.06; 0.06; 0.06];

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

Config.collisions.(Config.collisions.framesList{12}).radiuses = [0.08; 0.06; 0.06; 0.095];

%-------------------------------------------------------------------------%
%--------------------- JET CONES APPROXIMATION WITH LINES ----------------%
%-------------------------------------------------------------------------%

Config.cones.framesList = {'l_arm_jet_turbine', 'r_arm_jet_turbine', ...
                           'chest_l_jet_turbine', 'chest_r_jet_turbine'};

% cone lines rotations w.r.t. jet frame. The cone is approximated with 7
% lines spanning from 0 to 2*pi. The parameter alpha indicates how large is
% the cone. The parameter r indicates the positions of the points on the
% lines w.r.t. the turbine frame origin
alpha = 10;
r     = 0.025;
R_1   = wbc.rotationFromRollPitchYaw([alpha*cos(0),alpha*sin(0),0]*pi/180);
R_2   = wbc.rotationFromRollPitchYaw([alpha*cos(pi/4),alpha*sin(pi/4),0]*pi/180);
R_3   = wbc.rotationFromRollPitchYaw([alpha*cos(pi/2),alpha*sin(pi/2),0]*pi/180);
R_4   = wbc.rotationFromRollPitchYaw([alpha*cos(3*pi/4),alpha*sin(3*pi/4),0]*pi/180);
R_5   = wbc.rotationFromRollPitchYaw([alpha*cos(pi),alpha*sin(pi),0]*pi/180);
R_6   = wbc.rotationFromRollPitchYaw([alpha*cos(5*pi/4),alpha*sin(5*pi/4),0]*pi/180);
R_7   = wbc.rotationFromRollPitchYaw([alpha*cos(3*pi/2),alpha*sin(3*pi/2),0]*pi/180);
R_8   = wbc.rotationFromRollPitchYaw([alpha*cos(7*pi/4),alpha*sin(7*pi/4),0]*pi/180);

%---------------------------- arm turbines -------------------------------%
Config.cones.(Config.cones.framesList{1}).offsets = [r*sin(pi),     r*cos(pi),     0;
                                                     r*sin(3*pi/4), r*cos(3*pi/4), 0;
                                                     r*sin(pi/2),   r*cos(pi/2),   0;
                                                     r*sin(pi/4),   r*cos(pi/4),   0;
                                                     r*sin(0),      r*cos(0),      0;
                                                     r*sin(7*pi/4), r*cos(7*pi/4), 0;
                                                     r*sin(3*pi/2), r*cos(3*pi/2), 0;
                                                     r*sin(5*pi/4), r*cos(5*pi/4), 0];

Config.cones.(Config.cones.framesList{2}).offsets = Config.cones.(Config.cones.framesList{1}).offsets;

Config.cones.(Config.cones.framesList{1}).axes    = -[R_1(1:3,3)';
                                                      R_2(1:3,3)';
                                                      R_3(1:3,3)';
                                                      R_4(1:3,3)';
                                                      R_5(1:3,3)';
                                                      R_6(1:3,3)';
                                                      R_7(1:3,3)';
                                                      R_8(1:3,3)'];

Config.cones.(Config.cones.framesList{2}).axes    = Config.cones.(Config.cones.framesList{1}).axes;

%--------------------------- back turbines -------------------------------%
Config.cones.(Config.cones.framesList{3}).offsets = [r*sin(pi),     r*cos(pi),     0;
                                                     r*sin(3*pi/4), r*cos(3*pi/4), 0;
                                                     r*sin(pi/2),   r*cos(pi/2),   0;
                                                     r*sin(pi/4),   r*cos(pi/4),   0;
                                                     r*sin(0),      r*cos(0),      0;
                                                     r*sin(7*pi/4), r*cos(7*pi/4), 0;
                                                     r*sin(3*pi/2), r*cos(3*pi/2), 0;
                                                     r*sin(5*pi/4), r*cos(5*pi/4), 0];

Config.cones.(Config.cones.framesList{4}).offsets = Config.cones.(Config.cones.framesList{3}).offsets;

Config.cones.(Config.cones.framesList{3}).axes    = -[R_1(1:3,3)';
                                                      R_2(1:3,3)';
                                                      R_3(1:3,3)';
                                                      R_4(1:3,3)';
                                                      R_5(1:3,3)';
                                                      R_6(1:3,3)';
                                                      R_7(1:3,3)';
                                                      R_8(1:3,3)'];

Config.cones.(Config.cones.framesList{4}).axes    = Config.cones.(Config.cones.framesList{3}).axes;
