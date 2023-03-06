%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%              COMMON *JETS* CONFIGURATION PARAMETERS                     %
%                                                                         %
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Jets Dynamics Model
%
% ẋ  = F(x) + G(x)*u
%
% xdot(1,:) = Tdot;
%
% xdot(2,:) = K_T*T + K_TT*T^2 + K_D*Tdot + ... $x^2+e^{\pi i}$
%             K_DD*Tdot^2 + K_TD*T*Tdot + ...
%            (B_U + B_T*T + B_D*Tdot)*(u + B_UU*u^2) + c;
%

%% TODO: Hard Code or import?

% Diesel parameters
%                      K_T        K_TT       K_D       K_DD      K_TD      Bᵤ        Bₜ         B_d       Bᵤᵤ        c
% Config.jetC_P100 = [-1.496760, -0.045206, -2.433030, 0.020352, 0.064188, 0.589177, 0.016715, -0.021258, 0.013878, -19.926756];
% Config.jetC_P220 = [-0.482993, -0.013562,  1.292093, 0.055923, 0.006887, 0.130662, 0.022564, -0.052168, 0.004485, -5.436267];

% Kerosene parameters
%                    K_T        K_TT       K_D       K_DD       K_TD      Bᵤ        Bₜ         B_d       Bᵤᵤ        c
Config.jetC_P100 = [0.306530, -0.024867, -0.973422, 0.055786, 0.022618, 0.688710, 0.038025, -0.089879, -0.001393, -0.958953];
Config.jetC_P220 = [0.237755, -0.012799, -0.860374, 0.023606, 0.008599, 1.139026, 0.019914, -0.050257,  0.000302, -0.692913 ];
                  
% Just rewriting the coefficients in a different data structure: it is
% simpler to handle in the fl controller.
%
Config.jet.coeff         = [Config.jetC_P100; ...
                            Config.jetC_P100; ...
                            Config.jetC_P220; ...
                            Config.jetC_P220];
 
jets_config.coefficients = [Config.jetC_P100; ...
                            Config.jetC_P100; ...
                            Config.jetC_P220; ...
                            Config.jetC_P220];

% jets intial conditions
jets_config.init_thrust              = zeros(4,1);
Config.initT                         = 10.0;
Config.initTdot                      = 0.0;
Config.initialConditions.jets_thrust = 0.0;

% If TRUE, the thrust rate of change for jet control is estimated by
% relying on a dedicated EKF, and not taken from the momentum-based 
% EKF-thrust estimator
Config.TDot_for_jetControl_estimatedInternally = false;

% Jet control internal EKF parameters
Config.ekf.initP             = [10, 1; 1, 10] * 1e-1;
Config.ekf.process_noise     = [10, 1; 1, 10] * 1e-1;
Config.ekf.measurement_noise = 10e2 * 1e1;

% Thrust Gaussian noise, if we need it
%
% Note that I'm assuming that the noise does not affect the system but just 
% the "measurement". If the noise is not zero you should tune the EKF 
% parameters above
Config.thrust_noise = 0.0;

%% Gains for Jet Control

% T jet control
Config.fl.KP = [1, 1, 1, 1] * 50 ;
Config.fl.KD = 2 * sqrt(Config.fl.KP);
Config.fl.KI = [1, 1, 1, 1] * 0;

% Saturation on throttle
Config.jet.u_max = 120;
Config.jet.u_min = 0;

%% Parameters for thrust-to-RPM static model

Config.jet_T2rpm_LA = [18.66, 0.4053, -0.1373]; % P160-4243 - Identified on 10-Mar-2022
Config.jet_T2rpm_RA = [18.66, 0.4053, -0.1373]; % P160-4243 - Identified on 10-Mar-2022
Config.jet_T2rpm_LB = [14.41, 0.4304, -0.1330]; % P220-689  - Identified on 10-Mar-2022
Config.jet_T2rpm_RB = [14.41, 0.4304, -0.1330]; % P220-689  - Identified on 10-Mar-2022

Config.P_air     = 101185;    % Pa
Config.T_air     = 15+273.15; % Kelvin
Config.P_ref     = 101325;    % Pa
Config.T_ref     = 15+273.15; % Kelvin
Config.alpha_air = (Config.P_air/Config.T_air)/(Config.P_ref/Config.T_ref);
