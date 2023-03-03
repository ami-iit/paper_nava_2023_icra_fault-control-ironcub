%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%              COMMON *JETS* CONFIGURATION PARAMETERS                     %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% JETS MODEL

% ẋ  = F(x) + G(x)*u
% xdot(1,:) = Tdot;
% xdot(2,:) = K_T*T + K_TT*T^2 + K_D*Tdot + ... $x^2+e^{\pi i}$
%             K_DD*Tdot^2 + K_TD*T*Tdot + ...
%            (B_U + B_T*T + B_D*Tdot)*(u + B_UU*u^2) + c;

% Diesel parameters
%                      K_T        K_TT       K_D       K_DD      K_TD      Bᵤ        Bₜ         B_d       Bᵤᵤ        c
% Config.jetC_P100 = [-1.496760, -0.045206, -2.433030, 0.020352, 0.064188, 0.589177, 0.016715, -0.021258, 0.013878, -19.926756];
% Config.jetC_P220 = [-0.482993, -0.013562,  1.292093, 0.055923, 0.006887, 0.130662, 0.022564, -0.052168, 0.004485, -5.436267];

% Kerosene parameters
%                   K_T        K_TT       K_D       K_DD      K_TD      Bᵤ        Bₜ         B_d       Bᵤᵤ        c
Config.jetC_P100 = [0.306530, -0.024867, -0.973422, 0.055786, 0.022618, 0.688710, 0.038025, -0.089879, -0.001393, -0.958953];
Config.jetC_P220 = [0.237755, -0.012799, -0.860374, 0.023606, 0.008599, 1.139026, 0.019914, -0.050257,  0.000302, -0.692913 ];

% Just rewriting the coefficients in a different data structure: it is
% simpler to handle in the fl controller.
Config.jet.coeff = [Config.jetC_P100; ...
                    Config.jetC_P100; ...
                    Config.jetC_P220; ...
                    Config.jetC_P220];

jets_config.coefficients = [Config.jetC_P100; ...
                            Config.jetC_P100; ...
                            Config.jetC_P220; ...
                            Config.jetC_P220];

jets_config.init_thrust  = zeros(4,1);

% jets intial conditions
Config.initT    = 10.0;
Config.initTdot = 0.0;

Config.initialConditions.jets_thrust = 0.0;

% EKF parameters
Config.ekf.initP             = [10, 1; 1, 10] * 1e-1;
Config.ekf.process_noise     = [10, 1; 1, 10] * 1e-1;
Config.ekf.measurement_noise = 10e2 * 1e1; %% faking the jet

% Thrust Gaussian noise, if we need it. Note that I'm assuming that the noise 
% does not affect the system but just the "measurement". If the noise is not 
% zero you should tune the EKF parameters above
Config.thrust_noise          = 0.0;

%% JET CONTROLLER GAINS

Config.jet.u_max  = 100;
Config.jet.u_min  = 0.0;

% dotT jet control
Config.fl.KI = [4, 4, 4, 4] / 2; % * 1e-1 / 2;
Config.fl.KP = [10, 10, 10, 10] / 2;
Config.fl.KD = 1 * sqrt(Config.fl.KP) * 0;

% T jet control
% Config.fl.KP = [1, 1, 1, 1] * 0.25;
% Config.fl.KD = 2 * P)sqrt(Config.fl.K;
% Config.fl.KI = [1, 1, 1, 1] * 0;
