function [rpm] = T_to_rpm(Thrust,Config)

% The structure of thrust (T) to RPM (w) model is:
%  w = a*(T/alpha_air)^b + c*(T/alpha_air)
%
% This model is defined for each turbine. So each turbine has a different
% set of coefficients.
%
% The coefficients are defined in the following way:
%
% jet_T2rpm_XX = [a, b, c];
%
% where XX = LA or RA or LB or RB corresponding to Left Arm, Right Arm,
% Left Back or Right Back respectively
%
% UNITS: 
% angular speed (w) --> kiloRPM
% thrust (T) --> newtons (N)
%
a   = zeros(4,1);
b   = zeros(4,1);
c   = zeros(4,1);
rpm = zeros(4,1);

a(1) = Config.jet_T2rpm_LA(1); 
b(1) = Config.jet_T2rpm_LA(2);
c(1) = Config.jet_T2rpm_LA(3);

a(2) = Config.jet_T2rpm_RA(1); 
b(2) = Config.jet_T2rpm_RA(2);
c(2) = Config.jet_T2rpm_RA(3);

a(3) = Config.jet_T2rpm_LB(1); 
b(3) = Config.jet_T2rpm_LB(2);
c(3) = Config.jet_T2rpm_LB(3);

a(4) = Config.jet_T2rpm_RB(1); 
b(4) = Config.jet_T2rpm_RB(2);
c(4) = Config.jet_T2rpm_RB(3);

Thrust = Thrust./Config.alpha_air;

for ii=1:4

    if Thrust(ii) > 0
    
        rpm(ii) = a(ii)*Thrust(ii).^b(ii) + c(ii)*Thrust(ii);
    end
end

end
