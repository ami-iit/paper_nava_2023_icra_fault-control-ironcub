function [jetsIntensities_des, jointPos_des] = onlineFlightPlannerJoints_and_Thrust(accCoM_des, m, jointPos_0, Aj0, Config)

% Thrust intensities references for the postural task on TDot
f_grav              = m*Config.GRAVITY_ACC*[0; 0; 1; zeros(3,1)];
LDot_des            = [m*accCoM_des; zeros(3,1)];
jetsIntensities_des = Aj0\(LDot_des + f_grav);

% Joint position references
jointPos_des        = jointPos_0;

end
