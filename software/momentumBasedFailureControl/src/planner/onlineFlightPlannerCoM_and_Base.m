function [posCoM_des, accCoM_gravity_ref, rot_vel_acc_jerk_base_des] = onlineFlightPlannerCoM_and_Base(time, posCoM_0, w_R_b_0, Config)

% CoM position, acceleration and base rotation references
posCoM_des                = posCoM_0;
accCoM_gravity_ref        = zeros(3,1);
rot_vel_acc_jerk_base_des = [w_R_b_0, zeros(3,3)];

% Set the reference CoM acceleration to be the gravity acceleration. This is
% necessary as the contact forces are not included in the controller during
% the balancing phase
accCoM_gravity_ref(3)     = -(1-Config.planner.alpha0)*Config.GRAVITY_ACC;

if time > Config.planner.timeStartTakeOff && time < Config.planner.timeStartLanding

    % When the time to take off has come, remove the gravity. A minimum
    % jerk trajectory generator will smooth the CoM acceleration reference
    % to achieve the zero gravity only after the user-defined settling time
    accCoM_gravity_ref = zeros(3,1);
    
    if time > Config.planner.timeToReachHovering && time < Config.planner.timeStartLanding
        
        % When the robot hovers, we update the CoM references to reach the
        % desired hovering height
        posCoM_des = posCoM_0 + [0; 0; Config.planner.z_CoM_des];
    end        
end
end
