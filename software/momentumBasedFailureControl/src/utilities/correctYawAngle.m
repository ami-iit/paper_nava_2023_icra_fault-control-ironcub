function RPY_Corr = correctYawAngle(rollPitchYaw)

    % correct the Yaw angle to avoid discontinuities in the plots when the
    % robot is spinning along the yaw angle
    persistent yaw_0

    if isempty(yaw_0)
    
        yaw_0 = rollPitchYaw(3);
    end

    if rollPitchYaw(3) > yaw_0 + 180
    
        rollPitchYaw(3) = rollPitchYaw(3) -360;
    
    elseif rollPitchYaw(3) < yaw_0 - 180
    
        rollPitchYaw(3) = rollPitchYaw(3) +360;
    end

    yaw_0    = rollPitchYaw(3);
    RPY_Corr = rollPitchYaw;
end
