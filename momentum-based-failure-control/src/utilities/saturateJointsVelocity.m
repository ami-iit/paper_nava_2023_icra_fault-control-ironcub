function jointVel = saturateJointsVelocity(jointPos, jointVel_des, JetState, Config)
    
    maxJointVel = Config.saturation.maxJointVelDes;
    minJointVel = -Config.saturation.maxJointVelDes;
    minJoint = Config.saturation.jointPositionLimits(:,1);
    maxJoint = Config.saturation.jointPositionLimits(:,2);
    
    alpha = 50;
    
    minJointSaturation = min((minJoint - jointPos)/(alpha * Config.tStep), zeros(Config.N_DOF,1)); 
    maxJointSaturation = max((maxJoint - jointPos)/(alpha * Config.tStep), zeros(Config.N_DOF,1));
    disp(minJointSaturation(7))
    disp(maxJointSaturation(7))
    
    jointVel = zeros(Config.N_DOF,1);
    
    fprintf('JetState %f\n', JetState(1))
    if all(round(JetState) == 11) % if all the jets are ok
        disp('Jets ok')
        for i=1:Config.N_DOF
%             jointVel(i) = min(max(jointVel_des(i), minJointVel(i)), maxJointVel(i));
            jointVel(i) = min(max(max(minJointVel(i), jointVel_des(i)), minJointSaturation(i)), min(maxJointVel(i), maxJointSaturation(i)));
        end
    end
end
