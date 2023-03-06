function [pos_vel_acc_jerk_CoM_des, rot_vel_acc_jerk_base_des, jointPos_des] = ...
             computeReferencesBaseAndCoM(posCoM_init, w_R_b_init, jointPos_init, moveFront, moveLateral, moveUpDown, rotateRoll, rotatePitch, ...
                                         rotateYaw, turbo_mode, goLandingPos, feetContactIsActive, oneFootIsInContact, turbinesStatus, t, Config)

    % the function computes CoM position and base frame rotation references.
    % When the joystick button corresponding to a reference is pressed, the
    % current (costant) reference is updated by one step. The step length can
    % be defined by the user. 
    %
    persistent posCoM_0 rpyBase_0 posCoM_landing previous_feetInContact t_turbo time

    if isempty(time)

        time = 0 ;
    end
    if isempty(posCoM_0)
    
        posCoM_0  = posCoM_init;
    end
    if isempty(posCoM_landing)
    
        posCoM_landing  = posCoM_init;
    end
    if isempty(rpyBase_0)
    
        rpyBase_0 = wbc.rollPitchYawFromRotation(w_R_b_init);
    end
    if isempty(previous_feetInContact)
        
        previous_feetInContact = 1 - oneFootIsInContact;
    end
    if isempty(t_turbo)
        
        t_turbo = 0;
    end

    % new step length
    stepLength_CoM     = Config.references.stepLength_CoM;
    stepLength_rpyBase = Config.references.stepLength_rpyBase;

    % turbo mode. When turbo mode is enabled, the robot can only move
    % forward. The speed at which it moves forward progressively increases
    % from standard speed to max speed. In order to stop, the robot needs
    % time to decelerate otherwise it will front-flip in the air
    
    % avoid turbo mode to start when balancing
    if feetContactIsActive

        turbo_mode = 0;
    end
    if turbo_mode || t_turbo > 0
            
        moveFront   = min(0,moveFront);
        moveLateral = 0;
        moveUpDown  = 0;
        rotateRoll  = 0;
        rotateYaw   = 0;
            
        % the robot pitch must be inclined until the min value. In this
        % way the robot can better exploit the torso jets for moving
        if rpyBase_0 < (0.9*Config.references.minBaseRP_balancing(2))
                
            rotatePitch = 0;
        else
            rotatePitch = -1;
        end
                
        if moveFront < -0.5
        
            t_turbo           = min(Config.turbo.speedBoost,t_turbo + 2*Config.tStep);
            stepLength_CoM(1) = stepLength_CoM(1)*(1 + t_turbo);

        elseif moveFront > -0.5
        
            t_turbo           = max(0,t_turbo - 2*Config.tStep);
            stepLength_CoM(1) = stepLength_CoM(1)*(1 + t_turbo);
        end
            
        % do not allow the robot to suddenly stop, but decelerate safely instead
        if t_turbo > 0
                
            moveFront   = -1;
        end
    else
        t_turbo = 0;
    end
        
    % not smoothed references for CoM and base rotation. The orientation
    % pitch and roll are reset to zero if goLandingPos is true
    if ~goLandingPos
            
        rpyBase_ref    = rpyBase_0 + stepLength_rpyBase .* [rotateRoll; rotatePitch; rotateYaw];
    else
        rpyBase_ref    = [0; 0; rpyBase_0(3)];
    end
        
    % FAILURE DETECTION   
    if sum(turbinesStatus) > 0.5
       
        % if robot is traslating, stop it for a while      
        if time < Config.t_stopRobotDuringFailure
    
            stepLength_CoM = Config.references.stepLength_CoM*exp(-Config.failureDetection.eps_com_ref*time);      
        end

        time = time + Config.tStep;               
    end

    % CoM position reference is updated according to the current Yaw angle
    if ~goLandingPos
              
        posCoM_ref     = posCoM_0 + wbc.rotationFromRollPitchYaw([0; 0; rpyBase_ref(3)]) * (stepLength_CoM .* [moveFront; moveLateral; moveUpDown]);            
    else
        % limit the robot speed while landing
        posCoM_ref     = posCoM_0 + wbc.rotationFromRollPitchYaw([0; 0; rpyBase_ref(3)]) * ([0; 0; Config.references.zCoMlimitStepLengthLanding] .* [moveFront; moveLateral; moveUpDown]);
    end
    
    % SATURATIONS
    
    % register the CoM position when landing
    tol = 0.5;
    
    if previous_feetInContact < tol && oneFootIsInContact > tol && t > 0
        
        % update the CoM reference position at the instant of landing only
        posCoM_landing = posCoM_ref;
    end  
    
    % saturations on roll and pitch while flying
    rpyBase_ref(1:2)     = wbc.saturateInput(rpyBase_ref(1:2), Config.references.minBaseRP_flying, Config.references.maxBaseRP_flying);
   
    % saturation of the CoM is detected also when at least one foot is in 
    % contact, to detect landing when feet are not aligned  
    if feetContactIsActive || oneFootIsInContact
        
        % saturations on roll and pitch while balancing
        rpyBase_ref(1:2) = wbc.saturateInput(rpyBase_ref(1:2), Config.references.minBaseRP_balancing, Config.references.maxBaseRP_balancing);
   
        % saturation on CoM position while balancing
        posCoM_ref       = wbc.saturateInput(posCoM_ref, (posCoM_landing + Config.references.minCoMPos_balancing), (posCoM_landing + Config.references.maxCoMPos_balancing));  
    end
    
    % set jointPos_des
    jointPos_des = jointPos_init;

    % FAILURE DETECTION 
    if turbinesStatus(2) > 0 && Config.USE_OPTIMIZATION_REF_FOR_FAULT_RESPONSE
      
        jointPos_des     = Config.jointPos_rightArm_fail';   
        rpyBase_ref(1:2) = Config.attitude_upd_rightArm_fail(1:2)';              
    end
    if turbinesStatus(3) > 0 && Config.USE_OPTIMIZATION_REF_FOR_FAULT_RESPONSE
      
        jointPos_des     = Config.jointPos_leftBack_fail';   
        rpyBase_ref(1:2) = Config.attitude_upd_leftBack_fail(1:2)';              
    end

    % output reference (to be smoothed)
    pos_vel_acc_jerk_CoM_des  = [posCoM_ref, zeros(3)];
    rot_vel_acc_jerk_base_des = [wbc.rotationFromRollPitchYaw(rpyBase_ref), zeros(3)];
  
    % update initial CoM position, base rotation reference and feet in contact
    posCoM_0               = posCoM_ref;
    rpyBase_0              = rpyBase_ref;   
    previous_feetInContact = oneFootIsInContact;
 end
