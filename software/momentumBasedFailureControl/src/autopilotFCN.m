function [joyButtons,joyAxes] = autopilotFCN(time)
    
    persistent front_pressed pitch_c_pressed takeoff_pressed ...
               pitch_ac_pressed yaw_c_pressed yaw_ac_pressed  ...
               up_pressed down_pressed 
    
    front_pressed    = 0;
    pitch_c_pressed  = 0;
    pitch_ac_pressed = 0;
    yaw_c_pressed    = 0;
    yaw_ac_pressed   = 0;
    takeoff_pressed  = 0;
    down_pressed     = 0;
    up_pressed       = 0;
    
    joyAxes    = zeros(6,1);
    joyButtons = zeros(24,1);
    
    % if button pressed, set to 1
    if time > 1 && time < 5
        
        takeoff_pressed = 1;
    end
    if time > 5 && time < 10
        
        up_pressed = 1;   
    end
    if time > 25 && time < 28
        
        down_pressed = 1;   
    end  
    if time > 29 && time < 31
        
        yaw_c_pressed = 1;
    end
    if time > 33 && time < 40
        
        front_pressed = -1;
    end

%     if time > 25 && time < 28
%         
%         pitch_c_pressed = -1;
%     end
%     if time > 25 && time < 28
%         
%         pitch_ac_pressed = 1;
%     end
%     if time > 25 && time < 28
%         
%         yaw_c_pressed = 1;
%     end
 
    if front_pressed
        joyAxes(2) = front_pressed;
    end
%     if pitch_c_pressed
%         joyAxes(6) = pitch_c_pressed;
%     else
%         joyAxes(6) = pitch_ac_pressed;
%     end
        
    joyButtons(13) = up_pressed;
    joyButtons(6)  = takeoff_pressed;
    joyButtons(16) = yaw_ac_pressed;
    joyButtons(15) = yaw_c_pressed;
    joyButtons(14) = down_pressed; 
end
