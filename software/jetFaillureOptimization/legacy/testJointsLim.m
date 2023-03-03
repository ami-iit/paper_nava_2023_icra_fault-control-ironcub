jointPositionLimits = [-20,  70; -30,  30;  -50,  50; ...                                %torso
                       -90,  10;  0,   160; -35,  80;  17,  105; ...                     %larm
                       -90,  10;  0,   160; -35,  80;  17,  105; ...                     %rarm
                       -35,  80; -15,  90;  -70,  70; -100, 0;  -30,  30; -20,  20; ...  %rleg  
                       -35,  80; -15,  90;  -70,  70; -100, 0;  -30,  30; -20,  20];     %lleg

lowerBoundJointPos  = jointPositionLimits(:,1);
upperBoundJointPos  = jointPositionLimits(:,2);

% scaling joints limits
[lowerBoundJointPos, upperBoundJointPos] = scaleJointsPositionLimits(lowerBoundJointPos, upperBoundJointPos, Config)