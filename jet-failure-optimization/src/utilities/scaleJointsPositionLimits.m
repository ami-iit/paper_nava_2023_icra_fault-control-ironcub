function [lowerBound_scaled, upperBound_scaled] = scaleJointsPositionLimits(lowerBound, upperBound, Config)

scaleTorsoJointsLimits    = 0.3;
scaleLeftArmJointsLimits  = 0.5;

switch Config.FAILURE_TYPE

    case 1

        scaleRightArmJointsLimits = 0.2;
        scaleLegsJointsLimits     = 0.3;
    case 2

        scaleRightArmJointsLimits = 0.5;
        scaleLegsJointsLimits     = 0.2;
    otherwise

        scaleRightArmJointsLimits = 0.5;
        scaleLegsJointsLimits     = 0.5;
end

lowerBound_scaled         = zeros(size(lowerBound));
upperBound_scaled         = zeros(size(upperBound));

lowerBound_scaled(1:3)    = scaleTorsoJointsLimits*lowerBound(1:3);
lowerBound_scaled(4:6)    = scaleLeftArmJointsLimits*lowerBound(4:6);
lowerBound_scaled(7)      = lowerBound(7);
lowerBound_scaled(8:10)   = scaleRightArmJointsLimits*lowerBound(8:10);
lowerBound_scaled(11)     = lowerBound(11);
lowerBound_scaled(12:end) = scaleLegsJointsLimits*lowerBound(12:end);

upperBound_scaled(1:3)    = scaleTorsoJointsLimits*upperBound(1:3);
upperBound_scaled(4:6)    = scaleLeftArmJointsLimits*upperBound(4:6);
upperBound_scaled(7)      = upperBound(7)/2;
upperBound_scaled(8:10)   = scaleRightArmJointsLimits*upperBound(8:10);
upperBound_scaled(11)     = upperBound(11)/2;
upperBound_scaled(12:end) = scaleLegsJointsLimits*upperBound(12:end);


% another approach for scaling joints positions limits
% scalingConst              = 10;
% 
% lowerBound_scaled(1:3)    = -scalingConst + Config.initCond.jointPos_init(1:3);
% lowerBound_scaled(4:7)    = -scalingConst + Config.initCond.jointPos_init(4:7);
% lowerBound_scaled(8:11)   = -scalingConst + Config.initCond.jointPos_init(8:11);
% lowerBound_scaled(12:end) = -scalingConst + Config.initCond.jointPos_init(12:end);
% 
% upperBound_scaled(1:3)    =  scalingConst + Config.initCond.jointPos_init(1:3);
% upperBound_scaled(4:7)    =  scalingConst + Config.initCond.jointPos_init(4:7);
% upperBound_scaled(8:11)   =  scalingConst + Config.initCond.jointPos_init(8:11);
% upperBound_scaled(12:end) =  scalingConst + Config.initCond.jointPos_init(12:end);

end