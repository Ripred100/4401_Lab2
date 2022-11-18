function robotDeg = modelDeg2RobotDeg(modDeg)
%modelDeg2RobotDeg converts a vector of model degrees to degrees usable by
%the physical robot
%   Detailed explanation goes here

%These Angles read from the robot servos correspond to the DH convention
%model angles [0, 0, 0, 0, 0, 0]. 
Conversion = [150, 60, 240, 150, 150, 150, 0];%In degrees


robotDeg = modDeg + Conversion;

%CHANGE THESE AS NEEDED
OPEN_CLAW_VALUE = 150
CLOSED_CLAW_VALUE = 160

if robotDeg(7) == 0
    robotDeg(7) = OPEN_CLAW_VALUE
else
    robotDeg(7) = CLOSED_CLAW_VALUE
end
