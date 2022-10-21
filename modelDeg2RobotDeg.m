function robotDeg = modelDeg2RobotDeg(modDeg)
%modelDeg2RobotDeg converts a vector of model degrees to degrees usable by
%the physical robot
%   Detailed explanation goes here

%These Angles read from the robot servos correspond to the DH convention
%model angles [0, 0, 0, 0, 0, 0]. 
Conversion = [150, 60, 240, 150, 150, 150];%In degrees

robotDeg = modDeg + Conversion;

end

