%Lab2 code based off Laboratory #2 Document

% Defining the Model Robot (link lengths may be different)
Link1 = Link('a',0,'d',170,'alpha',pi/2);
Link2 = Link('a',175,'d',0,'alpha',0);
Link3 = Link('a',0,'d',0,'alpha',pi/2);
Link4 = Link('a',0,'d',110,'alpha',-pi/2);
Link5 = Link('a',0,'d',0,'alpha',pi/2);
Link6 = Link('a',0,'d',250,'alpha',0);
ModelBot = SerialLink([Link1, Link2, Link3, Link4, Link5, Link6],'name','MSE');
% Joint Limits should be defined (in terms of actual robot angles in degrees)

Q_UpperLimits = [230, 150, 150, 230, 230, 235, 190];
Q_LowerLimits = [0, 60, 55, 100, 100, 100, 100];
% Sample list of via points (7 angles in degrees per point) to be sent to the robot
% Can be found by manually moving the robot along the path and recording key points
% "..." is used to break a statement over several lines in MATLAB

Q_via = [
145 136 80 67 220 150;
160 136 65 92 220 138;
160 150 60 170 230 150;
90 150 70 160 230 157;
35 150 70 160 230 131;
10 150 70 160 230 131;
3 140 80 125 210 145];

NUMBER_OF_VIAPOINTS = 7


% Replace the line below with the Cubic or Quintic polynomial required to generate
% several points along a smooth path between every two via points in joint space.
% That is, you sample the Cubic or Quintic polynomial 10 times between the via
% points. This means that if you begin with 10 via points, you will end up with 91

T_f = 1; %The "time" between viapoints so that theta1(n*T_f) are all the viapoint positions
joints = joint_eqn(Q_via, T_f)

syms theta1(t) theta2(t) theta3(t) theta4(t) theta5(t) theta6(t) t

theta1(t) = joints(1)
theta2(t) = joints(2)
theta3(t) = joints(3)
theta4(t) = joints(4)
theta5(t) = joints(5)
theta6(t) = joints(6)

theta7(t) = piecewise( t <= 0.9, 160, (t > 0.9 & t < 5.8), 210, 0)%Figure this out. It's the gripper. Look at modelDeg2RobotDeg




Robot = MSE4401BOT(1234,4321); % Create robot objects
pause on; % Enable the use of pause command

disp('Ready to move the robot. Please press a key to continue...');
pause;


for i = 0:0.05:(NUMBER_OF_VIAPOINTS - 1)
    i
    if i == 4  
        pause(3)
    end
    Q_robot = [theta1(i), theta2(i), theta3(i), theta4(i), theta5(i), theta6(i), theta7(i)] % Get the next point to be sent


    Q_model = double(Q_robot);
    % Check for being close to singularity using the model, only first 6 angles
    J = ModelBot.jacob0(Q_model(1:6));
    DetJ = det(J);
    if abs(DetJ) < 0.0001
        Alarm_Singularity = 1;
    else
    Alarm_Singularity = 0;
end
% Check the joint limits
Alarm_Limits = 0;
for j = 1:7
    if Q_robot(j) > Q_UpperLimits(j) || Q_robot(j) < Q_LowerLimits(j)
        Alarm_Limits = 0;%FIX THIS LATER SHOULD BE 1 JUST FOR DEBUG
    end
end
Alarm = Alarm_Limits + Alarm_Singularity; % Any Fault in the system
% Send the point to the robot and wait for the robot to reach the point
if Alarm == 0
    Robot.sendPosition(Q_robot);
    Q_current = transpose(Robot.getPosition);
    while norm(Q_current - Q_robot,2) > 10
        Q_current = transpose(Robot.getPosition);
    end
    
else
    disp('The position is out of reach! Press enter to continue');
    pause;
        % Can do something like jumping over the point and sending the next one
end
end