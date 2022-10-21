

%my_bot.plot([ -2*pi/3 5*pi/2 pi/3 pi/5 pi/4 pi/3])
% 
% deg = rad2deg([pi, pi,pi,pi,pi,pi,pi])
Robot = MSE4401BOT(1234,4321);
% %Robot.sendPosition(deg)
% Robot.sendPosition([150,150,150,150,150,150,178])


%START OF LAB

%Develop Function to convert real angles to robot angles

%Consider Singularity and smoothness constraint to see if we use cubic or
%quintic trajectory. Justify

%Compute the required coefficients

%Using robotics toolbox, simulate path to ensure smoothness

% Defining the Model Robot (link lengths may be different)
Link1 = Link('a',0,'d',170,'alpha',pi/2);
Link2 = Link('a',175,'d',0,'alpha',0);
Link3 = Link('a',0,'d',0,'alpha',pi/2);
Link4 = Link('a',0,'d',110,'alpha',-pi/2);
Link5 = Link('a',0,'d',0,'alpha',pi/2);
Link6 = Link('a',0,'d',250,'alpha',0);
ModelBot = SerialLink([Link1, Link2, Link3, Link4, Link5, Link6],'name','MSE');