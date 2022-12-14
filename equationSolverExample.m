%Gives an example of how to use joint_eqn()

%this makes it have 10 samples per keyframe or viapoint
Time_Per_Keyframe = 1;
dx = 0.1;
T_f = Time_Per_Keyframe;


%Testing the coeffSolver function
a = coeffSolver(10, 30, T_f)
syms t
t_matrix = [1; t; t^2; t^3; t^4; t^5];
t_multi_matrix = [1 1 1; t (t-3) (t-6); t^2 (t-3)^2 (t-6)^2; t^3 (t-3)^3 (t-6)^3; t^4 (t-3)^4 (t-6)^4; t^5 (t-3)^5 (t-6)^5];
y = matlabFunction(a*t_matrix)
x_val = 0:dx:1
%plot(y(x_val))



%Using keyframes to solve for movement of each joing

keyframes = [   10 10 10 10 10 100;
                40 20 20 20 20 200;
                30 30 30 30 30 300;
                110 40 40 40 40 400;
                ]


joints = joint_eqn(keyframes, T_f)

syms theta1(t) theta2(t) theta3(t) theta4(t) theta5(t) theta6(t) t

theta1(t) = joints(1)
theta2(t) = joints(2)
theta3(t) = joints(3)
theta4(t) = joints(4)
theta5(t) = joints(5)
theta6(t) = joints(6)
theta7(t) = piecewise((t > 0) & (t < 1), 0, (t >= 1), 1)

for i = 0:0.1:3
    i
    Q_robot = [theta1(i), theta2(i), theta3(i), theta4(i), theta5(i), theta6(i), theta7(i)]
end

time = 0:dx:4;

figure
plot(time, theta1(time))
title("Theta1(t)")

figure
plot(time, theta2(time))
title("Theta2(t)")


% 
% %Solves for the piecewise joint functions
% function joint = joint_eqn(keyframes, T_f)
% 
% 
% 
% syms t
% t_multi_matrix = [  1 1 1 1 1; 
%                     t (t-T_f) (t-(2*T_f)) (t-(3*T_f)) (t-(4*T_f)); 
%                     t^2 (t-T_f)^2 (t-(2*T_f))^2 (t-(3*T_f))^2 (t-(4*T_f))^2; 
%                     t^3 (t-T_f)^3 (t-(2*T_f))^3 (t-(3*T_f))^3 (t-(4*T_f))^3; 
%                     t^4 (t-T_f)^4 (t-(2*T_f))^4 (t-(3*T_f))^4 (t-(4*T_f))^4; 
%                     t^5 (t-T_f)^5 (t-(2*T_f))^5 (t-(3*T_f))^5 (t-(4*T_f))^5];
%     for i = 1:6 %for every joint (Column in keyframes)
%         for j = 2:size(keyframes, 1) %every waypoint
%             curr_angle = keyframes(j-1,i);
%             next_angle = keyframes(j,i);
%         
%             coef(j-1,:) = coeffSolver(curr_angle, next_angle, T_f);
%             eqn_multi(j-1) = coef(j-1,:)*t_multi_matrix(:,j-1);
% 
%         end
%     
% 
%         
%         %IF YOU CHANGE THE AMMOUNT OF KEYFRAMES (VIAPOINTS) YOU NEED TO CHANGE THIS
%         %FUNCTION VERY IMPORTANT!!!!!!!!!!!!!!!!!!!!!!!!!!! for example for
%     syms theta(t)
% 
%     if size(keyframes, 1) == 4
%     theta(t) = piecewise(t <= T_f, eqn_multi(1), (T_f < t) & (t <= 2*T_f), eqn_multi(2), (2*T_f < t) & (t <= 3*T_f), eqn_multi(3))
% 
%     elseif size(keyframes, 1) == 5
%     %FOR 5 VIAPOINTS
%     theta(t) = piecewise(t <= T_f, eqn_multi(1), (T_f < t) & (t <= 2*T_f), eqn_multi(2), (2*T_f < t) & (t <= 3*T_f), eqn_multi(3), (3*T_f < t) & (t <= 4*T_f), eqn_multi(4));
% 
%     elseif size(keyframes, 1) == 6
%     %FOR 6 VIAPOINTS
%     theta(t) = piecewise(t <= T_f, eqn_multi(1), (T_f < t) & (t <= 2*T_f), eqn_multi(2), (2*T_f < t) & (t <= 3*T_f), eqn_multi(3), (3*T_f < t) & (t <= 4*T_f), eqn_multi(4), (4*T_f < t) & (t <= 5*T_f), eqn_multi(5))
%     end
%     
%     joint(i) = theta(t)
%     end
% end

