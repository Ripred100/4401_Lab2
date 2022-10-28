
Time_Per_Keyframe = 3;
dx = 0.05;
T_f = Time_Per_Keyframe;


%Testing the coeffSolver function
a = coeffSolver(10, 30, T_f)
syms t
t_matrix = [1; t; t^2; t^3; t^4; t^5];
t_multi_matrix = [1 1 1; t (t-3) (t-6); t^2 (t-3)^2 (t-6)^2; t^3 (t-3)^3 (t-6)^3; t^4 (t-3)^4 (t-6)^4; t^5 (t-3)^5 (t-6)^5];
y = matlabFunction(a*t_matrix)
x_val = 0:0.05:1
%plot(y(x_val))



%Using keyframes to solve for movement of each joing

keyframes = [   10 10 10 10 10 100;
                40 20 20 20 20 200;
                30 30 30 30 30 300;
                110 40 40 40 40 400;]


joints = joint_eqn(keyframes, T_f, dx)

syms theta1(t) theta2(t) theta3(t)

theta1(t) = joints(1)
theta2(t) = joints(2)
time = 0:dx:9;

figure
plot(time, theta1(time))
title("Theta1(t)")

figure
plot(time, theta2(time))
title("Theta2(t)")

%Solves for the piecewise joint functions
function joint = joint_eqn(keyframes, T_f)

syms t
t_multi_matrix = [  1 1 1; 
                    t (t-T_f) (t-(2*T_f)); 
                    t^2 (t-T_f)^2 (t-(2*T_f))^2; 
                    t^3 (t-T_f)^3 (t-(2*T_f))^3; 
                    t^4 (t-T_f)^4 (t-(2*T_f))^4; 
                    t^5 (t-T_f)^5 (t-(2*T_f))^5];
    for i = 1:6 %for every joint (Column in keyframes)
        for j = 2:size(keyframes, 1) %every waypoint
            curr_angle = keyframes(j-1,i);
            next_angle = keyframes(j,i);
        
            coef(j-1,:) = coeffSolver(curr_angle, next_angle, T_f);
            eqn_multi(j-1) = coef(j-1,:)*t_multi_matrix(:,j-1);

        end
    
    syms theta(t)
    theta(t) = piecewise(t <= T_f, eqn_multi(1), (T_f < t) & (t <= 2*T_f), eqn_multi(2), (2*T_f < t) & (t <= 3*T_f), eqn_multi(3))
    
    joint(i) = theta(t)
    end
end


