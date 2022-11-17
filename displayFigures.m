
Q_via = [
145 136 80 67 220 150;
160 136 65 92 220 138;
160 150 60 170 230 150;
90 150 70 160 230 157;
35 150 70 160 230 131;
10 150 70 160 230 131;
3 140 80 125 210 145];

[NUMBER_OF_VIAPOINTS ~] = size(Q_via);

T_f = 1; %The "time" between viapoints so that theta1(n*T_f) are all the viapoint positions
joints = joint_eqn(Q_via, T_f);

%syms theta1(t) theta2(t) theta3(t) theta4(t) theta5(t) theta6(t) t
syms t
% 
% theta1(t) = joints(1);
% theta1_dot = diff(theta1);
% theta1_dot_dot = diff(theta1_dot);
% theta2(t) = joints(2);
% theta3(t) = joints(3);
% theta4(t) = joints(4);
% theta5(t) = joints(5);
% theta6(t) = joints(6);
%theta7(t) = piecewise( t <= 0.9, 160, (t > 0.9 & t < 5.8), 210, 0);%Figure this out. It's the gripper. Look at modelDeg2RobotDeg

%1s between each viapoint with a timestep of 0.05 gives 20 points between
%each viapoint plus a 0. For 7 viapoints this gives 121 points.
x = 0:0.005:(NUMBER_OF_VIAPOINTS - 1);

for i = 1:6

    theta(t) = joints(i);
    theta_dot = diff(theta);
    theta_dot_dot = diff(theta_dot);

    fig(i) = figure

    str = sprintf("Theta%d", i);
    subplot(3,1,1)
    plot(x, theta(x),LineWidth=1, Color=[0 0.4470 0.7410])
    title(strcat(str, "(t)"))
    ylabel("Robot degrees")
    grid on
    subplot(3,1,2)
    plot(x, theta_dot(x), LineWidth=1, Color=[0.6350 0.0780 0.1840])
    title(strcat(str, "\_Dot(t)"))
    ylabel("Robot degrees/s")
    subplot(3,1,3)
    plot(x, theta_dot_dot(x), LineWidth=1, Color=[0.9290 0.6940 0.1250]);
    title(strcat(str, "\_DoubleDot(t)"))
    ylabel("Robot degrees/s^2")

    %saveas(fig(i), strcat(str,".png"))
    exportgraphics(fig(i), strcat(str,".png"),'Resolution',300)

end
