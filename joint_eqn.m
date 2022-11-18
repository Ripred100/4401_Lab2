
%Solves for the piecewise joint functions
function joint = joint_eqn(keyframes, T_f)

syms t
t_multi_matrix = [  1 1 1 1 1 1; 
                    t (t-T_f) (t-(2*T_f)) (t-(3*T_f)) (t-(4*T_f)) (t-(5*T_f)); 
                    t^2 (t-T_f)^2 (t-(2*T_f))^2 (t-(3*T_f))^2 (t-(4*T_f))^2 (t-(5*T_f))^2; 
                    t^3 (t-T_f)^3 (t-(2*T_f))^3 (t-(3*T_f))^3 (t-(4*T_f))^3 (t-(5*T_f))^3; 
                    t^4 (t-T_f)^4 (t-(2*T_f))^4 (t-(3*T_f))^4 (t-(4*T_f))^4 (t-(5*T_f))^4; 
                    t^5 (t-T_f)^5 (t-(2*T_f))^5 (t-(3*T_f))^5 (t-(4*T_f))^5 (t-(5*T_f))^5];
    for i = 1:6 %for every joint (Column in keyframes)
        for j = 2:size(keyframes, 1) %every waypoint
            curr_angle = keyframes(j-1,i);
            next_angle = keyframes(j,i);
        
            coef(j-1,:) = coeffSolver(curr_angle, next_angle, T_f);
% theta = matlabFunction(a*[1; t; t^2; t^3; t^4; t^5]);
% thetaDot = matlabFunction(a*[0; 1; 2*t; 3*t^2; 4*t^3; 5*t^4]);
% thetaDoubleDot = matlabFunction(a*[0; 0; 2; 6*t; 12*t^2; 20*t^3]);
            eqn_multi(j-1) = coef(j-1,:)*t_multi_matrix(:,j-1);

        end
    

        
        %IF YOU CHANGE THE AMMOUNT OF KEYFRAMES (VIAPOINTS) YOU NEED TO CHANGE THIS
        %FUNCTION VERY IMPORTANT!!!!!!!!!!!!!!!!!!!!!!!!!!! for example for
    syms theta(t)

    if size(keyframes, 1) == 4
    theta(t) = piecewise(t <= T_f, eqn_multi(1), (T_f < t) & (t <= 2*T_f), eqn_multi(2), (2*T_f < t) & (t <= 3*T_f), eqn_multi(3))

    elseif size(keyframes, 1) == 5
    %FOR 5 VIAPOINTS
    theta(t) = piecewise(t <= T_f, eqn_multi(1), (T_f < t) & (t <= 2*T_f), eqn_multi(2), (2*T_f < t) & (t <= 3*T_f), eqn_multi(3), (3*T_f < t) & (t <= 4*T_f), eqn_multi(4));

    elseif size(keyframes, 1) == 6
    %FOR 6 VIAPOINTS
    theta(t) = piecewise(t <= T_f, eqn_multi(1), (T_f < t) & (t <= 2*T_f), eqn_multi(2), (2*T_f < t) & (t <= 3*T_f), eqn_multi(3), (3*T_f < t) & (t <= 4*T_f), eqn_multi(4), (4*T_f < t) & (t <= 5*T_f), eqn_multi(5))
    
    elseif size(keyframes, 1) == 7
    %FOR 7 VIAPOINTS
    theta(t) = piecewise(t <= T_f, eqn_multi(1), (T_f < t) & (t <= 2*T_f), eqn_multi(2), (2*T_f < t) & (t <= 3*T_f), eqn_multi(3), (3*T_f < t) & (t <= 4*T_f), eqn_multi(4), (4*T_f < t) & (t <= 5*T_f), eqn_multi(5), (5*T_f < t) & (t <= 6*T_f), eqn_multi(6))
    end
    
    joint(i) = theta(t)
    
    end
end