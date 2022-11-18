function a_sol = coeffSolver(theta_init, theta_final, T_f)
%COEFFSOLVER Solves for the coefficients of a quintic function using
%boundary conditions


%T_f 
% Time between theta_init and
%theta_final

syms t
a = sym('a',[1 6]);

theta = matlabFunction(a*[1; t; t^2; t^3; t^4; t^5]);
thetaDot = matlabFunction(a*[0; 1; 2*t; 3*t^2; 4*t^3; 5*t^4]);
thetaDoubleDot = matlabFunction(a*[0; 0; 2; 6*t; 12*t^2; 20*t^3]);

%Declare all of the boundary conditions
eqn(1) = theta(a(1),a(2),a(3),a(4),a(5),a(6), 0)      == theta_init;
eqn(2) = theta(a(1),a(2),a(3),a(4),a(5),a(6), T_f)    == theta_final;
eqn(3) = thetaDot(a(2),a(3),a(4),a(5),a(6),   0)      == 0;
eqn(4) = thetaDot(a(2),a(3),a(4),a(5),a(6),   T_f)    == 0;
eqn(5) = thetaDoubleDot(a(3),a(4),a(5),a(6),  0)      == 0;
eqn(6) = thetaDoubleDot(a(3),a(4),a(5),a(6),  T_f)    == 0;

%Solves for coefficients using MATLAB built-in solver
[a1, a2, a3, a4, a5, a6] = solve(eqn, a);
a_sol = [a1, a2, a3, a4, a5, a6];
end
