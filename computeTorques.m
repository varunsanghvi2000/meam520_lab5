% Convert end effector forces into joint torques
% For 10V, and i = [1,2,1].*tau, we have tauLim = [0.7,0.5,0.9];

% Fill in the necessary inputs
function Tau = computeTorques(q, F)

a1 = 3*25.4;
a2 = 5.75*25.4;
a3 = 7.375*25.4;

% Fill this in
theta1 = q(1);
theta2 = q(2);
theta3 = q(3);

% Compute forces in computeForces.m 
% F = computeForces();

% Compute Jacobian for each joint
J0_1 = computeJacobian(theta1, theta2, theta3, a1, a2, a3, 1);
J1_2 = computeJacobian(theta1, theta2, theta3, a1, a2, a3, 2);
J2_3 = computeJacobian(theta1, theta2, theta3, a1, a2, a3, 3);

J0_1 = subs(J0_1);
J1_2 = subs(J1_2);
J2_3 = subs(J2_3);

% Find joint Tau
Tau1 = vpa(J0_1.' * F);
Tau2 = vpa(J1_2.' * F);
Tau3 = vpa(J2_3.' * F);

% Find final Tau
Tau = round(Tau1 + Tau2 + Tau3);

end