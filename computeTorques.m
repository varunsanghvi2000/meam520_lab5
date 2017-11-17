% Convert end effector forces into joint torques
% For 10V, and i = [1,2,1].*tau, we have tauLim = [0.7,0.5,0.9];

% Fill in the necessary inputs
function Tau = computeTorques(F)

global qs

a1 = 3*25.4;
a2 = 5.75*25.4;
a3 = 7.375*25.4;

% Fill this in
theta1 = qs(1);
theta2 = qs(2);
theta3 = qs(3);

% Compute forces in computeForces.m 
% F = computeForces();

% Compute Jacobian for each joint

J2_3 = computeJacobian(qs(1), qs(2), qs(3), a1, a2, a3);

J2_3 = subs(J2_3);

% Find joint Tau

Tau3 = vpa(J2_3.' * F);

% Find final Tau
Tau = round(Tau3);

end