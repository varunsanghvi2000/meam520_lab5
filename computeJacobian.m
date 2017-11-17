function Jv = computeJacobian(q1, q2, q3, a1, a2, a3, joint_number)

% Function has been modified to accomodate calculation of J for different
% joints

% Anticipate something to be wrong with this Jacobian function. Comparison of Jacobian values in the 2 methods given in the started
% started code DON'T MATCH! Have to get this checked.

% Uncomment the below line to view the symbolic outputs for Jacobians
syms theta1 theta2 theta3 a1 a2 a3 reals

T0_1 = [ cos(theta1)  0 -sin(theta1)   0           ;...
         sin(theta1)  0  cos(theta1)   0            ;...
              0       -1      0        a1            ;...
              0       0       0        1            ];
T1_2 = [ sin(theta2)  cos(theta2)  0  a2*sin(theta2);...
        -cos(theta2)  sin(theta2)  0 -a2*cos(theta2);...
              0            0       1       0             ;...
              0            0       0       1             ];
T2_3 = [-sin(theta3)  -cos(theta2)  0 -a3*sin(theta3);...
         cos(theta3)  -sin(theta2)  0  a3*cos(theta3);...
              0             0       1       0             ;...
              0             0       0       1             ];

T0_2 = T0_1 * T1_2;
T0_3 = T0_1 * T1_2 * T2_3;

o0_1 = T0_1 * [0 0 0 1]';
o0_2 = T0_2 * [0 0 0 1]';
o0_3 = T0_3 * [0 0 0 1]';

if joint_number == 1
    x = o0_1(1);
    y = o0_1(2);
    z = o0_1(3);
elseif joint_number == 2
    x = o0_2(1);
    y = o0_2(2);
    z = o0_2(3);
elseif joint_number == 3
    x = o0_3(1);
    y = o0_3(2);
    z = o0_3(3);
else
    disp('Can not compute Jacobian. Invalid joint number')
end

Jv(:,1) = diff([x;y;z], theta1);
Jv(:,2) = diff([x;y;z], theta2);
Jv(:,3) = diff([x;y;z], theta3);

theta1 = q1;
theta2 = q2;
theta3 = q3;

a1 = 3*25.4;
a2 = 5.75*25.4;
a3 = 7.375*25.4;

Jv = subs(Jv, {theta1, theta2, theta3, a1, a2, a3},{q1, q2, q3, 3*25.4, 5.75*25.4, 7.375*25.4});
%% Alternate
% Jv = [ -sin(theta1)*(a3*cos(theta2 + theta3) + a2*sin(theta2)), -cos(theta1)*(a3*sin(theta2 + theta3) - a2*cos(theta2)), -a3*sin(theta2 + theta3)*cos(theta1);
%      cos(theta1)*(a3*cos(theta2 + theta3) + a2*sin(theta2)), -sin(theta1)*(a3*sin(theta2 + theta3) - a2*cos(theta2)), -a3*sin(theta2 + theta3)*sin(theta1);
%                                              0,              - a3*cos(theta2 + theta3) - a2*sin(theta2),             -a3*cos(theta2 + theta3)];

end