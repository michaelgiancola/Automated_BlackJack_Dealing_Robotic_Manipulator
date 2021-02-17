function q = InverseKin(d) %d is the x,y,z coordinates of the end effector

%Michael Giancola
%Automated BlackJack Dealing Arm
%Inverse Kinematics Equations
%q = InverseKin(d) where d is the end effector position (x,y,z) and q are the joint
% positions q = [theta1, theta2, theta3] in radians
%Date: Nov 13, 2020

% Assign the x,y,z positions to variables
dx = d(1);
dy = d(2);
dz = d(3);

%constant link lengths(since all rotary joints in my application)
L1 = 20;
L2 = 55;
L3 = 55;

%First joint
%This solution is limited in theta1 to angles between 0 and pi
%Theta 1 can be calculated using the atan2(y,x)
theta1 = atan2(dy,dx);

%third joint
%This solution is limited in theta3 to angles between 0 and -pi/2
%Theta 3 must be in the elbow up position for my application
c3 = (dx^2 + dy^2 + (dz - L1)^2 - L3^2 - L2^2)/(2*L3*L2);
s3 = -sqrt(1 - c3^2);
theta3 = atan2(s3, c3);

%second joint
%This solution is limited in theta2 to angles between 0 and pi
%theta 2 is solved using this equation
theta2 = atan2(dz - L1, sqrt(dx^2 + dy^2)) - atan2(L3*s3, L2 + L3*c3);

%Computing the joint angles based off of the transformation matrix
%(this vector of joint angles is returned by the function)
q=[theta1 theta2 theta3];

end
