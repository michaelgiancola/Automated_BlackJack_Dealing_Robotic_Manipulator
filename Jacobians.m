function [J01, J02, J03] = Jacobians(q)

%Michael Giancola
%Automated BlackJack Dealing Arm
%Jacobian Matricies
%J = Jacobian(q) where q is a vector of joint positions in the form 
%q = [theta1, theta2, theta3] in radians
%J01, J02, J03 are the Jacobian matricies with velocity components only
%Date: Nov 13, 2020

%make the three joint variables the parameters of the function
theta1 = q(1);
theta2 = q(2);
theta3 = q(3);

%constant link lengths(since all rotary joints in my application)
L1 = 20;
L2 = 55;
L3 = 55;

%Each of these lines compute the homogenous matrix for each joint
%joint 1,2,3 respectively
A1 = [cos(theta1) 0 sin(theta1) 0; sin(theta1) 0 -cos(theta1) 0; 0 1 0 L1; 0 0 0 1];
A2 = [cos(theta2) -sin(theta2) 0 L2*cos(theta2); sin(theta2) cos(theta2) 0 L2*sin(theta2); 0 0 1 0; 0 0 0 1];
A3 = [cos(theta3) -sin(theta3) 0 L3*cos(theta3); sin(theta3) cos(theta3) 0 L3*sin(theta3); 0 0 1 0; 0 0 0 1];

%This computes the transformation matrix from joint 2 wrt the base
T2= A1*A2;

%This computes the transformation matrix from end effector wrt the base
T3= A1*A2*A3;

%these are the z axis vectors for each joint which are used to solve for
%the linear and angular velocity matrix (Jacobian) 
z0= [0;0;1];
z1= A1(1:3,3);
z2= T2(1:3,3); 

%these are the distances of the joint axes extracted from the
%transformation matricies
O0=[0;0;0];
O1=A1(1:3,4);
O2=T2(1:3,4);

%this is the distance of the end effector extracted from the transformation
%matrix
O3=T3(1:3,4);
    
% Jacobian matrix Computation
%Since all of the joints in my robot are revolute, the Jacobian matrix 
%linear velocity components are computed by doing the cross product of the z 
%axis and the distance between the end effector and the joint axis
%for intermediate Jacobians, the remaining columns are filled with zero vectors
J01 = [cross(z0,O3-O0), zeros(3,1), zeros(3,1)];
J02 = [cross(z0,O3-O0),cross(z1,O3-O1),zeros(3,1)];
J03 = [cross(z0,O3-O0),cross(z1,O3-O1),cross(z2,O3-O2)];

end