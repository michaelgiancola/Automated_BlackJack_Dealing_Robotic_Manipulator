%Michael Giancola
%Automated BlackJack Dealing Arm
%Computes the origin locations
%This function takes in the theta values for each joint and the necessary lengths when computing the forward kinematics
%This function was made to help account for the different required lengths to the COM's for each Jacobian used in the Euler-Lagrange Formulation
%Date: Dec 4, 2020

function [O1, O2, O3] = origins(thetas, Len1, Len2, Len3)

theta1 = thetas(1);
theta2 = thetas(2);
theta3 = thetas(3);

L1 = Len1;
L2 = Len2;
L3 = Len3;

%Each of these lines compute the homogenous matrix for each joint
%joint 1,2,3 respectively
A1 = [cos(theta1) 0 sin(theta1) 0; sin(theta1) 0 -cos(theta1) 0; 0 1 0 L1; 0 0 0 1];
A2 = [cos(theta2) -sin(theta2) 0 L2*cos(theta2); sin(theta2) cos(theta2) 0 L2*sin(theta2); 0 0 1 0; 0 0 0 1];
A3 = [cos(theta3) -sin(theta3) 0 L3*cos(theta3); sin(theta3) cos(theta3) 0 L3*sin(theta3); 0 0 1 0; 0 0 0 1];

%This computes the transformation matrix from joint 2 wrt the base
T2= A1*A2;

%This computes the transformation matrix from end effector wrt the base
T3= A1*A2*A3;

%these are the distances of the joint axes extracted from the
%transformation matricies
O1=A1(1:3,4);
O2=T2(1:3,4);

%this is the distance of the end effector extracted from the transformation matrix
O3=T3(1:3,4);


end
