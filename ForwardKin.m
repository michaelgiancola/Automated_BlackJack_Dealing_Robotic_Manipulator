function [T01, T02, T03] = ForwardKin(q)

%Michael Giancola
%Automated BlackJack Dealing Arm
%Forward Kinematics Equation
%T = ForwardKin(q) where q is a vector of joint positions in the form 
%q = [theta1, theta2, theta3] in radians
%T01, T02, T03 are the homogenous transformation matricies of each joint
%wrt to the robot's base
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

%computing the transformation matrix of joint 1 wrt base frame
T01 = A1;

%computing the transformation matrix of joint 1 wrt base frame
T02 = A1*A2;

%Computing the transformation matrix of the end effector position and 
%orientation wrt to the base (this matrix is returned by the function)
T03 = A1*A2*A3;

end