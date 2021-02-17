%Michael Giancola
%Automated BlackJack Dealing Arm
%Inverse Dynamic Equations of motion
%ddq = dynamicEqns(thetas, dthetas, ddthetas) 
%Returns a matrix of required torque values for each joint
%Date: Dec 4, 2020

function [Torq1, Torq2, Torq3] = dynamicEqns(thetas, dthetas, ddthetas)

theta1 = thetas(1);
theta2 = thetas(2);
theta3 = thetas(3);
dtheta1 = dthetas(1);
dtheta2 = dthetas(2);
dtheta3 = dthetas(3);
ddtheta1 = ddthetas(1);
ddtheta2 = ddthetas(2);
ddtheta3 = ddthetas(3);

%Inverse Dynamics Equations of Motion found using Euler Lagrange Formulation
Torq1 = ddtheta1*((1424533*cos(2*theta2 + theta3))/4 + (815624558777204737*cos(2*theta2))/1717986918400 + (203792294069927937*cos(2*theta2 + 2*theta3))/1717986918400 + (1424533*cos(theta3))/4 + 32855125699743187007/54975581388800) - 2*dtheta1*dtheta3*((1424533*sin(2*theta2 + theta3))/4 + (203792294069927937*sin(2*theta2 + 2*theta3))/858993459200 + (1424533*sin(theta3))/4) - 2*dtheta1*dtheta2*((1424533*sin(2*theta2 + theta3))/2 + (815624558777204737*sin(2*theta2))/858993459200 + (203792294069927937*sin(2*theta2 + 2*theta3))/858993459200);
 
 
Torq2 = ((1424533*sin(2*theta2 + theta3))/2 + (815624558777204737*sin(2*theta2))/858993459200 + (203792294069927937*sin(2*theta2 + 2*theta3))/858993459200)*dtheta1^2 + (343596100481210885*cos(theta2 + theta3))/137438953472 + (1030788301443632655*cos(theta2))/137438953472 - (1424533*dtheta3^2*sin(theta3))/2 + ddtheta2*((1424533*cos(theta3))/2 + 20400472916534231/17179869184) + ddtheta3*((1424533*cos(theta3))/4 + 8163827622388695/34359738368) - 1424533*dtheta2*dtheta3*sin(theta3);
 
 
Torq3 = ((1424533*sin(2*theta2 + theta3))/4 + (203792294069927937*sin(2*theta2 + 2*theta3))/858993459200 + (1424533*sin(theta3))/4)*dtheta1^2 + (1424533*sin(theta3)*dtheta2^2)/2 + (8163827622388695*ddtheta3)/34359738368 + (343596100481210885*cos(theta2 + theta3))/137438953472 + ddtheta2*((1424533*cos(theta3))/4 + 8163827622388695/34359738368);

end
