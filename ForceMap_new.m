function T = ForceMap_new(qc, qf, dealerShoePos, chipTrayPos, discardTrayPos) %positions in (x,y,z) format

%Michael Giancola
%Automated BlackJack Dealing Arm
%Attractive and repulsive forces, along with force mapping
%T = ForceMap_new(qc, qf, dealerShoePos, chipTrayPos, discardTrayPos)
%where T is the total joint torques at the current robot position
%qc and qf are the current and goal configurations of the robot
%dealerShoePos, chipTrayPos, discardTrayPos are the obstacle positions
%Date: Nov 13, 2020

%forward kinematics to get intermediate/final tranformation matricies 
[Tc01, Tc02, Tc03] = ForwardKin(qc);
[Tf01, Tf02, Tf03] = ForwardKin(qf);

%Extracting origins of frames from the current end effector position transformation matrix
O1c = Tc01(1:3, 4); 
O2c = Tc02(1:3, 4); 
O3c = Tc03(1:3, 4); 

%Extracting origins of frames from the final end effector position transformation matrix
O1f = Tf01(1:3, 4);
O2f = Tf02(1:3, 4);
O3f = Tf03(1:3, 4); 


%Formulating Attractive Forces

%predefined Zeta value which is influence of attractive potential [O1, O2, O3]
Z = [1, 1, 100];

%Parabolic attractive force calculation for each origin
Fatt1 = -Z(1)*(O1c - O1f);
Fatt2 = -Z(2)*(O2c - O2f);
Fatt3 = -Z(3)*(O3c - O3f);

%distances from current origin positions to final origin positions
dis1 = norm(O1c - O1f);
dis2 = norm(O2c - O2f);
dis3 = norm(O3c - O3f);

%predefined distance that causes transition from conic to parabolic
d = 10; 

%Implementation of the hybrid attractive field
%check if the distance of each current joint position is farther than d away from its goal joint position, and if so, use conic field
if(dis1 > d)
    Fatt1 = (d*Fatt1)/dis1; %conic potenial field calculation
end

if(dis2 > d)
    Fatt2 = (d*Fatt2)/dis2; %conic potenial field calculation
end

if(dis3 > d) 
    Fatt3 = (d*Fatt3)/dis3; %conic potenial field calculation
end


%Formulating Repulsive Forces

%predefined Eta value which is influence of repulsive potential [O1, O2, O3]
N = [100, 100, 10000];
    
%define the radius of influence of each obstacle [dealerShoe, chipTray, discardTray]
%NOTE: must define them such that two regions of influence do not overlap!
p0 = [10, 10, 10]; 
    
%putting current origin positions in a vector to iterate through
CurrOrigins = [O1c, O2c, O3c];
    
%store [Frep1, Frep2, Frep3] in this vector once they are calculated (default values of Frep = 0)
RepulsiveForces = zeros(3);

%transpose obstacle coordinates into a column vector
dealerShoePos = transpose(dealerShoePos);
chipTrayPos = transpose(chipTrayPos);
discardTrayPos = transpose(discardTrayPos);

%Iterate through each origin of robot to calculate the repulsive force on it
for i = 1:3
    
    %calculate the distance function between each origin and each obstacle
    dealerShoeDis = norm(CurrOrigins(1:3, i) - dealerShoePos);
    chipTrayDis = norm(CurrOrigins(1:3, i) - chipTrayPos); 
    discardTrayDis = norm(CurrOrigins(1:3, i) - discardTrayPos);
    
    %subtract obstacle radius from object's centroid to origin distance to account for obstacle size
    dealerShoeDis = dealerShoeDis - 5.75;
    chipTrayDis = chipTrayDis - 11.5;
    discardTrayDis = discardTrayDis - 5;
    
    %computing the gradient of the distance function with the closest point on each obstacle to the origin
    dealerShoeGrad = (CurrOrigins(1:3, i)- dealerShoePos)/dealerShoeDis;
    chipTrayGrad = (CurrOrigins(1:3, i)- chipTrayPos)/chipTrayDis;
    discardTrayGrad = (CurrOrigins(1:3, i)- discardTrayPos)/discardTrayDis;
        
    %if the origin is within the radius of influence of the dealer shoe then compute the repulsive force accordingly
    if(dealerShoeDis <= p0(1))
        RepulsiveForces(1:3, i) = N(i)*((1/dealerShoeDis)-(1/p0(1)))*(1/(dealerShoeDis^2))*(dealerShoeGrad);
    
    %if the origin is within the radius of influence of the chip tray then compute the repulsive force accordingly
    elseif(chipTrayDis <= p0(2)) 
        RepulsiveForces(1:3, i) = N(i)*((1/chipTrayDis)-(1/p0(2)))*(1/(chipTrayDis^2))*(chipTrayGrad);
    
    %if the origin is within the radius of influence of the discard tray then compute the repulsive force accordingly
    elseif(discardTrayDis <= p0(3))
        RepulsiveForces(1:3, i) = N(i)*((1/discardTrayDis)-(1/p0(3)))*(1/(discardTrayDis^2))*(discardTrayGrad);
    end
end

%putting calculated repulsive force values in variables
Frep1 = RepulsiveForces(1:3, 1);
Frep2 = RepulsiveForces(1:3, 2);
Frep3 = RepulsiveForces(1:3, 3);

%compute Jacobian to get matricies for each origin
[J01, J02, J03] = Jacobians(qc);

%mapping workspace forces to configuration torques
Tatt1 = (transpose(J01))*Fatt1; 
Tatt2 = (transpose(J02))*Fatt2; 
Tatt3 = (transpose(J03))*Fatt3; 
Trep1 = (transpose(J01))*Frep1; 
Trep2 = (transpose(J02))*Frep2; 
Trep3 = (transpose(J03))*Frep3; 

%total joint torque at current position (config space)
T = Tatt1 + Tatt2 + Tatt3 + Trep1 + Trep2 + Trep3;

end