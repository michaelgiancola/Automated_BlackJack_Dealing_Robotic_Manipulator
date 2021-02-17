%Michael Giancola
%Automated BlackJack Dealing Arm
%Euler Lagrange Formulation
%Computes the Dynamic Equations of Motion
%Date: Dec 4, 2020

%make the three joint variables the parameters of the function
syms theta1 theta2 theta3 real;
syms dtheta1 dtheta2 dtheta3 real;
syms ddtheta1 ddtheta2 ddtheta3 real;

thetas = [theta1, theta2, theta3];
dthetas = [dtheta1, dtheta2, dtheta3];
ddthetas = [ddtheta1, ddtheta2, ddtheta3];

%masses of links (lb)
%syms m1 m2 m3 real;
m1 = 85.62;
m2 = 235.46;
m3 = 235.46;

%gravity (inch/s^2)
g = 386.09;

%syms I1xx I1yy I1zz I2xx I2yy I2zz I3xx I3yy I3zz real;
I1xx = 3032.43;
I1yy = 2918.27;
I1zz = 242.59;
I2xx = 667.14;
I2yy = 59845.97;
I2zz = 59532.03;
I3xx = 667.14;
I3yy = 59845.97;
I3zz = 59532.03;

%intertias (assuming our links are symmetric)
I1 = [I1xx 0 0; 0 I1yy 0; 0 0 I1zz];
I2 = [I2xx 0 0; 0 I2yy 0; 0 0 I2zz];
I3 = [I3xx 0 0; 0 I3yy 0; 0 0 I3zz];

%constant link lengths(since all rotary joints in my application)
L1 = 20;
L2 = 55;
L3 = 55;

Lc1 = L1/2;
Lc2 = L2/2;
Lc3 = L3/2;

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

%Jacobian matrix Computation for COM of each link
%calls origin function to get transformation matricies with appropriate
%lengths to account for COM distance

[O1, O2, O3] = origins(thetas, Lc1, Lc2, Lc3);

Jvc1 = [cross(z0,O1-O0), zeros(3,1), zeros(3,1)];

[O1, O2, O3] = origins(thetas, L1, Lc2, Lc3);

Jvc2 = [cross(z0,O2-O0),cross(z1,O2-O1),zeros(3,1)];

[O1, O2, O3] = origins(thetas, L1, L2, Lc3);

Jvc3 = [cross(z0,O3-O0),cross(z1,O3-O1),cross(z2,O3-O2)];

Jwc1 = [z0, zeros(3,1), zeros(3,1)];
Jwc2 = [z0, z1, zeros(3,1)];
Jwc3 = [z0, z1, z2];

%Linear Kinetic Energy
Dkl = simplify(((m1*(Jvc1')*Jvc1) + (m2*(Jvc2')*Jvc2) + (m3*(Jvc3')*Jvc3)));

%Rotational Kinetic Energy

R1 = A1(1:3,1:3);
R2 = T2(1:3, 1:3);
R3 = T3(1:3, 1:3);

Dkr = simplify((((Jwc1')*R1*I1*(R1')*Jwc1) + ((Jwc2')*R2*I2*(R2')*Jwc2) + ((Jwc3')*R3*I3*(R3')*Jwc3)));

D = simplify(Dkl + Dkr);

%Christoffel Symbols
chris = sym(zeros(9,3));

%loop through i,j,k values to create the 27 Christoffel symbols
for k = 1:3
    z = 1;
    for j = 1:3
        for i = 1:3

            chris(z,k) = simplify(((diff(D(k,j), thetas(i))) + (diff(D(k,i), thetas(j))) - (diff(D(i,j), thetas(k))))*(dthetas(i))*(dthetas(j)));

            z = z + 1;
        end
    end
end

%Potential Energy
%optain the heights of each COM (referred to transformation matricies)
rc1 = Lc1;
rc2 = Lc2*sin(theta2) + L1;
rc3 = Lc3*sin(theta2 + theta3) + L1 + L2*sin(theta2);

%compute potential energies
P1 = m1*g*rc1;
P2 = m2*g*rc2;
P3 = m3*g*rc3;

P = simplify(P1 + P2 + P3);

%gravity forces for each link
g1 = simplify(diff(P, theta1));
g2 = simplify(diff(P, theta2));
g3 = simplify(diff(P, theta3));

%Equations of motion
Torq1 = simplify((D(1,:)*(ddthetas')) + (sum(chris(:,1))) + g1)
Torq2 = simplify(D(2,:)*(ddthetas')+ (sum(chris(:,2))) + g2)
Torq3 = simplify(D(3,:)*(ddthetas') + (sum(chris(:,3))) + g3)
            