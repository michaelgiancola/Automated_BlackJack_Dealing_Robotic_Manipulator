%Michael Giancola
%Automated BlackJack Dealing Arm
%Gets my desired path through workspace variables and gives it to simulink
%Date: Dec 4, 2020

%creates a time scale for my simulation (new config every 0.1 seconds) 
t = [(0:0.1:16.9)]';

%each of these variables is a matrix of the joints positition and its corresponding time step
%these are obtained by simulink through My Workspace when my
%GradientDesecent script is run so that my desired trajectorty is populated in the route matrix
q1d = [t, route(1,:)'];
q2d = [t, route(2,:)'];
q3d = [t, route(3,:)'];