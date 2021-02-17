%Michael Giancola
%Automated BlackJack Dealing Arm
%Gradient Descent Path Planning 
%The starting and goal positions are specificed along with the obstacle
%locations and then the path of the robot's end effector is computed and
%plotted
%Date: Nov 13, 2020

%Clear and close open figures before running 
close all
clear all

%initial and final position of end effector Q2 #1(x,y,z)
qsPos = [0, 5, 2]; 
qfPos = [60, 40, 1];

% %initial and final position of end effector Q2 #2(x,y,z)
% qsPos = [85, 10, 5]; 
% qfPos = [-15, 40, 2];

% %initial and final position of end effector Q3(x,y,z)
% qsPos = [-10, 20, 3]; 
% qfPos = [50, 10, 1];

% %initial and final position of end effector Q4(x,y,z)
% qsPos = [85, 15, 4]; 
% qfPos = [-6, 27, 2];

%inverse kinematics of current and final positions to get joint angles
qs = InverseKin(qsPos);
qf = InverseKin(qfPos);

%obstacle centroids
dealerShoePos = [0, 20, 2.1];
chipTrayPos = [35, 16.25, 2]; 
discardTrayPos = [70, 20, 2.5];

%define step size of ith iteration
alpha = 0.01;

%region of convergence 
e = 0.04;

%index tracker
i = 1;

%insert initial configuration into the routes matrix which will store all configurations of the path
route(1:3, i) = transpose(qs);

%infinite loop
while(true)
    
    %if the norm distance has not reached the region of convergence get the
    %current positon of the robot and compute the artifical forces by
    %calling the ForceMap function
    if ((norm(route(1:3,i) - transpose(qf))) > e)
        
        qc = transpose(route(1:3, i));
        T = ForceMap_new(qc, qf, dealerShoePos, chipTrayPos, discardTrayPos); %function call to get joint torque summation
        
        route(1:3, i+1) = route(1:3, i) + (alpha*(T)/norm(T)); %adding the new configuration to the routes matrix
        
        i = i + 1; %increment index tracker
        
    else %break out of infinite loop when the robot is within the region of convergence
        break       
    end
end


%Plotting the End Effector Route

%defining the radii of the obstacles (considering each obstacle as a sphere in the workspace for simplicity)
dealershoeRadius = 5.75; 
chiptrayRadius = 11.5; 
discardTrayRadius = 5; 

%get # of configurations inside route matrix
ncol = size(route, 2);

%loop through each configuration and use forward kinematics to get end effector position and plot it to show path
for j = 1:ncol
   [T01, T02, T03] = ForwardKin(transpose(route(1:3, j)));
   scatter3(T03(1,4), T03(2,4), T03(3,4), 'k.');
   hold on
end

title('Blackjack Dealing Robot End Effector Path'); %title of plot

%plotting intial end effector position
scatter3(qsPos(1), qsPos(2), qsPos(3), 750, 'm.');
text(qsPos(1)+5, qsPos(2)+3, qsPos(3)+10, 'Initial Position', 'Fontsize', 7)
hold on

%plotting goal end effector position
scatter3(qfPos(1), qfPos(2), qfPos(3), 750, 'm.');
text(qfPos(1), qfPos(2)-4, qfPos(3)+5, 'Goal Position', 'Fontsize', 7)
hold on

%plotting dealerShoe obstacle centroid
scatter3(dealerShoePos(1), dealerShoePos(2), dealerShoePos(3), 200, 'r*');
text(dealerShoePos(1)-2, dealerShoePos(2)+8, dealerShoePos(3)+20, 'Dealer Shoe', 'Fontsize', 7)
hold on

%plotting chipTray obstacle centroid
scatter3(chipTrayPos(1), chipTrayPos(2), chipTrayPos(3), 200, 'r*');
text(chipTrayPos(1), chipTrayPos(2)-10, chipTrayPos(3)+10, 'Chip Tray', 'Fontsize', 7)
hold on

%plotting discardTray obstacle centroid
scatter3(discardTrayPos(1), discardTrayPos(2), discardTrayPos(3), 200, 'r*');
text(discardTrayPos(1), discardTrayPos(2)-10, discardTrayPos(3)+10, 'Discard Tray', 'Fontsize', 7)
hold on

%plotting dealer shoe sphere shape
[x1,y1,z1] = sphere;
x1 = (x1*dealershoeRadius) + dealerShoePos(1);
y1 = (y1*dealershoeRadius) + dealerShoePos(2);
z1 = (z1*dealershoeRadius) + dealerShoePos(3);
surf(x1,y1,z1, 'FaceColor', 'r')
hold on

%plotting chip tray sphere shape
[x2,y2,z2] = sphere;
x2 = (x2*chiptrayRadius) + chipTrayPos(1);
y2 = (y2*chiptrayRadius) + chipTrayPos(2);
z2 = (z2*chiptrayRadius) + chipTrayPos(3);
surf(x2,y2,z2, 'FaceColor', 'r')
hold on

%plotting discard tray sphere shape
[x3,y3,z3] = sphere;
x3 = (x3*discardTrayRadius) + discardTrayPos(1);
y3 = (y3*discardTrayRadius) + discardTrayPos(2);
z3 = (z3*discardTrayRadius) + discardTrayPos(3);
surf(x3,y3,z3, 'FaceColor', 'r')
hold on

%axes labels
xlabel('x') 
ylabel('y')
zlabel('z')

%showing limits of workspace on plot
xlim([-20, 90])
ylim([-11.5, 58.5])
zlim([0, 110])





