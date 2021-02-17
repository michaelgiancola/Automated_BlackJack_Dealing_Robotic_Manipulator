function [Fatt1, Fatt2, Fatt3] = attractiveForces(O1s, O2s, O3s, O1f, O2f, O3f)

    %defining attractive force parameters
    Y = [1, 1, 1]; %zetas [origin 1, origin 2, origin 3]
    d = 10; %transition distance

    %calculating parabolic attractive forces by default for each joint 
    Fatt1 = -Y(1)*(O1s - O1f);
    Fatt2 = -Y(2)*(O2s - O2f);
    Fatt3 = -Y(3)*(O3s - O3f);
    
    %determining origin distances from current position to final position
    d1 = norm(O1s - O1f);
    d2 = norm(O2s - O2f);
    d3 = norm(O3s - O3f);
    
    %determining if parabolic or conic forces will be used for joint 1
    if(d1 > d) %if far, use conic 
        %calculating conic attractive forces by modifying parabolic force
        Fatt1 = (d*Fatt1)/d1;
    end
    
    %determining if parabolic or conic forces will be used for joint 2
    if(d2 > d) %if far, use conic 
        %calculating conic attractive forces by modifying parabolic force
        Fatt2 = (d*Fatt2)/d2;
    end
    
    %determining if parabolic or conic forces will be used for joint 3
    if(d3 > d) %if far, use conic 
        %calculating conic attractive forces by modifying parabolic force
        Fatt3 = (d*Fatt3)/d3;
    end
end