function Async_NoAnimate(obj_coords)

hold on;
workspace = [-2 2 -2 2 -5.67 5];   
camlight;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ENVIRONMENT

    % Displaying a concrete floor
        % note - the [x,x,x,x],[y,y,y,y],[z,z,z,z] are corner points so we
                 % know where to map the image
surf([-2,-2;3.25,3.25],[-2.5,2;-2.5,2],[-0.65,-0.65;-0.65,-0.65],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

    %Coords for person and laser
laser_origin = [1.5,2,0];

%         %outside
% person_coords = [2.2, 4, -0.65];

%         inside
% person_coords = [2.2 ,2 ,-0.65];

person_coords = obj_coords;

    %Display environment objects
PlaceObject('person_async.ply', person_coords);    
 hold on;
% PlaceObject('lasergate.ply', [3,2, 0.05]);     
% hold on;
% PlaceObject('lasergate.ply', [1.5,2,0.05]);  
% hold on;


% function ufo() 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LASER BEAMS

    %Laser beam 1
laserStartPnt1 = laser_origin;
laserEndPnt1 = [3,2,0];

%This projects a line out of the irb end effector 
    %choose colour using hexidecimal
laserPlot_h = plot3([laserStartPnt1(1),laserEndPnt1(1)],[laserStartPnt1(2),laserEndPnt1(2)],[laserStartPnt1(3),laserEndPnt1(3)],'Color', 'r');
axis equal;

    %Laser beam 2
laserStartPnt2 = [1.5,2,0.2];
laserEndPnt2 = [3,2,0.2];

%This projects a line out of the irb end effector 
    %choose colour using hexidecimal
laserPlot_h2 = plot3([laserStartPnt2(1),laserEndPnt2(1)],[laserStartPnt2(2),laserEndPnt2(2)],[laserStartPnt2(3),laserEndPnt2(3)],'Color', 'r');
axis equal;

    %Laser beam 3
laserStartPnt3 = [1.5,2,0.4];
laserEndPnt3 = [3,2,0.4];

%This projects a line out of the irb end effector 
    %choose colour using hexidecimal
laserPlot_h3 = plot3([laserStartPnt3(1),laserEndPnt3(1)],[laserStartPnt3(2),laserEndPnt3(2)],[laserStartPnt3(3),laserEndPnt3(3)],'Color', 'r');
axis equal;

    %Laser beam 4
laserStartPnt4 = [1.5,2,-0.2];
laserEndPnt4 = [1.5,2,-0.2];

%This projects a line out of the irb end effector 
    %choose colour using hexidecimal
laserPlot_h4 = plot3([laserStartPnt4(1),laserEndPnt4(1)],[laserStartPnt4(2),laserEndPnt4(2)],[laserStartPnt4(3),laserEndPnt4(3)],'Color', 'r');
axis equal;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DIST CALC

x_dist = person_coords(1,1) - laser_origin(1,1)
y_dist = person_coords(1,2) - laser_origin(1,2)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONCLUSION

if (0 < x_dist) && (x_dist < 2) && (-0.05 < y_dist) && (y_dist < 0.05)
    disp("LEAVE THE AREA")
else 
    disp("HAVE A NICE DAY")
end 

end





