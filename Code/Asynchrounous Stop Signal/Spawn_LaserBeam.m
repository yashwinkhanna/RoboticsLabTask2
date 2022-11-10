    %Laser beam 1
laserStartPnt = laser_origin;
laserEndPnt = [3,2,0];

%This projects a line out of the irb end effector 
    %choose colour using hexidecimal
laserPlot_h = plot3([laserStartPnt(1),laserEndPnt(1)],[laserStartPnt(2),laserEndPnt(2)],[laserStartPnt(3),laserEndPnt(3)],'Color', 'r');
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
laserEndPnt4 = [3,2,-0.2];

%This projects a line out of the irb end effector 
    %choose colour using hexidecimal
laserPlot_h4 = plot3([laserStartPnt4(1),laserEndPnt4(1)],[laserStartPnt4(2),laserEndPnt4(2)],[laserStartPnt4(3),laserEndPnt4(3)],'Color', 'r');
axis equal;