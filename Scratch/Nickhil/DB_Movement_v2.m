
% 15/10

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Create robot and environment

clf
    % Docking the figure to the window on the right hand side 
set(0,'DefaultFigureWindowStyle','docked')              

    %move the dispensing robot ee to this coordinate
 grill_pos = [-0.3 ,0.2, 0];

    %initialise dispensing robot
robot = IRB_910sc;

    %note - let us manually interact with the robot  
robot.model.teach;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Move robot arm to grill location

    %Create a vector of initial joint angles, in this case the joint angles 
    %are zero
q = zeros(1,3);

    %Inverse kinematics to find joint angles needed to move to brick spawn
    %coords brick1_xyz.
            %note - trotx(pi) makes end-effector face down when going to a 
                    %brick, so arm does not go into floor. 
    q1 = robot.model.ikine(transl(grill_pos) * trotx(pi), q, [1,1,1,0,0,0]);


    %jtraj creates a path between one set of joint positions (q) and a second
    %set of joint positions (q1) using a certain amount of set increments (50)
path = jtraj (q,q1,50);

    %Put the jtraj to action usiing for loop 
     for i = 1:50
        pause(0.01);

        %Animate actually makes the arm move 
        %i,: is just the saying the current ith row and all columns
      robot.model.animate(path(i,:));
      
        %drawnow() displays the arm movement 
      drawnow()
    end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Make dispensing projection line

hold on;
blastStartTr = robot.model.fkine(q1);
blastStartPnt = blastStartTr(1:3,4)';

%note - this sets up the blast stream length in the x,y or z direction
blastEndTr = robot.model.fkine(q1) * transl(0,0,-0.7);
blastEndPnt = blastEndTr(1:3,4)';

%note - this projects line out of the end effector 
    %choose colour using hexidecimal
blastPlot_h = plot3([blastStartPnt(1),blastEndPnt(1)],[blastStartPnt(2),blastEndPnt(2)],[blastStartPnt(3),blastEndPnt(3)],'Color', "#FFE7B4");
axis equal;

