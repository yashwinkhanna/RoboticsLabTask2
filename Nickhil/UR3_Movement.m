
%15/10 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Create robot and environment 

clf
    % Docking the figure to the window on the right hand side 
set(0,'DefaultFigureWindowStyle','docked')              

    %move the UR3 to this x,y,z
 grill_pos = [-0.4 ,0.5, 0];

    %Create the Linear UR3 arm
robot = LinearUR3(false);

    %To interact with q joints 
robot.model.teach;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Move robot arm to grill location

    %Create a vector of initial joint angles, in this case the joint angles 
    %are zero
q = zeros(1,7);

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

