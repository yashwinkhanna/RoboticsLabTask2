
% 14/10

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Create robot and environment

clf

    % Docking the figure to the window on the right hand side 
set(0,'DefaultFigureWindowStyle','docked')

% Make a 3DOF model
% L1 = Link('d',1,'a',1,'alpha',-pi,'qlim',[-pi/2 pi/2]);
%     %change pi/2 to 0 if we want to change it back to top down arm set up 
%     %L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi/2 pi/2])
% %L2 = Link('d',0,'a',1,'alpha',pi/2,'qlim',[-pi/2 pi/2])
% L2 = Link('d',0,'a',1,'alpha',pi,'qlim',[-pi/2 pi/2]);    
% L3 = Link('d',-0.5,'a',0,'alpha',pi,'qlim',[-pi/2 pi/2]);

L1 = Link('d',0.2577,'a',0.3,'alpha',0,'qlim',deg2rad([-360 360]), 'offset',0);
L2 = Link('d',0,'a',0.25,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
L3 = Link('d',0.2577*2,'a',0,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);

robot = SerialLink([L1 L2 L3],'name','myRobot');                     

% Make workspace big except in the negative z
workspace = [-4 4 -4 4 0 4];                                       % Set the size of the workspace when drawing the robot
        
scale = 0.5;

% Create a vector of initial joint angles        
q = zeros(1,3);                                                     
       
% Plot the robot
robot.plot(q,'workspace',workspace,'scale',scale);                  

    %change this to move the end-effector of dispensing bot 
 sandwich_pos = [1.5 ,-1, 0.7];
%  swiggle_pos1 = [1.3,-1  ,1];
%  swiggle_pos2 = [1.6,-1.5,1];
%  swiggle_pos3 = [2  ,-1  ,1];

%robot = DispensingBot2;

    %note - let us manually interact with the robot  
%robot.model.teach;

robot.teach;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Move robot arm to sandwich location

    %Create a vector of initial joint angles, in this case the joint angles 
    %are zero
q = zeros(1,3);

    %Inverse kinematics to find joint angles needed to move to brick spawn
    %coords brick1_xyz.
            %note - trotx(pi) makes end-effector face down when going to a 
                    %brick, so arm does not go into floor. 
    q1 = robot.ikine(transl(sandwich_pos) * trotx(pi), q, [1,1,1,0,0,0]);


    %jtraj creates a path between one set of joint positions (q) and a second
    %set of joint positions (q1) using a certain amount of set increments (50)
path = jtraj (q,q1,50);

    %Put the jtraj to action usiing for loop 
     for i = 1:50
        pause(0.01);

        %Animate actually makes the arm move 
        %i,: is just the saying the current ith row and all columns
      robot.animate(path(i,:));
      
        %drawnow() displays the arm movement 
      drawnow()
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Make liquid projection line
hold on;
blastStartTr = robot.fkine(q1);
blastStartPnt = blastStartTr(1:3,4)';

% 2.8 (Bonus) Get the end of the stream with TR * transl (blast stream length (i.e. 1m along z) 
blastEndTr = robot.fkine(q1) * transl(0,-1,0);
blastEndPnt = blastEndTr(1:3,4)';

% 2.9	(Bonus) Project a line out of the end effector (i.e. a mock grit-blasting stream)
blastPlot_h = plot3([blastStartPnt(1),blastEndPnt(1)],[blastStartPnt(2),blastEndPnt(2)],[blastStartPnt(3),blastEndPnt(3)],'b');
axis equal;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %% Draw swiggle 1 with sauce 
% 
%     %Inverse kinematics to find joint angles needed to move to brick spawn
%     %coords brick1_xyz.
%             %note - trotx(pi) makes end-effector face down when going to a 
%                     %brick, so arm does not go into floor. 
%     q2 = robot.ikine(transl(swiggle_pos1) * trotx(pi), q, [1,1,1,0,0,0]);
% 
%     %jtraj creates a path between one set of joint positions (q) and a second
%     %set of joint positions (q1) using a certain amount of set increments (50)
% path = jtraj (q1,q2,15);
% 
%     %Put the jtraj to action usiing for loop 
%      for i = 1:15
%         pause(0.01);
% 
%         %Animate actually makes the arm move 
%         %i,: is just the saying the current ith row and all columns
%       robot.animate(path(i,:));
% 
%         %drawnow() displays the arm movement 
%       drawnow()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TRYING TO MAKE SAUCE PROJECTION MOVE WITH ARM (NOT WORKING)
% hold on;
% 
% x = robot.getpos;
% pos = robot.fkine(x);
% 
% q_sauce = robot.ikcon(transl(pos(1,4), pos(2,4), pos(3,4)) *trotx(pi), q);
% 
% blastStartTr = robot.fkine(q_sauce);
% blastStartPnt = blastStartTr(1:3,4)';
% 
% % 2.8 (Bonus) Get the end of the stream with TR * transl (blast stream length (i.e. 1m along z) 
% blastEndTr = robot.fkine(q_sauce) * transl(0,-1,0);
% blastEndPnt = blastEndTr(1:3,4)';
% 
% % 2.9	(Bonus) Project a line out of the end effector (i.e. a mock grit-blasting stream)
% blastPlot_h = plot3([blastStartPnt(1),blastEndPnt(1)],[blastStartPnt(2),blastEndPnt(2)],[blastStartPnt(3),blastEndPnt(3)],'b');
% axis equal;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%end


% %% Draw swiggle 2 with sauce 
% 
%     %Inverse kinematics to find joint angles needed to move to brick spawn
%     %coords brick1_xyz.
%             %note - trotx(pi) makes end-effector face down when going to a 
%                     %brick, so arm does not go into floor. 
%     q3 = robot.ikine(transl(swiggle_pos2) * trotx(pi), q, [1,1,1,0,0,0]);
% 
% 
%     %jtraj creates a path between one set of joint positions (q) and a second
%     %set of joint positions (q1) using a certain amount of set increments (50)
% path = jtraj (q2,q3,15);
% 
%     %Put the jtraj to action usiing for loop 
%      for i = 1:15
%         pause(0.01);
% 
%         %Animate actually makes the arm move 
%         %i,: is just the saying the current ith row and all columns
%       robot.animate(path(i,:));
%       
%         %drawnow() displays the arm movement 
%       drawnow()
% end
% 
% 
% %% Draw swiggle 3 with sauce 
% 
%     %Inverse kinematics to find joint angles needed to move to brick spawn
%     %coords brick1_xyz.
%             %note - trotx(pi) makes end-effector face down when going to a 
%                     %brick, so arm does not go into floor. 
%     q4 = robot.ikine(transl(swiggle_pos3) * trotx(pi), q, [1,1,1,0,0,0]);
% 
% 
%     %jtraj creates a path between one set of joint positions (q) and a second
%     %set of joint positions (q1) using a certain amount of set increments (50)
% path = jtraj (q3,q4,15);
% 
%     %Put the jtraj to action usiing for loop 
%      for i = 1:15
%         pause(0.01);
% 
%         %Animate actually makes the arm move 
%         %i,: is just the saying the current ith row and all columns
%       robot.animate(path(i,:));
%       
%         %drawnow() displays the arm movement 
%       drawnow()
% end

