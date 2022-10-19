
% 15/10

clf
    % Docking the figure to the window on the right hand side 
set(0,'DefaultFigureWindowStyle','docked')   

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Create robots and environment           

    % To retain current plot or graphic 
hold on; 

    % Displaying a concrete floor
        % note - the [x,x,x,x],[y,y,y,y],[z,z,z,z] are corner points so we
                 % know where to map the image
surf([-2,-2;3.25,3.25],[-2,2;-2,2],[-0.65,-0.65;-0.65,-0.65],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

    %Table
PlaceObject('newroboticstable.ply', [0,0,-0.0844]);
hold on;

PlaceObject('kitchenenvironment.ply', [-2,-2,-0.7]);     %Loading in kitchen environment
hold on;

PlaceObject('cookingplate.ply', [2,0,0]);     %Loading in kitchen environment
hold on;

PlaceObject('griddle.ply', [-0.35,-0.15,0]);     %Loading in kitchen environment
hold on;

PlaceObject('pancake_150.ply', [-0.6,0,0.1]);     %Loading in kitchen environment
hold on;


    %Move the robots end effector to this coordinate
 irb_pos = [ -1, 0, 0.01];
 ur3_pos = [ 0.1,0.4, 0];

    %Initialise the IRB and UR3 robot
irb = IRB_910sc;
ur3 = Linear_UR3(false);

    %Manually interact with the robots  
irb.model.teach;
ur3.model.teach;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Creating the IRB ee Spatula 

    %SpatulaClass is a class and () indicates how many spatulas we want made
create_spatula = SpatulaClass(1);

    %SpatulaSpawn is a function we made in SpatulaClass to create and spawn
    %the spatula at an x,y,theta coordinate   
create_spatula.SpatulaSpawn(1,-0.7,0.5,0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Move IRB to grill position

    %Create a vector of initial joint angles, in this case the joint angles 
    %are zero
q = zeros(1,3);

    %Inverse kinematics to find joint angles needed to move to brick spawn
    %coords brick1_xyz.
            %note - trotx(pi) makes end-effector face down when going to a 
                    %brick, so arm does not go into floor. 
    q1 = irb.model.ikine(transl(irb_pos) * trotx(pi), q, [1,1,1,0,0,0]);


    %jtraj creates a path between one set of joint positions (q) and a second
    %set of joint positions (q1) using a certain amount of set increments (50)
path = jtraj (q,q1,50);

    %Put the jtraj to action usiing for loop 
     for i = 1:50
        pause(0.01);

        %Animate actually makes the arm move 
        %i,: is just the saying the current ith row and all columns
      irb.model.animate(path(i,:));
      
        %drawnow() displays the arm movement 
      drawnow()
     end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Make IRB dispensing line

hold on;
blastStartTr = irb.model.fkine(q1);
blastStartPnt = blastStartTr(1:3,4)';

%Blast stream length in x,y,z axis
blastEndTr = irb.model.fkine(q1) * transl(0,0,-0.75);
blastEndPnt = blastEndTr(1:3,4)';

%This projects a line out of the irb end effector 
    %choose colour using hexidecimal
blastPlot_h = plot3([blastStartPnt(1),blastEndPnt(1)],[blastStartPnt(2),blastEndPnt(2)],[blastStartPnt(3),blastEndPnt(3)],'Color', "#FFE7B4");
axis equal;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Pancake pouring animation

    %animation speed
pause(0.5);
    PlaceObject('pancake_50.ply', irb_pos);

pause(0.5);
    PlaceObject('pancake_75.ply', irb_pos);
        %deleting previous pancake
    delete(PlaceObject('pancake_50.ply', irb_pos));

pause(0.5);
    PlaceObject('pancake_100.ply', irb_pos);
    delete(PlaceObject('pancake_75.ply', irb_pos));

pause(0.5);
    PlaceObject('pancake_125.ply', irb_pos);
    delete(PlaceObject('pancake_100.ply', irb_pos));

pause(0.5);
    PlaceObject('pancake_150.ply', irb_pos);
    delete(PlaceObject('pancake_125.ply', irb_pos));

    %stop dispensing projection line
 delete(blastPlot_h);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Move IRB back to home pose

    %Inverse kinematics to find joint angles needed to move to brick spawn
    %coords brick1_xyz.
            %note - trotx(pi) makes end-effector face down when going to a 
                    %brick, so arm does not go into floor. 
    qh = irb.model.ikine(transl(0.5,0.2,0) * trotx(pi), q, [1,1,1,0,0,0]);

    %jtraj creates a path between one set of joint positions (q) and a second
    %set of joint positions (q1) using a certain amount of set increments (50)
path = jtraj (q1,qh,50);

    %Put the jtraj to action usiing for loop 
     for i = 1:50
        pause(0.01);

        %Animate actually makes the arm move 
        %i,: is just the saying the current ith row and all columns
      irb.model.animate(path(i,:));
      
        %drawnow() displays the arm movement 
      drawnow()
     end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Move UR3 to pancake position

    %Create a vector of initial joint angles, in this case the joint angles 
    %are zero
q = zeros(1,7);

    %We do nverse kinematics to find joint angles needed to move to the 
    %ur3_pos coords.
            %note - trotx(pi) makes end-effector face down when going to  
                    %coords, so arm does not go into floor. 
    q1 = ur3.model.ikine(transl(ur3_pos) * trotx(pi), q, [1,1,1,0,0,0]);

    %jtraj creates a path between one set of joint positions (q) and a second
    %set of joint positions (q1) using a certain amount of set increments (50)
path = jtraj (q,q1,50);

    %Put the jtraj to action usiing for loop 
     for i = 1:50
        pause(0.01);

        %Animate actually makes the arm move 
        %i,: is just the saying the current ith row and all columns
      ur3.model.animate(path(i,:));
 
      %This makes the base of our spatula equal to the trajectory of our
      %robot arm found using path variable above, hence making it look like the spatula is attatched to the
      %end-effector
            %These are from Lab 4 extension 
  create_spatula.spatula{1}.base = ur3.model.fkine(path(i,:));
  create_spatula.spatula{1}.animate(0);

        %drawnow() displays the arm movement 
      drawnow()
     end

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Move UR3 up to lift pancake 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Rotate UR3 to flip pancake


