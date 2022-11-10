
                        % ROBOT PANCAKE CHEF

% By:
    %  Nickhil Naiker 
    %  Stefano Mazzotta 
    %  Yashwin Khanna 

% Steps:
    %  1) Move IRB end-effector above grill
    %  2) Dispense pancake batter
    %  3) Animate pancake pouring
    %  4) Move IRB away
    %  5) Move LinUR3 to grill
    %  6) Flip pancake
    %  7) Spatula to grill, side of pancake
    %  8) LinUR3 pick up pancake
    %  9) Move pancake to plates
    % 10) Drop pancake on plates

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% File Set up

clf
clear all    
set(0,'DefaultFigureWindowStyle','docked')   % Docking the figure to the window on the right hand side  
estop = ES_GUI;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Create robots and environment           

hold on; % To retain current plot or graphic

% Displaying a concrete floor
    % note - the [x,x,x,x],[y,y,y,y],[z,z,z,z] are corner points so we
    % know where to map the image
surf([-2,-2;3.25,3.25],[-2.5,4;-2.5,4],[-0.65,-0.65;-0.65,-0.65],'CData',imread('concrete.jpg'),'FaceColor','texturemap'); 

%Pancake position on griddle
    cakepos = [-1.25+0.4, 0.035, 0.075];
 
%Pancake dispensing on grill position for IRB 910
    cakepos_irb = [-1.25+0.4, 0.035, 0.16];

%Pancake picking up from grill position for LinUR3
    cakepos_ur3 = [-1.25+0.5, 0, 0.075]; 

%Final position where pancake is put onto the plates
    finish_pos = [0.1,0,0];
plate_stack = [-0.35, 0, 0];

%How high to lift up the pancake when picking it up/putting it down 
    lift = 0.065; 

%Height of a pancake
    cake_H = 0.05; 

%Table
PlaceObject('newroboticstable.ply', [0,0,-0.0844]);
hold on;

%Griddle
PlaceObject('griddle.ply', [-0.55,-0.15,0]);
hold on;

%Plate pancake to be placed on
plate_obj = PlaceObject('plate.ply', [plate_stack(1), plate_stack(2), plate_stack(3)+0.08]);

%Plate stack
PlaceObject('plate.ply', [plate_stack(1),plate_stack(2),plate_stack(3)+0.00]); hold on;
PlaceObject('plate.ply', [plate_stack(1),plate_stack(2),plate_stack(3)+0.02]); hold on;
PlaceObject('plate.ply', [plate_stack(1),plate_stack(2),plate_stack(3)+0.04]); hold on;
PlaceObject('plate.ply', [plate_stack(1),plate_stack(2),plate_stack(3)+0.06]); hold on;

%Status traffic lights
trafficlightgreen = PlaceObject('trafficlightgreen.ply', [-0.75,0.75,0]); 
hold on;
trafficlightyellow = PlaceObject('trafficlightyellow.ply', [-0.75,0.75,0]); 
hold on;
delete(trafficlightyellow);

        %%%%%%%
%Kitchen Environment Enable = 1/Disable = 0
enviro = 1;
if enviro == 1
    environmentRPC;
end
        %%%%%%%

irb = IRB_910sc; %Initialise the IRB and UR3 robot
ur3 = Linear_UR3(false);

% irb.model.teach; %Manually interact with the robot
% ur3.model.teach;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Asyncronous Setup 

PlaceObject('lasergate.ply', [3,2, 0.05]);    %Laser gate  
hold on;
PlaceObject('lasergate.ply', [1.5,2,0.05]);  % laser gate
hold on;

        %%%%%%%
%Change to 1, to have a person go through gate and trigger Asyncronous Stop
    %Signal (calls Async_MainImp file in step 5)
Async_Mode = 0;

        %%%%%%%

    %Coords for person and laser gate
laser_origin = [1.5,2,0];
    %Where we want the person to spawn intially, for Async Stop Signal
person_coords = [2.2, 4, -0.65];

steps = 50;

%Code that creates all the laser beams for gate (inside Asyncronous folder)
Spawn_LaserBeam;

    %Initialise the RMRC class. Class performs traj and movement animations
resolve = RMRC(); 

offset_y = 0.1  ;
offset_z = 0.045;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1) Move IRB dispenser above grill

pause(5);
    
q = zeros(1,3);  %Create a vector of initial joint angles, beginning at 0

    %Inverse kinematics to find joint angles needed to move IRB to grill 
    %position (cakepos_irb). trotx(pi) makes ee face down when going to a point, so arm does not go into floor
q1 = irb.model.ikine(transl(cakepos_irb) * trotx(pi), q, [1,1,1,0,0,0]);

    %jtraj creates a path between one set of joint positions (q) and a second
    %set (q1) with a certain amount of set increments (50)
path = jtraj (q,q1,50);

    %Light System = Green to indicate the system is running 
booleantrafficlightgreen = 1;

    %Put the jtraj to action using for loop 
     for i = 1:50
         pause(0.01);
         irb.model.animate(path(i,:));  %Animate plots the arm movement. i,: is current ith row and all columns
         drawnow()  %drawnow(), displays the arm movement in the figure

            %Checks if the E-stop is pressed after each robot animation
         isEStop;
     end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2) Make the IRB dispensing line

hold on;
%Forward kinematics to determine robot end-effector position in the form of a 4 by 4 transformation matrix 
    blastStartTr = irb.model.fkine(q1);
%We only want the first 3 rows and 4th coloumn (x,y,z) 
    blastStartPnt = blastStartTr(1:3,4)';

%Blast stream length in x,y,z axis
    %length in z is end-effector above grill (cakepos_irb) - position of pancake
    %(cakepos)
blastEndTr = irb.model.fkine(q1) * transl(0,0,cakepos_irb(3)-cakepos(3));
blastEndPnt = blastEndTr(1:3,4)';

%This projects a line out of the irb end effector 
    %choose colour using hexidecimal
blastPlot_h = plot3([blastStartPnt(1),blastEndPnt(1)],[blastStartPnt(2),blastEndPnt(2)],[blastStartPnt(3),blastEndPnt(3)],'Color', "#FFE7B4");
axis equal;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3) Pancake batter pour animation

%Animation speed
anim_speed = 0.5; 

num_cakes = 5;
p_index = 50;
p_index0 = p_index;

%Start with the smallest pancake size 
    cake_now = PlaceObject('pancake_50.ply', cakepos);

%Iterate through all our pancake sizes 
for n = 1:num_cakes
    pause(anim_speed);
    delete(cake_now)
        cake_now = PlaceObject(['pancake_',num2str(p_index),'.ply'], cakepos);
    p_index = p_index0 + n*25;
        %Checking if E-stop is being pressed during this
    isEStop;
    
    %SinisEStop only stops robot movement. if statement below is needed to
    %stop blast pouring anim
    if estop.ButtonValuePublic == 1
        delete(blastPlot_h);
    end
end
    %Stop dispensing projection line
delete(blastPlot_h);       

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4) Move IRB back to the home pose

 %Home coords   
     home = [-1.25,0,0.2577];     

 %Inverse kinematics to find joint angles needed to move the IRB to 
 %home (home). trotx(pi) makes ee face down when going to a point, so arm does not go into floor
    qh = irb.model.ikine(transl(home) * trotx(pi), q,[1,1,1,0,0,0]);   

 %jtraj creates a path between one set of joint positions (q) and a second
 %set (q1) with a certain amount of set increments (50)
    path = jtraj (q1,qh,50);

    %Light System = Green to indicate that the system is running 
booleantrafficlightgreen = 1;

    %Put the jtraj to action using for loop 
     for i = 1:50
        pause(0.01);

      %Animate actually makes the arm move 
      %i,: is just the saying the current ith row and all columns
        irb.model.animate(path(i,:));
      
      %drawnow() displays the arm movement 
        drawnow()
        
      %Checking if the E-stop button is being pressed during this
        isEStop;
     end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 5) Move UR3 to pancake's position

%Create a vector of initial joint angles
    q = zeros(1,7);

%Tried to do inverse kinematics to find the joint angles needed to move to the 
%cakepos_ur3 coords. 
    % q1 = ur3.model.ikine(transl(cakepos_ur3) * trotx(pi), q, [1,1,1,0,0,0]);

%But preset joint angles provided by ikine were 'broken' arm positions that prevented next movement. Preset joint 
%angles found using teach, given we have pre-set griddle and pancake position
    q1 = [-0.600         0    0.5341    1.8221    0.7854    1.5708        0];

%Creating a trajectory between two joint angle positions
    path = jtraj (q,q1,steps);

    %Light System = Green to indicate system is running 
booleantrafficlightgreen = 1;

    %Put jtraj to action using for loop 
for i = 1:steps
    pause(0.01);

  %Animate actually makes the arm move 
  %i,: is just the saying the current ith row and all columns
    ur3.model.animate(path(i,:));

 %drawnow(), displays the arm movement
    drawnow()  

 %Asyncronous Stop 
    %Runs Async_MainImp file that makes person spawn in and move towards gate, stopping the robots once the person has 
    %passed it. 
              if Async_Mode == 1
                 Async_MainImp;
              end 

    isEStop; %Checks if EStop button is pressed after each animation movement
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 6) Move to slide spatula under cake

    %Get current UR3 joint angle position 
q0 = ur3.model.getpos;
    %Assign it to q(1)
q1 = q0;
    %Change only the first joint value by -0.14 (to move it along linear rail)
q1(1) = q1(1) - 0.14;

    %Create a trajectory between q0 and q1
path = jtraj(q0, q1, steps);

    %Animate each step
for i = 1:steps
    pause(0.01);
    ur3.model.animate(path(i,:));
    drawnow()
    
    %Check if E-stop is pressed during thi s
    isEStop;
end
     
pause(1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 7) Moves UR3 up to lift up the cake

    %Get current UR3 joint angle position 
q0 = ur3.model.getpos;

   % resolve.axial is a function that allows us to input a robot, the axis
   % we want to move in, the current end-effector position, displacement we want to move, time we want it done in 
   %
   % Outputs a matrix, like "path" of joint angle positins to go from point 
   % A to B in rmrc.  
   rmMatrix = resolve.axial(ur3 , 'z' , ur3.model.fkine(q0) , 0.06 , 1);

    %Light System = Green (robots running)
booleantrafficlightgreen = 1;

    %Animating the outputted RMRC rmMatrix  
for i = 1:resolve.steps
  pause(0.01);    
  ur3.model.animate(rmMatrix(i,:)); 
  drawnow() 
  
  delete(cake_now);
    %Get position of end-effector using fkine 
  EE_pos = ur3.model.fkine(ur3.model.getpos);
    %Make spat_pos = to cake postion on grill (cakepos)
  spat_pos = cakepos; 
    %Make cake move up with UR3 spatula and Subtract cake height from cakepos so spatula is below pancake, not inside
  spat_pos(3) = EE_pos(3, 4) - 0.05; 
  cake_now = PlaceObject('pancake_150.ply', spat_pos);
  
  isEStop; %Check if E-stop is being pressed
end
         
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 8) Rotate UR3 to flip cake onto grill

    %Get current UR3 joint angle position 
q0 = ur3.model.getpos;
    %Assign it to q(1)
q1 = q0;
    %Desired spatula rotation 
EE_rot = pi; 
 %We only change the 7th joint value by pi, to rotate ee/spatula by 180 degrees
q1(7) = q1(7)+EE_rot;
    
    %Create a trajectory between q0 and q1
path = jtraj(q0, q1, steps);

%Arguements needed to use the rotation function
    %
    %Angle increment for rotation in each step
alpha = 180/steps; 
    %To use rotate func, need to give xyz 'origin' for cake_now to rotate about
flip_about = spat_pos; 
flip_about(3) = spat_pos(3) + 0.05;

    %Light System = Green (robots are running)
booleantrafficlightgreen = 1;

    %Animate the path trajectory from q0 to q1
for i = 1:steps
    pause(0.01);
    ur3.model.animate(path(i,:));
    drawnow()

        %Rotate function to rotate our pancake place object
    rotate(cake_now, [0 1 0], alpha, flip_about); % rotate the pancake placeobject. matlab function (not toolbox) [0 1 0] to rotate about y axis - masking off 
                                                  % syntax: rotate(object, axis mask, angle to rotate each step, xyz origin to rotate about)
    isEStop; 
end

delete(cake_now);
    %cake_now is variable for where the pancake placeobject currently is in
    %the workplace 
cake_now = PlaceObject('pancake_150.ply', cakepos);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 9) Move UR3 to pick up the cake again

%%%% Moving spatula to left and rotate it

    %Get current UR3 joint angle position 
q0 = ur3.model.getpos;
    %Assign it to q(1)
q1 = q0;
    %We change the 1st joint value, to move the UR3 along linear rail and
    %we change the last joint value to rotate the end-effector / spatula.
q1(1) = q1(1) + 0.085;
q1(7) = q1(7) + pi/ 2;

    %Creating trajectory between q0 and q1 of joint angle positions within
    %steps 
path = jtraj(q0, q1, steps);

    %Animate this generated trajectory  
for i = 1:steps
    pause(0.01);
    ur3.model.animate(path(i,:));
    drawnow()
end

%%%% Moving spatula down to the griddle

   %Get current UR3 joint angle position 
q0 = ur3.model.getpos;

   % resolve.axial is a function that allows us to input a robot, the axis
   % we want to move in, the current end-effector position, displacement we want to move, time we want it done in 
   %
   % Outputs a matrix, like "path" of joint angle positins to go from point 
   % A to B in rmrc. 
newm = resolve.axial(ur3, 'z', ur3.model.fkine(ur3.model.getpos), - lift, 1);

   %Light System = Green (robots running)
booleantrafficlightgreen = 1;

    %Animate RMRC (newm matrix) trajectory
for i = 1:size(newm, 1)
  pause(0.01);    
  ur3.model.animate(newm(i,:)); %Animate plots arm movement
  drawnow() %drawnow(), displays the arm movement in figure   
  isEStop;
end

% An alternative method to lift pancake 
    %Did not work
% q0 = ur3.model.getpos;
% q1 = ur3.model.ikcon(transl(cakepos_ur3(1) + 0.15,cakepos_ur3(2)...
% ,cakepos_ur3(3))*trotx(-pi),q0);
% path = jtraj(q0, q1, steps);
% for i = 1:steps
%     pause(0.01);
%     ur3.model.animate( path(i, :));
%     drawnow()
% end

%%%% Move spatula right and scoop pancake

   %Get current UR3 joint angle position 
q0 = ur3.model.getpos;
    %Assign it to q(1)
q1 = q0;
    %We change the 1st joint value, to move the UR3 along linear rail and
    %we change the last joint value to rotate the end-effector / spatula.
q1(1) = q1(1) - 0.075;
q1(7) = q1(7) + pi/ 2;

    %Creating trajectory between q0 and q1 of joint angle positions within
    %steps 
path = jtraj(q0, q1, steps);

    %Animate this generated trajectory  
for i = 1:steps
    pause(0.01);
    ur3.model.animate(path(i,:));
    drawnow()
end

%%%% Move spatula upward to lift pancake

   %Get current UR3 joint angle position 
q0 = ur3.model.getpos;

   % resolve.axial is a function that allows us to input a robot, the axis
   % we want to move in, the current end-effector position, displacement we want to move, time we want it done in 
   %
   % Outputs a matrix, like "path" of joint angle positins to go from point 
   % A to B in rmrc. 
rmMatrix = resolve.axial(ur3, 'z', ur3.model.fkine(q0), lift, 1);

   %Light System = Green (robots running)
booleantrafficlightgreen = 1;

    %Animating RMRC (rmMatrix) trajectory
for i = 1:resolve.steps
    pause(0.01);    
    ur3.model.animate(rmMatrix(i,:));  % animate plots the arm movement
    drawnow()  % drawnow(), displays the robot arm movement in a figure 
  
        %Delete cake placeobject at old spatula location
    delete(cake_now);
        %Making sure pancake placeobject follows spatula 
    EE_pos = ur3.model.fkine(ur3.model.getpos);
    spat_pos = cakepos;
    spat_pos(3) = EE_pos(3, 4)-0.05;
        %Spawns cake placeobject in new spatula location
    cake_now = PlaceObject('pancake_150.ply', spat_pos);

  %Check if E-stop is being pressed
  isEStop;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 10) Move UR3/cake to plating position

   %Get current UR3 joint angle position 
q0 =  ur3.model.getpos ;
    %Assign it to q(1)
q1 = q0;
    %We change the 1st joint value, to move the UR3 along linear rail 
q1(1) = q1(1) - (cakepos(1) - plate_stack(1)); % Subtracting (cakepos) position of cake when on griddle by plate stack will give us the distance needed to travel to plate stack

    %Creating trajectory between q0 and q1 of joint angle positions in
    %steps 
path =jtraj(q0,q1,steps);

   %Light System = Green (robots running)
booleantrafficlightgreen = 1;

    %Animate path trajectory from q0 to q1
for i = 1:steps
    pause(0.01);
    ur3.model.animate(path(i,:));
    drawnow()
    
            %Delete cake placeobject at old spatula location
    delete(cake_now);
            %Making sure pancake placeobject follows spatula 
    EE_pos = ur3.model.fkine(ur3.model.getpos);
    spat_pos(1) = EE_pos (1, 4);
    spat_pos(2) = cakepos(2); 
    spat_pos(3) = EE_pos (3, 4) - 0.05;
            %Spawns cake placeobject in new spatula location
    cake_now = PlaceObject('pancake_150.ply', spat_pos);

    %Check if E-stop is being pressed 
    isEStop;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 11) Turn spatula to put cake on plate

   %Get current UR3 joint angle position 
q0 = ur3.model.getpos;
    %Assign it to q(1)
q1 = q0;
EE_rot = pi;
    %We only change the last joint value to rotate the end-effector/spatula.
q1(7) = q1(7)+EE_rot;

    %Creating trajectory between q0 and q1 of joint angle positions within
    %steps 
path = jtraj(q0, q1, steps);


%Arguements needed to use the rotation function
    %
    % Angle increment for rotation in each step
alpha = 180/ steps;
    %In rotate func, need to give xyz 'origin' for cake_now to rotate about
flip_about = spat_pos; 
flip_about(3) = spat_pos(3) + 0.05;

   %Light System = Green (the robot is running)
booleantrafficlightgreen = 1;

    %Animate the path trajectory from q0 to q1
for i = 1:steps
    pause(0.01);
    ur3.model.animate(path(i,:));
    drawnow()

        %Rotate function to rotate our pancake place object    
    rotate(cake_now, [0 1 0], alpha, flip_about); % rotate the pancake placeobject. matlab function (not toolbox) [0 1 0] to rotate about y axis - masking off 
                                                  % syntax: rotate(object, axis mask, angle to rotate each step, xyz origin to rotate about)
    %Checking if E-stop button is being pressed
    isEStop;
end
    %Delete the pancake that was on the spatula 
delete(cake_now);

    %Place finished pancake on stack of plates 
cake_now = PlaceObject('pancake_150.ply', [plate_stack(1), plate_stack(2), plate_stack(3)+0.095]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %Light System = Red, robot task completed
if booleantrafficlightgreen == 1
    delete (trafficlightgreen);
       trafficlightred = PlaceObject('trafficlightred.ply', [-0.75,0.75,0]);  
       hold on;
end

pause(5);