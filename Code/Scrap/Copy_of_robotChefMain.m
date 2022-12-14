%% LAB TASK 2 - PANCAKE CHEF
%  MASTER MAIN FILE
%  Nickhil Naiker - 
%  Stefano Mazzotta - 
%  Yashwin Khanna - 12919116

% 22/10

% Process:
%  1) Move IRB above grill
%  2) Dispense batter
%  3) Animate pancake pouring
%  4) Move IRB away
%  5) Move LinUR3 to grill
%  6) Flip pancake
%  7) Spatula to grill, side of pancake
%  8) LinUR3 pick up pancake
%  9) LinUR3 pancake to plate
% 10) Pick up plate, move above grill
% 11) Scara dispense syrup
% 12) LinUR3 present plate



% 8) Pick up pancake with Lin UR3
% 9) Place pancake on plate
% 10) IRB Move to plate
% 11) IRB dispense syrup


%% File set up
clf
clear all    
set(0,'DefaultFigureWindowStyle','docked')   % Docking the figure to the window on the right hand side  
estop = GUItest;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Create robots and environment           
hold on; % To retain current plot or graphic

    % Displaying a concrete floor
        % note - the [x,x,x,x],[y,y,y,y],[z,z,z,z] are corner points so we
                 % know where to map the image
surf([-2,-2;3.25,3.25],[-2.5,4;-2.5,4],[-0.65,-0.65;-0.65,-0.65],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

% surf([-2,-2;-2,-2],[-2.6,2;-2.6,2],[-0.65,-0.65;1.5,1.5],'CData',imread('wall.jpg'),'FaceColor','texturemap');

% surf([-2,3.25;-2,3.25],[-2.6,-2.6;-2.6,-2.6],[-0.65,-0.65;1.5,1.5],'CData',imread('wall.jpg'),'FaceColor','texturemap');

%PlaceObject('roboticstable.ply', [-0.25,0.5,-0.07]);    %Table

cakepos = [-1.25+0.4, 0.035, 0.075]; %Pancake position on griddle
cakepos_irb = [-1.25+0.4, 0.035, 0.16]; %[ 0.1 ,0.49, 0.01];                       %Pancake dispense position for IRB 910
cakepos_ur3 = [-1.25+0.5, 0, 0.075]; %[ -0.1   ,0.5,    0.12];    %[ 0.1 ,0.4, 0]        %Pancake pick up pos for LinUR3
finish_pos = [0.1,0,0];
plate_stack = [-0.35, 0, 0];
lift = 0.065;
cake_H = 0.05;

    %Table
PlaceObject('newroboticstable.ply', [0,0,-0.0844]);
hold on;

PlaceObject('griddle.ply', [-0.55,-0.15,0]);     %Loading in kitchen environment
hold on;

plate_obj = PlaceObject('plate.ply', [plate_stack(1), plate_stack(2), plate_stack(3)+0.08]);

% plate_stack = [-0.6,0.1,0]; 
    PlaceObject('plate.ply', [plate_stack(1),plate_stack(2),plate_stack(3)+0.00]); hold on;
    PlaceObject('plate.ply', [plate_stack(1),plate_stack(2),plate_stack(3)+0.02]); hold on;
    PlaceObject('plate.ply', [plate_stack(1),plate_stack(2),plate_stack(3)+0.04]); hold on;
    PlaceObject('plate.ply', [plate_stack(1),plate_stack(2),plate_stack(3)+0.06]); hold on;
%     PlaceObject('plate.ply', [plate_stack(1),plate_stack(2),plate_stack(3)+0.08]); hold on;
%     PlaceObject('plate.ply', [plate_stack(1),plate_stack(2),plate_stack(3)+0.10]); hold on;

% trafficlightgreen = PlaceObject('trafficlightgreen.ply', [-0.75,0.75,0]);     %Loading in kitchen environment
% hold on;
% trafficlightyellow = PlaceObject('trafficlightyellow.ply', [-0.75,0.75,0]);    %Loading traffic cone
% hold on;
% delete(trafficlightyellow);

enviro = 0;
if enviro == 1
    environmentRPC;
end

irb = IRB_910sc;                                        %Initialise the IRB and UR3 robot
ur3 = Linear_UR3(false);

% irb.model.teach;                                        %Manually interact with the robots
% ur3.model.teach;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Async_Mode = 0;
            
                %Coords for person and laser
            laser_origin = [1.5,2,0];
            person_coords = [2.2, 4, -0.65];
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Setup 

% PlaceObject('lasergate.ply', [3,2, 0.05]);    %Laser gate  
% hold on;
% PlaceObject('lasergate.ply', [1.5,2,0.05]);  % laser gate
% hold on;

steps = 50;

%resolve = RMRC(); %initialise RMRC class. Class performs traj and movement animations

offset_y = 0.1;
offset_z = 0.045;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1) Move IRB to Grill Position

pause(5);
    
q = zeros(1,3);         %Create a vector of initial joint angles, beginning at 0

    %Inverse kinematics to find joint angles needed to move to brick spawn
    %coords brick1_xyz.
            %note - trotx(pi) makes end-effector face down when going to a 
                    %brick, so arm does not go into floor. 
q1 = irb.model.ikine(transl(cakepos_irb) * trotx(pi), q, [1,1,1,0,0,0]);


    %jtraj creates a path between one set of joint positions (q) and a second
    %set of joint positions (q1) using a certain amount of set increments (50)
path = jtraj (q,q1,50);

booleantrafficlightgreen = 1;

    %Put the jtraj to action usiing for loop 
     for i = 1:50
         pause(0.01);
         irb.model.animate(path(i,:));     %Animate plots the arm movement. i,: is current ith row and all columns
         drawnow()         %drawnow() displays the arm movement in figure

         %Asyncronous Stop 
              if Async_Mode == 1
                 Async_MainImp;
              end 

         isEStop;
     end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2) Make IRB dispensing line

hold on;
blastStartTr = irb.model.fkine(q1);
blastStartPnt = blastStartTr(1:3,4)';

%Blast stream length in x,y,z axis
blastEndTr = irb.model.fkine(q1) * transl(0,0,cakepos_irb(3)-cakepos(3));
blastEndPnt = blastEndTr(1:3,4)';

%This projects a line out of the irb end effector 
    %choose colour using hexidecimal
blastPlot_h = plot3([blastStartPnt(1),blastEndPnt(1)],[blastStartPnt(2),blastEndPnt(2)],[blastStartPnt(3),blastEndPnt(3)],'Color', "#FFE7B4");
axis equal;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3) Pancake pouring animation

anim_speed = 0.5;       %animation speed 
% pause(anim_speed); 
% p50 = PlaceObject('pancake_50.ply', cakepos);
% 
% pause(anim_speed);
% delete(p50);
% p75 = PlaceObject('pancake_75.ply', cakepos); 
% 
% pause(anim_speed);
% delete(p75)
% p100 = PlaceObject('pancake_100.ply', cakepos);
% 
% pause(anim_speed);
% delete(p100);
% p125 = PlaceObject('pancake_125.ply', cakepos);
% 
% pause(anim_speed);
% delete(p125);
% p150 = PlaceObject('pancake_150.ply', cakepos);

num_cakes = 5;
p_index = 50;
p_index0 = p_index;
cake_now = PlaceObject('pancake_50.ply', cakepos);
for n = 1:num_cakes
    pause(anim_speed);
    delete(cake_now)
    cake_now = PlaceObject(['pancake_',num2str(p_index),'.ply'], cakepos);
    p_index = p_index0 + n*25;
    isEStop;
    if estop.ButtonValuePublic == 1
        delete(blastPlot_h);
    end
end

delete(blastPlot_h);       %stop dispensing projection line

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4) Move IRB back to home pose

    %Inverse kinematics to find joint angles needed to move to brick spawn
    %coords brick1_xyz.
            %note - trotx(pi) makes end-effector face down when going to a 
                    %brick, so arm does not go into floor.
    home = [-1.25, 0, 0.2577];                
%     qh = irb.model.ikine(transl(0.5,0.2,0) * trotx(pi), q, [1,1,1,0,0,0]);
    qh = irb.model.ikine(transl(home) * trotx(pi), q, [1,1,1,0,0,0]);

    %jtraj creates a path between one set of joint positions (q) and a second
    %set of joint positions (q1) using a certain amount of set increments (50)
path = jtraj (q1,qh,50);

booleantrafficlightgreen = 1;

    %Put the jtraj to action usiing for loop 
     for i = 1:50
        pause(0.01);

        %Animate actually makes the arm move 
        %i,: is just the saying the current ith row and all columns
        irb.model.animate(path(i,:));
      
        %drawnow() displays the arm movement 
        drawnow()
        
        isEStop;
     end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 5) Move UR3 to pancake position

    %Create a vector of initial joint angles, in this case the joint angles 
    %are zero
q = zeros(1,7);

    %We do nverse kinematics to find joint angles needed to move to the 
    %ur3_pos coords.
            %note - trotx(pi) makes end-effector face down when going to  
                    %coords, so arm does not go into floor. 
     q1 = ur3.model.ikcon(transl(cakepos_ur3)*trotx(pi), q);
% q1 = [-0.1000         0    0.7854    1.5708    0.7854    1.5708         0]
% q1 = [-0.7400         0    0.7854    1.5708    0.7854    1.5708         0];
% q1 = [-0.7400         0    0.5341    1.8221    0.7854    1.5708         0];
%q1 = [-0.600         0    0.5341    1.8221    0.7854    1.5708         0];
    %jtraj creates a path between one set of joint positions (q) and a second
    %set of joint positions (q1) using a certain amount of set increments (50)
path = jtraj (q,q1,steps);

booleantrafficlightgreen = 1;

    %Put the jtraj to action usiing for loop 
for i = 1:steps
    pause(0.01);

    %Animate actually makes the arm move 
    %i,: is just the saying the current ith row and all columns
    ur3.model.animate(path(i,:));

    %This makes the base of our spatula equal to the trajectory of our
    %robot arm found using path variable above, hence making it look like the spatula is attatched to the
    %end-effector
    drawnow() %drawnow() displays the arm movement 

    isEStop;
end

q0 = ur3.model.getpos;
q1 = ur3.model.ikcon(transl(-0.89, 0, 0.075)*trotx(pi), q);

path = jtraj(q0, q1, steps);
for i = 1:steps
    pause(0.01);
    ur3.model.animate(path(i,:));
    drawnow()
         
    isEStop;
end
     
pause(1);
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 6) Move UR3 up to lift pancake
q0 = ur3.model.getpos;
%rmMatrix = resolve.axial(ur3, 'z', ur3.model.fkine(q0), 0.06, 1);
% cake_now = PlaceObject('pancake_150.ply', cakepos);
% delete(p150);

q1 = ur3.model.ikcon(transl(-0.89, 0, 0.135)*trotx(pi), q);

path = jtraj(q0, q1, steps);

booleantrafficlightgreen = 1;

for i = 1:steps
  pause(0.01);    
  ur3.model.animate(path(i,:)); %Animate plots the arm movement
  drawnow() %drawnow() displays the arm movement in figure 
  
  delete(cake_now);
  EE_pos = ur3.model.fkine(ur3.model.getpos);
%   spat_pos = [spat_Jangles(1,4) spat_Jangles(2,4)+0.05 spat_Jangles(3,4)]
  spat_pos = cakepos;
  spat_pos(3) = EE_pos(3, 4)-0.05;
  cake_now = PlaceObject('pancake_150.ply', spat_pos);
  
  isEStop;
end


            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 7) Rotate UR3 to flip pancake

q0 = ur3.model.getpos;
q1 = q0;
EE_rot = pi;
q1(7) = q1(7)+EE_rot;
path = jtraj(q0, q1, steps);

alpha = 180/steps;
spat_face = spat_pos;
spat_face(3) = spat_pos(3) + 0.05;

booleantrafficlightgreen = 1;

for i = 1:steps
    pause(0.01);
    ur3.model.animate(path(i,:));
    drawnow()

    rotate(cake_now, [0 1 0], alpha, spat_face); % rotate the pancake placeobject
    
    isEStop;
end

delete(cake_now);

cake_now = PlaceObject('pancake_150.ply', cakepos);

% rotate(cake_now, [1 0 0], pi/2, cakepos_irb)

%% 8) Pick up pancake
% Take spatula to left and rotate
q0 = ur3.model.getpos;
q1 = ur3.model.ikcon(transl(-0.805, 0, 0.135)*trotx(pi), q);

q1(7) = q1(7) + pi/2;
% q1 = [-0.6400         0    0.5341    1.8221    0.7854    1.5708         0.01];
path = jtraj(q0, q1, steps);

for i = 1:steps
    pause(0.01);
    ur3.model.animate(path(i,:));
    drawnow()
end

% Take spatula down to griddle
q0 = ur3.model.getpos;
q1 = ur3.model.ikcon(transl(-0.89, 0, 0.07)*trotx(pi), q);

path = jtraj(q0, q1, steps);

%newm = resolve.axial(ur3, 'z', ur3.model.fkine(ur3.model.getpos), - lift, 1);

booleantrafficlightgreen = 1;

for i = 1:steps
  pause(0.01);    
  ur3.model.animate(path(i,:)); %Animate plots the arm movement
  drawnow() %drawnow() displays the arm movement in figure   
  isEStop;
end

% q0 = ur3.model.getpos;
% q1 = ur3.model.ikcon(transl(cakepos_ur3(1) + 0.15...
%                            ,cakepos_ur3(2)...
%                            ,cakepos_ur3(3))*trotx(-pi)...
%                     ,q0);
% path = jtraj(q0, q1, steps);
% for i = 1:steps
%     pause(0.01);
%     ur3.model.animate(path(i,:));
%     drawnow()
% end


% Take spatula to right and rotate 90 - scoop pancake
q0 = ur3.model.getpos;
q1 = ur3.model.ikcon(transl(-0.965, 0, 0.07)*trotx(pi), q);

q1(7) = q1(7) + pi/2;
% q1 = [-0.6400         0    0.5341    1.8221    0.7854    1.5708         0.01];
path = jtraj(q0, q1, steps);
for i = 1:steps
    pause(0.01);
    ur3.model.animate(path(i,:));
    drawnow()
end


%Lift up pancake
q0 = ur3.model.getpos;
q1 = ur3.model.ikcon(transl(-0.965, 0, 0.135)*trotx(pi), q);

path = jtraj(q0, q1, steps);

%rmMatrix = resolve.axial(ur3, 'z', ur3.model.fkine(q0), lift, 1);
% delete(cake_now);
% cake_now = PlaceObject('pancake_150.ply', cakepos);

booleantrafficlightgreen = 1;

for i = 1:steps
    pause(0.01);    
    ur3.model.animate(path(i,:)); %Animate plots the arm movement
    drawnow() %drawnow() displays the arm movement in figure 
  
    delete(cake_now);
    EE_pos = ur3.model.fkine(ur3.model.getpos);
    spat_pos = cakepos;
    spat_pos(3) = EE_pos(3, 4)-0.05;
    cake_now = PlaceObject('pancake_150.ply', spat_pos);
  
  isEStop;
end


%Take cake across to plate
q0 = ur3.model.getpos;
q1 = ur3.model.ikcon(transl(-0.465, 0, 0.135)*trotx(pi), q);

%q1(1) = q1(1) - (cakepos(1) - plate_stack(1)); %works as ur3 base is at 0,0,0 and L(1) is prismatic, therefore q(1) provides distance from 0,0
path = jtraj(q0, q1, steps);

booleantrafficlightgreen = 1;

for i = 1:steps
    pause(0.01);
    ur3.model.animate(path(i,:));
    drawnow()
    
    delete(cake_now);
    EE_pos = ur3.model.fkine(ur3.model.getpos);
    spat_pos(1) = EE_pos(1, 4);
    spat_pos(2) = cakepos(2); %EE_pos(2, 4);
    spat_pos(3) = EE_pos(3, 4) -0.05;
    
    cake_now = PlaceObject('pancake_150.ply', spat_pos);
    
    isEStop;
end

%Rotate spat to drop pancake on plate
q0 = ur3.model.getpos;
q1 = q0;
EE_rot = pi;
q1(7) = q1(7)+EE_rot;
path = jtraj(q0, q1, steps);

alpha = 180/steps;
spat_face = spat_pos; %spat pos is in line with centre axis of EE, while spat face is the plane coincident with spat surface
spat_face(3) = spat_pos(3) + 0.05;

booleantrafficlightgreen = 1;

for i = 1:steps
    pause(0.01);
    ur3.model.animate(path(i,:));
    drawnow()

    rotate(cake_now, [0 1 0], alpha, spat_face);
    
    isEStop;
end

delete(cake_now);
cake_now = PlaceObject('pancake_150.ply', [-0.465, plate_stack(2), plate_stack(3)+0.095]);

% q00 = zeros(1, 7);
% q0 = ur3.model.getpos;
% path = jtraj(q0, q00, steps);
% for i = 1:steps
%     pause(0.01);
%     ur3.model.animate(path(i,:));
%     drawnow();
% end


%%

% if booleantrafficlightgreen == 1
%     delete (trafficlightgreen);
%        trafficlightred = PlaceObject('trafficlightred.ply', [-0.75,0.75,0]);  
%        hold on;
% end

pause(5);