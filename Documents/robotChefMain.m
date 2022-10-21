%% LAB TASK 2 - PANCAKE CHEF
%  MASTER MAIN FILE
%  Nickhil Naiker - 
%  Stefano Mazzotta - 
%  Yashwin Khanna - 12919116

% 19/10

% Process:
% 1) Move IRB above grill
% 2) Dispense batter
% 3) Animate pancake pouring
% 4) Move IRB away
% 5) Move LinUR3 to grill
% 6) Flip pancake
% 7) Move LinUR3 away
% 8) Pick up pancake with Lin UR3
% 9) Place pancake on plate
% 10) IRB Move to plate
% 11) IRB dispense syrup






%%
clf
clear all    
set(0,'DefaultFigureWindowStyle','docked')   % Docking the figure to the window on the right hand side  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Create robots and environment           
hold on; % To retain current plot or graphic

    % Displaying a concrete floor
        % note - the [x,x,x,x],[y,y,y,y],[z,z,z,z] are corner points so we
                 % know where to map the image
surf([-2,-2;3.25,3.25],[-2.5,2;-2.5,2],[-0.65,-0.65;-0.65,-0.65],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

% surf([-2,-2;-2,-2],[-2.6,2;-2.6,2],[-0.65,-0.65;1.5,1.5],'CData',imread('wall.jpg'),'FaceColor','texturemap');

% surf([-2,3.25;-2,3.25],[-2.6,-2.6;-2.6,-2.6],[-0.65,-0.65;1.5,1.5],'CData',imread('wall.jpg'),'FaceColor','texturemap');

%PlaceObject('roboticstable.ply', [-0.25,0.5,-0.07]);    %Table

cakepos_irb = [-1.25+0.4, 0.035, 0.075]; %[ 0.1 ,0.49, 0.01];                       %Pancake dispense position for IRB 910
cakepos_ur3 = [-1.25+0.5, 0, 0.075]; %[ -0.1   ,0.5,    0.12];    %[ 0.1 ,0.4, 0]        %Pancake pick up pos for LinUR3

    %Table
PlaceObject('newroboticstable.ply', [0,0,-0.0844]);
hold on;

PlaceObject('griddle.ply', [-0.55,-0.15,0]);     %Loading in kitchen environment
hold on;

enviro = 1;
if enviro == 1
    environmentRPC;
end


irb = IRB_910sc;                                        %Initialise the IRB and UR3 robot
ur3 = Linear_UR3(false);

% irb.model.teach;                                        %Manually interact with the robots
% ur3.model.teach;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Setup 

steps = 50;

% create_spatula = SpatulaClass(1); %SpatulaClass is a class and () indicates how many spatulas we want made
% create_spatula.SpatulaSpawn(1,-0.7,0.5,0);    %fn in SpatulaClass to create and spawn spatula at an x,y,theta coordinate   

resolve = RMRC(); %initialise RMRC class. Class performs traj and movement animations

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

    %Put the jtraj to action usiing for loop 
     for i = 1:50
         pause(0.01);
         irb.model.animate(path(i,:));     %Animate plots the arm movement. i,: is current ith row and all columns
         drawnow()         %drawnow() displays the arm movement in figure
     end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2) Make IRB dispensing line

hold on;
blastStartTr = irb.model.fkine(q1);
blastStartPnt = blastStartTr(1:3,4)';

%Blast stream length in x,y,z axis
blastEndTr = irb.model.fkine(q1) * transl(0,0,-0.70);
blastEndPnt = blastEndTr(1:3,4)';

%This projects a line out of the irb end effector 
    %choose colour using hexidecimal
blastPlot_h = plot3([blastStartPnt(1),blastEndPnt(1)],[blastStartPnt(2),blastEndPnt(2)],[blastStartPnt(3),blastEndPnt(3)],'Color', "#FFE7B4");
axis equal;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3) Pancake pouring animation

anim_speed = 0.5;       %animation speed 
pause(anim_speed); 
p50 = PlaceObject('pancake_50.ply', cakepos_irb);

pause(anim_speed);
delete(p50);
p75 = PlaceObject('pancake_75.ply', cakepos_irb); 
% delete(PlaceObject('pancake_50.ply', cakepos_irb));     %deleting previous pancake


pause(anim_speed);
delete(p75)
p100 = PlaceObject('pancake_100.ply', cakepos_irb);
% delete(PlaceObject('pancake_75.ply', cakepos_irb));

pause(anim_speed);
delete(p100);
p125 = PlaceObject('pancake_125.ply', cakepos_irb);
% delete(PlaceObject('pancake_100.ply', cakepos_irb));

pause(anim_speed);
delete(p125);
p150 = PlaceObject('pancake_150.ply', cakepos_irb);
% delete(PlaceObject('pancake_125.ply', cakepos_irb));

 delete(blastPlot_h);       %stop dispensing projection line

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4) Move IRB back to home pose

    %Inverse kinematics to find joint angles needed to move to brick spawn
    %coords brick1_xyz.
            %note - trotx(pi) makes end-effector face down when going to a 
                    %brick, so arm does not go into floor.
    home = [-1.25, 0, 0];                
%     qh = irb.model.ikine(transl(0.5,0.2,0) * trotx(pi), q, [1,1,1,0,0,0]);
    qh = irb.model.ikine(transl(home) * trotx(pi), q, [1,1,1,0,0,0]);

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
%% 5) Move UR3 to pancake position

    %Create a vector of initial joint angles, in this case the joint angles 
    %are zero
q = zeros(1,7);

    %We do nverse kinematics to find joint angles needed to move to the 
    %ur3_pos coords.
            %note - trotx(pi) makes end-effector face down when going to  
                    %coords, so arm does not go into floor. 
%     q1 = ur3.model.ikine(transl(cakepos_ur3) * trotx(pi), q, [1,1,1,0,0,0]);
% q1 = [-0.1000         0    0.7854    1.5708    0.7854    1.5708         0]
% q1 = [-0.7400         0    0.7854    1.5708    0.7854    1.5708         0];
q1 = [-0.7400         0    0.5341    1.8221    0.7854    1.5708         0];

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
%   create_spatula.spatula{1}.base = ur3.model.fkine(path(i,:));
%   create_spatula.spatula{1}.animate(0);

        %drawnow() displays the arm movement 
      drawnow()
     end
     
pause(1);
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 6) Move UR3 up to lift pancake 
% premove = ur3.model.getpos
% mat = resolve.axial(ur3, 'z', ur3.model.fkine(ur3.model.getpos), 0.2);
% postmove = ur3.model.getpos
% res2 = RMRC_2;
% res2.axial(ur3, 'z', ur3.model.fkine(ur3.model.getpos), 0.2);

rmMatrix = resolve.axial(ur3, 'z', ur3.model.fkine(q1), 0.06, 1);
cake_now = PlaceObject('pancake_150.ply', cakepos_irb);
delete(p150);

for i = 1:resolve.steps
  pause(0.01);    
  ur3.model.animate(rmMatrix(i,:)); %Animate plots the arm movement
  drawnow() %drawnow() displays the arm movement in figure 
  
  delete(cake_now);
  EE_pos = ur3.model.fkine(ur3.model.getpos);
%   spat_pos = [spat_Jangles(1,4) spat_Jangles(2,4)+0.05 spat_Jangles(3,4)]
  spat_pos = cakepos_irb;
  spat_pos(3) = EE_pos(3, 4)-0.05;
  cake_now = PlaceObject('pancake_150.ply', spat_pos);
  
end


            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 7) Rotate UR3 to flip pancake

q0 = ur3.model.getpos;
q1 = q0;
EE_rot = pi;
q1(7) = q1(7)+EE_rot;
path = jtraj(q0, q1, steps);

% cake.spawnLocation(1, spat_pos(1), spat_pos(2), spat_pos(3));
% delete(cake_now);
alpha = 180/steps;
spat_face = spat_pos;
spat_face(3) = spat_pos(3) + 0.05;

for i = 1:steps
    pause(0.01);
    ur3.model.animate(path(i,:));
    drawnow()
    
    
    rotate(cake_now, [0 1 0], alpha, spat_face);
    
    
%     delete(cake_now);
%     EE_pos = ur3.model.fkine(ur3.model.getpos);
%     spat_pos(1) = EE_pos(1,4);
%     spat_pos(2) = EE_pos(2, 4)+offset_y; %0.1;
%     spat_pos(3) = EE_pos(3, 4)+offset_z; %-0.05;
%     cake_now = PlaceObject('pancake_150.ply', spat_pos);
    
%     cake.brick{1}.base = EE_pos;
%     cake.brick{1}.base(2,4) = EE_pos(2, 4)+offset_y;
%     cake.brick{1}.base(3,4) = EE_pos(3, 4)+offset_z;
%     cake.brick{1}.base = ur3.model.fkine(path(i,:))*trotx(pi/2);
%     cake.brick{1}.base(2,4) = cake.brick{1}.base(2,4) + offset_y;
%     cake.brick{1}.base(3,4) = cake.brick{1}.base(3,4) + offset_z;
%     cake.brick{1}.animate(0);
end

delete(cake_now);

cake_now = PlaceObject('pancake_150.ply', cakepos_irb);

% rotate(cake_now, [1 0 0], pi/2, cakepos_irb)




% 
% resolve.axial(ur3, 'z', ur3.model.fkine(q1), -0.2);     %Move back to grill
% 
% q0 = ur3.model.getpos                                   %Flip and scoop cake from side
% q1 = q0
% EE_rot = pi;
% q1(7) = q1(7)+EE_rot
% path = jtraj(q0, q1, steps);
% for i = 1:steps
%     pause(0.01);
%     ur3.model.animate(path(i,:));
%     
%     drawnow()
% end
% 
% 
% % %% 8) Move UR3 to side
% % 
% % resolve.axial(ur3, 'x', ur3.model.fkine(q1), 0.1)
% % 
% % %% 9) Flip spatula (UR3) back to normal
% % 
% % q0 = ur3.model.getpos
% % q1 = q0
% % EE_rot = pi;
% % q1(7) = q1(7)+EE_rot
% % path = jtraj(q0, q1, steps);
% % for i = 1:steps
% %     pause(0.01);
% %     ur3.model.animate(path(i,:));
% %     
% %     drawnow()
% % end
% % 
% % %% 10) move back down to grill level
% % 
% % resolve.axial(ur3, 'z', ur3.model.fkine(q1), -0.2);
% % q1 = bot.model.getpos;
% % resolve.axial(ur3, 'x', ur3.model.fkine(q1), -0.1);
% % 

