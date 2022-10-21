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

enviro = 0;
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

%% Spawn ur3 and irb at hover spot

hover_pos = [-1.02, -0.53, 0.06];

q00 = [-0.7400    0.0000    0.5341    1.5708    1.0367    1.5708         0];
ur3.model.animate(q00);
% cake_now = PlaceObject('plate.ply', hover_pos);

    
q = zeros(1,3);         %Create a vector of initial joint angles, beginning at 0
q1 = irb.model.ikine(transl(cakepos_irb) * trotx(pi), q, [1,1,1,0,0,0]);
irb.model.animate(q1);
pause(1);
     
%% 4) Move IRB back to home pose

    %Inverse kinematics to find joint angles needed to move to brick spawn
    %coords brick1_xyz.
            %note - trotx(pi) makes end-effector face down when going to a 
                    %brick, so arm does not go into floor.
    home = [-1.25, 0, 0];                
%     qh = irb.model.ikine(transl(0.5,0.2,0) * trotx(pi), q, [1,1,1,0,0,0]);
    qh = irb.model.ikine(transl(home) * trotx(pi) * trotz(pi), q, [1,1,1,0,0,0]);
    irb.model.animate(qh);

%     %jtraj creates a path between one set of joint positions (q) and a second
%     %set of joint positions (q1) using a certain amount of set increments (50)
% path = jtraj (q1,qh,50);
% 
%     %Put the jtraj to action usiing for loop 
%      for i = 1:50
%         pause(0.01);
% 
%         %Animate actually makes the arm move 
%         %i,: is just the saying the current ith row and all columns
%       irb.model.animate(path(i,:));
%       
%         %drawnow() displays the arm movement 
%       drawnow()
%      end

%% Move plate to safe spot
rmMatrix = resolve.axial(ur3, 'x', ur3.model.fkine(ur3.model.getpos), 0.65, 0.5);
% rmMatrix = resolve.axial(ur3, 'x', ur3.model.fkine(q00), 0.65, 0.5);
cake_now = PlaceObject('pancake_150_syrupBandplate.ply', hover_pos);

spat_pos = zeros(1, 3);

for i = 1:resolve.steps
  pause(0.01);    
  ur3.model.animate(rmMatrix(i,:)); %Animate plots the arm movement
  drawnow() %drawnow() displays the arm movement in figure 
  
  delete(cake_now);
  EE_pos = ur3.model.fkine(ur3.model.getpos);
%   spat_pos = [spat_Jangles(1,4) spat_Jangles(2,4)+0.05 spat_Jangles(3,4)]
  spat_pos(1) = EE_pos(1, 4);
  spat_pos(2) = EE_pos(2, 4)+offset_y+0.1;
  spat_pos(3) = EE_pos(3, 4)-0.05;
  cake_now = PlaceObject('pancake_150_syrupBandplate.ply', spat_pos);
  
end

%% Move plate to table

% rmMatrix = resolve.axial(ur3, 'z', ur3.model.fkine(ur3.model.getpos), -0.168, 0.5);
rmMatrix = resolve.axial(ur3, 'z', ur3.model.fkine(ur3.model.getpos), -0.168, 0.5);
% cake_now = PlaceObject('pancake_150.ply', cakepos_irb);

spat_pos = zeros(1, 3);

for i = 1:resolve.steps
  pause(0.01);    
  ur3.model.animate(rmMatrix(i,:)); %Animate plots the arm movement
  drawnow() %drawnow() displays the arm movement in figure 
  
  delete(cake_now);
  EE_pos = ur3.model.fkine(ur3.model.getpos);
%   spat_pos = [spat_Jangles(1,4) spat_Jangles(2,4)+0.05 spat_Jangles(3,4)]
  spat_pos(1) = EE_pos(1, 4);
  spat_pos(2) = EE_pos(2, 4)+offset_y+0.1;
  spat_pos(3) = EE_pos(3, 4)-0.05;
  cake_now = PlaceObject('pancake_150_syrupBandplate.ply', spat_pos);
  
end




