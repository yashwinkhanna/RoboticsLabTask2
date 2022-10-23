%% Environment Robo Pancake Chef

surf([-2,-2;-2,-2],[-2.6,2;-2.6,2],[-0.65,-0.65;1.5,1.5],'CData',imread('wall.jpg'),'FaceColor','texturemap');

surf([-2,3.25;-2,3.25],[-2.6,-2.6;-2.6,-2.6],[-0.65,-0.65;1.5,1.5],'CData',imread('wall.jpg'),'FaceColor','texturemap');


PlaceObject('kitchenenvironment.ply', [-2,-2.5,-0.7]);     %Loading in kitchen environment
hold on;

    % get rid of these
% PlaceObject('cookingplate.ply', [3,0,0]);     %Loading in kitchen environment
% hold on;
% PlaceObject('griddle.ply', [-0.35,-0.15,0]);     %Loading in kitchen environment
% hold on;
% PlaceObject('pancake_150.ply', [-0.6,0,0.1]);     %Loading in kitchen environment
% hold on;
    %%%%%%%%%%%%%%%%%

PlaceObject('plate.ply', [-1.2,-0.4,0]);     %Loading in kitchen environment
hold on;

% PlaceObject('plate.ply', [0.4,-0.5,0]);     %Loading in kitchen environment
% hold on;

% PlaceObject('plate.ply', [0,0,0]);     %Loading in kitchen environment
% hold on;

PlaceObject('plate.ply', [0.4,0,0]);     %Loading in kitchen environment
hold on;

PlaceObject('plate.ply', [0.4,0,0.02]);     %Loading in kitchen environment
hold on;

PlaceObject('plate.ply', [0.4,0,0.04]);     %Loading in kitchen environment
hold on;

PlaceObject('plate.ply', [0.4,0,0.06]);     %Loading in kitchen environment
hold on;

PlaceObject('plate.ply', [0.4,0,0.08]);     %Loading in kitchen environment
hold on;

PlaceObject('plate.ply', [0.4,0,0.1]);     %Loading in kitchen environment
hold on;

% PlaceObject('roboticshuman.ply', [-0.5,-1.25,-0.7]);     %Loading in kitchen environment
% hold on;

PlaceObject('roboticsemergencystop.ply', [-1.25,-0.8,0.05]);     %Loading in kitchen environment
hold on;

PlaceObject('roboticsfireextinguisher.ply', [-1.5,-1.75,-0.7]);  %Loading fire extinguisher
hold on;

PlaceObject('roboticsfireextinguisher.ply', [2,-1.75,-0.7]);  %Loading fire extinguisher
hold on;

% PlaceObject('roboticssafetyhat.ply', [-0.5,-1.25,1.3]);    %Loading a hard safety hat
% hold on;

PlaceObject('roboticscamera.ply', [-0.5,0.75,0]); %Loading a camera
hold on;

PlaceObject('roboticsclock.ply', [1,-0.5,0.05]);  %loading an alarm clock
hold on;

PlaceObject('pancakemixbottle.ply', [-1.25,0.5,0]); %Loading a camera
hold on;

PlaceObject('pancakemixbottle.ply', [-1.15,0.5,0]); %Loading a camera
hold on;

PlaceObject('jammixbottle.ply', [-1.05,0.5,0]); %Loading a camera
hold on;

 PlaceObject('chair.ply', [1.6,0,-0.65]); %Loading a camera
 hold on;

% PlaceObject('roboticsfence.ply', [-1,2,-0.7]);     %Loading in fence
% hold on;

PlaceObject('roboticscone.ply', [3,1.5,-0.65]);    %Loading traffic cone
hold on;

PlaceObject('roboticscone.ply', [3,1,-0.65]);    %Loading traffic cone
hold on;

PlaceObject('roboticscone.ply', [3,0.5,-0.65]);    %Loading traffic cone
hold on;

PlaceObject('roboticscone.ply', [3,0,-0.65]);    %Loading traffic cone
hold on;

PlaceObject('roboticscone.ply', [3,-0.5,-0.65]);    %Loading traffic cone
hold on;

trafficlightgreen = PlaceObject('trafficlightgreen.ply', [-0.75,0.75,0]);     %Loading in kitchen environment
hold on;

       trafficlightyellow = PlaceObject('trafficlightyellow.ply', [-0.75,0.75,0]);    %Loading traffic cone
       hold on;
       delete(trafficlightyellow);
