close all

Flag = 1; %Flag to choose between running the script which plot the grippers max reach (point cloud) or starts to run the building of the 3x3 wall

%% Loading environment in

fprintf('LOADING ENVRIONMENT IN \n');   %Log messages

surf([-2,-2;2,2],[-2,2;-2,2],[-0.7,-0.7;-0.7,-0.7],'CData',imread('concrete.jpg'),'FaceColor','texturemap');    %Loading in concrete floor
hold on;

PlaceObject('roboticstable.ply', [-0.35,0,-0.0844]);    %Loading in table
hold on;

PlaceObject('roboticsfence.ply', [-1.25,1.5,-0.7]);     %Loading in fence
hold on;

PlaceObject('roboticsfence.ply', [-1.25,-1.5,-0.7]);    %Loading in another fence locate somewhere else
hold on;

PlaceObject('roboticsemergencystop.ply', [-1.25,0.3,0.05])  %Loading emergercy stop button
hold on;

PlaceObject('roboticsfireextinguisher.ply', [-1.5,-1.4,-0.7]);  %Loading fire extinguisher
hold on;

PlaceObject('roboticsfireextinguisher.ply', [1.5,1.3,-0.7]);    %Loasing another fire extinguisher in another location
hold on;

PlaceObject('roboticscone.ply', [1.5,0.5,-0.7]);    %Loading traffic cone
hold on;

PlaceObject('roboticscone.ply', [1.5,0,-0.7]);  %Loading another traffic cone
hold on;

PlaceObject('roboticscone.ply', [1.5,-0.5,-0.7]);   %Loading another traffic cone
hold on;

PlaceObject('roboticscone.ply', [1.5,-1,-0.7]); %Loading another traffic cone
hold on;

PlaceObject('roboticshuman.ply', [-0.5,-1,-0.7]);   %Loading a human of average size
hold on;

PlaceObject('roboticscamera.ply', [0.5,0.4,0]); %Loading a camera
hold on;

PlaceObject('roboticsclock.ply', [-1.15,-0.375,0.05]);  %loading an alarm clock
hold on;

PlaceObject('roboticssafetyhat.ply', [-0.5,-1,1.3]);    %Loading a hard safety hat
hold on;

wallxyz = [0,-2,-0.7];
PlaceObject('roboticswall.ply', wallxyz);   %Loading a brick wall
hold on;

%% Initalisation of robot and brick spawns
 
fprintf('STARTING ROBOT AND INITIALISING BRICKS \n');   %Logging messages

r = LinearUR3(false);   %Starting up robot
 
 
startbx{1} = -0.2;  %Intialising first bricks starting coordinates
startby{1} = -0.35; 
startbz{1} = 0;
startlocationbrickxyz{1} = [startbx{1},startby{1},startbz{1}];
brick{1} = PlaceObject('HalfSizedRedGreenBrick.ply', [startlocationbrickxyz{1}]);

dropoffbx{1} = 0.375; %Intialising first bricks final coordinates
dropoffby{1} = -0.15; 
dropoffbz{1} = 0;
dropofflocationbrickxyz{1} = [dropoffbx{1},dropoffby{1},dropoffbz{1}];

startbx{2} = -0.3;  %Intialising second bricks starting coordinates
startby{2} = -0.35;
startbz{2} = 0;
startlocationbrickxyz{2} = [startbx{2},startby{2},startbz{2}];
brick{2} = PlaceObject('HalfSizedRedGreenBrick.ply', [startlocationbrickxyz{2}]);

dropoffbx{2} = 0.375;   %Intialising second bricks final coordinates
dropoffby{2} = -0.15 + 0.1334;
dropoffbz{2} = 0;
dropofflocationbrickxyz{2} = [dropoffbx{2},dropoffby{2},dropoffbz{2}];

startbx{3} = -0.4;  %Intialising third bricks starting coordinates
startby{3} = -0.35;
startbz{3} = 0;
startlocationbrickxyz{3} = [startbx{3},startby{3},startbz{3}];
brick{3} = PlaceObject('HalfSizedRedGreenBrick.ply', [startlocationbrickxyz{3}]);

dropoffbx{3} = 0.375;   %Intialising third bricks final coordinates
dropoffby{3} = -0.15 + 0.1334 + 0.1334;
dropoffbz{3} = 0;
dropofflocationbrickxyz{3} = [dropoffbx{3},dropoffby{3},dropoffbz{3}];

startbx{4} = -0.5; %Intialising foruth bricks starting coordinates
startby{4} = -0.35;
startbz{4} = 0;
startlocationbrickxyz{4} = [startbx{4},startby{4},startbz{4}];
brick{4} = PlaceObject('HalfSizedRedGreenBrick.ply', [startlocationbrickxyz{4}]);

dropoffbx{4} = 0.375;   %Intialising foruth bricks final coordinates
dropoffby{4} = -0.15; 
dropoffbz{4} = 0.0334;
dropofflocationbrickxyz{4} = [dropoffbx{4},dropoffby{4},dropoffbz{4}];

startbx{5} = -0.6;  %Intialising fifth bricks starting coordinates
startby{5} = -0.35;
startbz{5} = 0;
startlocationbrickxyz{5} = [startbx{5},startby{5},startbz{5}];
brick{5} = PlaceObject('HalfSizedRedGreenBrick.ply', [startlocationbrickxyz{5}]);

dropoffbx{5} = 0.375;   %Intialising fifth bricks starting coordinates, 0.1334 is the y length of s brick, 0.0334 is the z length/height of  brick
dropoffby{5} = -0.15 + 0.1334;
dropoffbz{5} = 0.0334;
dropofflocationbrickxyz{5} = [dropoffbx{5},dropoffby{5},dropoffbz{5}];

startbx{6} = -0.2; %Intialising sixth bricks starting coordinates
startby{6} = 0.35; 
startbz{6} = 0;
startlocationbrickxyz{6} = [startbx{6},startby{6},startbz{6}];
brick{6} = PlaceObject('HalfSizedRedGreenBrick.ply', [startlocationbrickxyz{6}]);

dropoffbx{6} = 0.375; %Intialising sixth bricks final coordinates
dropoffby{6} = -0.15 + 0.1334 + 0.1334;
dropoffbz{6} = 0.0334;
dropofflocationbrickxyz{6} = [dropoffbx{6},dropoffby{6},dropoffbz{6}];

startbx{7} = -0.3;  %Intialising seventh bricks starting coordinates
startby{7} = 0.35;
startbz{7} = 0;
startlocationbrickxyz{7} = [startbx{7},startby{7},startbz{7}];
brick{7} = PlaceObject('HalfSizedRedGreenBrick.ply', [startlocationbrickxyz{7}]);

dropoffbx{7} = 0.375;   %Intialising seventh bricks final coordinates
dropoffby{7} = -0.15; 
dropoffbz{7} = 0.0334 + 0.0334;
dropofflocationbrickxyz{7} = [dropoffbx{7},dropoffby{7},dropoffbz{7}];

startbx{8} = -0.4;  %Intialising eigth bricks starting coordinates
startby{8} = 0.35;
startbz{8} = 0;
startlocationbrickxyz{8} = [startbx{8},startby{8},startbz{8}];
brick{8} = PlaceObject('HalfSizedRedGreenBrick.ply', [startlocationbrickxyz{8}]);

dropoffbx{8} = 0.375;   %Intialising eigth bricks final coordinates
dropoffby{8} = -0.15 + 0.1334;
dropoffbz{8} = 0.0334 + 0.0334;
dropofflocationbrickxyz{8} = [dropoffbx{8},dropoffby{8},dropoffbz{8}];

startbx{9} = -0.5;  %Intialising ninth bricks starting coordinates
startby{9} = 0.35;
startbz{9} = 0;
startlocationbrickxyz{9} = [startbx{9},startby{9},startbz{9}];
brick{9} = PlaceObject('HalfSizedRedGreenBrick.ply', [startlocationbrickxyz{9}]);

dropoffbx{9} = 0.375;   %Intialising ninth bricks final coordinates
dropoffby{9} = -0.15 + 0.1334 + 0.1334;
dropoffbz{9} = 0.0334 + 0.0334;
dropofflocationbrickxyz{9} = [dropoffbx{9},dropoffby{9},dropoffbz{9}];


offset = transl(0,0,0.0334)*trotx(pi); %Offset used to ensure end effector touches edge of brick and brick does not go inside end effector, also a rotation of pi to ensure joint 5 is facing downwars when placing a brick on the table, and not upwards 

waypointQ = [0,4.71239,0,-1.5708,0,-4.71239,6.28319]; %Waypoint generated, robot moves to waypoint after every brick movement it completes, moving to a known waypoint ensures the robot does not travel through the ground to reach its next location and instead rises to the waypoint and then falls back down to where it needs to go

%% CALCULATE AND PLOT WORKSPACE RADIUS AND VOLUME

if Flag == 0    %When flag = 0 the cloud point graph and maximum reach method functions are started up and ran through

% Use glyphs to draw robot, don't display the name
r.model.plotopt = {'nojoints', 'noname', 'noshadow', 'nowrist'}; 
    
% 2.4 Sample the joint angles within the joint limits at 30 degree increments between each of the joint limits
% & 2.5 Use fkine to determine the point in space for each of these poses, so that you end up with a big list of points
stepRads = deg2rad(30);
qlim = r.model.qlim;
% Don't need to worry about joint 6
pointCloudeSize = prod(floor((qlim(1:6,2)-qlim(1:6,1))/stepRads + 1));
pointCloud = zeros(pointCloudeSize,3);
counter = 1;
tic

for q1 = qlim(1,1):stepRads:qlim(1,2)
    for q2 = qlim(2,1):stepRads:qlim(2,2)
        for q3 = qlim(3,1):stepRads:qlim(3,2)
            for q4 = qlim(4,1):stepRads:qlim(4,2)
                for q5 = qlim(5,1):stepRads:qlim(5,2)
                    for q6 = qlim(6,1):stepRads:qlim(6,2)
                    % Don't need to worry about joint 6, just assume it=0
                    q7 = 0;
                    q6 = 0;
%                     for q6 = qlim(6,1):stepRads:qlim(6,2)
                        q = [q1,q2,q3,q4,q5,q6,q7];
                        tr = r.model.fkine(q);                        
                        pointCloud(counter,:) = tr(1:3,4)';
                        counter = counter + 1; 
                        if mod(counter/pointCloudeSize * 100,1) == 0
                            display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                        end
                     end
                end
            end
        end
    end
end

% 2.6 Create a 3D model showing where the end effector can be over all these samples.  
plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');


[maxxvalue,idx1] = max(pointCloud(:,1)); % Calculating robot reach %Max x value robot can reach
disp('maxxvalue = ');
disp(maxxvalue);

[minxvalue,idx2] = min(pointCloud(:,1)); %Min x value robot can reach
disp('minxvalue = ');
disp(minxvalue);

[maxyvalue,idx3] = max(pointCloud(:,2));    %Max y value robot can reach
disp('maxyvalue = ');
disp(maxyvalue);

[minyvalue,idx4] = min(pointCloud(:,2)); %Min y value robot can reach
disp('minyvalue = ');
disp(minyvalue);

[maxzvalue,idx5] = max(pointCloud(:,3));    %Max z value robot can reach
disp('maxzvalue = ');
disp(maxzvalue);

[minzvalue,idx6] = min(pointCloud(:,3)); %Min z value robot can reach
disp('minzvalue = ');
disp(minzvalue);

approxxradius = (abs(maxxvalue) + abs(minxvalue)) / 2;  %Approx radius of reach in x direction
disp('Approximate radius of sphere in x direction = ');
disp(approxxradius);
approxyradius = (abs(maxyvalue) + abs(minyvalue)) / 2;  %Approx radius of reach in y direction
disp('Approximate radius of sphere in y direction = ');
disp(approxyradius);
approxzradius = (abs(maxzvalue) + abs(0)) / 2;  %Approx radius of reach in z direction
disp('Approximate radius of sphere in z direction = ');
disp(approxzradius);


xcolumnmatrixofpointcloud = pointCloud(:,1); %Assigning all x values in pointcloud matrix into a new matrix
ycolumnmatrixofpointcloud = pointCloud(:,2);    %Assigning all y value in pointcloud matrix into a new matrix
zcolumnmatrixofpointcloud = pointCloud(:,3);    %Assigning all z value in pointcloud matrix into a new matrix

logicalarrayofwhatnumbersaregreaterthanzero = zcolumnmatrixofpointcloud > 0; % Get logic matrix of numbers greater than 0
logicalindexingtomakevectorofnumbersgreaterthanzero = zcolumnmatrixofpointcloud(logicalarrayofwhatnumbersaregreaterthanzero);   %Use logic matrix multiplied by original matrix to get a new matrix that provides only z values greater than 0


for g = 1:190000 %For the convhull function to work all x, y and z matrices need to be of equal size. As we've delete negative numbers from the z matrix we then need to increase the size of this matrix again by adding zeros to the end so it matches the size of the others
logicalindexingtomakevectorofnumbersgreaterthanzero(end+1) = 0;
end

[K,V] = convhull(xcolumnmatrixofpointcloud,ycolumnmatrixofpointcloud,logicalindexingtomakevectorofnumbersgreaterthanzero); %Convhull function for computing volume of a convex shape using multiple x, y and z coordinates
disp('Volume of convex hull(m^3) =');
disp(V);        %Display volume answer


end



%% MOVING FROM INITIAL STARTING TO WAYPOINT

if Flag == 1    %If flag = 1 start to begin wall buildiing
    
fprintf('MOVING FROM INITIAL STARTING POSITION TO WAYPOINT \n');    %Logging messages

arminitialstartingq = zeros(1,7);   %Initial q matrix

armmovetowaypointafterinitialisationjtraj = jtraj(arminitialstartingq,waypointQ,100); %Calculating trajectory plan from initial starting q matrix to waypoint

for i = 1:100
  r.model.animate(armmovetowaypointafterinitialisationjtraj(i,:));  %Visually animating the robot to see it moving through space to go to the waypoint
  drawnow() 
 
  pause(0.005); 
end

%% MOVING TO S BRICK

for s = 1:9     %for loop iterating through each of the bricks, each brick has their own unique coordinates

 fprintf('MOVING TO BRICK %d \n',s);    %Logging messages
    
armpickupbrickQ{s} = r.model.ikcon(transl(startlocationbrickxyz{s})*offset,waypointQ); %Inverse kinematics to find joint positions needed to reach brick



calcfinalendeffectorpositiontolerancemarks = r.model.fkine(armpickupbrickQ{s});  %Calculating where we think we are arriving, the calculating where we have actually arrived once we moved there, find the difference and show it is less than the +-5mm tolerance limit
calcendxposition = calcfinalendeffectorpositiontolerancemarks(1,4); 
calcendyposition = calcfinalendeffectorpositiontolerancemarks(2,4); 
calcendzposition = calcfinalendeffectorpositiontolerancemarks(3,4);
disp('Calculated x position =');
disp(calcendxposition);
disp('Calculated y position =');
disp(calcendyposition);
disp('Calculated z position =');
disp(calcendzposition);



armpickupbrickjtraj{s} = jtraj(waypointQ,armpickupbrickQ{s},100);   %Calculating trajectory plan from waypoint to picking up a brick

for i = 1:100
        pause(0.005);
      r.model.animate(armpickupbrickjtraj{s}(i,:)); %Visually animating the robot to see it moving through space to go to the brick
      drawnow()
end


findingacutualcurrentendeffectorpositiontolerancemarksQ = r.model.getpos(); %Continued from above, calculating tolerance within +-5mm
fourbyfourmatrixacutualcurrentendeffectorpositiontolerancemarks = r.model.fkine(findingacutualcurrentendeffectorpositiontolerancemarksQ);
actualendxposition = fourbyfourmatrixacutualcurrentendeffectorpositiontolerancemarks(1,4); 
actualendyposition = fourbyfourmatrixacutualcurrentendeffectorpositiontolerancemarks(2,4); 
actualendzposition = fourbyfourmatrixacutualcurrentendeffectorpositiontolerancemarks(3,4);
disp('Actual x position =');
disp(actualendxposition);
disp('Actual y position =');
disp(actualendyposition);
disp('Actual z position =');
disp(actualendzposition);

disp('Difference between calculated x and actual x ='); %Displaying differences to show output is less than +-5mm
display(calcendxposition - actualendxposition);
disp('Difference between calculated y and actual y =');
display(calcendyposition - actualendyposition);
disp('Difference between calculated z and actual z =');
display(calcendzposition - actualendzposition);


%% %% MOVING TO WAYPOINT WITH BRICK

fprintf('MOVING TO WAYPOINT WITH BRICK %d \n',s); %Logging messages

armpickupbrickQ{s} = r.model.getpos;    %Getting matrix of the current joint positons


calcfinalendeffectorpositiontolerancemarks = r.model.fkine(waypointQ);  %Calculating where we think we are arriving, the calculating where we have actually arrived once we moved there, find the difference and show it is less than the +-5mm tolerance limit
calcendxposition = calcfinalendeffectorpositiontolerancemarks(1,4); 
calcendyposition = calcfinalendeffectorpositiontolerancemarks(2,4); 
calcendzposition = calcfinalendeffectorpositiontolerancemarks(3,4);
disp('Calculated x position =');
disp(calcendxposition);
disp('Calculated y position =');
disp(calcendyposition);
disp('Calculated z position =');
disp(calcendzposition);


armmovebricktowaypointjtraj{s} = jtraj([armpickupbrickQ{s}],[waypointQ],100);   %Calculating trajectory plan from picking up a brick to waypoint


 for i = 1:100
     
   r.model.animate(armmovebricktowaypointjtraj{s}(i,:)); %Visually animating the robot to see it moving through space to go to the waypoint
 
   drawnow() 

brickstaticlocationendeffector = r.model.getpos; %Updating the location of end effector, then updating the location that the brick is required to be, and then replotting the brick in every iteration of the for loop
brickendeffectorlocationmatrix = r.model.fkine(brickstaticlocationendeffector); 
delete(brick{s}); 
brickmovingposition = [brickendeffectorlocationmatrix(1,4) brickendeffectorlocationmatrix(2,4) brickendeffectorlocationmatrix(3,4)-0.0334];
brick{s} = PlaceObject('HalfSizedRedGreenBrick.ply', [brickmovingposition]);
pause(0.005);  
   
 end
 
 
findingacutualcurrentendeffectorpositiontolerancemarksQ = r.model.getpos(); %Continued from above, calculating tolerance within +-5mm
fourbyfourmatrixacutualcurrentendeffectorpositiontolerancemarks = r.model.fkine(findingacutualcurrentendeffectorpositiontolerancemarksQ);
actualendxposition = fourbyfourmatrixacutualcurrentendeffectorpositiontolerancemarks(1,4); 
actualendyposition = fourbyfourmatrixacutualcurrentendeffectorpositiontolerancemarks(2,4); 
actualendzposition = fourbyfourmatrixacutualcurrentendeffectorpositiontolerancemarks(3,4);
disp('Actual x position =');
disp(actualendxposition);
disp('Actual y position =');
disp(actualendyposition);
disp('Actual z position =');
disp(actualendzposition);

disp('Difference between calculated x and actual x ='); %Displaying differences to show output is less than +-5mm
display(calcendxposition - actualendxposition);
disp('Difference between calculated y and actual y =');
display(calcendyposition - actualendyposition);
disp('Difference between calculated z and actual z =');
display(calcendzposition - actualendzposition);
  
 
 

%% DROPPING OFF S BRICK

fprintf('DROPPING OFF BRICK %d \n',s);

armmovebricktowallQ{s} = r.model.ikcon(transl(dropofflocationbrickxyz{s})*offset,waypointQ);    %Inverse kinematics to find joint positions needed to reach brick


calcfinalendeffectorpositiontolerancemarks = r.model.fkine(armmovebricktowallQ{s}); %Calculating where we think we are arriving, the calculating where we have actually arrived once we moved there, find the difference and show it is less than the +-5mm tolerance limit
calcendxposition = calcfinalendeffectorpositiontolerancemarks(1,4); 
calcendyposition = calcfinalendeffectorpositiontolerancemarks(2,4); 
calcendzposition = calcfinalendeffectorpositiontolerancemarks(3,4);
disp('Calculated x position =');
disp(calcendxposition);
disp('Calculated y position =');
disp(calcendyposition);
disp('Calculated z position =');
disp(calcendzposition);



armmovebricktowalljtraj{s} = jtraj([waypointQ],[armmovebricktowallQ{s}],100);   %Calculating trajectory plan from waypoint to moving towards a brick

 for i = 1:100
     
 r.model.animate(armmovebricktowalljtraj{s}(i,:));  %Visually animating the robot to see it moving through space to go to a brick
 
 drawnow() 

brickstaticlocationendeffector = r.model.getpos;    %Updating the location of end effector, then updating the location that the brick is required to be, and then replotting the brick in every iteration of the for loop
brickendeffectorlocationmatrix = r.model.fkine(brickstaticlocationendeffector); 
delete(brick{s}); 
brickmovingposition = [brickendeffectorlocationmatrix(1,4) brickendeffectorlocationmatrix(2,4) brickendeffectorlocationmatrix(3,4)-0.0334];
brick{s} = PlaceObject('HalfSizedRedGreenBrick.ply', [brickmovingposition]);
pause(0.005);  
  
end
delete(brick{s});
brick{s} = PlaceObject('HalfSizedRedGreenBrick.ply', dropofflocationbrickxyz{s}); %Place brick on the floor in required location



findingacutualcurrentendeffectorpositiontolerancemarksQ = r.model.getpos(); %Continued from above, calculating tolerance within +-5mm
fourbyfourmatrixacutualcurrentendeffectorpositiontolerancemarks = r.model.fkine(findingacutualcurrentendeffectorpositiontolerancemarksQ);
actualendxposition = fourbyfourmatrixacutualcurrentendeffectorpositiontolerancemarks(1,4); 
actualendyposition = fourbyfourmatrixacutualcurrentendeffectorpositiontolerancemarks(2,4); 
actualendzposition = fourbyfourmatrixacutualcurrentendeffectorpositiontolerancemarks(3,4);
disp('Actual x position =');
disp(actualendxposition);
disp('Actual y position =');
disp(actualendyposition);
disp('Actual z position =');
disp(actualendzposition);

disp('Difference between calculated x and actual x ='); %Displaying differences to show output is less than +-5mm
display(calcendxposition - actualendxposition);
disp('Difference between calculated y and actual y =');
display(calcendyposition - actualendyposition);
disp('Difference between calculated z and actual z =');
display(calcendzposition - actualendzposition);




%% MOVING TO WAYPOINT WITHOUT BRICK

fprintf('MOVING TO WAYPOINT WITHOUT BRICK \n'); %Logging messages

armmovebricktowallQ{s} = r.model.getpos; %Get position of current joint angles


calcfinalendeffectorpositiontolerancemarks = r.model.fkine(waypointQ);  %Calculating where we think we are arriving, the calculating where we have actually arrived once we moved there, find the difference and show it is less than the +-5mm tolerance limit
calcendxposition = calcfinalendeffectorpositiontolerancemarks(1,4); 
calcendyposition = calcfinalendeffectorpositiontolerancemarks(2,4); 
calcendzposition = calcfinalendeffectorpositiontolerancemarks(3,4);
disp('Calculated x position =');
disp(calcendxposition);
disp('Calculated y position =');
disp(calcendyposition);
disp('Calculated z position =');
disp(calcendzposition);



armmovetowaypointafterbrickdropoffjtraj{s} = jtraj([armmovebricktowallQ{s}],[waypointQ],100);   %Calculating trajectory plan from brickwall to moving towards waypoint

for i = 1:100

  r.model.animate(armmovetowaypointafterbrickdropoffjtraj{s}(i,:)); %Visually animating the robot to see it moving through space to go to waypoint
  drawnow() 
  pause(0.005);  
  
end



findingacutualcurrentendeffectorpositiontolerancemarksQ = r.model.getpos(); %Continued from above, calculating tolerance within +-5mm
fourbyfourmatrixacutualcurrentendeffectorpositiontolerancemarks = r.model.fkine(findingacutualcurrentendeffectorpositiontolerancemarksQ);
actualendxposition = fourbyfourmatrixacutualcurrentendeffectorpositiontolerancemarks(1,4); 
actualendyposition = fourbyfourmatrixacutualcurrentendeffectorpositiontolerancemarks(2,4); 
actualendzposition = fourbyfourmatrixacutualcurrentendeffectorpositiontolerancemarks(3,4);
disp('Actual x position =');
disp(actualendxposition);
disp('Actual y position =');
disp(actualendyposition);
disp('Actual z position =');
disp(actualendzposition);

disp('Difference between calculated x and actual x ='); %Displaying differences to show output is less than +-5mm
display(calcendxposition - actualendxposition);
disp('Difference between calculated y and actual y =');
display(calcendyposition - actualendyposition);
disp('Difference between calculated z and actual z =');
display(calcendzposition - actualendzposition);



end

fprintf('FINISHED BUILDING WALL\n');    %Logging messages
fprintf('ROBOT SHUTTING DOWN \n');  %Logging messages

end
