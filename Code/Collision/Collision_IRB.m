
function [  ] = Collision_IRB( )

clf
close all;
set(0,'DefaultFigureWindowStyle','docked')   % Docking the figure to the window on the right hand side  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% ENVIRONMENT 

    %For collision detection the robot moves to the plate stack
        %cooordinates 
    %For collsion avoidance the robot moves to a point you give      
        %it, make sure the plate stack coords are in it's path.

   %Table
PlaceObject('newroboticstable.ply', [0,0,-0.0844]);
hold on;
 
    %%%%%%%%%%%%%%%%%%%%%%%

    %Plate coordinates
 plate_stack = [-0.75,0,0]; 

    %Robot end-effector 
        %For Collision Avoidance, where you want robot end-effector to go 
 robot_eepos = [-0.9,0.1,0];

    %0 = Detection mode, 1 = Avoidance 
 col_mode = 0;

    %%%%%%%%%%%%%%%%%%%%%%%

    PlaceObject('plate.ply', [plate_stack(1),plate_stack(2),plate_stack(3)+0.00]); hold on;
    PlaceObject('plate.ply', [plate_stack(1),plate_stack(2),plate_stack(3)+0.02]); hold on;
    PlaceObject('plate.ply', [plate_stack(1),plate_stack(2),plate_stack(3)+0.04]); hold on;
    PlaceObject('plate.ply', [plate_stack(1),plate_stack(2),plate_stack(3)+0.06]); hold on;
    PlaceObject('plate.ply', [plate_stack(1),plate_stack(2),plate_stack(3)+0.08]); hold on;
    PlaceObject('plate.ply', [plate_stack(1),plate_stack(2),plate_stack(3)+0.10]); hold on;

    %Rectangular prism coords and side length
        %we treat the surface of the prism as triangles (12 triangles
        %total)
centerpnt = [plate_stack(1),plate_stack(2),plate_stack(3)+0.07];
side = 0.2;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
axis equal
  camlight
  
      %Create IRB_910sc
q = zeros(1,3);
irb = IRB_910sc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% DETECTION MODE

if col_mode == 0
    disp('Collsion Detection Mode')

        % Start point 
    q1 = [0,0,0];
        %Finish point
    q2 = irb.model.ikcon(transl(plate_stack)*trotx(pi),q1); 

     %Get the transform of every joint (start and end of every link)
tr =zeros(4,4,irb.model.n+1);
tr (:,:, 1) = irb.model.base;
L = irb.model.links;
for i = 1 : irb.model.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end
 
%Initally set steps = 2 from q1 to q2
    steps = 2;
%Go through q1 to q2, increasing the steps until there are no step sizes > than 1 degree
while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);

result = true(steps,1);
    %Iterate through all generated steps
for i = 1: steps
        %Checking IsCollision function, which checks for LinePlaneIntersection and 
        %IsIntersectionPointInsideTriangle
      result(i) = IsCollision(irb,qMatrix(i,:),faces,vertex,faceNormals,false);
      display(result(i));
        %If there is a collision detected
      if result(i) == 1
          for j = 1:irb.model.n %iterating through 1-7 links 
              qMatrix(:, j) = qMatrix(i, j); %overwrite all values in matrix with qmatrix value of current joint angles for loop index (to make it not go beyond collision)
          end
      end
      irb.model.animate(qMatrix(i,:));
        drawnow()
      pause(0.01);
        %If there is a collision break loop
      if result(i) == 1
            break
      end
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% AVOIDANCE MODE
       
if col_mode == 1
    disp('Collsion Avoidance Mode')    

         % Start and end point
 q1 = [0,0,0];
 q2 = irb.model.ikcon(transl(robot_eepos)* trotx(pi),q1); 

    %Animate start
irb.model.animate(q1);
    drawnow() 

qWaypoints = [q1;q2];
isCollision = true;
checkedTillWaypoint = 1;
qMatrix = [];

while (isCollision)
    startWaypoint = checkedTillWaypoint;
        %Keep iterating for the amount of waypoints there are between q1,
        %q2
    for i = startWaypoint:size(qWaypoints,1)-1
            %Make waypoint between one point to another inbetween q1, q2
        qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));

            %If a path between the waypoints can be made without collision
        if ~IsCollision(irb,qMatrixJoin,faces,vertex,faceNormals)
            qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>

            isCollision = false;
            checkedTillWaypoint = i+1;
                %Now try make waypoint between the start and end point
            qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q2],deg2rad(10));
              
                %If this possible, animate it and break loop 
            if ~IsCollision(irb,qMatrixJoin,faces,vertex,faceNormals)
                qMatrix = [qMatrix;qMatrixJoin];
                
                %Animate 
            for j=1:size(qMatrix,1)
                pause(0.05);
                irb.model.animate(qMatrix(j,:));
                drawnow()
            end
                % Reached goal without collision, so break out
                break;
            end
            %Else there is a collision between waypoints, find another path 
        else
            % Randomly pick a new pose that is not in collision
            qRand = (2 * rand(1,3) - 1) * pi;

                %Setting limits for link 3 (rod) rand generator so it does
                %not give it a value outside the rod length 
            rodmin = -0.180;
            rodmax = 0;
            rod = rodmin+rand(1,1)*(rodmax-rodmin);
            qRand(3) = rod;

            while IsCollision(irb,qRand,faces,vertex,faceNormals)
                qRand = (2 * rand(1,3) - 1) * pi;

                 %Again setting limits for link 3 (rod) rand generator so 
                 %it does not give value outside rod length
            rodmin = -0.180;
            rodmax = 0;
            rod = rodmin+rand(1,1)*(rodmax-rodmin);
            qRand(3) = rod;

            end
            qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
            isCollision = true;
            break;
        end
    end
end
%ur3.model.animate(qMatrix)
    %drawnow()%%%%%%%%%%%%%added this
% keyboard

end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % FUNCTIONS USED

%% IsIntersectionPointInsideTriangle

% Given a point which is known to be on the same plane as the triangle, determine if the point is inside (result == 1) or 
% outside a triangle (result == 0 ).

function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                    % intersectP is inside Triangle
end

%% IsCollision

% Syntax = robot model (irb), and trajectory (i.e. joint state vector) (qMatrix) and triangle obstacles in the environment 
% (faces,vertex,faceNormals)

function result = IsCollision(irb,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
            %the original line from Lab 5 was commented out and replaced with line below because collision was not working 
   
     %tr = GetLinkPoses( qMatrix(qIndex, :), ur3);
     [~, tr] = irb.model.fkine(qMatrix(qIndex,:));

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);

                %First, checks if a plane coincident (in line) with the rectangular prism, intersects with a robot's link 
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                
                %Next, it checks if this intersection on the plane, lies within a triangle face on the rectangular prism  
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                %Red dot
                plot3(intersectP(1),intersectP(2),intersectP(3),'r*');
                display('IRB is detecting collison');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end    
    end
end
end

%% GetLinkPoses
% Syntax: q = the robots joint angle positions, irb = seriallink robot model and transforms = the list of transforms

function [ transforms ] = GetLinkPoses( q, irb)

links = irb.model.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = irb.model.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
end
end

%% FineInterpolation
% Use results from Q2.6 to keep calling jtraj until all step sizes are
% smaller than a given max steps size
%
% Interpolating until waypoints between q0, q1 are smaller than a set
% degree.

function qMatrix = FineInterpolation(q1,q2,maxStepRadians)
if nargin < 3
    maxStepRadians = deg2rad(1);
end
    
steps = 2;
while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);
end

%% InterpolateWaypointRadians
% Given a set of waypoints, finely intepolate them
%
% Interpolating is the act of inserting an intermediate value between two other values. 
    % Since we know q0, q1, we need to interpolate joint angle positions inbetween q0, q1 to create a trajectory of 
    % waypoints

function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
if nargin < 2
    maxStepRadians = deg2rad(1);
end

qMatrix = [];
for i = 1: size(waypointRadians,1)-1
    qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
end
end
