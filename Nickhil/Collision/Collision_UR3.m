
function [  ] = Collision_UR3( )

clf
close all;
set(0,'DefaultFigureWindowStyle','docked')   % Docking the figure to the window on the right hand side  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% ENVIRONMENT 

   %Table
PlaceObject('newroboticstable.ply', [0,0,-0.0844]);
hold on;

    %For collision detection the robot moves to the plate stack
        %cooordinates 
    %For collsion avoidance the robot moves to a point -0.4 away
        %in the x-axis, from the stack of plates 

    %%%%%%%%%%%%%%%%%%%%%%%

    %Plate coords
 plate_stack = [-0.5,-0.1,0]; 

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
centerpnt = [plate_stack(1),plate_stack(2),plate_stack(3)+0.07];
side = 0.2;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
axis equal
  camlight
  
      %Create Col_Linear_UR3
q = zeros(1,7);
ur3 = Col_Linear_UR3(false);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% DETECTION MODE

if col_mode == 0
    disp('Collsion Detection Mode')
           
        %Start and end points 
    q1 = [0,0,0, 85*pi/180, 0, 0, 0];
    q2 = ur3.model.ikcon(transl(plate_stack)*trotx(pi),q1);

     %Get the transform of every joint (start and end of every link)
tr =zeros(4,4,ur3.model.n+1);
tr (:,:, 1) = ur3.model.base;
L = ur3.model.links;
for i = 1 : ur3.model.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end
 
 %Go through q1 to q2 until there are no step sizes > than 1 degree
    steps = 2;

while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);

result = true(steps,1);
    %Iterate through all generated steps
for i = 1: steps
      result(i) = IsCollision(ur3,qMatrix(i,:),faces,vertex,faceNormals,false);
      display(result(i));
        %If there is a collision detected
      if result(i) == 1
          for j = 1:ur3.model.n %iterating through 1-7 links 
              qMatrix(:, j) = qMatrix(i, j); %overwrite all values in matrix with qmatrix value of current for loop index
          end
      end
      ur3.model.animate(qMatrix(i,:));
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
 q1 = [0,0,pi/4,85*pi/180,pi/4,pi/2,0];
    %-0.4 = 0.4 passed plates in x-axis
 q2 = ur3.model.ikcon(transl(plate_stack(1)-0.4,plate_stack(2),plate_stack(3))* trotx(pi),q1); 
    disp('Collsion Avoidance Mode')    

ur3.model.animate(q1);
    drawnow() %%%%%%%%%%%%%%%added this
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
        if ~IsCollision(ur3,qMatrixJoin,faces,vertex,faceNormals)
            qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>

%             ur3.model.animate(qMatrixJoin);
%                 drawnow() %%%%%%%%%%%%%%%added this
            %size(qMatrix)

            isCollision = false;
            checkedTillWaypoint = i+1;
                %Now try make waypoint between the start and end point
            qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q2],deg2rad(10));
              
                %If this possible, animate it and break loop 
            if ~IsCollision(ur3,qMatrixJoin,faces,vertex,faceNormals)
                qMatrix = [qMatrix;qMatrixJoin];
                
                %Animate 
            for j=1:size(qMatrix,1)
                pause(0.05);
                ur3.model.animate(qMatrix(j,:));
                drawnow()
            end
                % Reached goal without collision, so break out
                break;
            end
            %Else there is a collision between waypoints, find another path 
        else
            % Randomly pick a new pose that is not in collision
            qRand = (2 * rand(1,7) - 1) * pi;

                %setting limits for link 1 (rail) rand generator so it does
                %not give it a value outside the rail length 
            lk1min = -0.8;
            lk1max = 0;
            lk1 = lk1min+rand(1,1)*(lk1max-lk1min);
            qRand(1) = lk1;

            while IsCollision(ur3,qRand,faces,vertex,faceNormals)
                qRand = (2 * rand(1,7) - 1) * pi;

                 %setting limits for link 1 (rail) rand generator
            lk1min = -0.8;
            lk1max = 0;
            lk1 = lk1min+rand(1,1)*(lk1max-lk1min);
            qRand(1) = lk1;

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
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
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

result = 1;                      % intersectP is in Triangle
end

%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(ur3,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)

        %original line from lab - commented out and replaced with line below to show collision      
        %points 
    %tr = GetLinkPoses(qMatrix(qIndex,:), ur3);

        % can use either line 
       %[~, tr] = ur3.model.fkine(ur3.model.getpos);
          %error "q must have 7 columns error"
     [~, tr] = ur3.model.fkine(qMatrix(qIndex,:));

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'r*');
                display('UR3 is detecting collison');
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
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, ur3)

links = ur3.model.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = ur3.model.base;

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
function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
if nargin < 2
    maxStepRadians = deg2rad(1);
end

qMatrix = [];
for i = 1: size(waypointRadians,1)-1
    qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
end
end
