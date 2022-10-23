%% Robotics
% Lab 5 - Questions 2 and 3: 3-link plannar collision check and avoidance
function [  ] = Collision_UR3( )

clf
close all;
set(0,'DefaultFigureWindowStyle','docked')   % Docking the figure to the window on the right hand side  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% ENVIRONMENT 

    %Plate coords - ONLY NEED TO CHANGE THIS
plate_stack = [-0.6,0.1,0]; 

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

     %UR3 Start and end point 
 q1 = [   0,0,0,85*pi/180,0,0,0];
 q2 = ur3.model.ikcon(transl(plate_stack)*trotx(pi),q1); 
%q2 = [-0.8,0,0,0,0,0,2*pi]; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% PROCESS

    %Get the transform of every joint (i.e. start and end of every link)
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

% 2.7
result = true(steps,1);
for i = 1: steps
%         offset_weighted = round(steps/45);
%         offset = 25;
%         result(i) = IsCollision(ur3,qMatrix(i+round(offset*(steps/(steps-i))),:),faces,vertex,faceNormals,false);
        result(i) = IsCollision(ur3,qMatrix(i,:),faces,vertex,faceNormals,false);
        display(result(i));
        if result(i) == 1
            for j = 1:ur3.model.n %iterating through 1-7 links 
                qMatrix(:, j) = qMatrix(i, j); %overwrite all values in matrix with qmatrix value of current for loop index
            end
            
            %qMatrix(:, 1) = qMatrix(i, 1); %overwrite all values in matrix with qmatrix value of current for loop index            
        end
        ur3.model.animate(qMatrix(i,:));
        drawnow()
        pause(0.01);
        if result(i) == 1
            break
        end
end


%% Question 3
% 3.1: Manually move robot with teach and add some waypoints to qMatrix
% ur3.model.animate(q1);
% ur3.model.teach
% qWaypoints = [q1 ...
%     ; -pi/4,deg2rad([-111,-72]) ...
%     ; deg2rad([169,-111,-72]) ...
%     ; q2];
% qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));
% if IsCollision(ur3,qMatrix,faces,vertex,faceNormals)
%     error('Collision detected!!');
% else
%     display('No collision found');
% end
% ur3.model.animate(qMatrix);

% 3.2: Manually create cartesian waypoints
% ur3.model.animate(q1);
% qWaypoints = [q1 ; ur3.model.ikcon(transl(0,-0.5,0.5),q1)];
% qWaypoints = [qWaypoints; ur3.model.ikcon(transl(-0.1,-0.5,0.5),qWaypoints(end,:))];
% qWaypoints = [qWaypoints; ur3.model.ikcon(transl(-0.2,-0.5,0.5),qWaypoints(end,:))];
% qWaypoints = [qWaypoints; ur3.model.ikcon(transl(-0.3,-0.5,0.5),qWaypoints(end,:))];
% qWaypoints = [qWaypoints; ur3.model.ikcon(transl(-0.4,-0.5,0.5),qWaypoints(end,:))];
% qWaypoints = [qWaypoints; ur3.model.ikcon(transl(-0.5,-0.5,0.5),qWaypoints(end,:))];
% qWaypoints = [qWaypoints; ur3.model.ikcon(transl(-0.6,-0.5,0.5),q2)];
% qWaypoints = [qWaypoints; q2];
% qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));
% if IsCollision(ur3,qMatrix,faces,vertex,faceNormals)
%     error('Collision detected!!');
% else
%     display('No collision found');
% end
% ur3.model.animate(qMatrix);        

% 3.3: Randomly select waypoints (primative RRT)
% ur3.model.animate(q1);
%     drawnow() %%%%%%%%%%%%%%%added this
% qWaypoints = [q1;q2];
% isCollision = true;
% checkedTillWaypoint = 1;
% qMatrix = [];
% while (isCollision)
%     startWaypoint = checkedTillWaypoint;
%     for i = startWaypoint:size(qWaypoints,1)-1
%         qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));
%         if ~IsCollision(ur3,qMatrixJoin,faces,vertex,faceNormals)
%             qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
%             ur3.model.animate(qMatrixJoin);
%                 drawnow() %%%%%%%%%%%%%%%added this
%             size(qMatrix)
%             isCollision = false;
%             checkedTillWaypoint = i+1;
%             % Now try and join to the final goal (q2)
%             qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q2],deg2rad(10));
%             if ~IsCollision(ur3,qMatrixJoin,faces,vertex,faceNormals)
%                 qMatrix = [qMatrix;qMatrixJoin];
%                 % Reached goal without collision, so break out
%                 break;
%             end
%         else
%             % Randomly pick a pose that is not in collision
%             qRand = (2 * rand(1,3) - 1) * pi;
%             while IsCollision(ur3,qRand,faces,vertex,faceNormals)
%                 qRand = (2 * rand(1,3) - 1) * pi;
%             end
%             qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
%             isCollision = true;
%             break;
%         end
%     end
% end
% ur3.model.animate(qMatrix)
%     drawnow()%%%%%%%%%%%%%added this
% % keyboard

end

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
%       [~, tr] = ur3.model.fkine(ur3.model.getpos);
          %error "q must have 7 columns error"
     [~, tr] = ur3.model.fkine(qMatrix(qIndex,:));

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'r*');
%                 q1 = 
                display('UR3 is at a collison');
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
