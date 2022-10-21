
%15/10 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Create robot and environment 

clf
    % Docking the figure to the window on the right hand side 
set(0,'DefaultFigureWindowStyle','docked')              

    %move the UR3 to this x,y,z
 grill_pos = [-0.4 ,0.5, 0];
 pos2 = [0, 0.5, 0];

    %Create the Linear UR3 arm
robot = Linear_UR3(false);

    %To interact with q joints 
robot.model.teach;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Move robot arm to grill location

    %Create a vector of initial joint angles, in this case the joint angles 
    %are zero
q = zeros(1,7);

    %Inverse kinematics to find joint angles needed to move to brick spawn
    %coords brick1_xyz.
            %note - trotx(pi) makes end-effector face down when going to a 
                    %brick, so arm does not go into floor. 
    q1 = robot.model.ikine(transl(grill_pos) * trotx(pi), q, [1,1,1,0,0,0]);


    %jtraj creates a path between one set of joint positions (q) and a second
    %set of joint positions (q1) using a certain amount of set increments (50)
path = jtraj (q,q1,50);

    %Put the jtraj to action usiing for loop 
     for i = 1:50
        pause(0.01);

        %Animate actually makes the arm move 
        %i,: is just the saying the current ith row and all columns
      robot.model.animate(path(i,:));
      
        %drawnow() displays the arm movement 
      drawnow()
     end

     %% RMRC - move from gill_pos to pos_2 (from Lab 6)
% % 3.6
% x1 = [1.5 1]';
% x2 = [1.5 -1.5]';
% deltaT = 0.05;                                        % Discrete time step
% 
% % % 3.7
% % x = zeros(2,steps);
% % s = lspb(0,1,steps);                                 % Create interpolation scalar
% % for i = 1:steps
% %     x(:,i) = x1*(1-s(i)) + s(i)*x2;                  % Create trajectory in x-y plane
% % end
% % 
% % % 3.8
% % qMatrix = nan(steps,2);
% % 
% % % 3.9
% % qMatrix(1,:) = p2.ikine(T1,[0 0],M);                 % Solve for joint angles
% % 
% % % 3.10
% % for i = 1:steps-1
% %     xdot = (x(:,i+1) - x(:,i))/deltaT;                             % Calculate velocity at discrete time step
% %     J = p2.jacob0(qMatrix(i,:));            % Get the Jacobian at the current state
% %     J = J(1:2,:);                           % Take only first 2 rows
% %     qdot = inv(J)*xdot;                             % Solve velocitities via RMRC
% %     qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';                   % Update next joint state
% % end
% 
% Kv = 0.3; %linear velocity gain
% Kw = 0.8; %angular velocity gain;
% 
% qtest = [1,1.5,1,0,0,0];
% J = robot.model.jacob0(qtest);
% % dq = pinv(J)*

    %To get the robots current joint angles
%x = r.model.getpos and 

    %To get the robots current end-effector position, by using fkine, which
    %let us determine the end-effector position if we know the joint angles 
% pos = r.model.fkine(x)

%t2rpy 
