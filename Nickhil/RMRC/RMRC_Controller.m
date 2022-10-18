classdef RMRC_Controller
    %RMRC_CONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        time
        deltaT
        steps
        deltaTheta
        epsilon
        Weightmatricx
        Controller
    end
    
    methods
        function Controller = RMRC_Controller()
            %RMRC_CONTROLLER Construct an instance of this class
            %   Detailed explanation goes here
            Controller.time = 5;             % Total time (s) time should be calculated from trjectory
            Controller.deltaT = 0.02;      % Control frequency
            Controller.steps = Controller.time/Controller.deltaT ;   % No. of steps for simulation
            Controller.deltaTheta = 2*pi/Controller.steps; % Small angle change
            Controller.epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
            Controller.Weightmatricx = diag([0.1 0.1 0.1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector

        end
        
        function RMRC(Controller, Robot, Traj)
            % T is trajectory
            robot = Robot.model;
            T = [rpy2r(Traj.theta(1,1),Traj.theta(2,1),Traj.theta(3,1)) Traj.x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
            q0 = robot.getpos;                                                            % Initial guess for joint angles
            qMatrix(1,:) = robot.ikcon(T,q0);                                            % Solve joint angles to achieve first waypoint

            for i = 1:Controller.steps-1
                T = robot.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
                deltaX = Traj.x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(Traj.theta(1,i+1),Traj.theta(2,i+1),Traj.theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/Controller.deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/Controller.deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                %Controller.deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                xdot = Controller.Weightmatricx*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                J = robot.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
                m(i) = sqrt(det(J*J'));
                if m(i) < Controller.epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/Controller.epsilon)*5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda *eye(robot.n))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:6                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + Controller.deltaT*qdot(i,j) < robot.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + Controller.deltaT*qdot(i,j) > robot.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + Controller.deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities

            end
            
            figure(1)
            plot3(Traj.x(1,:),Traj.x(2,:),Traj.x(3,:),'k.','LineWidth',1)
            
            Robot.appicablemovement = true;
            Robot.QMatrix = qMatrix;
            
        end
    end
end