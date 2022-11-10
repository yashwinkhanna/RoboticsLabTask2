classdef RMRC < handle
    properties
        t;                               % The total time (s) default
        deltaT;                          % Control frequency default
        steps;                           % No. of steps for simulation
        delta;                           % Small angle change
        epsilon;                         % Threshold value for manipulability/ Damped Least Squares
        W;                               % Weighting matrix for velocity vector
        workspace = [-2 2 -2 2 -0.3 2];   

    end
    
    methods
        %% structor
           % Initiallise the variables from properties with default values 
           % from Lab 9
        function self = RMRC()

                % Control frequency, default
            self.deltaT = 0.02;      
                % No. of steps for simulation
            self.steps = self.t/self.deltaT;
                % Threshold value for manipulability/Damped Least Squares
            self.epsilon = 0.1;   
                % Weighting matrix for the velocity vector
            self.W = diag([0.1 0.1 0.1 0.1 0.1 0.1]); 
            
           % Note: since 'time' is an input arugment in our axial function
                 % we set it to 1, in main
        end
        
        %% Movement colinear to given axis

        % Syntax:
                % robot_in = which robot to move
                % axis = axis to translate along: 'x' 'y' or 'z' (appostrophe included)
                % location = end effector current pose as homo 4x4 transform
                % displacement = desired travel distance
                % time = time default -> use 10 if unsure. minimum is 0.5

        function matrix = axial(self, robot_in, axis, location, displacement, time)

            robot = robot_in.model;
            self.t = time;
                % No. of steps for the simulation
            self.steps = self.t/self.deltaT;      
            
        % Allocating all the array data
            m = zeros(self.steps,1);              % Array for Measure of Manipulability
            qMatrix = zeros(self.steps,7);        % Array for joint anglesR
            qdot = zeros(self.steps,7);           % Array for joint velocities
            theta = zeros(3,self.steps);          % Array for roll-pitch-yaw angles
            x = zeros(3,self.steps);              % Array for x-y-z trajectory
            positionError = zeros(3,self.steps);  % For plotting trajectory error
            angleError = zeros(3,self.steps);     % For plotting trajectory error
            
        % Trapezoidal trajectory scalar
            s = lspb(0,1,self.steps);               
          % 'lspb' stands for Linear segment with parabolic blend which creates a matrix 
          % (rows = steps by coloumn = 1), where first value is 0 and last is 1 with an increment depending on steps  
          % input.
              %this will then be mutliplied by a given position as a 'weight' to develop 
              % our trajectory
              
         % Extracting the x,y,z values from 'location', a 4 by 4 transformation matrix and 
         % storing them 
                % location is a transformation matrix, since that is what we input into it 
                % inside the main code
            loc_x = location(1, 4);
            loc_y = location(2, 4);
            loc_z = location(3, 4);

         % Extracting roll, pitch, yaw from 'location' using tr2rpy and storing in variables 
            rpy = tr2rpy(location);
            loc_r = rpy(1);
            loc_p = rpy(2);
            loc_yw =rpy(3);

            % If x is the input arguement, move in global x-axis 
            if axis == 'x'
                for i=1:self.steps

                    %Equation uses 'lspb' to create a trajectory in our desired displacement 
                        %inputted
                    x(1,i) = ((1-s(i))*loc_x) + (s(i)*(loc_x+displacement));  % Points in x
                    x(2,i) = loc_y;  % Points in y (do not change)
                    x(3,i) = loc_z;  % Points in z (do not change)

                    theta(1,i) = loc_r;  % Roll
                    theta(2,i) = loc_p;  % Pitch 
                    theta(3,i) = loc_yw; % Yaw 
                end
            end

            % If y is the input arguement, move in global y-axis 
            if axis == 'y'
                for i=1:self.steps
                    x(1,i) = loc_x; % Points in x (do not change)
                    %Equation uses 'lspb' to create a trajectory in our desired displacement 
                        %inputted
                    x(2,i) = ((1-s(i))*loc_y) + (s(i)*(loc_y+displacement));  % Points in y
                    x(3,i) = loc_z; % Points in z (do not change)

                    theta(1,i) = loc_r;  % Roll  
                    theta(2,i) = loc_p;  % Pitch 
                    theta(3,i) = loc_yw; % Yaw
                end
            end

            % If z is the input arguement, move in global z-axis 
            if axis == 'z'
                for i=1:self.steps

                    x(1,i) = loc_x; % Points in x (do not change)
                    x(2,i) = loc_y; % Points in y (do not change)
                    %Equation uses 'lspb' to create a trajectory in our desired displacement 
                        %inputted
                    x(3,i) = ((1-s(i))*loc_z) + (s(i)*(loc_z+displacement));  % Points in z

                    theta(1,i) = loc_r;  % Roll  
                    theta(2,i) = loc_p;  % Pitch 
                    theta(3,i) = loc_yw; % Yaw 
                end
            end
            
            % stg1 = 's(i) calced'

            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
            q0 = zeros(1,7);                                                            % Initial guess for joint angles
            qMatrix(1,:) = robot.ikcon(T,q0);                                           % Solve joint angles, to achieve first waypoint

         % Track trajectory with RMRC - same as Lab 9 code
                % This section of code ensures the robot end-effector stays on path, following a straight line by constantly checking its
                % current position and determining its position error, before moving to next waypoint in the steps trajectory from 'lspb' 
                %
                % It also uses Dampled Least Squares incase it has to deal with singularities and avoids going past robot's joint limits.
            for i = 1:self.steps-1

                T = robot.fkine(qMatrix(i,:));                                          % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/self.deltaT)*(Rd - Ra);                                       % Calculating the rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/self.deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                xdot = self.W*[linear_velocity;angular_velocity];                       % Calculate end-effector velocity to reach next waypoint.
                J = robot.jacob0(qMatrix(i,:));                                         % Gets Jacobian at current joint state
                m(i) = sqrt(det(J*J'));
                if m(i) < self.epsilon                                                  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/self.epsilon)*5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda *eye(7))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                               % Solve RMRC equation (you may need to transpose the vector)
                for j = 1:7                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + self.deltaT*qdot(i,j) < robot.qlim(j,1)           % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0;                                                  % Stop the motor
                    elseif qMatrix(i,j) + self.deltaT*qdot(i,j) > robot.qlim(j,2)       % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0;                                                  % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + self.deltaT*qdot(i,:);                  % Update next joint state based on joint velocities
                positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
                angleError(:,i) = deltaTheta;                                           % For plotting
            end
            matrix = qMatrix;
             % stg2 = 'rmrc class'
    
        end  % end linear function
        
    end  % end methods
        
end  % end class
