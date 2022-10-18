classdef rmrcTraj < handle
    properties
        %> Robot model
        model;
        t = 10;             % Total time (s) default
        deltaT = 0.02;      % Control frequency default
        steps = t/deltaT;   % No. of steps for simulation
        delta = 2*pi/steps; % Small angle change
        epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
        W = diag([0.1 0.1 0.1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
        workspace = [-2 2 -2 2 -0.3 2];   

    end
    
    methods
        function linear(self, axis, displacement)
            m = zeros(steps,1);             % Array for Measure of Manipulability
            qMatrix = zeros(steps,7);       % Array for joint anglesR
            qdot = zeros(steps,7);          % Array for joint velocities
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            x = zeros(3,steps);             % Array for x-y-z trajectory
            positionError = zeros(3,steps); % For plotting trajectory error
            angleError = zeros(3,steps);    % For plotting trajectory error
            
            s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
            for i=1:steps
            %     x(1,i) = (1-s(i))*-0.35 + s(i)*0.35; % Points in x
            %     x(2,i) = (1-s(i))*-0.5 + s(i)*-0.5; % Points in y
                x(3,i) = (1-s(i))*0.25 + s(i)*0.5; % Points in z

                x(1,i) = -1; %(1-s(i)) + s(i); % Points in x
                x(2,i) = 0.35; %(1-s(i)) + s(i); % Points in y
            %     x(3,i) = 0.25; %(1-s(i)); % + s(i); % Points in z

                theta(1,i) = 0;                 % Roll angle 
                theta(2,i) = pi/2; %5*pi/9;            % Pitch angle
                theta(3,i) = 0;                 % Yaw angle
            end

            
    
        end
        
    %% RMRC(axis, displacement)
    % RMRC(z, 0.5)