
%% Setup virtual pendant 

pendant = VirtualTeachPendant;

%% Setting up robot

robot = Linear_UR3(false); %UR3

    %my teach
Joint_Controller(robot);

q = zeros(1,7);  % Set initial robot configuration 'q'

   %Table
PlaceObject('newroboticstable.ply', [0,0,-0.0844]);
hold on;

camlight;

HF = figure(1);  % Initialise figure to display robot
robot.model.plot3d(q,'workspace',[-2 2 -2 2 -0.67 2]);  % Plot robot in initial configuration
robot.model.delay = 0.001;  % Set smaller delay when animating
set(HF,'Position',[0.1 0.1 0.8 0.8]);

duration = 300;  % Set duration of the simulation (seconds)
dt = 0.15;       % Set up time step for simulation(seconds)

n = 0;  % Initialise step count to zero 
tic;    % Records simulation start time

while( toc < duration)
    
    n=n+1; % increment step count

%% Global Cartesian Movements 

    %Calling teach pendant
          axes = pendant.read;
 
    % 1 - turn joystick input into an end-effector velocity command
    Kv = 0.3; % linear velocity gain
    Kw = 0.8; %angular velocity gain

        %Multiplying linear velocity gain (Kv) with x,y,z axis to move
        %robot in each global axis 

    vx = Kv*axes(1); %x
    vy = Kv*axes(2); %y
    vz = Kv*axes(3); %z 

         %Multiplying angular velocity gain (Kw) with r,p,y axis to move
        %robot in each global axis

    wx = Kw*axes(4); %r
    wy = Kw*axes(5); %p
    wz = Kw*axes(6); %y
    
    dx = [vx;vy;vz;wx;wy;wz]; % combined velocity vector
    
    % 2 - use DLS J inverse to calculate joint velocity
    lambda = 0.5;
    J = robot.model.jacob0(q);
    Jinv_dls = inv((J'*J)+lambda^2*eye(7))*J';
    dq = Jinv_dls*dx;
    
    % 3 - apply joint velocity to step robot joint angles 
    q = q + dq'*dt;
      
    % -------------------------------------------------------------
    
    % Update plot
    robot.model.animate(q);  
    
    % wait until loop time elapsed
    if (toc > dt*n)
        warning('Loop %i took too much time - consider increating dt',n);
    end
    while (toc < dt*n); % wait until loop time (dt) has elapsed 
    end
end

