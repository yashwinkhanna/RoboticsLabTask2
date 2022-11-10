
%% Setting pendant 
clf;
%Create our GUI teach pendant, with interactive sliders and gain values
%that can be communicated to the robot here
pendant = VirtualTeachPendant;

%% Setting up robot

robot = Linear_UR3(false); %UR3

        %%%%%

%To Enable Joint controller: uncomment line below and set break at line
    %"set(HF,'Position',[0.1 0.1 0.8 0.8]);"

    %Joint_Controller(robot); 

        %%%%%

q = zeros(1,7);  % Set initial robot configuration 'q'

   %Table
PlaceObject('newroboticstable.ply', [0,0,-0.0844]);
hold on;

camlight;

HF = figure(1);                                         % Initialises figure to display robot
robot.model.plot3d(q,'workspace',[-2 2 -2 2 -0.67 2]);  % Plot robot in initial configuration
robot.model.delay = 0.001;                              % Sets a smaller delay when animating
set(HF,'Position',[0.1 0.1 0.8 0.8]);                   % Break here for the joint controller

duration = 300;  % Set duration of the simulation (seconds)
dt = 0.15;       % Set up time step for simulation(seconds)

n = 0;  % Initialise step count to zero 
tic;    % Records simulation start time

while( toc < duration)
    
    n=n+1; % increment step count

%% Global Cartesian Movements 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%Read teach pendant values (from slider) and store into (6x1) axes
    %matrix
          axes = pendant.read;
 
    %1 - Turn teach pendant input into an end-effector velocity command
    Kv = 0.3; % linear velocity gain
    Kw = 0.8; %angular velocity gain

        %Multiplying linear velocity gain (Kv) with x,y,z axis value selected on the pendant to move
        %robot in each global axis 

    vx = Kv*axes(1); %x
    vy = Kv*axes(2); %y
    vz = Kv*axes(3); %z 

        %Multiplying angular velocity gain (Kw) with r,p,y axis value selected on the pendant to move
        %robot in each global axis

    wx = Kw*axes(4); %r
    wy = Kw*axes(5); %p
    wz = Kw*axes(6); %y
    
    dx = [vx;vy;vz;wx;wy;wz]; % Combine all values in one velocity vector
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Calculating the Joint velocity 

    %2 - Here we use DLS J inverse to calculate joint velocity
       % without DLS the robot arm would move violently when approaching a
       % singularity

    lambda = 0.5;
    J = robot.model.jacob0(q);

    % Finding the DLS Inverse Jacobian by adding Inverse Jacobian to DLS
        Jinv_dls = inv((J'*J)+lambda^2*eye(7))*J';

    % Multiplies J Inverse DLS by combined velocity vector taken from our
    % manual teach pendant input 
       dq = Jinv_dls*dx;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Convert Join velocity to Joint angles 

    %3 - Now that we have the Joint velocities we need to convert to robot  
       % joint angles, so robot knows how to move according to what was inputted using pendant slider/ gain 
       
       % Updating value of q as it moves
       % current q + complex conjugate of dq multiplied by dt (time step)
    q = q + dq'*dt;
          
    % Finally, animate the robots movement 
       robot.model.animate(q);  
    
    % wait until loop time elapsed
    if (toc > dt*n)
        warning('Loop %i took too much time - consider increating dt',n);
    end
    while (toc < dt*n); % wait until loop time (dt) has elapsed 
    end
end

