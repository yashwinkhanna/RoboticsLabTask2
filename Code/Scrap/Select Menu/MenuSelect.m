
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %ROBOT PANCAKE CHEF 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %Change this to select from the modes below: 
mode = 0; 

    %% Pancake Making Mode  

if mode == 0
    
    %This mode goes through creating and plating a pancake with e-stop 
    %functionality to stop and resume the process.

    robotChefMain;
end

    %% Collision Avoidance

if mode == 1

    %This mode undergoes collision detection or collision avoidance for
    %the robots, by interacting with plates

    Collision_IRB;
    Collision_UR3;
end

   %% Asyncronous Stop Signal

if mode == 2

    %This mode runs the pancake making process but stops if a person has
    %entered the area using a light curtain 

    Async_StopSignal;
end

    %% Advanced Teach Mode

if mode == 3

    %This mode lets the user interact with the robot arm manually in the
    %global cartesian plane and joint angles


end