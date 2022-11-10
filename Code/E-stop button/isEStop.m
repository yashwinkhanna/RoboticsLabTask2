
% estop = ES_GUI;

    %Storing the mlapp ButtonValuePublic function state into a variable
        %this will = 1 if estop button has been pressed 
button = estop.ButtonValuePublic;

    %Storing the mlapp ResumeStatePublic function state into a variable
        %this will = 1 if estop button has been pressed 
res_state = estop.ResumeStatePublic; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Light System

% The yellow light is initially turned off
booleantrafficlightyellow = 0; 

% If estop is pressed, delete green light
if button == 1 && booleantrafficlightgreen == 1
    delete(trafficlightgreen);
    booleantrafficlightgreen = 0;
end

% If estop is pressed, enable yellow light
if button == 1
    booleantrafficlightyellow = 1;
    trafficlightyellow = PlaceObject('trafficlightyellow.ply', [-0.75,0.75,0]);
    hold on;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       
resume_wait = 0;  %variable: ON/ OFF toggle to know if to wait for resume button press before killing while loop
                  %necessary for 2-action resume

% Logic: If the estop is engaged, code will be stuck in while loop until disengaged. By calling isEstop in every 
% animation loop of robot in main, we will be stuck in the estop while, preventing the next robot animation step.

if button == 1 % e-stop pressed
    resume_wait = 1;% set resume to high
    while resume_wait == 1 % loop will repeat until resume_wait change to 0
        if button == 0 % if estop de-pressed
            if res_state == 1 % if resume button pressed
                resume_wait = 0; % change resume_wait to 0 to break out from while loop 
            end
            res_state = estop.ResumeStatePublic; % re-fills current resume button value
            fprintf('STOPPED - EStop disengaged. Press Resume to continue\n');
        end
        fprintf('STOPPED - Disengage E-Stop to continue\n');
        pause(1);

            % checking state of estop and res_state button before looping 
            % while 
        button = estop.ButtonValuePublic;
        res_state = estop.ResumeStatePublic;
        
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Light System

% If E-stop disengaged turn yellow light to green
if booleantrafficlightyellow == 1 && button == 0 
    delete(trafficlightyellow);
    booleantrafficlightyellow = 0;
    booleanresumebutton = 0;

    trafficlightgreen = PlaceObject('trafficlightgreen.ply', [-0.75,0.75,0]);    
    hold on;
    booleantrafficlightgreen = 1;
end

fprintf('EStop DISENGAGED. Resuming process now...\n');