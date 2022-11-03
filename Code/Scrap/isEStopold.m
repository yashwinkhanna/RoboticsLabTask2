

% function breaktest()
%      button = estop.ButtonValuePublic;
%      while button == 1
%         fprintf('STOPPED - Disengage E-Stop to continue\n');
%         pause(0.5);
%         button = estop.ButtonValuePublic;
%      end
% end


     %ledsvalue = 0;
     button = estop.ButtonValuePublic;
     booleantrafficlightyellow = 0;
    % booleantrafficlightgreen = 1;
   % booleanresumebutton = 0;
     
          if button == 1 && booleantrafficlightgreen == 1
              delete(trafficlightgreen);
              booleantrafficlightgreen = 0;
          end
     
     if button == 1 
       
      % delete(trafficlightgreen);
       booleantrafficlightyellow = 1;
       trafficlightyellow = PlaceObject('trafficlightyellow.ply', [-0.75,0.75,0]);    %Loading traffic cone
       hold on;
     end
     
     
     while button == 1
        fprintf('STOPPED - Disengage E-Stop to continue\n');
        pause(2);
        button = estop.ButtonValuePublic;   
        
%         while button == 0 && booleanresumebutton == 0
%          fprintf('E-Stop has been disengaged, please press resume\n')
%          pause(1);
%          
%         end
              
       % ledsvalue = 1;
     end
     

     
     
     if booleantrafficlightyellow == 1 && button == 0 %&& booleanresumebutton == 1
     delete(trafficlightyellow);
     booleantrafficlightyellow = 0;
     booleanresumebutton = 0;
     
      trafficlightgreen = PlaceObject('trafficlightgreen.ply', [-0.75,0.75,0]);     %Loading in kitchen environment
        hold on;
        booleantrafficlightgreen = 1;
     end