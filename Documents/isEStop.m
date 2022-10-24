

% function breaktest()
%      button = estop.ButtonValuePublic;
%      while button == 1
%         fprintf('STOPPED - Disengage E-Stop to continue\n');
%         pause(0.5);
%         button = estop.ButtonValuePublic;
%      end
% end
% estop = GUItest1;

     %ledsvalue = 0;
     button = estop.ButtonValuePublic;
%      res_button = estop.R;
     
     
     res_state = estop.ResumeStatePublic;
%      res_state = 0;
     booleantrafficlightyellow = 0;
    % booleantrafficlightgreen = 1;
   
    
%     booleanresumebutton = 0;
     
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
     
% res_prev = res_state;     
resume_wait = 0;  


if button == 1
    resume_wait = 1;
    while resume_wait == 1
        if button == 0
            if res_state == 1
                resume_wait = 0;
            end
            res_state = estop.ResumeStatePublic;
            fprintf('STOPPED - EStop disengaged. Press Resume to continue\n');
        end
        fprintf('STOPPED - Disengage E-Stop to continue\n');
        pause(1);
        button = estop.ButtonValuePublic;
        res_state = estop.ResumeStatePublic;
        
    end
%     estop.ResumeStateButtonValuePublic = 0;
end

    
%     while button == 1
%         fprintf('STOPPED - Disengage E-Stop to continue\n');
%         pause(1);
%         button = estop.ButtonValuePublic
%         resume_wait
%         
% %         while button == 0 && booleanresumebutton == 0
% %          fprintf('E-Stop has been disengaged, please press resume\n')
% %          pause(1);
% %          
% %         end
% 
%             if button == 0
%                resume_wait = 1;
%             end
%               
%        % ledsvalue = 1;
%     end
%    res_state = estop.ResumeStateButtonValuePublic
% end

while resume_wait ==1
    
    
end
     

     
     
     if booleantrafficlightyellow == 1 && button == 0 %&& booleanresumebutton == 1
     delete(trafficlightyellow);
     booleantrafficlightyellow = 0;
     booleanresumebutton = 0;
     
      trafficlightgreen = PlaceObject('trafficlightgreen.ply', [-0.75,0.75,0]);     %Loading in kitchen environment
        hold on;
        booleantrafficlightgreen = 1;
     end