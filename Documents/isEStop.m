

% function breaktest()
%      button = estop.ButtonValuePublic;
%      while button == 1
%         fprintf('STOPPED - Disengage E-Stop to continue\n');
%         pause(0.5);
%         button = estop.ButtonValuePublic;
%      end
% end

     button = estop.ButtonValuePublic;
     while button == 1
        fprintf('STOPPED - Disengage E-Stop to continue\n');
        pause(0.5);
        button = estop.ButtonValuePublic;
     end