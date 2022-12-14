 %SerialLink.teach Graphical teach pendant
%
% R.teach(Q, OPTIONS) allows the user to "drive" a graphical robot by means
% of a graphical slider panel. If no graphical robot exists one is created
% in a new window.  Otherwise all current instances of the graphical robot
% are driven.  The robots are set to the initial joint angles Q.
%
% R.teach(OPTIONS) as above but with options and the initial joint angles
% are taken from the pose of an existing graphical robot, or if that doesn't
% exist then zero.
%
% Options::
% 'eul'           Display tool orientation in Euler angles (default)
% 'rpy'           Display tool orientation in roll/pitch/yaw angles
% 'approach'      Display tool orientation as approach vector (z-axis)
% '[no]deg'       Display angles in degrees (default true)
% 'callback',CB   Set a callback function, called with robot object and
%                 joint angle vector: CB(R, Q)
%
% Example::
%
% To display the velocity ellipsoid for a Puma 560
%
%        p560.teach('callback', @(r,q) r.vellipse(q));
%
% GUI::
%
% - The specified callback function is invoked every time the joint configuration changes.
%   the joint coordinate vector.
% - The Quit (red X) button destroys the teach window.
%
% Notes::
% - If the robot is displayed in several windows, only one has the
%   teach panel added.
% - The slider limits are derived from the joint limit properties.  If not
%   set then for
%   - a revolute joint they are assumed to be [-pi, +pi]
%   - a prismatic joint they are assumed unknown and an error occurs.
%
% See also SerialLink.plot, SerialLink.getpos.


% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

% a ton of handles and parameters created by this function are stashed in 
% a structure which is passed into all callbacks 

function Joint_Controller(robot, varargin)
    
    %-------------------------------
    % parameters for teach panel
    bgcol = [0.00,0.45,0.74];  % background color
    height = 0.06;  % height of slider rows
    %-------------------------------
    
    
    % Handle options
    opt.deg = true;
    opt.orientation = {'rpy', 'eul', 'approach'};
    opt.callback = [];    
    [opt,args] = tb_optparse(opt, varargin);
    
    % Stash some options into the persistent object
    handles.orientation = opt.orientation;
    handles.callback = opt.callback;
    handles.opt = opt;

    
    % We need to have qlim set to finite values for a prismatic joint
    qlim = robot.model.qlim;
    if any(isinf(qlim))
        error('RTB:teach:badarg', 'Must define joint coordinate limits for prismatic axes, set qlim properties for prismatic Links');
    end
    
    if isempty(args)
        q = [];
    else
        q = args{1};
    end
    
    % Set up scale factor, from actual limits in radians/metres to display units
    qscale = ones(robot.model.n,1);
    for j=1:robot.model.n
        L=robot.model.links(j);
        if opt.deg && L.isrevolute
            qscale(j) = 180/pi;
        end
    end
    
    handles.qscale = qscale;
    handles.robot = robot;
    robot = robot.model;
   
    
    % Install the panel at the side of the figure
    
    % find the right figure to put it in
    c = findobj(gca, 'Tag', robot.name);  % check the current axes
    if isempty(c)
        % doesn't exist in current axes, look wider
        c = findobj(0, 'Tag', robot.name);  % check all figures
        if isempty(c)
            % create robot in arbitrary pose
            robot.plot( zeros(1, robot.n) );
            ax = gca;
        else
            ax = get(c(1), 'Parent'); % get first axis holding the robot
        end
    else
        % found it in current axes
        ax = gca;
    end
    handles.fig = get(ax, 'Parent');  % get the figure that holds the axis
    
    % shrink the current axes to make room
    %   [l b w h]
    set(ax, 'OuterPosition', [0.25 0 0.70 1])
    
    handles.curax = ax;

    
    % Create the panel that holds everything
    panel = uipanel(handles.fig, ...
        'BackGroundColor',[0.00,0.45,0.74] ,...
        'Position', [0 0 .25 1],...
        'FontUnits', 'normalized', ...
        'FontSize', 0.06);
    set(panel, 'Units', 'pixels'); % stop automatic resizing
    handles.panel = panel;
    set(handles.fig, 'Units', 'pixels');

    set(handles.fig, 'ResizeFcn', @(src,event) resize_callback(robot, handles));
    
         %Display text: Joint 
    uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor',[0.00,0.45,0.74], ...
        'Position', [0.03 0 1.5 1], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.06, ...
        'HorizontalAlignment', 'left', ...
        'String', 'Joint',...
        'ForegroundColor', 'white',...
        'FontAngle','italic',...
        'FontName','Franklin Gothic Demi');

        %Display text: Controller 
    uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor',[0.00,0.45,0.74], ...
        'Position', [0.03 -0.06 1.5 1], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.06, ...
        'HorizontalAlignment', 'left', ...
        'String', 'Controller',...
        'ForegroundColor', 'white',... 
        'FontAngle','italic',...
        'FontName','Franklin Gothic Demi');

    
    % Get the current robot state
    
    if isempty(q)
        % check to see if there are any graphical robots of this name
        rhandles = findobj('Tag', robot.name);
        
        % find the graphical element of this name
        if isempty(rhandles)
            error('RTB:teach:badarg', 'No graphical robot of this name found');
        end
        % get the info from its Userdata
        info = get(rhandles(1), 'UserData');
        
        % the handle contains current joint angles (set by plot)
        if ~isempty(info.q)
            q = info.q;
        end
    else
    robot.plot(q);
    end
    handles.q = q;
    T6 = robot.fkine(q);

    
    % Now make the sliders
    n = robot.n;
    for j=1:n
        % slider 'J' label
        uicontrol(panel, 'Style', 'text', ...
            'Units', 'normalized', ...
            'BackgroundColor', bgcol, ...
            'Position', [0.045 height*(n-j+2.8) 0.15 height], ...
            'FontUnits', 'normalized', ...
            'FontSize', 0.4, ...
            'String', sprintf('J%d', j));
        
        % slider itself
        q(j) = max( qlim(j,1), min( qlim(j,2), q(j) ) ); % clip to range
        handles.slider(j) = uicontrol(panel, 'Style', 'slider', ...
            'Units', 'normalized', ...
            'Position', [0.2 height*(n-j+3) 0.59 height], ...
            'Min', qlim(j,1), ...
            'Max', qlim(j,2), ...
            'Value', q(j), ...
            'Tag', sprintf('Slider%d', j));
        
        % text box showing slider value, also editable
        handles.edit(j) = uicontrol(panel, 'Style', 'edit', ...
            'Units', 'normalized', ...
            'Position', [0.80 height*(n-j+2.9)+.01 0.15 0.9*height], ...
            'BackgroundColor', bgcol, ...
            'String', num2str(qscale(j)*q(j), 3), ...
            'HorizontalAlignment', 'left', ...
            'FontUnits', 'normalized', ...
            'FontSize', 0.4, ...
            'Tag', sprintf('Edit%d', j));
    end
    
    % Set up the position display box
    
    % X display text
    uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.1 0.83-height 0.3 height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.4, ...
        'HorizontalAlignment', 'left', ...
        'String', 'X:');
    
    % X value
    handles.t6.t(1) = uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.25 0.85-height 0.65 0.04], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.6, ...
        'String', sprintf('%.3f', T6(1,4)), ...
        'Tag', 'T6');
    
    % Y display text
    uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.1 0.78-height 0.2 height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.4, ...
        'HorizontalAlignment', 'left', ...
        'String', 'Y:');
    
    % Y value
    handles.t6.t(2) = uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.25 0.80-height 0.65 0.04], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.6, ...
        'String', sprintf('%.3f', T6(2,4)));
    
    % Z display text
    uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.1 0.73-height 0.2 height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.4, ...
        'HorizontalAlignment', 'left', ...
        'String', 'Z:');
    
    % Z value
    handles.t6.t(3) = uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.25 0.75-height 0.65 0.04], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.6, ...
        'String', sprintf('%.3f', T6(3,4)));
    
    % Orientation
    switch opt.orientation
        case 'approach'
            labels = {'ax:', 'ay:', 'az:'};
        case 'eul'
            labels = {[char(hex2dec('3c6')) ':'], [char(hex2dec('3b8')) ':'], [char(hex2dec('3c8')) ':']}; % phi theta psi
        case'rpy'
            labels = {'R:', 'P:', 'Y:'};
    end
     
    % Exit button
    uicontrol(panel, 'Style', 'pushbutton', ...
        'Units', 'normalized', ...
        'Position', [0.16 0.04 0.7 0.06], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.5, ...
        'CallBack', @(src,event) quit_callback(robot, handles), ...
        'BackgroundColor', 'white', ...
        'ForegroundColor',[0.00,0.45,0.74], ...
        'String', 'Exit');    
    
    % Record button
    handles.record = [];
    if ~isempty(opt.callback)
    uicontrol(panel, 'Style', 'pushbutton', ...
        'Units', 'normalized', ...
        'Position', [0.1 height*(0)+.01 0.30 height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.6, ...
        'CallBack', @(src,event) record_callback(robot, handles), ...
        'BackgroundColor', 'red', ...
        'ForegroundColor', 'white', ...
        'String', 'REC');
    end
    
    % Assign the callbacks

    for j=1:n
        % text edit box
        set(handles.edit(j), ...
            'Interruptible', 'off', ...
            'Callback', @(src,event)teach_callback(src, robot.name, j, handles));
        
        % slider
        set(handles.slider(j), ...
            'Interruptible', 'off', ...
            'BusyAction', 'queue', ...
            'Callback', @(src,event)teach_callback(src, robot.name, j, handles));
    end
end
    
    % Called on changes to a slider or to the edit box showing joint coordinate
function teach_callback(src, name, j, handles)
    
    % Syntax:
    % src      the object that caused the event
    % name     name of the robot
    % j        the joint index concerned (1..N)
    % slider   true if the
    
    qscale = handles.qscale;
    
    switch get(src, 'Style')
        case 'slider'
            % slider changed, get value and reflect it to edit box
            newval = get(src, 'Value');
            set(handles.edit(j), 'String', num2str(qscale(j)*newval, 3));
        case 'edit'
            % edit box changed, get value and reflect it to slider
            newval = str2double(get(src, 'String')) / qscale(j);
            set(handles.slider(j), 'Value', newval);
    end
    %fprintf('newval %d %f\n', j, newval);
    

    
    % find all graphical objects tagged with the robot name, this is the
    % instancs of that robot across all figures
    
    h = findobj('Tag', name);
    
    
    % find the graphical element of this name
    if isempty(h)
        error('RTB:teach:badarg', 'No graphical robot of this name found');
    end
    % get the info from its Userdata
    info = get(h(1), 'UserData');
    
    % update the stored joint coordinates
    info.q(j) = newval;
    % and save it back to the graphical object
    set(h(1), 'UserData', info);
    
    % update all robots of this name
    animate(handles.robot.model, info.q);
    
    
    % compute the robot tool pose
    T6 = handles.robot.model.fkine(info.q);
    
    % convert orientation to desired format
%     switch handles.orientation
%         case 'approach'
%             orient = T6(:,3);    % approach vector
%         case 'eul'
%             orient = tr2eul(T6, 'setopt', handles.opt);
%         case'rpy'
%             orient = tr2rpy(T6, 'setopt', handles.opt);
%     end
    
    % update the display in the teach window
    for i=1:3
        set(handles.t6.t(i), 'String', sprintf('%.3f', T6(i,4)));
        %set(handles.t6.r(i), 'String', sprintf('%.3f', orient(i)));
    end
    
    if ~isempty(handles.callback)
        handles.callback(handles.robot, info.q);
    end
    
    %notify(handles.robot, 'Moved');

end

function record_callback(robot, handles)
    
    if ~isempty(handles.callback)
        handles.callback(h.q);
    end
end

function quit_callback(robot, handles)
    set(handles.fig, 'ResizeFcn', '');
    delete(handles.panel);
    set(handles.curax, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1])
end

function resize_callback(robot, handles)

    % come here on figure resize events
    fig = gcbo;   % this figure (whose callback is executing)
    fs = get(fig, 'Position');  % get size of figure
    ps = get(handles.panel, 'Position');  % get position of the panel
    % update dimensions of the axis area
    set(handles.curax, 'Units', 'pixels', ...
        'OuterPosition', [ps(3) 0 fs(3)-ps(3) fs(4)]);
    % keep the panel anchored to the top left corner
    set(handles.panel, 'Position', [1 fs(4)-ps(4) ps(3:4)]);
end
