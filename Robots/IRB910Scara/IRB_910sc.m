classdef IRB_910sc < handle
    properties
        %> Robot model
        model;
        
        %> workspace
        workspace = [-2 2 -2 2 -0.67 2]; %[-0.6 0.6 -0.6 0.6 -0.67 1.1];   %[-0.6 0.6 -0.6 0.6 -0.2 1.1]
      
    end
    
    methods%% Class for IRB_910sc robot simulation
        function self = IRB_910sc(toolModelAndTCPFilenames)
            if 0 < nargin
                if length(toolModelAndTCPFilenames) ~= 2
                    error('Please pass a cell with two strings, toolModelFilename and toolCenterPointFilename');
                end
                self.toolModelFilename = toolModelAndTCPFilenames{1};
                self.toolParametersFilenamure = toolModelAndTCPFilenames{2};
            end
            
            self.GetIRB_910scRobot();
            self.PlotAndColourRobot();%robot,workspace);

            drawnow
        end

        %% GetIRB_910scRobot
        % Given a name (optional), create and return a IRB_910sc robot model
        function GetIRB_910scRobot(self)
            pause(0.001);
            name = ['IRB_910sc_',datestr(now,'yyyymmddTHHMMSSFFF')];
            L(1) = Link('d',0.1916,           'a',0.3,        'alpha',0,  'qlim',deg2rad([-140 140]), 'offset',0);
            L(2) = Link('d',0.2577-0.1916,    'a',0.25,       'alpha',0,  'qlim',deg2rad([-150 150]), 'offset',0);
            %Prismatic link: set sigma (5th coloumn) 0 if revolute, 1 if prismatic
            L(3) = Link([0     0       0       0    1]);
%            L3 = Link('d',0,         'a',0,          'alpha',0,  'qlim',deg2rad([-400 400]), 'offset', 0);
            
            L(3).qlim = [-0.180 0];

%             self.model = SerialLink([L1 L2 L3],'name',name);
            self.model = SerialLink(L,'name',name);

                %this moves the robot base position 
%                 self.model.base = self.model.base * transl(0.5,0.5,0);
                self.model.base = self.model.base * transl(-1.25,0,0) * trotz(-pi/2);
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['IRB_910sc_Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end        
    end
end
