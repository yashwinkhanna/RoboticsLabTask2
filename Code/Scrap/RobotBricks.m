classdef RobotBricks < handle
    %ROBOTBRICKS A way of creating a stack of robot bricks
    %   The bricks can be moved around randomly. It is then possible to query
    %   the current location (base) of the bricks.
    
    properties (Constant)
        %> Max height is for plotting of the workspace
        maxHeight = 10;
    end
    
    properties
        %> Number of bricks as default
        brickCount = 2;
        
        %> A cell structure of \c brickCount brick models
        brick;
        
        %> paddockSize in meters
        paddockSize = [1, 0.5];        
        
        %> Dimensions of the workspace in regard to the padoc size
        workspaceDimensions;
    end
    
    methods
        %% ...structors
        function self = RobotBricks(brickCount)
            if 0 < nargin
                self.brickCount = brickCount;
            end
            
            self.workspaceDimensions = [-self.paddockSize(1)/2, self.paddockSize(1)/2 ...
                                       ,-self.paddockSize(2)/2, self.paddockSize(2)/2 ...
                                       ,0,self.maxHeight];

            % Create the required number of bricks
            for i = 1:self.brickCount
                self.brick{i} = self.GetBrickModel(['brick',num2str(i)]);
%                 % Random spawn
%                 self.brick{i}.base = se3(se2(1, 1, 0));
% %                 self.brick{i}.base = se3(se2((2 * rand()-1) * self.paddockSize(1)/2 ...
% %                          , (2 * rand()-1) * self.paddockSize(2)/2 ...
% %                          , 0));                     
%                                      
%                  % Plot 3D model
%                 plot3d(self.brick{i},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0);
%                 % Hold on after the first plot (if already on there's no difference)
%                 if i == 1 
%                     hold on;
%                 end
            end

%             axis equal
            camlight;
        end
        
        function delete(self)
%             cla;
        end       
        
        %% brick Spawn
        function spawnLocation(self, index, x, y , z)
            self.brick{index}.base = se3(se2(x, y, z));
            self.brick{index}.base(3, 4) = z;
            plot3d(self.brick{index},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0);
            
        end
        
        function spawnLocation2(self, index, loc)
            self.brick{index}.base = se3(se2(loc(1), loc(2), loc(3)));
            plot3d(self.brick{index},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0);   
        end

        %% PlotSingleRandomStep
        % Move each of the bricks forward and rotate some rotate value around
        % the z axis
        function PlotSingleRandomStep(self)
            for brickIndex = 1:self.brickCount
                % Move Forward
                self.brick{brickIndex}.base = self.brick{brickIndex}.base * se3(se2(0.2, 0, 0));
                animate(self.brick{brickIndex},0);
                % Turn randomly
                self.brick{brickIndex}.base(1:3,1:3) = self.brick{brickIndex}.base(1:3,1:3) *  rotz((rand-0.5) * 30 * pi/180);
                animate(self.brick{brickIndex},0);                

                % If outside workspace rotate back around
                if self.brick{brickIndex}.base(1,4) < self.workspaceDimensions(1) ...
                || self.workspaceDimensions(2) < self.brick{brickIndex}.base(1,4) ...
                || self.brick{brickIndex}.base(2,4) < self.workspaceDimensions(3) ...
                || self.workspaceDimensions(4) < self.brick{brickIndex}.base(2,4)
                    self.brick{brickIndex}.base = self.brick{brickIndex}.base * se3(se2(-0.2, 0, 0)) * se3(se2(0, 0, pi));
                end
            end
            % Do the drawing once for each interation for speed
            drawnow();
        end 
        
%         %% setBrickTarget
%         function setBrickTarget(x,y,z)
%             xloc = x;
%             yloc = y;
%             zloc = z;
%             
%         end
        
        
%         %% PlotSingleGivenStep
%         % Move each of the bricks forward and rotate some rotate value around
%         % the z axis
%         function PlotSingleGivenStep(self)
%             for brickIndex = 1:self.brickCount
%                 % Move Forward
%                 self.brick{brickIndex}.base = self.brick{brickIndex}.base * se3(se2(0, 0, 0));
%                 animate(self.brick{brickIndex},0);            
%             end
%             % Do the drawing once for each interation for speed
%             drawnow();
%         end   
%         
        %% TestPlotManyStep
        % Go through and plot many random walk steps
        function TestPlotManyStep(self,numSteps,delay)
            if nargin < 3
                delay = 0;
                if nargin < 2
                    numSteps = 200;
                end
            end
            for i = 1:numSteps
                self.PlotSingleRandomStep();
                pause(delay);
            end
        end
    end
    
    methods (Static)
        %% GetBrickModel
        function model = GetBrickModel(name)
            if nargin < 1
                name = 'Brick';
            end
            [faceData,vertexData] = plyread('pancake_150.ply','tri');
            L1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            model = SerialLink(L1,'name',name);
            model.faces = {faceData,[]};
            vertexData(:,2) = vertexData(:,2);
            model.points = {vertexData * rotz(-pi/2),[]};
            
            for linkIndex = 0:1 %self.model.n
                handles = findobj('Tag', model.name);
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
        
        
%         %% PlotAndColourRobot
%     % Given a robot index, add the glyphs (vertices and faces) and
%     % colour them in if data is available 
%     function PlotAndColourRobot(self)%robot,workspace)
%         for linkIndex = 0:self.model.n
%             if self.useGripper && linkIndex == self.model.n
%                 [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['LinUR3Link',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
%             else
%                 [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['LinUR3Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
%             end
%             self.model.faces{linkIndex+1} = faceData;
%             self.model.points{linkIndex+1} = vertexData;
%         end
% 
%         % Display robot
%         self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
%         if isempty(findobj(get(gca,'Children'),'Type','Light'))
%             camlight
%         end  
%         self.model.delay = 0;
% 
%         % Try to correctly colour the arm (if colours are in ply file data)
%         for linkIndex = 0:self.model.n
%             handles = findobj('Tag', self.model.name);
%             h = get(handles,'UserData');
%             try 
%                 h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
%                                                               , plyData{linkIndex+1}.vertex.green ...
%                                                               , plyData{linkIndex+1}.vertex.blue]/255;
%                 h.link(linkIndex+1).Children.FaceColor = 'interp';
%             catch ME_1
%                 disp(ME_1);
%                 continue;
%             end
%         end
%     end 
    end    
end

