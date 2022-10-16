
%Repurposed lab 2, Robotcow class

classdef SpatulaClass < handle
    %SpatulaClass A way of creating a group of spatulas
    %   The spatulas can be moved around randomly. It is then possible to query
    %   the current location (base) of the spatulas.
    
    properties (Constant)
        %> Max height is for plotting of the workspace
        maxHeight = 10;
        
    end
    
    properties
        %> Default number of spatulas to spawn
        spatulaCount = 2;
        
        %> A cell structure of \c spatulaCount spatula models
        spatula;
        
        %> paddockSize in meters
        paddockSize = [0.5,0.5];        
        
        %> Dimensions of the workspace in regard to the padoc size
        workspaceDimensions;
    end
    
    methods
        %% ...structors

            %note - this creates the amount of spatulas we want to spawn inside (spatulaCount), the default is 
                    % 2 as we set it above, which it will result to if we dont give a number
        function self = SpatulaClass(spatulaCount)

            %note - nargin is our input argument (in this case for the number of spatulas we want to spawn) 
                    % where if our nargin is not greater than 0, it will assume we have not chosen a valid option and set 
                    % it to our default value
            if 0 < nargin
                self.spatulaCount = spatulaCount;
            end
            
            self.workspaceDimensions = [-self.paddockSize(1)/2, self.paddockSize(1)/2 ...
                                       ,-self.paddockSize(2)/2, self.paddockSize(2)/2 ...
                                       ,0,self.maxHeight];

            % Create the required number of spatulas on Matlab workspace
            for i = 1:self.spatulaCount
                self.spatula{i} = self.GetspatulaModel(['spatula',num2str(i)]);

                % Hold on after the first plot (if already on there's no difference)
                if i == 1 
                    hold on;
                end
            end

            axis equal
            camlight;
        end

                 % Spawn a spatula at x, y,theta 
function SpatulaSpawn(self, i, enter_x,enter_y,enter_theta)
                self.spatula{i}.base = se3(se2(enter_x, enter_y, enter_theta));
                
                 % Plot 3D model of this spatula
                plot3d(self.spatula{i},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0);
end  
        
        function delete(self)
%             cla;
        end       
        
        %% PlotSingleRandomStep

        % Move each of the spatulas forward and rotate some rotate value around
        % the z axis
        function PlotSingleRandomStep(self)
            for spatulaIndex = 1:self.spatulaCount
                % Move Forward
                self.spatula{spatulaIndex}.base = self.spatula{spatulaIndex}.base * se3(se2(0.2, 0, 0));
                animate(self.spatula{spatulaIndex},0);
                % Turn randomly
                self.spatula{spatulaIndex}.base(1:3,1:3) = self.spatula{spatulaIndex}.base(1:3,1:3) *  rotz((rand-0.5) * 30 * pi/180);
                animate(self.spatula{spatulaIndex},0);                

                % If outside workspace rotate back around
                if self.spatula{spatulaIndex}.base(1,4) < self.workspaceDimensions(1) ...
                || self.workspaceDimensions(2) < self.spatula{spatulaIndex}.base(1,4) ...
                || self.spatula{spatulaIndex}.base(2,4) < self.workspaceDimensions(3) ...
                || self.workspaceDimensions(4) < self.spatula{spatulaIndex}.base(2,4)
                    self.spatula{spatulaIndex}.base = self.spatula{spatulaIndex}.base * se3(se2(-0.2, 0, 0)) * se3(se2(0, 0, pi));
                end
            end
            % Do the drawing once for each interation for speed
            drawnow();
        end    
        
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
        %% GetspatulaModel
            %note - this chunk of code allows Matlab to interpret the ply model we give it and 
                    % prepare it using a vertices appearence
        function model = GetspatulaModel(name)
            if nargin < 1
                name = 'spatula';
            end
            [faceData,vertexData] = plyread('spatula.ply','tri');
            L1 = Link('alpha',0,'a',0,'d',0.3,'offset',0);
            model = SerialLink(L1,'name',name);
            model.faces = {faceData,[]};
            vertexData(:,2) = vertexData(:,2);
                %Had to multiply with rotz to make spatula orientated to make
                %wall
            model.points = {vertexData * rotz(-pi/2),[]};
        end
    end  
end

  
