classdef humanMale < handle    
    %#ok<*TRYNC>    

    properties (Constant)
        %> Max height is for plotting of the workspace
        maxHeight = 10;
    end
    
    properties
        %> Number of humans
        humanCount = 1;
        
        %> A cell structure of \c cowCount cow models
        humanModel;
        % workspaceDimensions;
    end
    
    methods
        %% ...structors
        function self = humanMale
            if 0 < nargin
                self.humanCount = 1;
            end
            
            %self.workspaceDimensions = workspace;
            lighting none;

            % Create the required number of cows
            %for i = 1:self.humanCount
                self.humanModel{1} = self.GethumanModel(['cow',num2str(1)]);
                % Random spawn
                %basePose = SE3(SE2(-0.1,0.5,0.45));
                self.humanModel{1}.base = transl(0.5, -0.3, 0.465);
                
                 % Plot 3D model
                plot3d(self.humanModel{1},0,'workspace',[-1,1,-1,1,-1,1],'view',[-30,30],'delay',0,'noarrow','nowrist','notiles');
                % Hold on after the first plot (if already on there's no difference)
                %if i == 1 
                    hold on;
                %end
            %end

            %axis equal
            %if isempty(findobj(get(gca,'Children'),'Type','Light'))
                %camlight
            %end 
        end
        
        function delete(self)
            for index = 1:self.humanCount
                handles = findobj('Tag', self.humanModel{index}.name);
                h = get(handles,'UserData');
                try delete(h.robot); end
                try delete(h.wrist); end
                try delete(h.link); end
                try delete(h); end
                try delete(handles); end
            end
        end       
        
        %% PlotSingleRandomStep
        % Move each of the cows forward and rotate some rotate value around
        % the z axis
        function UpdateModel(self)
            for index = 1:self.humanCount
                % % Move Forward
                % self.humanModel{index}.base = self.humanModel{index}.base * SE3(SE2(0, 0, 0));
                % animate(self.humanModel{index},0);
                % 
                % % Turn randomly
                % % Save base as a temp variable
                % tempBase = self.humanModel{index}.base.T;
                % rotBase = tempBase(1:3, 1:3);
                % posBase = tempBase(1:3, 4);
                % newRotBase = rotBase * rotz((rand-0.5) * 30 * pi/180);
                % newBase = [newRotBase posBase ; zeros(1,3) 1];
                % 
                % % Update base pose
                % self.humanModel{index}.base = newBase;
                animate(self.humanModel{index},0);                

                % If outside workspace rotate back around
                % Get base as temp
                %tempBase = self.humanModel{index}.base.T;
                
                % if tempBase(1,4) < self.workspaceDimensions(1) ...
                % || self.workspaceDimensions(2) < tempBase(1,4) ...
                % || tempBase(2,4) < self.workspaceDimensions(3) ...
                % || self.workspaceDimensions(4) < tempBase(2,4)
                %     self.humanModel{index}.base = self.humanModel{index}.base * SE3(SE2(-0.2, 0, 0)) * SE3(SE2(0, 0, pi));
                % end
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
                self.UpdateModel();
                pause(delay);
            end
        end
    end
    
    methods (Static)
        %% GethumanModel
        function model = GethumanModel(name)
            if nargin < 1
                name = 'human';
            end
            [faceData,vertexData] = plyread('personMaleConstruction.ply','tri');
            link1 = Link('alpha',0,'a',0,'d',0.1,'offset',0);
            model = SerialLink(link1,'name',name);
            
            % Changing order of cell array from {faceData, []} to 
            % {[], faceData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.faces = {[], faceData};

            % Changing order of cell array from {vertexData, []} to 
            % {[], vertexData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.points = {[], vertexData};
        end
    end    
end