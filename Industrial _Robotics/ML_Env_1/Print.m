classdef Print < RobotBaseClass
    %% Printe on a non-standard linear rail created by a student

    properties(Access = public)              
        plyFileNameStem = 'Print';
    end
    
    methods
%% Define robot Function 
        function self = Print(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);			
            end
            self.model.base = baseTr; 
            
            self.PlotAndColourRobot();         
        end

%% Create the robot model
        function CreateModel(self)   
            % Create the UR3 model mounted on a linear rail
            link(1) = Link([0    0.0885       0       0    1]); % PRISMATIC Link
            link(2) = Link([0    1.1       0       0    1]); % PRISMATIC Link
            link(3) = Link([0    1.1      0       0    1]); % PRISMATIC Link

            % Incorporate joint limits
            link(1).qlim = [-0.8 -0.01];
            link(2).qlim = [-0.8 -0.01];
            
            self.model = SerialLink(link,'name',self.name);

            % self.model.teach;

            axis equal
        end
     
    end
end