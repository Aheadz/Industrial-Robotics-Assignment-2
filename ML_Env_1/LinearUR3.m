classdef LinearUR3 < RobotBaseClass
    %% LinearUR3 UR3 on a non-standard linear rail created by a student

    properties(Access = public)              
        plyFileNameStem = 'LinearUR3';
    end
    
    methods
%% Define robot Function 
        function self = LinearUR3(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);			
            end
            self.model.base = baseTr * trotx(pi/2) * troty(-pi/2); 
            
            self.PlotAndColourRobot();         
        end

%% Create the robot model
        function CreateModel(self)   
            % Create the UR3 model mounted on a linear rail
            link(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link
            link(2) = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',-pi/2);
            link(4) = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            link(5) = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(6) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(7) = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);


            % ur3d = [0.1519, 0, 0, 0.11235, 0.08535, 0.0819];
            % ur3a = [0, -0.24365, -0.21325, 0, 0, 0];
            % ur3alpha = [(90*pi)/180, (0*pi)/180, (0*pi)/180, (90*pi)/180, (-90*pi)/180, (0*pi)/180];
            % 
            % for i = 1:6
            %     link(i+1) = Link('d', ur3d(i), 'a', ur3a(i), 'alpha', ur3alpha(i), 'offset', 0);
            % end      
            
            % Incorporate joint limits
            link(1).qlim = [-0.8 -0.01];
            
            self.model = SerialLink(link,'name',self.name);
            

        end
     
    end
end