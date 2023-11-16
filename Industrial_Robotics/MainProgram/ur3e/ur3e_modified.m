classdef ur3e_modified < RobotBaseClass
    %% UR3e Universal Robot 3kg payload robot model

    properties(Access = public)   
        plyFileNameStem = 'ur3e';
        qz = [0,0,0,0,0,0];
        maxspeed = [pi pi pi 2*pi 2*pi 2*pi];
    end

    properties(Access = private)
        homePose = SE3(0,0,0);
        elbowUpQ = [0,0,0,0,0,0];
        jointLimits = [0,0,0,0,0,0];
       % jointStatePub = rospublisher('/tm5_joint_state', 'sensor_msgs/JointState');
    end
    methods
%% Constructor
        function self = ur3e_modified(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 1
                    %Passed in a base translation location
                    baseTr = baseTr;
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(-0.75,1,1.343);  
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
			self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
			%warning('The DH parameters are correct. But as of July 2023 the ply files for this UR3e model are definitely incorrect, since we are using the UR3 ply files renamed as UR3e. Once replaced remove this warning.')  
            self.PlotAndColourRobot();
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.15185,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0,'a',-0.24355,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0,'a',-0.2132,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            link(4) = Link('d',0.13105,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(5) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(6) = Link('d',	0.0921,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
             
            self.model = SerialLink(link,'name',self.name);
        end
    end

    methods (Access = private)
        function publish_joint_state(self,Q)
            jointStateMsg = rosmessage(self.jointStatePub);
            jointStateMsg.Name = {'UR3e'};
            jointStateMsg.Position = Q; % Example values
            jointStateMsg.Velocity = zeros(1,6); % Example values
            jointStateMsg.Effort = zeros(1,6); % Example values
            send(self.jointStatePub,jointStateMsg);
        end
    end
end