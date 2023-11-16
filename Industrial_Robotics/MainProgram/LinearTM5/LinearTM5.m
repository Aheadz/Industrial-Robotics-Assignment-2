classdef LinearTM5 < RobotBaseClass

%% Attributes
    properties(Access = public)   
        plyFileNameStem = 'TM5';
        maxspeed = [pi pi pi deg2rad(225) deg2rad(225) deg2rad(225)];
    end

    properties(Access = private)
        homePose = SE3(0,0,0);
        elbowUpQ = [0,0,0,0,0,0];
        jointLimits = [0,0,0,0,0,0];
        %%jointStatePub = rospublisher('/tm5_joint_state', 'sensor_msgs/JointState');
    end
    methods(Access = public)
        %% Constructor
        function self = LinearTM5(baseTr,useTool,toolFilename)
            self.CreateModel();
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 1
                    %Passed in a base translation location
                    baseTr = baseTr * trotx(pi/2);
                elseif nargin == 0 % Nothing passed
                    %baseTr = transl(0.3,1,1.38)*trotx(pi/2)*troty(-pi/2);  
                    baseTr = transl(0.3,1,1.38)*trotx(pi/2)*troty(-pi/2)
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
            self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
        end
        %Model Creation
        function CreateModel(self)
            link(1) = Link([pi     0       0       pi/2    1]);
            link(2) = Link('d',0.1452,'a',0,'alpha',-pi/2,'qlim',deg2rad([-270, 270]), 'offset',0);
            link(3) = Link('d',0,'a',.329,'alpha',0,'qlim', deg2rad([-180, 180]), 'offset',0);
            link(4) = Link('d',0,'a', .3115,'alpha',0,'qlim', deg2rad([-180, 180]), 'offset', 0);
            link(5) = Link('d',0.106,'a',0,'alpha',pi/2,'qlim',deg2rad([-180, 180]),'offset', 0);
            link(6) = Link('d',0.106,'a',0,'alpha',pi/2,'qlim',deg2rad([-180, 180]), 'offset',0);
            link(7) = Link('d',0.1144,'a',0,'alpha',0,'qlim',deg2rad([-270, 270]), 'offset', 0);

            link(1).qlim = [-0.8 -0.01];
             
            self.model = SerialLink(link,'name',self.name);          
        end 
        
        function self = setHomePose(self,pose)
            self.homePose = pose;
        end

        function self = setElbowPos(self,Q)
            self.elbowUpQ = Q;
        end

        function self = setJointLimits(self,Q)
            self.jointLimits = Q;
        end
    end

    % There are many more support methods that could be added here such as those
    % repeatedly used as part of the movement sysm
    methods (Access = private)
        function self = publish_joint_state(self,Q)
            jointStateMsg = rosmessage(self.jointStatePub);
            jointStateMsg.Name = {'TM5'};
            jointStateMsg.Position = Q;
            jointStateMsg.Velocity = zeros(1,6);
            jointStateMsg.Effort = zeros(1,6);
            %%send(self.jointStatePub,jointStateMsg);
        end
    end
end