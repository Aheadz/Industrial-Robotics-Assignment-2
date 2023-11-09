classdef LinearTM5 < RobotBaseClass

%% Attributes
    properties(Access = public)   
        plyFileNameStem = 'TM5';
    end

    properties(Access = private)
        homePose = SE3(0,0,0);
        elbowUpQ = [0,0,0,0,0,0];
        jointLimits = [0,0,0,0,0,0];
        jointStatePub = rospublisher('/tm5_joint_state', 'sensor_msgs/JointState');
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
                    baseTr = transl(0,0,0) * trotx(pi/2);  
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
            link(6) = Link('d',.106,'a',0,'alpha',pi/2,'qlim',deg2rad([-180, 180]), 'offset',0);
            link(7) = Link('d',.1144,'a',0,'alpha',0,'qlim',deg2rad([-270, 270]), 'offset', 0);

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

        function self = goToPrinter(self,pose)
            % 1. Move To a Position Where the Tool Can Slot into the bed or attach to it.
            % 1.1. It is necessary to move in a highly constrained way such that there is no collision with
            %      the environment.
            % 2. The pose must line the tool up with the mounting holes in the print bed
            % self.plot() as you change the joint state.
            % self.publish_joint_state(q) publish joint states to ROS as you move aswell
        end

        function self = attachtool(self)
            %1. Routine needs move in linearly to slot into the bed brace.
            %2. Should move in nice and slow.
            % self.plot() as you change the joint state.
            % self.publish_joint_state(q) publish joint states to ROS as you move aswell
        end

        function self = removeBed(self)
            %1. Robot must keep the build plate level whilst also avoiding 
            % collisions with the printer frame.
            %2. Should move directly upwards then pull back and then rotate.
            %3. This step should also be done slowly to minimize reaction forces.
            % self.plot() as you change the joint state.
            % self.publish_joint_state(q) publish joint states to ROS as you move aswell
        end

        function self = moveBed(self,pose)
            %1. Robot moves to bed.
            %2. No Movement in the Z-axis is allowed until mountBed() is called.
            % self.plot() as you change the joint state.
            % self.publish_joint_state(q) publish joint states to ROS as you 
            % move aswell
        end

        function self = mountBed(self,pose)
            %1. Robot lowers the bed onto the magnetic mount of the UR3e bench
            % self.plot() as you change the joint state.
            % self.publish_joint_state(q) publish joint states to ROS as you move
            %  aswell
        end

        function self = placeBed(self)
            %This Function places the bed back onto the printer
        end

        function self = detach(self)
            %1. Robot dettaches the end-effector from the bed (basically inverse 
            % of attach, could make one function?)
        end

        function self = home(self)
            %1. Move Back to Home Pose
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
            send(self.jointStatePub,jointStateMsg);
        end
    end
end