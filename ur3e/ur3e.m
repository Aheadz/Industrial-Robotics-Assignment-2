classdef ur3e < RobotBaseClass
    %% UR3e Universal Robot 3kg payload robot model

    properties(Access = public)   
        plyFileNameStem = 'ur3e';
    end

    properties(Access = private)
        homePose = SE3(0,0,0);
        elbowUpQ = [0,0,0,0,0,0];
        jointLimits = [0,0,0,0,0,0];
    end

    methods
%% Constructor
        function self = UR3e(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);  
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
			warning('The DH parameters are correct. But as of July 2023 the ply files for this UR3e model are definitely incorrect, since we are using the UR3 ply files renamed as UR3e. Once replaced remove this warning.')  
            self.PlotAndColourRobot();

            drawnow
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
        
        function self = setHomePose(self,pose)
            self.homePose = pose;
        end

        function self = setElbowPos(self,Q)
            self.elbowUpQ = Q;
        end

        function self = setJointLimits(self,Q)
            self.jointLimits = Q;
        end

        function object_poses =  start_scan(self,heights)
            %1. There should be a ROS Service call or publish to a ROS 
            % message thatActivates the python script for executing the scan.
            % it can be done completley in python or the movement can be 
            % in MATLAB and the scans can
            % be done in python and so would the image processing.
            % no matter the, result must be an SE3 Matrix that gives the 
            % poses of the objects
            % at this points the objects will be rectangular blocks of 
            % different heights
            object_poses = zeros(4,4,4);
        end

        function self = sort_objects(poses, bin_allocation, bin_locations)
            %we take the object_poses and we get which bin each object will go into.
            %doesnt matter the order as long as they sorted separately.
            %we also get another array of SE3 that include the bin locations.
            %this function can be implemented in python if necessary
        end

        function isBedClear = check_bed(self)
            isBedClear = true;
        end

        function isBedClear = clear_bed(self)
            %We remove anything on the bed and call the check_bed again.
            isBedClear = self.check_bed();
        end
    end

    methods (Access = private)
        function publish_joint_state(self,Q)
            
        end
    end
end