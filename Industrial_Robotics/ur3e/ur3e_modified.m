classdef ur3e_modified < RobotBaseClass
    %% UR3e Universal Robot 3kg payload robot model

    properties(Access = public)   
        plyFileNameStem = 'ur3e';
        qz = [0,0,0,0,0,0];
    end

    properties(Access = private)
        homePose = SE3(0,0,0);
        elbowUpQ = [0,0,0,0,0,0];
        jointLimits = [0,0,0,0,0,0];
        maxspeed = [pi pi pi 2*pi 2*pi 2*pi]
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
%%Custom Functions start Here
        function self = setHomePose(self,pose)
            self.homePose = pose;
        end

        function self = setElbowPos(self,Q)
            self.elbowUpQ = Q;
        end
        
        function self = setJointLimits(self,Q)
            self.jointLimits = Q;
        end

        function object_poses =  start_scan(self)
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

        %function self = sort_objects(poses, bin_allocation, bin_locations)
        function completed = sort_objects(self)
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

        %Generalized RMRC that will move between pose changing rotation and translation.
         function trajectory = RMRC_general(self, startPose, targetPose, deltaT, time)
            %1.Parameter Setup
            steps = time/deltaT;
            threshold = 0.01;
            epsilon = 0.1;                          % Threshold value for manipulability/Damped Least Squares. 
            W = diag([1 1 1 0.1 0.1 0.1]);          % Weighting matrix for the velocity vector.
            N = 10; %Points to generate for interpolation.
            %2. Allocate Array Data.
            trajectory = zeros(steps,length(q0));
            qMatrix = zeros(steps,length(q0));       % Array for joint angles.
            m = zeros(steps,1);                      % Array for Measure of Manipulability.
            qdot = zeros(steps,6);                   % Array for joint velocities.
            %3. Starting Joint Angles.
            qMatrix(1,:) = self.model.ikcon(startPose.T,q0);
            for i = 1:steps-1
                %1.Compute the current end-effector pose.
                x1 = self.model.fkine(qMatrix(i,:)).T;                
                x2  = targetPose.T;
                %2.Calculate Translation Delta only Between Starting Pose and Desired, Starting Pose will be the current index, desired is index + 1.
                deltaX = x2(1:3,4)' - x1(1:3,4)';
                %3.Calculate the Rotation Delta.
                r1 = x1(1:3,1:3);
                r2 = x2(1:3,1:3);
                Rdot= (1/deltaT)*(r2-r1);
                deltaTheta = tr2rpy(r2*r1');  
                %4.Calculate Skew Symmetric Matrix (Used to find angular vel)
                S = Rdot*r1';
                %5.Calculate Angular Velocity (Using Skew Symmetric)
                angular_velocity = [S(3,2);S(1,3);S(2,1)];
                %6.Calculate Linear Velocity
                linear_velocity = (1/deltaT)*deltaX;
                linear_velocity = linear_velocity';
                %7.Calculate end-effector velocity to reach next waypoint.
                xdot = W*[linear_velocity;angular_velocity];
                %8.Get Jacobian at current joint state
                J = self.model.jacob0(qMatrix(i,:));
                %9.Calculate Measure of Manipulabillity.
                m(i) = sqrt(det(J*J'));
                %10.Check if manipulabillity is below threshold.
                if m(i) < epsilon
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
                %11.Calculate the DLS inverse jacobian
                invJ = inv(J'*J + lambda *eye(6))*J';
                %12.Solve RMRC equation and find joint velocities.
                qdot(i,:) = (invJ*xdot)'; 
                % Check termination condition
                if norm(linear_velocity) < threshold
                    qMatrix = qMatrix(1:i,:); % Trim unused columns
                    break;
                end
                %13.Check to ensure joint change doesnt exceed joint limits.
                %Might want to break the code here.
                for j = 1:6                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.model.qlim(j,1)           % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; %Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.model.qlim(j,2)       % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; %Stop the motor
                    end
                end
                % Cap the joint velocities
                for j = 1:6
                    if qdot(j) > self.maxspeed(j)
                        qdot(j)
                        qdot(j) = self.maxspeed(j);
                    elseif qdot(j) < -self.maxspeed(j)
                        qdot(j)
                        qdot(j) = -self.maxspeed(j);
                    end
                end  
                %14.Add new joint state to trajectory.
                %new joint state = (previous joint state) + ((joint velocity)*(change in time)) 
                qMatrix(i+1,:) = qMatrix(i,:) + qdot(i,:)*deltaT;                       % Update next joint state based on joint velocities
            end

        end 

        %Linear RMRC that does change end-effector orientation
        function qMatrix = RMRC_noRot(self, q0, targetPose, deltaT, time)
            %1.Parameter Setup
            steps = time/deltaT;
            threshold = 0.01;
            epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares  
            W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
            N = 10; %Points to generate for interpolation

            %2. Allocate Array Data
            trajectory = zeros(steps,length(q0));
            qMatrix = zeros(steps,length(q0));       % Array for joint anglesR
            m = zeros(steps,1);                      % Array for Measure of Manipulability
            qdot = zeros(steps,6);                   % Array for joint velocities

            
            %2. Interpolation between two points
            x1 = self.model.fkine(q0).T;
            x2 = targetPose.T;
            % x1 and x2 are vectors containing the coordinates of the two points
            % N is the number of points to generate on the line segment including P1 and P2
            % Initialize the vector to hold the points
            points = zeros(N, 3); % Each row will be a point (x, y, z)
            % Calculate the step size for the parameter t
            t_step = 1 / (N - 1);
            % Generate the points
            for i = 1:N
                t = (i - 1) * t_step;
                points(i, :) = x1(1:3,4)' + t * (x2(1:3,4)' - x1(1:3,4)');
            end

            %Starting joint angle comes from getpos()
            qMatrix(1,:) = q0;

            for i = 1:steps-1
                %1.Compute the current end-effector pose
                x1 = self.model.fkine(qMatrix(i,:)).T;
                %x2 = points(i+1,:);
                x2  = targetPose.T;
                %2.Calculate Translation Delta only Between Starting Pose and Desired, Starting Pose will be the current index, desired is index + 1.
                deltaX = x2(1:3,4)' - x1(1:3,4)';

                %3.Calculate the Rotation Delta.
                r1 = x1(1:3,1:3);
                r2 = x2(1:3,1:3);
                Rdot= (1/deltaT)*(r2-r1);
                deltaTheta = tr2rpy(r2*r1');  
                %4.Calculate Skew Symmetric Matrix (Used to find angular vel)
                S = Rdot*r1';
                %5.Calculate Angular Velocity (Using Skew Symmetric)
                angular_velocity = [S(3,2);S(1,3);S(2,1)];

                %6.Calculate Linear Velocity
                linear_velocity = (1/deltaT)*deltaX;
                linear_velocity = linear_velocity';
                %7.Calculate end-effector velocity to reach next waypoint.
                xdot = W*[linear_velocity;angular_velocity];
                %8.Get Jacobian at current joint state
                J = self.model.jacob0(qMatrix(i,:));

                %9.Calculate Measure of Manipulabillity.
                m(i) = sqrt(det(J*J'));

                %10.Check if manipulabillity is below threshold.
                if m(i) < epsilon
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end

                %11.Calculate the DLS inverse jacobian
                invJ = inv(J'*J + lambda *eye(6))*J';

                %12.Solve RMRC equation and find joint velocities.
                qdot(i,:) = (invJ*xdot)'; 

                % Check termination condition
                if norm(linear_velocity) < threshold
                    qMatrix = qMatrix(1:i,:); % Trim unused columns
                    break;
                end

                %12.Check to ensure joint change doesnt exceed joint limits.
                %Might want to break the code here.
                for j = 1:6                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.model.qlim(j,1)           % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; %Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.model.qlim(j,2)       % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; %Stop the motor
                    end
                end
                % Cap the joint velocities
                for j = 1:6
                    if qdot(j) > self.maxspeed(j)
                        qdot(j)
                        qdot(j) = self.maxspeed(j);
                    elseif qdot(j) < -self.maxspeed(j)
                        qdot(j)
                        qdot(j) = -self.maxspeed(j);
                    end
                end  
                %13.Add new joint state to trajectory
                %new joint state = (previous joint state) + ((joint velocity)*(change in time)) 
                qMatrix(i+1,:) = qMatrix(i,:) + qdot(i,:)*deltaT;                       % Update next joint state based on joint velocities
            end
        end

        function qMatrix = trap_ikine(self,q1,T1,T2,mask,steps)
            %1. Solve Inverse Kinematics
            q2 = self.model.ikine(T2,q1,mask);
            %2. Generate trajectory
            s = lspb(0,1,steps);                        % First, create the scalar function
            qMatrix = nan(steps,6);                     % Create memory allocation for variables
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;   % Generate interpolated joint angles
            end
        end

        function qMatrix = trap_ikcon(self,q1,T1,T2,steps)
            %1. Solve Inverse Kinematics
            q2 = self.model.ikcon(T2,q1);

            s = lspb(0,1,steps);                                             	% First, create the scalar function
            qMatrix = nan(steps,6);                                             % Create memory allocation for variables
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;                   	% Generate interpolated joint angles
            end

        end

        function qMatrix = quintic_ikine(self,q1,T1,T2,mask,steps)
            %1. Solve Inverse Kinematics
            self.model.ikine(T2,q1,mask);
            %2. Generate
            qMatrix = jtraj(q1,q2,steps);
        end

        function qMatrix = quintic_ikcon(self,q1,T1,T2)
            self.model.ikine(T2,q1,mask);
        end
        
        function testMove(self,targetPose,type)
            switch type
                case 1
                    %RMRC General
                    
                case 2
                    %RMRC No Rotation
                    qMatrix = self.RMRC_noRot(self.model.getpos(),targetPose,0.25,5);
                case 3
                    %Trapezoidal ikine
                    qMatrix = self.trap_ikine(self.model.getpos(),0,targetPose,[1 1 1 1 1 1],50);
                case 4
                    %Trapezoidal ikcon
                    qMatrix = self.trap_ikcon(self.model.getpos(),0,targetPose,50);
                case 5
                    %Quintic ikine
                    qMatrix = self.quintic_ikine(self.model.getpos(),0,targetPose,[1 1 1 1 1 1],50);
                case 6
                    %Quintic ikcon
                    qMatrix = self.quintic_ikcon(self.model.getpos(),0,targetPose);
            end
            for i = 1:length(qMatrix)
                self.model.animate(qMatrix(i,:))
                drawnow();
                pause(0.1);
            end
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