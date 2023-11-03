classdef RobotBaseClass_modified < handle
    %% RobotBaseClass_modified The base robot class from which other UTS robot models should inherit 

    %#ok<*TRYNC>    

    properties (Abstract) % An inheriing class must implement and assign these properties 
        %> The first few letters or the name of ply files in the same directory
        plyFileNameStem;
    end

    properties
        %> Robot model
        model;
        
        % Name of the robot (to be copied into model.name)
        name;

        %> workspace (this changes based on the DH parameters)  
        workspace = [-10 10 -10 10 -0.01 10];

        %> The home location in radians 
        homeQ = [];

        %> Flag to indicate if tool (such as a gripper) is used
        useTool = false; 

        %> The fule filename and extension of the tool ply filename
        toolFilename = [];

        %> Tool transform (only relevant if there is a tool)
        toolTr = transl(0,0,0);
    end

    properties (Hidden)
        axis_h = [];
        figure_h = [];
        lightAdded = false;
        surfaceAdded = false;
    end

    properties (Access = private)
        delaySecondsForInnerAnimation = 0.2;
        stepsForInnerAnimation = 20;        
    end   
    
    methods (Abstract) % An inheriting class must implement these methods
        % Inherriting class must implement a method which sets
        % self.model to be a SerialLink object
        CreateModel(self);
    end
        
    methods
    
%% General class for multiDOF robot simulation
        function self = RobotBaseClass_modified()
            % This is intentionally left empty. Implement the class
            % constructor in the inhereting class.
            pause(0.001);
            try 
                self.name = [self.plyFileNameStem,datestr(now,'yyyymmddTHHMMSSFFF')];
            catch
                self.name = ['RobotBaseClass_modified',datestr(now,'yyyymmddTHHMMSSFFF')];
                warning(['Please include a variable called plyFileNameStem in your inherreting class. For now the robot is named: ',self.name])                
            end
            
        end

%% delete
        % Try to clean up gracefully by deleting just this robot plot
        function delete(self)
            handles = findobj('Tag', self.model.name);
            h = get(handles,'UserData');
            try delete(h.robot); end 
            try delete(h.wrist); end
            try delete(h.link); end
            try delete(h); end
            try delete(handles);end

            % Cleaning up the axis by removing the tiled floor and lights
            % if the figure and axis still exist
            if ishandle(self.figure_h) && ishandle(self.axis_h)
                % If there are any surface (ground floor) left then delete
                % one if one was initially added
                floor_h = findobj(gca, 'Type', 'surface', 'Tag', 'tiled_floor');
                if self.surfaceAdded
                    try delete(floor_h);end
                end

                light_h = findobj(gca, 'Type', 'light');
                axisChildren_h = get(self.axis_h,'Children');
                % If self added a light and there is now only a single light left then clear (or close) the figure
                if self.lightAdded && size(axisChildren_h,1) == 1 && ~isempty(light_h)
                    % clf % CLEAR
                    delete(self.figure_h); % CLOSE
                end                                             
            end
        end

%% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and colour them in if data is available 
        function PlotAndColourRobot(self)
            if isempty(self.homeQ)
                self.homeQ = zeros(1,self.model.n);
            end

            if exist([self.plyFileNameStem,'.mat'],'file') == 2 && exist([self.plyFileNameStem,'Link0.ply'],'file') ~= 2
                warning('There are no ply files for the links but there is a mat file. You should use PlotAndColourRobotMat to create and colour a 3D robot model plot. I am doing this for you now.')
                self.PlotAndColourRobotMat()
                return;
            end

            for linkIndex = 0:self.model.n
                if self.useTool && linkIndex == self.model.n
                    if ~isempty(self.toolFilename)
                        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.toolFilename],'tri'); %#ok<AGROW>
                    else
                        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.plyFileNameStem,'Link',num2str(linkIndex),'Tool.ply'],'tri'); %#ok<AGROW>
                    end
                else
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.plyFileNameStem,'Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                end
                
                % Obtain faceData and vertexData for the current link and save to the cell.
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end
        
            h = self.InitiliseRobotPlot();
            if 1 < length(h)
                self.MultirobotWarningMessage();
                h = h{1};
            end

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                vertexColours = [0.5,0.5,0.5]; % Default if no colours in plyData
                try 
                     vertexColours = [plyData{linkIndex+1}.vertex.red ...
                                     , plyData{linkIndex+1}.vertex.green ...
                                     , plyData{linkIndex+1}.vertex.blue]/255;
                    
                catch ME_1
                    disp(ME_1);
                    disp('No vertex colours in plyData');
                    try 
                         vertexColours = [plyData{linkIndex+1}.face.red ...
                                     , plyData{linkIndex+1}.face.green ...
                                     , plyData{linkIndex+1}.face.blue]/255;
                    catch ME_1
                        disp(ME_1);
                        disp(['Also, no face colours in plyData, so using a default colour: ',num2str(vertexColours)]);
                    end
                end
                
                h.link(linkIndex+1).Children.FaceVertexCData = vertexColours;
                h.link(linkIndex+1).Children.FaceColor = 'interp';
            end
            drawnow();
        end

%% PlotAndColourRobotMat 
% An alternative plot and colour method which uses the .mat file structure
% containing data for the vertex, face and colour data in it 
        function PlotAndColourRobotMat(self)
            % Choose one models with different (or no) end effector
            shapeData = load([self.plyFileNameStem,'.mat']);

            for linkIndex = 0:self.model.n               
                % Obtain faceData for the current link and save to the cell.
                self.model.faces{linkIndex+1} = shapeData.shapeModel(linkIndex+1).face;
                self.model.points{linkIndex+1} = shapeData.shapeModel(linkIndex+1).vertex;
            end
        
            h = self.InitiliseRobotPlot();                   

            if 1 < length(h)
                self.MultirobotWarningMessage();
                h = h{1};
            end

            % Colour the links
            for linkIndex = 0:self.model.n 
                vertexColours = repmat(shapeData.shapeModel(linkIndex+1).colour, size(self.model.points{linkIndex+1}, 1), 1);
                h.link(linkIndex+1).Children.FaceVertexCData = vertexColours;
                h.link(linkIndex+1).Children.FaceColor = 'interp';
            end          
        end
%%Movement Functions
        %Generalized RMRC that will move between pose changing rotation and translation.
        function trajectory = RMRC_general(self,r,startPose, targetPose, deltaT, time)
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
            qdot = zeros(steps,link_length);                   % Array for joint velocities.
            %3. Starting Joint Angles.
            qMatrix(1,:) = r.model.ikcon(startPose.T,q0);
            for i = 1:steps-1
                %1.Compute the current end-effector pose.
                x1 = r.model.fkine(qMatrix(i,:)).T;                
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
                J = r.model.jacob0(qMatrix(i,:));
                %9.Calculate Measure of Manipulabillity.
                m(i) = sqrt(det(J*J'));
                %10.Check if manipulabillity is below threshold.
                if m(i) < epsilon
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
                %11.Calculate the DLS inverse jacobian
                invJ = inv(J'*J + lambda *eye(link_length))*J';
                %12.Solve RMRC equation and find joint velocities.
                qdot(i,:) = (invJ*xdot)'; 
                % Check termination condition
                if norm(linear_velocity) < threshold
                    qMatrix = qMatrix(1:i,:); % Trim unused columns
                    break;
                end
                %13.Check to ensure joint change doesnt exceed joint limits.
                %Might want to break the code here.
                for j = 1:link_length                                                            % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < r.model.qlim(j,1)           % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; %Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > r.model.qlim(j,2)       % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; %Stop the motor
                    end
                end
                % Cap the joint velocities
                for j = 1:link_length
                    if qdot(j) > r.maxspeed(j)
                        qdot(j)
                        qdot(j) = r.maxspeed(j);
                    elseif qdot(j) < -r.maxspeed(j)
                        qdot(j)
                        qdot(j) = -r.maxspeed(j);
                    end
                end  
                %14.Add new joint state to trajectory.
                %new joint state = (previous joint state) + ((joint velocity)*(change in time)) 
                qMatrix(i+1,:) = qMatrix(i,:) + qdot(i,:)*deltaT;                       % Update next joint state based on joint velocities
            end

        end 
        %Linear RMRC that does change end-effector orientation
        function qMatrix = RMRC_noRot(self,r,q0, targetPose, deltaT, time)
            %1.Parameter Setup
            link_length = length(r.model.links());
            steps = time/deltaT;
            threshold = 0.01;
            epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares  
            W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
            N = 10; %Points to generate for interpolation

            %2. Allocate Array Data
            trajectory = zeros(steps,length(q0));
            qMatrix = zeros(steps,length(q0));       % Array for joint anglesR
            m = zeros(steps,1);                      % Array for Measure of Manipulability
            qdot = zeros(steps,link_length);                   % Array for joint velocities

            
            %2. Interpolation between two points
            x1 = r.model.fkine(q0).T;
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
                x1 = r.model.fkine(qMatrix(i,:)).T;
                %x2 = points(i+1,:);
                x2  = targetPose.T;
                %2.Calculate Translation Delta only Between Starting Pose and Desired, Starting Pose will be the current index, desired is index + 1.
                deltaX = x2(1:3,4)' - x1(1:3,4)';

                %3.Calculate the Rotation Delta.
                r1 = x1(1:3,1:3);
                r2 = x2(1:3,1:3);
                Rdot= (1/deltaT)*(r2-r1);
                %deltaTheta = tr2rpy(r2*r1');  
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
                J = r.model.jacob0(qMatrix(i,:));

                %9.Calculate Measure of Manipulabillity.
                m(i) = sqrt(det(J*J'));

                %10.Check if manipulabillity is below threshold.
                if m(i) < epsilon
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end

                %11.Calculate the DLS inverse jacobian
                invJ = inv(J'*J + lambda *eye(link_length))*J';

                %12.Solve RMRC equation and find joint velocities.
                qdot(i,:) = (invJ*xdot)'; 

                % Check termination condition
                if norm(linear_velocity) < threshold
                    qMatrix = qMatrix(1:i,:); % Trim unused columns
                    break;
                end

                %12.Check to ensure joint change doesnt exceed joint limits.
                %Might want to break the code here.
                for j = 1:link_length                                                             % Loop through joints 1 to link_length
                    if qMatrix(i,j) + deltaT*qdot(i,j) < r.model.qlim(j,1)           % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; %Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > r.model.qlim(j,2)       % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; %Stop the motor
                    end
                end
                % Cap the joint velocities
                for j = 1:length(r.maxspeed)
                    if qdot(i,j) > r.maxspeed(j)
                        qdot(i,j) = r.maxspeed(j);
                    elseif qdot(j) < -r.maxspeed(j)
                        qdot(i,j) = -r.maxspeed(j);
                    end
                end  
                %13.Add new joint state to trajectory
                %new joint state = (previous joint state) + ((joint velocity)*(change in time)) 
                qMatrix(i+1,:) = qMatrix(i,:) + qdot(i,:)*deltaT;                       % Update next joint state based on joint velocities
            end
        end

        function qMatrix = trap_ikine(self,r,q1,T1,T2,mask,steps)
            %1. Solve Inverse Kinematics
            link_length = length(r.model.links());
            q2 = r.model.ikine(T2,q1,mask)
            %2. Generate trajectory
            s = lspb(0,1,steps);                        % First, create the scalar function
            qMatrix = nan(steps,link_length);                     % Create memory allocation for variables
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;   % Generate interpolated joint angles
            end
        end

        function qMatrix = trap_ikcon(self,r,q1,T1,T2,steps)
            link_length = length(r.model.links());
            %1. Solve Inverse Kinematics
            q2 = r.model.ikcon(T2,q1);
            s = lspb(0,1,steps);                                            % First, create the scalar function
            qMatrix = nan(steps,link_length);                               % Create memory allocation for variables
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;                   	% Generate interpolated joint angles
            end
        end

        function qMatrix = quintic_ikine(self,r,q1,T1,T2,mask,steps)
            %1. Solve Inverse Kinematics
            q2 = r.model.ikine(T2,q1,mask);
            %2. Generate Trajectory
            qMatrix = jtraj(q1,q2,steps);
        end

        function qMatrix = quintic_ikcon(self,r,q1,T1,T2,steps)
            q2 = r.model.ikcon(T2,q1);
            qMatrix = jtraj(q1,q2,steps);
        end    
    end

    methods (Hidden)
%% InitiliseRobotPlot
% First and only time to plot the robot
        function h = InitiliseRobotPlot(self)
            self.figure_h = gcf;
            self.axis_h = gca;
            initialSurfaceCount = self.CountTiledFloorSurfaces();
            % Display robot
            [ax,by] = view;
            
            roughMinMax = sum(abs(self.model.d) + abs(self.model.a));
            self.workspace = [-roughMinMax roughMinMax -roughMinMax roughMinMax -0.01 roughMinMax]; 

            self.model.plot3d(self.homeQ,'noarrow','workspace',self.workspace,'view',[ax,by]);%,'notiles');            

            % Check if a single surface has been added by plot3d
            if self.CountTiledFloorSurfaces() - initialSurfaceCount == 1
                self.surfaceAdded = true;
            end

            % Check if a light needs to be added
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
                self.lightAdded = true;
            end

            self.model.delay = 0;
            handles = findobj('Tag', self.model.name);
            h = get(handles,'UserData');
        end

%% TestMoveJoints      
        % Simple test move of the joints
        function TestMoveJoints(self)
            qPath = jtraj(self.model.qlim(:,1)',self.model.qlim(:,2)',50);
            initialDelay = self.model.delay;
            self.model.delay = self.delaySecondsForInnerAnimation;
            self.model.animate(qPath);
            self.model.delay = initialDelay;
        end

%% TestMoveBase
        % Simple test move of the base
        function TestMoveBase(self)
            startBaseTR = self.model.base.T;
            self.MoveBaseToTR(startBaseTR,transl(-1,-1,0));
            self.MoveBaseToTR(transl(-1,-1,0),transl(1,1,0));
            self.MoveBaseToTR(transl(1,1,0),startBaseTR);
        end

%% MoveBaseToTR 
        % move robot base through a ctraj generated path
        function MoveBaseToTR(self,startTR, endTR)
            trPath = ctraj(startTR,endTR,self.stepsForInnerAnimation);
            for i = 1:size(trPath,3)
                self.model.base = trPath(:,:,i);
                self.model.animate(self.model.getpos);
                pause(self.delaySecondsForInnerAnimation);
            end
        end

%% CountTiledFloorSurfaces
        % A way of checking if a base tiled floor surface has been added
        % that needs deleting
        function surfaceCount = CountTiledFloorSurfaces(self)
            surfaceCount = numel(findobj(self.axis_h, 'Type', 'surface', 'Tag', 'tiled_floor'));
        end

%% MultirobotWarningMessage
        function MultirobotWarningMessage(self)
            disp('There are more than 1 of these robots in the plot although they may be plotted over each other. This shouldnt break anything, but it is a good idea to clean up by deleting identical robots before you replot a new one. For now I will recolour the first (and probably latest) one.')
        end
    end
end