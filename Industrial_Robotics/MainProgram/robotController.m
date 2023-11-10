classdef robotController < handle
    %robotController builds control system around the robot models
    %Detailed explanation goes here
    % Public properties can be accessed from outside the class
    properties (Access = public)
        r;
        estop = false;
    end
    
    % Protected properties can only be accessed from within the class and subclasses
    properties (Access = protected)

    end
    
    % Private properties can only be accessed from within the class
    properties (Access = private)
        current_trajectory;
        current_position;
        current_index;
        l2;
    end

    methods
        function self = robotController(r)
            %Set r to a robot class object
            self.r = r;
        end
%%Trajectory Generation methods
        %Generalized RMRC that will move between pose changing rotation and translation.
        function qMatrix = RMRC_general(self,startPose,targetPose, deltaT, time)
            %1.Parameter Setup
            steps = time/deltaT;
            threshold = 0.01;
            epsilon = 0.1;                          % Threshold value for manipulability/Damped Least Squares. 
            W = diag([1 1 1 0.1 0.1 0.1]);          % Weighting matrix for the velocity vector.
            N = 10; %Points to generate for interpolation.
            %2. Allocate Array Data.
            qMatrix = zeros(steps,length(q0));
            qMatrix = zeros(steps,length(q0));       % Array for joint angles.
            m = zeros(steps,1);                      % Array for Measure of Manipulability.
            qdot = zeros(steps,link_length);                   % Array for joint velocities.
            %3. Starting Joint Angles.
            qMatrix(1,:) = self.r.model.ikcon(startPose.T,q0);
            for i = 1:steps-1
                %1.Compute the current end-effector pose.
                x1 = self.r.model.fkine(qMatrix(i,:)).T;                
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
                J = self.r.model.jacob0(qMatrix(i,:));
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
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.r.model.qlim(j,1)           % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; %Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.r.model.qlim(j,2)       % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; %Stop the motor
                    end
                end
                % Cap the joint velocities
                for j = 1:link_length
                    if qdot(j) > self.r.maxspeed(j)
                        qdot(j)
                        qdot(j) = self.r.maxspeed(j);
                    elseif qdot(j) < - (self.r.maxspeed(j))
                        qdot(j)
                        qdot(j) = - (self.r.maxspeed(j));
                    end
                end  
                %14.Add new joint state to trajectory.
                %new joint state = (previous joint state) + ((joint velocity)*(change in time)) 
                qMatrix(i+1,:) = qMatrix(i,:) + qdot(i,:)*deltaT;                       % Update next joint state based on joint velocities
            end

        end 
        function qMatrix = increment_end_effector(self,q0,dx,dy,dz,wx,wy,wz,deltaT,time)
           currentPose = self.r.model.fkine(q0);
           increment_transform = transl(dx,dy,dz) * rpy2tr(deg2rad(wx),deg2rad(wy),deg2rad(wz));
        
           targetPose = currentPose * SE3(increment_transform);
           qMatrix = self.RMRC_noRot(q0,targetPose,deltaT,time);
        end
        %Linear RMRC that does change end-effector orientation
        function qMatrix = RMRC_noRot(self,q0, targetPose, deltaT, time)

            %1.Parameter Setup
            link_length = length(self.r.model.links());
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
            x1 = self.r.model.fkine(q0).T;
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
                x1 = self.r.model.fkine(qMatrix(i,:)).T;
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
                J = self.r.model.jacob0(qMatrix(i,:));

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
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.r.model.qlim(j,1)           % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; %Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.r.model.qlim(j,2)       % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; %Stop the motor
                    end
                end
                % Cap the joint velocities
                for j = 1:length(self.r.maxspeed)
                    if qdot(i,j) > self.r.maxspeed(j)
                        qdot(i,j) = self.r.maxspeed(j);
                    elseif qdot(j) < -(self.r.maxspeed(j))
                        qdot(i,j) = -(self.r.maxspeed(j));
                    end
                end  
                %13.Add new joint state to trajectory
                %new joint state = (previous joint state) + ((joint velocity)*(change in time)) 
                qMatrix(i+1,:) = qMatrix(i,:) + qdot(i,:)*deltaT;                       % Update next joint state based on joint velocities
            end
        end
        function qMatrix = trap_ikine(self,q1,T1,T2,mask,steps)
            %1. Solve Inverse Kinematics
            link_length = length(self.r.model.links());
            q2 = self.r.model.ikine(T2,q1,mask)
            %2. Generate trajectory
            s = lspb(0,1,steps);                        % First, create the scalar function
            qMatrix = nan(steps,link_length);                     % Create memory allocation for variables
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;   % Generate interpolated joint angles
            end
        end
        function qMatrix = trap_ikcon(self,q1,T1,T2,steps)
            link_length = length(self.r.model.links());
            %1. Solve Inverse Kinematics
            q2 = self.r.model.ikcon(T2,q1);
            s = lspb(0,1,steps);                                            % First, create the scalar function
            qMatrix = nan(steps,link_length);                               % Create memory allocation for variables
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;                   	% Generate interpolated joint angles
            end
        end
        function qMatrix = quintic_ikine(self,q1,T1,T2,mask,steps)
            %1. Solve Inverse Kinematics
            q2 = self.r.model.ikine(T2,q1,mask);
            %2. Generate Trajectory
            qMatrix = jtraj(q1,q2,steps);
        end
        function qMatrix = quintic_ikcon(self,q1,T1,T2,steps)
            q2 = self.r.model.ikcon(T2,q1);
            qMatrix = jtraj(q1,q2,steps);
        end

%Movement System
        function move(self,targetPose,type)
            switch type
                case 1
                    %RMRC General
                    qMatrix = self.RMRC_general(self.r.model.fkine(self.r.model.getpos()),targetPose,0.25,5);
                case 2
                    %RMRC No Rotation
                    qMatrix = self.RMRC_noRot(self.r.model.getpos(),targetPose,0.25,5);
                case 3
                    %Trapezoidal ikine
                    qMatrix = self.trap_ikine(self.r.model.getpos(),0,targetPose,[1 1 1 1 1 1],50);
                case 4
                    %Trapezoidal ikcon
                    qMatrix = self.trap_ikcon(self.r.model.getpos(),0,targetPose,50);
                case 5
                    %Quintic ikine
                    qMatrix = self.quintic_ikine(self.r.model.getpos(),0,targetPose,[1 1 1 1 1 1],50);
                case 6
                    %Quintic ikcon
                    qMatrix = self.quintic_ikcon(self.r.model.getpos(),0,targetPose,50);
            end
            %self.plotTrajectory(qMatrix)
            self.executeTraj(qMatrix)
        end

        function executeTraj(self,qMatrix)
            self.current_trajectory = qMatrix;
            for i = 1:length(qMatrix)
                if (self.estop)
                    self.current_position = qMatrix(i,:);
                    self.current_index = i;
                    break;
                else
                    self.r.model.animate(qMatrix(i,:));
                    drawnow();
                    pause(0.1);
                end
            end
        end

        function continueTraj(self)
            qMatrix = self.current_trajectory;
            for i = self.current_index:length(qMatrix)
                if (self.estop)
                    self.current_position = qMatrix(i,:);
                    self.current_index = i;
                    break;
                else
                    self.r.model.animate(qMatrix(i,:));
                    drawnow();
                    pause(0.1);
                end
            end
        end

        function plotTrajectory(self,qMatrix)
            pointsMatrix = zeros(length(qMatrix),3);
            for i = 1:length(qMatrix)
                point = self.r.model.fkine(qMatrix(i,:)).T;
                pointsMatrix(i,:) = point(1:3,4)';
            end
            x = pointsMatrix(:,1);
            y = pointsMatrix(:,2);
            z = pointsMatrix(:,3);
            delete(self.l2);
            self.l2 = plot3(x,y,z,'-o','LineWidth', 2);
        end

    end
    
    methods (Access = private)
        % Private method
        function result = privateMethod(obj)
            %PRIVATEMETHOD An example of a private method
            %   This method can only be called from within the class.
            result = obj.PrivateProperty1 * obj.PrivateProperty2;
        end
    end
    
    methods (Access = protected)
        % Protected method
        function result = protectedMethod(obj)
            %PROTECTEDMETHOD An example of a protected method
            %   This method can be called from within the class and subclasses.
            result = obj.ProtectedProperty1 - obj.ProtectedProperty2;
        end
    end
    
    methods (Static)
        % Static method
        function result = staticMethod(inputArg)
            %STATICMETHOD An example of a static method
            %   This method can be called without creating an instance of the class.
            result = MyClass.someStaticOperation(inputArg);
        end
    end
    
    methods (Access = private, Static)
        % Private static method
        function result = privateStaticMethod(inputArg)
            %PRIVATESTATICMETHOD An example of a private static method
            %   This method can only be called from other static methods within the class.
            result = MyClass.somePrivateStaticOperation(inputArg);
        end
    end
end