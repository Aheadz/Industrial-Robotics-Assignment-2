% Initialize the ROS node
rosinit;
% Create a publisher on the /joint_states topic with the message type sensor_msgs/JointState
jointStatePub = rospublisher('/joint_states', 'sensor_msgs/JointState');
% Create a blank message of type sensor_msgs/JointState
jointStateMsg = rosmessage(jointStatePub);

r1 = UR5;
r1.model.base = transl(0,0,1);

q1 = [0,-pi/2,0,0,0,0];
q2 = r1.model.ikine(transl(-0.6,-0.5,0.3),'q0',q1);
q_matrix = jtraj(q1,q2,50);

for i = 1:length(q_matrix)
    %Send Joint State to Swift
    jointStateMsg.Name = {'UR5'};
    jointStateMsg.Position = q_matrix(i,:); % Example values
    jointStateMsg.Velocity = zeros(1,6); % Example values
    jointStateMsg.Effort = zeros(1,6); % Example values
    % Publish the message
    send(jointStatePub, jointStateMsg);
    r1.model.animate(q_matrix(i,:));
end
% Shut down the ROS node
rosshutdown;