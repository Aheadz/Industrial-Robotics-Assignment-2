%rosinit

%logPub = rospublisher('/log_info', 'std_msgs/String');



ur3e_homePose = SE3(eye(4) * transl(0,0,0));
ur3e_base = transl(0.6,0,1.0);
ur3e_elbowPos = [0,0,0,0,0,0];
ur3e_jointLimit =[0,0,0,0,0,0];


test_env_ur3e()
ur3e_1 = ur3e_modified(ur3e_base);
ur3e_1.setHomePose(ur3e_homePose);
%ue3e_1.setElbowPos(ur3e_elbowPos);
ur3e_1.setJointLimits(ur3e_jointLimit);
axis equal

% Example usage:
q0 = [0, 0, 0, 0, 0, 0];  % Initial joint angles
targetT = transl(0.5, 0.5, 0.5) * trotx(pi/4) * troty(pi/4);  % Desired SE(3) pose
targetPose = [targetT(1:3, 4); tr2rpy(targetT)];  % Convert SE(3) pose to position + Euler angles
dt = 0.1;
maxIterations = 100;




%rosshutdown