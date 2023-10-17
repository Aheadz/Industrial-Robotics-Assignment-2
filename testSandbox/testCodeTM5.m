rosinit

logPub = rospublisher('/log_info', 'std_msgs/String');

tm5_homePose = SE3(eye(4) * transl(0,0,0));
tm5_base = transl(1.5,1,1.324) * trotz(pi/2);
tm5_elbowPos =[0,0,0,0,0,0];
tm5_jointLimit =[0,0,0,0,0,0];

%Initialize Robots
tm5_1 = LinearTM5(tm5_base);
tm5_1.setHomePose(tm5_homePose);
tm5_1.setElbowPos(tm5_elbowPos);
tm5_1.setJointLimits(tm5_jointLimit);

%Initialize Test Environment
test_env_tm5();

axis equal;
