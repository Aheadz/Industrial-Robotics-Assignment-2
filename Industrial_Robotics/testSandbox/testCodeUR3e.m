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
ur3e_1.move(pose)

%rosshutdown