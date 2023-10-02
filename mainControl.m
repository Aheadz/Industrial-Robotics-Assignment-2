%Initialize ROS
rosinit

%Hard-Coded Variables Go HERE
printer_poses = zeros(4,4,3);
printer_poses(:,:,1) = SE3(0,0,0);
printer_poses(:,:,2) = SE3(0,0,0);
printer_poses(:,:,3) = SE3(0,0,0);

bench_position = SE3(0,0,0);
bed_mount_position = SE3(0,0,0);

%Assuming there are 4 objects on the print bed and 4 bins.
bin_locations = zeros(4,4,4);
bin_locations(:,:,1) = SE3(0,0,0);
bin_locations(:,:,2) = SE3(0,0,0);
bin_locations(:,:,3) = SE3(0,0,0);
bin_locations(:,:,4) = SE3(0,0,0);

printed_object_heights = [0,0,0,0];
%Object 1 will go to BIN 1 etc
object_bin_allocation = [1,2,3,4];


tm5_homePose = SE3(eye(4) * transl(0,0,0));
tm5_base = transl(0,0,0);
tm5_elbowPos =[0,0,0,0,0,0];
tm5_jointLimit =[0,0,0,0,0,0];

ur3e_homePose = SE3(eye(4) * transl(0,0,0));
ur3e_base = transl(0,0,0);
ur3e_elbowPos = [0,0,0,0,0,0];
ur3e_jointLimit =[0,0,0,0,0,0];


%Initialize Environment
init_env();
logPub = rospublisher('/log_info', 'std_msgs/String');
logMsg = rosmessage(logPub);
logMsg.Data = 'Environment Initialized';
send(logPub, logMsg);

%Initialize Robots
tm5_1 = tm5();
tm5_1.setHomePose(tm5_homePose);
tm5_1.model.base = tm5_base;
tm5_1.setElbowPos = tm5_elbowPos;
tm5_1.setJointLimits = tm5_jointLimit;

ur3e_1 = ur3e_custom();
ur3e_1.setHomePose(ur3e_homePose);
ur3e_1.model.base = ur3e_base;
ue3e_1.setElbowPos = ur3e_elbowPos
ur3e_1.setJointLimits = ur3e_jointLimit;

%Check Printer Status
printer_id = printerState();
logMsg.Data = ['Starting Print Clearing Operation for Printer #' num2str(printer_id)];
send(logPub,logMsg);


tm5_1.goToPrinter(printer_poses(:,:,printer_id));
tm5_1.attachtool();
tm5_1.removeBed();
tm5_1.moveBed(bench_position);
tm5_1.mountBed(bed_mount_position);
tm5_1.dettach();
tm5_1.home();

%Return SE3 array indicating location of all objects
object_poses = ur3e_1.start_scan(printed_object_heights);
ur3e_1.sort_objects(object_poses, object_bin_allocation, bin_locations);

bedClear = false;
bedClear = ur3e_1.check_bed();

while ~bedClear
    bedClear = ur3e_1.clear_bed();
end

tm5_1.moveBed(bench_position);
tm5_1.mountBed(bed_attach_position);




rosshutdown