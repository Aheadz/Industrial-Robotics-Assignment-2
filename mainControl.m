%Initialize ROS
rosinit
logPub = rospublisher('/log_info', 'std_msgs/String');


%Hard-Coded Variables Go HERE
printer_poses = zeros(4,4,3);
printer_poses(:,:,1) = SE3(0,0,0);
printer_poses(:,:,2) = SE3(0,0,0);
printer_poses(:,:,3) = SE3(0,0,0);

bench_position = SE3(0,0,0);
bed_mount_position = SE3(0,0,0);
bed_attach_position = SE3(0,0,0);

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
SetUp();
logger(logPub,'Environment Initialized');


LinearUR3(transl(0.75,1,1.324));
%Initialize Robots
tm5_1 = tm5;
tm5_1.setHomePose(tm5_homePose);
tm5_1.model.base = tm5_base;
tm5_1.setElbowPos(tm5_elbowPos);
tm5_1.setJointLimits(tm5_jointLimit);

ur3e_1 = ur3e;
ur3e_1.setHomePose(ur3e_homePose);
ur3e_1.model.base = ur3e_base;
%ue3e_1.setElbowPos(ur3e_elbowPos);
ur3e_1.setJointLimits(ur3e_jointLimit);

%Check Printer Status
printer_id = printerState();
logger(logPub,['Starting Print Clearing Operation for Printer #' num2str(printer_id)]);

logger(logPub,'Moving to Printer...');
tm5_1.goToPrinter(printer_poses(:,:,printer_id));
logger(logPub,'Attaching End-Effector...');
tm5_1.attachtool();
logger(logPub,'Removing Print Bed...');
tm5_1.removeBed();
logger(logPub,'Moving to Sorting Bench...');
tm5_1.moveBed(bench_position);
logger(logPub,'Mounting Bed to Bench..');
tm5_1.mountBed(bed_mount_position);
logger(logPub,'Dettaching End-Effector...');
tm5_1.detach();
logger(logPub,'Moving TM5 to Home Position...');
tm5_1.home();

%Return SE3 array indicating location of all objects
logger(logPub,'Beginning Object Scan...');
object_poses = ur3e_1.start_scan(printed_object_heights);
logger(logPub,'Sorting Objects...');

%supposed to take 3 arguments but for some reason matlab says too many when
%i pass 
ur3e_1.sort_objects(object_poses, object_bin_allocation);

bedClear = false;
bedClear = ur3e_1.check_bed();

while ~bedClear
    bedClear = ur3e_1.clear_bed();
end

logger(logPub,'Moving TM5 to pickup empty Print Bed...');
tm5_1.moveBed(bench_position);
%Can re-use the mountBed Function as long as we give it different
% co-ordinates.
logger(logPub,'Moving to Attach position...');
tm5_1.mountBed(bed_attach_position);
logger(logPub,'Attaching Print Bed...');
tm5_1.attachtool();
logger(logPub,'Removing Print Bed From Bench...');
tm5_1.removeBed();
logger(logPub,'Moving Print Bed Back to Printer...');
tm5_1.moveBed(printer_poses(:,:,printer_id));
logger(logPub,'Placing Print Bed onto Printer...');
tm5_1.placeBed();
logger(logPub,'Sorting Objects...');
tm5_1.detach();
logger(logPub,'Sending TM5 Back to Home Position...');
tm5_1.home();

logger(logPub,['Printer #' num2str(printer_id) ' Is Now Ready for printing again']);

updatePrinterStatus(printer_id);


rosshutdown


function logger(publisher,str)
    logMsg = rosmessage(publisher);
    logMsg.Data = str;
    send(publisher, logMsg);
end