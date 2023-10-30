%Initialize ROS
rosinit
logPub = rospublisher('/log_info', 'std_msgs/String');
modeControl_sub = rossubscriber('/program_mode' ,'std_msgs/Int8',@modeControl);
octo_print_status_sub = rossubscriber('/octoprint_status','std_msgs/Bool', @print_status_cb);
octo_print_printer_id_sub = rossubscriber('/octoprint_finished_printers','std_msgs/Int8',@printer_id_cb);
global mode;
global print_end;
global printer_id;

mode = 0;
print_end = 0;
printer_id = 0;


%Hard-Coded Variables Go HERE
printer_poses = zeros(4,4,3);
printer_poses(:,:,1) = SE3(0,0,0);
printer_poses(:,:,2) = SE3(0,0,0);
printer_poses(:,:,3) = SE3(0,0,0);

bench_position = SE3(0,0,0);
bed_mount_position = SE3(0,0,0);
bed_attach_position = SE3(0,0,0);

printed_object_heights = [0,0,0,0];
%Object 1 will go to BIN 1 etc
object_bin_allocation = [1,2,3,4];

tm5_homePose = SE3(eye(4) * transl(0,0,0));
tm5_base = transl(1.5,1,1.324) * trotz(pi/2);
tm5_elbowPos =[0,0,0,0,0,0];
tm5_jointLimit =[0,0,0,0,0,0];

ur3e_homePose = SE3(eye(4) * transl(0,0,0));
ur3e_base = transl(-0.75,1,1.343);
ur3e_elbowPos = [0,0,0,0,0,0];
ur3e_jointLimit =[0,0,0,0,0,0];


%Initialize Environment
SetUp();
logger(logPub,'Environment Initialized');


%Initialize Robots
tm5_1 = LinearTM5(tm5_base);
tm5_1.setHomePose(tm5_homePose);
tm5_1.setElbowPos(tm5_elbowPos);
tm5_1.setJointLimits(tm5_jointLimit);

ur3e_1 = ur3e_modified(ur3e_base);
ur3e_1.setHomePose(ur3e_homePose);
%ue3e_1.setElbowPos(ur3e_elbowPos);
ur3e_1.setJointLimits(ur3e_jointLimit);
logger(logPub,'Robots Initialized');

axis equal;

while mode ~= 3
    switch mode
        case 1
            logger(logPub,['Program Entering Automatic Control Mode ' num2str(mode)]);
            logger(logPub,"Waiting For Printer To Finish");
            % Wait until the boolean is true
            while (((~print_end) && (printer_id == 0)) && mode == 1)
                %% Allow Program To Exit from Automatic Mode by changing modes to something
                %other than 1.
                pause(0.1); % Pause for a short time to prevent excessive CPU usage
            end
            if ((print_end) && (printer_id ~= 0) && (mode == 1))
                logger(logPub,['Printer # ' num2str(printer_id) ' has finished!']);
                processPart(printer_poses,printer_id,logPub, ...
                            tm5_1,ur3e_1, ...
                            bench_position,bed_mount_position, ...
                            bed_attach_position);
                updatePrinterStatus(printer_id);
                print_end = false;
                printer_id = 0;
            end
        case 2
            logger(logPub,['Received mode: ' num2str(mode)]);
            logger(logPub,"Entering Manual Control Mode");
            manualMode(tm5_1,ur3e_1);
        case 3

        otherwise
            %fprintf('Invalid mode. Please select 1, 2, or 3.\n');
    end
    pause(0.1); % Pause for a short time to prevent excessive CPU usage
end

logger(logPub,'Exiting the program.');
rosshutdown
clear

function print_status_cb(~, message)
    % Callback function to handle received boolean messages
    global print_end;
    print_end = message.Data;
end

function printer_id_cb(~, message)
    % Callback function to handle received integer array messages
    global printer_id;
    printer_id = message.Data;
end

function modeControl(~,message)
    global mode;
    mode = message.Data;
end