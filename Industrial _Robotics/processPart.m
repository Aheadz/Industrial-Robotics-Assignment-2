function completed = processPart(printer_poses,printer_id,logPub,tm5,ur3e, ...
                                 bench_position,bed_mount_position, bed_attach_position)
%PROCESSPRINT Summary of this function goes here
% This function runs in the automatic mode of the program, it waits to
% recieve a call from Octoprint indicating that a printer has finished
% printing.
    logger(logPub,['Starting Print Clearing Operation for Printer #' num2str(printer_id)]);
    logger(logPub,'Moving to Printer...');
    tm5.goToPrinter(printer_poses(:,:,printer_id));
    logger(logPub,'Attaching End-Effector...');
    tm5.attachtool();
    logger(logPub,'Removing Print Bed...');
    tm5.removeBed();
    logger(logPub,'Moving to Sorting Bench...');
    tm5.moveBed(bench_position);
    logger(logPub,'Mounting Bed to Bench..');
    tm5.mountBed(bed_mount_position);
    logger(logPub,'Dettaching End-Effector...');
    tm5.detach();
    logger(logPub,'Moving TM5 to Home Position...');
    tm5.home();
    
    %Return SE3 array indicating location of all objects
    logger(logPub,'Beginning Object Scan...');
    object_poses = ur3e.start_scan();
    logger(logPub,'Sorting Objects...');
    
    %supposed to take 3 arguments but for some reason matlab says too many when
    %i pass 
    %ur3e.sort_objects(object_poses, object_bin_allocation);
    ur3e.sort_objects();
    
    bedClear = false;
    bedClear = ur3e.check_bed();
    
    while ~bedClear
        bedClear = ur3e.clear_bed();
    end
    
    logger(logPub,'Moving TM5 to pickup empty Print Bed...');
    tm5.moveBed(bench_position);
    %Can re-use the mountBed Function as long as we give it different
    % co-ordinates.
    logger(logPub,'Moving to Attach position...');
    tm5.mountBed(bed_attach_position);
    logger(logPub,'Attaching Print Bed...');
    tm5.attachtool();
    logger(logPub,'Removing Print Bed From Bench...');
    tm5.removeBed();
    logger(logPub,'Moving Print Bed Back to Printer...');
    tm5.moveBed(printer_poses(:,:,printer_id));
    logger(logPub,'Placing Print Bed onto Printer...');
    tm5.placeBed();
    logger(logPub,'Sorting Objects...');
    tm5.detach();
    logger(logPub,'Sending TM5 Back to Home Position...');
    tm5.home();
    
    logger(logPub,['Printer #' num2str(printer_id) ' Is Now Ready for printing again']);
    
    updatePrinterStatus(printer_id);
    completed = true;
end

