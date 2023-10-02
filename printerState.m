function printer_id = printerState()
    % Variables to hold the received data
    % Variable to hold the state of the received boolean message
    print_end = false;
    printer_id = 0;

    % Create a subscriber with a callback function
    octo_print_status_sub = rossubscriber('/octoprint_status','std_msgs/Bool', @print_status_cb);
    octo_print_printer_id_sub = rossubscriber('/octoprint_finished_printers','std_msgs/Int8',@printer_id_cb);


    % Wait until the boolean is true
    while ((~print_end) | (printer_id == 0))
        pause(0.1); % Pause for a short time to prevent excessive CPU usage
    end

    function print_status_cb(~, message)
        % Callback function to handle received boolean messages
        print_end = message.Data;
    end

    function printer_id_cb(~, message)
        % Callback function to handle received integer array messages
        printer_id = message.Data;
    end
end

function updatePrinterStatus(printer_id)

end