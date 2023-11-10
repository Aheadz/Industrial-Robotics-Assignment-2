
app = projectGUI_exported;
SetUp();

tm5 = LinearTM5();
ur3e = ur3e_modified();


tm5_ctrl = robotController(tm5);
app.setTM5_ctrl(tm5_ctrl);
ur3e_ctrl = robotController(ur3e);
app.setUR3e_ctrl(ur3e_ctrl);

while true

    % Prompt user for translation input
    dx = input('Enter the x translation: ');
    dy = input('Enter the y translation: ');
    dz = input('Enter the z translation: ');



    % Prompt user for rotation input in degrees
    wx = input('Enter the roll angle in degrees: ');
    wy = input('Enter the pitch angle in degrees: ');
    wz = input('Enter the yaw angle in degrees: ');




    qMatrix = ur3e_ctrl.increment_end_effector(ur3e.model.getpos(),dx,dy,dz,wx,wy,wz,0.05,5);
    ur3e_ctrl.plotTrajectory(qMatrix);
    ur3e_ctrl.executeTraj(qMatrix);


end