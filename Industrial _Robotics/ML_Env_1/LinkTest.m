L1 = Link([pi     0       1      0     1]); % PRISMATIC Link
L2 = Link('d',1,'a',0,'alpha',pi/2,'qlim',[-pi/2 pi/2])
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi/2 pi/2])
L4 = Link('d',0,'a',1,'alpha',-pi/2,'qlim',[-pi/2 pi/2])
        
robot = SerialLink([L1 L2 L3 L4], 'name', 'myRobot')                   % Create the robot model

workspace = [-4 4 -4 4 -4 4];                                       % Define the boundaries of the workspace        
scale = 0.5;                                                        % Scale the robot down        
q = zeros(1,4);                                                     % Generate a vector of joint angles
% L1.qlim = [-0.8 -0.01];

robot.plot(q,'workspace',workspace,'scale',scale)                   % Plot the robot

% robot.teach;