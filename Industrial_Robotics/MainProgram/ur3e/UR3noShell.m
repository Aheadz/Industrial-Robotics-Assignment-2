function plot_robot_no_ply(self)
            L1 = Link('d',0.089159,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            L2 = Link('d',0,'a',-0.425,'alpha',0,'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]);
            L3 = Link('d',0,'a',-0.39225,'alpha',0,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
            L4 = Link('d',0.10915,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            L5 = Link('d',0.09465,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            L6 = Link('d',0.0823,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);   
             
            self.robot = SerialLink([L1,L2,L3,L4,L5,L6],'name','random_robot');
 
            self.robot.plot(zeros(1,6));
end