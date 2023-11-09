location = {
            [0,0,0]
            [1.25,1,0.324] %Bench_Omran %BECAUSE OF ROTATION OF Z THIS ONE HAS NEGATIVE BUT IT IS [1.25,1,0.324]
            [1.25,2,0.324]   %Bench_Print
            [-1.25,1,0.324]  %Bench_UR3E

            [-0.5,0.603,1.445] %Bench_UR3E_BinR1
            [-1,0.603,1.445] %Bench_UR3E_BinR2
            [-0.5,1.397,1.445] %Bench_UR3E_BinL1
            [-1,1.397,1.445] %Bench_UR3E_BinL2

            [0.3,1.85,1.343]   %Bench_Print_Printer1
            [.9,1.85,1.343]   %Bench_Print_Printer2
            [1.5,1.85,1.343]   %Bench_Print_Printer3
            
           }; 

PlaceObject('Garage.PLY',location{1});
hold on

PlaceObject('Bench.PLY',location{2});
PlaceObject('Bench.PLY',location{3});
PlaceObject('WBench.PLY',location{4});

PlaceObject('BinR.ply',location{5});
PlaceObject('BinR.ply',location{6});
PlaceObject('BinL.ply',location{7});
PlaceObject('BinL.ply',location{8});


printer_1 = printerSpawn(location{9});
printer_2 = printerSpawn(location{10});
printer_3 = printerSpawn(location{11});
PlaceObject('emergencyStopButton.ply', [2.25,0.75,1.343])
PlaceObject('emergencyStopButton.ply', [-2.25,0.75,1.343])

PlaceObject('fireExtinguisher.ply', [2.75,0,1.343])
PlaceObject('fireExtinguisher.ply', [-2.75,0,1.343])

PlaceObject('fireExtinguisher.ply', [2.75,0,1.343])
PlaceObject('fireExtinguisher.ply', [-2.75,0,1.343])

PlaceObject('fenceFinals.ply', [0,-1,0.343])
PlaceObject('fenceFinals.ply', [1.8,-1,0.343])
PlaceObject('fenceFinals.ply', [-1.8,-1,0.343])

printer_1 = printerSpawn(location{9});
printer_2 = printerSpawn(location{10});
printer_3 = printerSpawn(location{11});

omron = LinearTM5();
ur = ur3e_modified();
urEndPose = ur.model.fkine(ur.model.getpos());
gripL = g_2F85;
gripR = g_2F85;

gripR.model.base = urEndPose.T*trotx(-pi/2)*trotz(-pi);
gripL.model.base = urEndPose.T*trotx(pi/2);
gripL.model.animate([0,0,0]);
gripR.model.animate([0,0,0]);
omron = LinearTM5();
ur = ur3e_modified();
urEndPose = ur.model.fkine(ur.model.getpos());
gripL = g_2F85;
gripR = g_2F85;

gripR.model.base = urEndPose.T*trotx(-pi/2)*trotz(-pi);
gripL.model.base = urEndPose.T*trotx(pi/2);
gripL.model.animate([0,0,0]);
gripR.model.animate([0,0,0]);

Plane(-2.8,-0.92,1.354, -2.8,-0.92,1.354, 2.8,-0.92,0, 2.8,-0.92,0);

pb1 = printBed();
% pb2 = printBed();
% pb3 = printBed();

pb1.bedModel{1}.base = transl(.275,1.806,1.354);
% pb2.bedModel{1}.base = translationmat*transl(location{10});
% pb3.bedModel{1}.base = translationmat*transl(1.475,1.806,1.354);

pb1.UpdateModel
% pb2.UpdateModel
% pb3.UpdateModel
% 
% Printer(location{9});
% Printer(location{10});
% Printer(location{11});

% P1_Shell = PlaceObject('PrintLink0.ply', location{9});
% P1_BedMover = PlaceObject('PrintLink1.ply', location{9});
% P1_Bed = PlaceObject('PrintLink2.ply', location{9});
% P1_Printer = PlaceObject('PrintLink3.ply', location{9});
% 
% P2_Shell = PlaceObject('PrintLink0.ply', location{10});
% P2_BedMover = PlaceObject('PrintLink1.ply', location{10});
% P2_Bed = PlaceObject('PrintLink2.ply', location{10});
% P2_Printer = PlaceObject('PrintLink3.ply', location{10});
% 
% P3_Shell = PlaceObject('PrintLink0.ply', location{11});
% P3_BedMover = PlaceObject('PrintLink1.ply', location{11});
% P3_Bed = PlaceObject('PrintLink2.ply', location{11});
% P3_Printer = PlaceObject('PrintLink3.ply', location{11});