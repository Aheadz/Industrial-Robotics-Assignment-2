location = {
            [0,0,0]
            [1.25,1,0.324] %Bench_Omran %BECAUSE OF ROTATION OF Z THIS ONE HAS NEGATIVE BUT IT IS [1.25,1,0.324]
            [1.25,2,0.324]   %Bench_Print
            [-1.25,1,0.324]  %Bench_UR3E

            [-0.5,0.603,1.445] %Bench_UR3E_BinR1
            [-1,0.603,1.445] %Bench_UR3E_BinR2
            [-0.5,1.397,1.445] %Bench_UR3E_BinL1
            [-1,1.397,1.445] %Bench_UR3E_BinL2

            [0.5,1.85,1.343]   %Bench_Print_Printer1
            [.9,1.85,1.343]   %Bench_Print_Printer2
            [2,1.85,1.343]   %Bench_Print_Printer3
            
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