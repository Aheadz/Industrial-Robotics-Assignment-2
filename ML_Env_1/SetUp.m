clear;
clc;

location = {
            [0,0,0]
            [1.25,1,0.324] %Bench_Omran %BECAUSE OF ROTATION OF Z THIS ONE HAS NEGATIVE BUT IT IS [1.25,1,0.324]
            [1.25,2,0.324]   %Bench_Print
            [-1.25,1,0.324]  %Bench_UR3E

            [-0.5,0.603,1.445] %Bench_UR3E_BinR1
            [-1,0.603,1.445] %Bench_UR3E_BinR2
            [-0.5,1.397,1.445] %Bench_UR3E_BinL1
            [-1,1.397,1.445] %Bench_UR3E_BinL2

            [0.5,2,1.343]   %Bench_Print_Printer1
            [1.25,2,1.343]   %Bench_Print_Printer2
            [2,2,1.343]   %Bench_Print_Printer3
            
           }; 

PlaceObject('Garage.ply',location{1});
hold on

PlaceObject('Bench.ply',location{2});
PlaceObject('Bench.ply',location{3});
PlaceObject('WBench.ply',location{4});

PlaceObject('BinR.ply',location{5});
PlaceObject('BinR.ply',location{6});
PlaceObject('BinL.ply',location{7});
PlaceObject('BinL.ply',location{8});

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

tm5(transl(0.75,1,1.324));
% LinearUR3(0.75,1,1.324));

axis equal