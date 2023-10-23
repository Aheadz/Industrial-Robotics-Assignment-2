location = {
            [0,0,0]
            [1.25,1,0.324] %Bench_Omran %BECAUSE OF ROTATION OF Z THIS ONE HAS NEGATIVE BUT IT IS [1.25,1,0.324]
            [1.25,2,0.324]   %Bench_Print
            [-1.25,1,0.324]  %Bench_UR3E

            [0.5,-0.4,1.121] %Bench_UR3E_BinR1
            [1,-0.4,1.121] %Bench_UR3E_BinR2

            [0.5,0.4,1.121] %Bench_UR3E_BinL1
            [1.0,0.4,1.121] %Bench_UR3E_BinL2

            [0.5,2,1.343]   %Bench_Print_Printer1
            [1.25,2,1.343]   %Bench_Print_Printer2
            [2,2,1.343]   %Bench_Print_Printer3
            
           }; 
hold on
PlaceObject('WBench.PLY',location{1});
PlaceObject('BinR.ply',location{5});
PlaceObject('BinR.ply',location{6});
PlaceObject('BinL.ply',location{7});
PlaceObject('BinL.ply',location{8});
PlaceObject('PrintBedFixed.ply',[1,0,1.015]);