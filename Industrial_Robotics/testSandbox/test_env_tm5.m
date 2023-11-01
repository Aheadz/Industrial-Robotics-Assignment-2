location = {
            [0,0,0]
            [0,0,0] %Bench_Omran %BECAUSE OF ROTATION OF Z THIS ONE HAS NEGATIVE BUT IT IS [1.25,1,0.324]
            [0,1.0,0]   %Bench_Print
            [-1.25,1,0.324]  %Bench_UR3E

            [-0.5,0.603,1.445] %Bench_UR3E_BinR1
            [-1,0.603,1.445] %Bench_UR3E_BinR2
            [-0.5,1.397,1.445] %Bench_UR3E_BinL1
            [-1,1.397,1.445] %Bench_UR3E_BinL2

            [0,1.0,1.0]   %Bench_Print_Printer1
            [1.25,2,1.343]   %Bench_Print_Printer2
            [2,2,1.343]   %Bench_Print_Printer3
            
           }; 
hold on
PlaceObject('Bench.PLY',location{2});
PlaceObject('Bench.PLY',location{3})
printer_1 = printerSpawn(location{9});