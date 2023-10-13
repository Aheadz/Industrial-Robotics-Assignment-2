clear;
clc;

location = {
            [0,0,0]
            [-1.25,-1,0.324] %Bench_Omran
            [1.25,2,0.324]   %Bench_Print
            [1.25,-1,0.324]  %Bench_UR3E

            [-0.5,0.603,1.445] %Bench_UR3E_BinR1
            [-1,0.603,1.445] %Bench_UR3E_BinR2
            [-0.5,1.397,1.445] %Bench_UR3E_BinL1
            [-1,1.397,1.445] %Bench_UR3E_BinL2
            
            
           }; 

% Garage(location{1});
% Bench_Omran(location{2});
Bench_Print(location{3});
hold on
% Bench_UR3E(location{4});
% 
% BinR(location{5});
% BinR(location{6});
% BinL(location{7});
% BinL(location{8});
% 
% LinearUR3(transl(0.75,1,1.324));
%UR3e(transl(-1.25,1,1.324));

axis equal