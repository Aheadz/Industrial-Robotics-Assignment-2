SetUp();

TM5 = LinearTM5();
UR3e = ur3e_modified();

trajectory1 = traj1(TM5);
trajectory2 = traj2(TM5);
trajectory3 = traj3(TM5);
trajectory4 = traj4(UR3e);


for i = 1:length(trajectory1)
    TM5.model.animate(trajectory1(i,:));
    drawnow();
    pause(0.1);
end

for i = 1:length(trajectory2)
    TM5.model.animate(trajectory2(i,:));
    drawnow();
    pause(0.1);
end

for i = 1:length(trajectory3)
    TM5.model.animate(trajectory3(i,:));
    UR3e.model.animate(trajectory4(i,:));
    drawnow();
    pause(0.1);
end



function tm5traj = traj1(TM5)
    %using quintic traj gen for more complex movements to minimise jerk
    tm5traj1 = TM5.quintic_ikcon(TM5,TM5.model.getpos(),0,TM5.model.fkine([0    0.1745   -0.7854    0.1745    0.1745    0.1745    3.1416]),50);
    tm5traj2 = TM5.trap_ikcon(TM5,TM5.model.getpos(),0,TM5.model.fkine([-0.2000    0.1745   -0.7854    0.5236    0.1745    0.5236    3.1416]),50);
    tm5traj3 = TM5.quintic_ikcon(TM5,TM5.model.getpos(),0,TM5.model.fkine([-0.2000    0.3427   -0.7997    1.1424    2.7418   -1.5422    3.1416]),50);
    tm5traj4 = TM5.quintic_ikcon(TM5,TM5.model.getpos(),0,TM5.model.fkine([-0.2000    0.3319   -0.8866    1.8326    2.1681   -1.1947    3.1416]),25);
    tm5traj5 = TM5.quintic_ikcon(TM5,TM5.model.getpos(),0,TM5.model.fkine([-0.2000    0.3319   -0.9599    1.8326    2.2689   -1.2392    3.1416]),10);
    tm5traj = [tm5traj1;tm5traj2;tm5traj3;tm5traj4;tm5traj5];
    %Move to Print Bed and pick up

end

function tm5traj = traj2(TM5)
    
    tm5traj1 = TM5.quintic_ikcon(TM5,TM5.model.getpos(),0,TM5.model.fkine([-0.2000    0.3319   -0.9599    1.8326    2.2689   -1.2392    3.1416]),25);
    tm5traj2 = TM5.quintic_ikcon(TM5,TM5.model.getpos(),0,TM5.model.fkine([-0.2000    0.3319   -0.8866    1.8326    2.1681   -1.1947    3.1416]),10);
    %proceed towards UR3e
    tm5traj3 = TM5.trap_ikcon(TM5,TM5.model.getpos(),0,TM5.model.fkine([ -0.2520    1.2427   -1.0751    1.3090    2.7336   -1.8326    3.1416]),50);
    tm5traj4 = TM5.trap_ikcon(TM5,TM5.model.getpos(),0,TM5.model.fkine([ -0.1888    1.3369   -0.8238    1.5603    2.5451   -1.8326    3.1416]),50);
    %Return the TM5 toward the printers, activate the UR3e
    tm5traj5 = TM5.quintic_ikcon(TM5,TM5.model.getpos(),0,TM5.model.fkine([-0.2000    0.1745   -0.7854    0.5236    0.1745    0.5236    3.1416]),100);
    tm5traj = [tm5traj1;tm5traj2;tm5traj3;tm5traj4;tm5traj5];
end
function tm5traj = traj3(TM5)
    %Return the TM5 toward the printers, activate the UR3e
    tm5traj1 = TM5.quintic_ikcon(TM5,TM5.model.getpos(),0,TM5.model.fkine([-0.2000    0.1745   -0.7854    0.5236    0.1745    0.5236    3.1416]),200);
    tm5traj = tm5traj1;
end

function UR3etraj = traj4(UR3e)
    urtraj1 = UR3e.quintic_ikcon(UR3e,UR3e.model.getpos(),0,UR3e.model.fkine([    2.7837   -0.6363    1.4316    3.8397   -1.5708         0]),100); 
    urtraj2 = UR3e.quintic_ikcon(UR3e,UR3e.model.getpos(),0,UR3e.model.fkine([1.7498   -0.9544    1.3521    3.8397   -1.5708         0]),50);
    urtraj3 = UR3e.quintic_ikcon(UR3e,UR3e.model.getpos(),0,UR3e.model.fkine([3.1416   -1.4316    1.3521    3.8397   -1.5708         0]),50);
    UR3etraj = [urtraj1;urtraj2;urtraj3];
end

