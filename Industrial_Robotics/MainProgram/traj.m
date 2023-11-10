function traj(tm5_ctrl,ur3e_ctrl)

    tm5traj1 = tm5_ctrl.quintic_ikcon(tm5_ctrl.r.model.getpos(),0,tm5_ctrl.r.model.fkine([0    0.1745   -0.7854    0.1745    0.1745    0.1745    3.1416]),50);
    tm5traj2 = tm5_ctrl.trap_ikcon(tm5_ctrl.r.model.getpos(),0,tm5_ctrl.r.model.fkine([-0.2000    0.1745   -0.7854    0.5236    0.1745    0.5236    3.1416]),50);
    tm5traj3 = tm5_ctrl.quintic_ikcon(tm5_ctrl.r.model.getpos(),0,tm5_ctrl.r.model.fkine([-0.2000    0.3427   -0.7997    1.1424    2.7418   -1.5422    3.1416]),50);
    tm5traj4 = tm5_ctrl.quintic_ikcon(tm5_ctrl.r.model.getpos(),0,tm5_ctrl.r.model.fkine([-0.2000    0.3319   -0.8866    1.8326    2.1681   -1.1947    3.1416]),25);
    tm5traj5 = tm5_ctrl.quintic_ikcon(tm5_ctrl.r.model.getpos(),0,tm5_ctrl.r.model.fkine([-0.2000    0.3319   -0.9599    1.8326    2.2689   -1.2392    3.1416]),10);
    trajectory1 = [tm5traj1;tm5traj2;tm5traj3;tm5traj4;tm5traj5];

    tm5traj1 = tm5_ctrl.quintic_ikcon(tm5_ctrl.r.model.getpos(),0,tm5_ctrl.r.model.fkine([-0.2000    0.3319   -0.9599    1.8326    2.2689   -1.2392    3.1416]),25);
    tm5traj2 = tm5_ctrl.quintic_ikcon(tm5_ctrl.r.model.getpos(),0,tm5_ctrl.r.model.fkine([-0.2000    0.3319   -0.8866    1.8326    2.1681   -1.1947    3.1416]),10);
    %Proceed towards UR3e
    tm5traj3 = tm5_ctrl.trap_ikcon(tm5_ctrl.r.model.getpos(),0,tm5_ctrl.r.model.fkine([ -0.2520    1.2427   -1.0751    1.3090    2.7336   -1.8326    3.1416]),50);
    tm5traj4 = tm5_ctrl.trap_ikcon(tm5_ctrl.r.model.getpos(),0,tm5_ctrl.r.model.fkine([ -0.1888    1.3369   -0.8238    1.5603    2.5451   -1.8326    3.1416]),50);
    %Return the tm5_ctrl toward the printers, activate the UR3e
    tm5traj5 = tm5_ctrl.quintic_ikcon(tm5_ctrl.r.model.getpos(),0,tm5_ctrl.r.model.fkine([-0.2000    0.1745   -0.7854    0.5236    0.1745    0.5236    3.1416]),100);
    trajectory2 = [tm5traj1;tm5traj2;tm5traj3;tm5traj4;tm5traj5];

    tm5traj1 = tm5_ctrl.quintic_ikcon(tm5_ctrl.r.model.getpos(),0,tm5_ctrl.r.model.fkine([-0.2000    0.1745   -0.7854    0.5236    0.1745    0.5236    3.1416]),200);
    trajectory3 = tm5traj1;

    urtraj1 = ur3e_ctrl.quintic_ikcon(ur3e_ctrl.r.model.getpos(),0,ur3e_ctrl.r.model.fkine([    2.7837   -0.6363    1.4316    3.8397   -1.5708         0]),100); 
    urtraj2 = ur3e_ctrl.quintic_ikcon(ur3e_ctrl.r.model.getpos(),0,ur3e_ctrl.r.model.fkine([1.7498   -0.9544    1.3521    3.8397   -1.5708         0]),50);
    urtraj3 = ur3e_ctrl.quintic_ikcon(ur3e_ctrl.r.model.getpos(),0,ur3e_ctrl.r.model.fkine([3.1416   -1.4316    1.3521    3.8397   -1.5708         0]),50);
    trajectory4 = [urtraj1;urtraj2;urtraj3];

    tm5_ctrl.executeTraj(trajectory1);
    tm5_ctrl.executeTraj(trajectory2);
    tm5_ctrl.executeTraj(trajectory3);
    ur3e_ctrl.executeTraj(trajectory4);


end