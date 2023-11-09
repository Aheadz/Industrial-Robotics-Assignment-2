clear
clc
hold on
g1 = g_2F85;
g2 = g_2F85;
g2.model.base = g2.model.base.T*troty(-pi);
g1.model.animate([0,0,0]);
g2.model.animate([0,0,0]);
gripControl(0, g1, g2)

function gripControl(state, g1, g2)
steps = 100;
if state == 0;
    curPos = g1.model.getpos();
    joint1 = linspace(curPos([1]),0,steps)';
    joint2 = linspace(0,0,steps)';
    joint3 = linspace(curPos([3]),0,steps)';
    qTraj = ([joint1,joint2,joint3]);
    for i = 1:steps
        g1.model.animate(qTraj(i,:));
        g2.model.animate(qTraj(i,:));
    end
else state == 1;
    curPos = g1.model.getpos();
    joint1 = linspace(curPos([1]),0.5,steps)';
    joint2 = linspace(0,0,steps)';
    joint3 = linspace(curPos([3]),-0.5,steps)';
    qTraj = ([joint1,joint2,joint3]);
    for i = 1:steps
        g1.model.animate(qTraj(i,:));
        g2.model.animate(qTraj(i,:));
    end
end
end