close all
clear all

base1 = transl(-1, 0, 0);
base2 = transl(1, 0, 0);
robot = UR3(base1);
robot_2 = UR3(base2);
robot.model.teach()
q = zeros(1,6);
% robot.model.plot(q,'workspace',[-2 2 -2 2 -0.05 2]);
% robot.PlotAndColourRobot(self)
hold on
PlaceObject('brick.ply',[0,0,0])

