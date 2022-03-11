clear all
close all
% Link('theta',__,'d',__,'a',__,'alpha',__,'offset',__,'qlim',[ ... ])
figure(1)
robot = UR5;


L1 = robot.model.links(1,1)

L2 = robot.model.links(1,2)

L3 = robot.model.links(1,3)

L4 = robot.model.links(1,4)

L5 = robot.model.links(1,5)

L6 = robot.model.links(1,6)


myRobot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'UR5')

q = zeros(1,6)
figure(2)
myRobot.plot(q)

myRobot.teach
