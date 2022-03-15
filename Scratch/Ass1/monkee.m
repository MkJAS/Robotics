close all
clear all

base1 = transl(-1, 0, 0);
base2 = transl(1, 0, 0);
robot = UR3(base1);
% robot_2 = UR3(base2);
% robot.model.teach()
q = zeros(1,6);
% robot.model.plot(q,'workspace',[-2 2 -2 2 -0.05 2]);
% robot.PlotAndColourRobot(self)
hold on
PlaceObject('brick.ply',[0,0,0])

q(1,2) = pi/2;
% robot.model.plot(q);


Q = q;
Q = robot.model.ikine(transl(-0.25,0.1,0),Q,[1,1,1,0,0,0]);
robot.model.plot(Q);
% pos = robot.fkine(Q);
pos = robot.model.fkine(q)
XYplane = zeros(3,(370/5));
XYZ = pos(1:3,4);
XYplane(:,1) = XYZ;
index = 2;
for i=1:5:370
    
    
    pos = robot.model.fkine(q);
%     robot.model.plot(q);
    XYZ = pos(1:3,4);
    XYplane(:,index) = XYZ;
    q(1,1) = i*pi/180;
    index = index + 1;
    
end
%%
q = zeros(1,6);

pos = robot.model.fkine(q)
XZplane = zeros(3,(370/5));
XYZ = pos(1:3,4);
XZplane(:,1) = XYZ;
index = 2;
for i=1:5:370
    
    
    pos = robot.model.fkine(q);
%     robot.model.plot(q);
    XYZ = pos(1:3,4);
    XZplane(:,index) = XYZ;
    q(1,2) = i*pi/180;
    index = index + 1;
    
end


plot3(XYplane(1,:),XYplane(2,:),XYplane(3,:));
plot3(XZplane(1,:),XZplane(2,:),XZplane(3,:));




