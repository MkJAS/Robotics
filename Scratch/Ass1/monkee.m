close all
clear all
load('brick.mat')

base1 = transl(-1, 0, 0);
base2 = transl(1, 0, 0);
robot = UR3(base1);

% robot_2 = UR3(base2);
% robot.model.teach()
q = zeros(1,6);
% robot.model.plot(q,'workspace',[-2 2 -2 2 -0.05 2]);
% robot.PlotAndColourRobot(self)
hold on
% PlaceObject('brick.ply',[0,0,0])

q(1,2) = pi/2;
% robot.model.plot(q);

Q = q;
ogpos = robot.model.fkine(zeros(1,6));
T1 = transl(ogpos(1,4),ogpos(2,4),ogpos(3,4));
q1 = robot.model.ikcon(T1)

T2 = transl(-0.6,0.4,0.4);
q2 = robot.model.ikcon(T2)

steps = 50;
s = lspb(0,1,steps);
qMatrix = nan(steps,6);
for i = 1:steps
   qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
end

figure(1)
for i=1:steps
robot.model.animate(qMatrix(i,:));
pause(0.1)
end

velocity = zeros(steps,6);
acceleration = zeros(steps,6);
for i = 2:steps
   velocity(i,:) = qMatrix(i,:) - qMatrix(i-1,:);
   acceleration(i,:) = velocity(i,:) - velocity(i-1,:);
end

% for i = 1:6
%    figure(2)
%    subplot(3,2,i)
%    plot(qMatrix(:,i),'k','LineWidth',1)
%    title(['Joint ', num2str(i)])
%    xlabel('Step')
%    ylabel('Joint Angle (rad)')
%    refline(0,robot.model.qlim(i,1)) % Reference line on the lower joint limit for joint i
%    refline(0,robot.model.qlim(i,2)) % Reference line on the upper joint limit for joint i
% 
%    figure(3)
%    subplot(3,2,i)
%    plot(velocity(:,i),'k','LineWidth',1)
%    title(['Joint ', num2str(i)])
%    xlabel('Step')
%    ylabel('Joint Velocity')
%  
%    figure(4)
%    subplot(3,2,i)
%    plot(acceleration(:,i),'k','LineWidth',1)
%    title(['Joint ', num2str(i)])
%    xlabel('Step')
%    ylabel('Joint Acceleration')
%  end
%%

% Q = robot.model.ikine(transl(-0.8,0.1,0),Q,[1,1,1,0,0,0]);
% robot.model.animate(Q);
% pos = robot.fkine(Q);
pos = robot.model.fkine(q)
XYplane = zeros(3,(370/5));
XYZ = pos(1:3,4);
XYplane(:,1) = XYZ;
index = 2;
for i=1:5:370
    
    
    pos = robot.model.fkine(q);
    robot.model.animate(q);
    XYZ = pos(1:3,4);
    XYplane(:,index) = XYZ;
    q(1,1) = i*pi/180;
    index = index + 1;
    pause(0.01);
    
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


