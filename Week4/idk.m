clear all
close all

L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
robot = SerialLink([L1 L2 L3],'name','myRobot');

robot.base = troty(pi);

q = zeros(1,3);
robot.plot(q,'workspace',[-2 2 -2 2 -0.05 2],'scale',0.5);
robot.teach;


%% 

Q = robot.ikine(transl(-0.75,-0.5,0),q,[1,1,0,0,0,0]);

robot.plot(Q)
hold on
ppos = robot.fkine(Q)
pXYZ = ppos(1:3,4);

for i=-0.5:0.05:0.5
    
%     Q = robot.ikine(transl(-0.75,i,0),Q,[1,1,0,0,0,0]);
%     robot.animate(Q);
%     drawnow();
    
    Q = robot.ikine(transl(-0.75,i,0),Q,[1,1,0,0,0,0]);
    robot.plot(Q);
    pos = robot.fkine(Q);
    XYZ = pos(1:3,4);
    Line = plot3([pXYZ(1,1),XYZ(1,1)],[pXYZ(2,1),XYZ(2,1)],[pXYZ(3,1),XYZ(3,1)],'-b');
    pXYZ = XYZ;    
    
end
%% 
try delete(Line); end
Q = robot.ikine(transl(-0.5,0,0),Q,[1,1,0,0,0,0]);
ppos = robot.fkine(Q)
pXYZ = ppos(1:3,4);



for i=-0.5:0.05:0.5
    
    y = (0.5^2 - i^2);
    if y<0
        y = y*-1;
        y = y^0.5;
        y = y*-1;
    end
    if y>0
        y = y^0.5;
    end
    
    Q = robot.ikine(transl(i,y,0),Q,[1,1,0,0,0,0]);
    robot.animate(Q);
    pos = robot.fkine(Q);
    XYZ = pos(1:3,4);
    Line = plot3([pXYZ(1,1),XYZ(1,1)],[pXYZ(2,1),XYZ(2,1)],[pXYZ(3,1),XYZ(3,1)],'-b');
    pXYZ = XYZ; 
    drawnow();
end
for i=0.5:-0.05:-0.5
    
    y = (0.5^2 - i^2);
    if y<0
        y = y*-1;
        y = y^0.5;
        y = y*-1;
    end
    if y>0
        y = y^0.5;
    end
    y = y*-1;
    
    Q = robot.ikine(transl(i,y,0),Q,[1,1,0,0,0,0]);
    robot.animate(Q);
    pos = robot.fkine(Q);
    XYZ = pos(1:3,4);
    Line = plot3([pXYZ(1,1),XYZ(1,1)],[pXYZ(2,1),XYZ(2,1)],[pXYZ(3,1),XYZ(3,1)],'-b');
    pXYZ = XYZ; 
    drawnow();
end
%%
clear all
close all

mesh_h = PlaceObject('penVertexColour.ply');
axis equal
vertices = get(mesh_h,'Vertices');

transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0,0,0.1)';
set(mesh_h,'Vertices',transformedVertices(:,1:3));

transformedVertices = [vertices,ones(size(vertices,1),1)] * trotx(pi/2)';
set(mesh_h,'Vertices',transformedVertices(:,1:3));

mdl_planar3
hold on
p3.plot([0,0,0])
p3.delay = 0;
% ppos = p3.fkine([0 0 0])
% pXYZ = ppos(1:3,4);

axis([-3,3,-3,3,-0.5,8])
%%

for i = -pi/4:0.01:pi/4
    p3.animate([i,i,i])
    tr = p3.fkine([i,i,i]);
%     XYZ = tr(1:3,4);
%     Line = plot3([pXYZ(1,1),XYZ(1,1)],[pXYZ(2,1),XYZ(2,1)],[pXYZ(3,1),XYZ(3,1)],'-b');
%     pXYZ = XYZ;
    transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
    set(mesh_h,'Vertices',transformedVertices(:,1:3));
    drawnow();
    pause(0.01);
end
%%
close all
clear all

mdl_puma560

T1 = transl(0.5,-0.4,0.5);
q1 = p560.ikine(T1);

T2 = transl(0.5,0.4,0.1);
q2 = p560.ikine(T2); 

steps = 50;

qMatrix = jtraj(q1,q2,steps);

figure(1)
p560.plot(qMatrix,'trail','r-')

s = lspb(0,1,steps);
qMatrix = nan(steps,6);
for i = 1:steps
   qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
end
figure(1)
p560.plot(qMatrix,'trail','r-')















