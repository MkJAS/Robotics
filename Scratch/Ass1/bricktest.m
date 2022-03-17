clear all
close all
load('brick.mat');
%%
base1 = transl(-1,0,0);
robot1 = LinearUR5(base1);
q = zeros(1,7);
q(1,1) = -0.4;
robot1.model.animate(q)
hold on
brick2pos = [-1,-0.5,0];
brick1pos = [-0.6,0.2,0];
brick1 = PlaceObject('brick.ply');
brick2 = PlaceObject('brick.ply');
Bricks = {brick1 brick1pos;brick2 brick2pos};
brickposes = {brick1pos brick2pos};

steps = 50;
s = lspb(0,1,steps);
qMatrix = nan(steps,7);

numB = size(Bricks); %number of bricks
for i=1:numB(1,1)
    vertices(1,i) = {get(Bricks{i,1},'Vertices')};
    transformedVertices = [vertices{1,i},ones(size(vertices{1,i},1),1)] * transl(Bricks{i,2})';
    set(Bricks{i,1},'Vertices',transformedVertices(:,1:3));
end



% getbrickcoord = zeros(1,numB);
gripbrick = struct('x',0,'y',0,'z',0);
for i=1:numB(1,1)
    gripbrick.x = Bricks{i,2}(1,1);
    gripbrick.y = Bricks{i,2}(1,2);
    gripbrick.z = Bricks{i,2}(1,3);
    getbrickcoord(1,i) = gripbrick;
end

q = zeros(1,7);
q(1,1) = -0.4;
for i=0:-1:-90
       
    pos = robot1.model.fkine(q);
    robot1.model.animate(q);
    q(1,3) = i*pi/180;
    pause(0.01);
    
end

q = zeros(1,7);
q(1,1) = -0.4;
q(1,5) = -pi/2;
pos = robot1.model.fkine(q)
XYplane = zeros(3,(370/5));
XYZ = pos(1:3,4);
XYplane(:,1) = XYZ;
index = 2;
for i=1:5:370
    
    
    pos = robot1.model.fkine(q);
%     robot1.model.animate(q);
    XYZ = pos(1:3,4);
    XYplane(:,index) = XYZ;
    q(1,2) = i*pi/180;
    index = index + 1;
    pause(0.01);
    
end
try delete(maxXY); end;
maxXY = plot3(XYplane(1,:),XYplane(2,:),XYplane(3,:));
%%

% ogpos = robot1.model.fkine(robot1.model.getpos())
% 
% T1 = transl(ogpos(1,4),ogpos(2,4),ogpos(3,4));
q1 = robot1.model.getpos();

T2 = transl(-0.6,getbrickcoord(1,1).y,getbrickcoord(1,1).z+0.2)*troty(pi);
q2 = robot1.model.ikcon(T2);


for i = 1:steps
   qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
end


for i=1:steps
    robot1.model.animate(qMatrix(i,:));
    pause(0.1)
end

% q1 = robot1.model.getpos();
% 
% T2 = transl(getbrickcoord(1,1).x,getbrickcoord(1,1).y,getbrickcoord(1,1).z+0.25)*troty(pi);
% q2 = robot1.model.ikcon(T2,q1);
% 
% 
% for i = 1:steps
%    qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
% end
% 
% 
% for i=1:steps
%     robot1.model.animate(qMatrix(i,:));
%     pause(0.1)
% end
%%

q1 = q2;

T3 = transl(getbrickcoord(1,1).x,getbrickcoord(1,1).y,getbrickcoord(1,1).z+brick.z+0.05)*troty(pi);
q2 = robot1.model.ikcon(T3);

for i = 1:steps
   qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
end


for i=1:steps
robot1.model.animate(qMatrix(i,:));
pause(0.1)
end


q1 = q2;
brick1drop = transl(-1,0.5,brick.z+0.1)*troty(pi);
T4 = brick1drop;
q2 = robot1.model.ikcon(T4);
for i = 1:steps
   qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
end


for i=1:steps
    robot1.model.animate(qMatrix(i,:));
    tr = robot1.model.fkine(qMatrix(i,:)) * transl(0,0,0.08); %transl because the end of the effector is for some reason not where the visual end is???
    transformedVertices = [vertices{1,1},ones(size(vertices{1,1},1),1)] * tr';
    set(brick1,'Vertices',transformedVertices(:,1:3));
    drawnow();
    pause(0.1)
end

transformedVertices = [vertices{1,1},ones(size(vertices{1,1},1),1)] * transl(brick1drop(1,4),brick1drop(2,4),0)';
set(brick1,'Vertices',transformedVertices(:,1:3));
drawnow();


q1 = q2;
%%
% T5 = tr/trotz(pi/2);
% q2(1,6) = q2(1,6) + pi/2;
% pos = tr;
% for i=0:9:90
%        
%     
%     robot1.model.animate(q1);
%     transformedVertices = [vertices{1,1},ones(size(vertices{1,1},1),1)] * pos';
%     set(brick1,'Vertices',transformedVertices(:,1:3));
%     drawnow();
%    
%     q1(1,6) = i*pi/180;
%     pos = robot1.model.fkine(q1);
%     pause(0.01);
%     
% end

% for i = 1:steps
%    qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
% end
% for i=1:steps
%     robot1.model.animate(qMatrix(i,:));
%     tr = robot1.model.fkine(qMatrix(i,:));
%     transformedVertices = [vertices{1,1},ones(size(vertices{1,1},1),1)] * tr';
%     set(brick1,'Vertices',transformedVertices(:,1:3));
%     drawnow();
%     pause(0.1)
% end
% % 
% % 
% 
% tr = tr * trotz(pi/2);
% transformedVertices = [vertices{1,1},ones(size(vertices{1,1},1),1)] * tr';
% set(brick1,'Vertices',transformedVertices(:,1:3));
% drawnow();



%% Get 2nd Brick


T1 = pos;
q1 = robot1.model.getpos();

T2 = transl(getbrickcoord(1,2).x,getbrickcoord(1,2).y,getbrickcoord(1,2).z+0.25)*troty(pi);
q2 = robot1.model.ikcon(T2);


for i = 1:steps
   qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
end


for i=1:steps
robot1.model.animate(qMatrix(i,:));
pause(0.1)
end

q1 = q2;

T3 = transl(getbrickcoord(1,2).x,getbrickcoord(1,2).y,getbrickcoord(1,2).z+brick.z+0.1)*troty(pi);
q2 = robot1.model.ikcon(T3);

for i = 1:steps
   qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
end


for i=1:steps
robot1.model.animate(qMatrix(i,:));
pause(0.1)
end


q1 = q2;
brick2drop = brick1drop;
brick2drop(2,4) = brick2drop(2,4) - brick.y;
brick2drop(3,4) = brick2drop(3,4) + brick.z;
T4 = brick2drop;
q2 = robot1.model.ikcon(T4);
%%
for i = 1:steps
   qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
end


for i=1:steps
    robot1.model.animate(qMatrix(i,:));
    tr = robot1.model.fkine(qMatrix(i,:)) * transl(0,0,0.08);
    transformedVertices = [vertices{1,2},ones(size(vertices{1,2},1),1)] * tr';
    set(Bricks{2,1},'Vertices',transformedVertices(:,1:3));
    drawnow();
    pause(0.1)
end
%%
transformedVertices = [vertices{1,1},ones(size(vertices{1,1},1),1)] * transl(brick2drop(1,4),brick2drop(2,4),0)';
set(brick2,'Vertices',transformedVertices(:,1:3));
drawnow();




