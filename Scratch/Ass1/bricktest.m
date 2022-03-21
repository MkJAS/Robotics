clear all
close all
load('brick.mat');

getbrickcoord = cell(1,9);
Bricks = cell(9,2);
vertices = cell(1,9);
steps = 50;
s = lspb(0,1,steps);
q1Matrix = nan(steps,6);
q2Matrix = nan(steps,7);

base1 = [0,0.4,0];
base2 = [0.4,-0.6,0];
robot1 = UR3(transl(base1));
robot2 = LinearUR5(transl(base2));

qr2 = zeros(1,7);         %
qr2(1,1) = -0.4;
robot2.model.animate(qr2) %Move UR5 so its starts at centre of rail
hold on
%% Get brick positions and plot them
brick1pos = [-0.4,0.2,0];
brick2pos = [0.4,0.2,0];
brick3pos = [-0.4,-0.2,0];
brick4pos = [0.4,-0.2,0];
brick5pos = [0,0,0];
brick6pos = [0.3,0,0];
brick7pos = [-0.3,0,0];
brick8pos = [0,-0.2,0];
brick9pos = [0,0.2,0];

%store brick positions
brickposes = {brick1pos brick2pos brick3pos brick4pos brick5pos brick6pos brick7pos...
                brick8pos brick9pos}; 

%store bricks and their corresponsing positions in 9x2 array
for i=1:9
    Bricks{i,1} = PlaceObject('brick.ply');
    Bricks{i,2} = brickposes{1,i};
end



for i=1:9
    vertices(1,i) = {get(Bricks{i,1},'Vertices')};
    transformedVertices = [vertices{1,i},ones(size(vertices{1,i},1),1)] * transl(Bricks{i,2})';
    set(Bricks{i,1},'Vertices',transformedVertices(:,1:3));
end

%Store brick centre coords in array of structs??? Not sure why I did this
%tbh
gripbrick = struct('x',0,'y',0,'z',0);

for i=1:9
    gripbrick.x = Bricks{i,2}(1,1);
    gripbrick.y = Bricks{i,2}(1,2);
    gripbrick.z = Bricks{i,2}(1,3);
    getbrickcoord{1,i} = gripbrick;
end


%animate UR3 moving to standing position
qr1 = zeros(1,6);
for i=0:-1:-90
       
    robot1.model.animate(qr1);
    qr1(1,2) = i*pi/180;
    pause(0.01);
    
end
%get and plot max reach radius in XY plane around UR3
qr1 = zeros(1,6);
qr1(1,4) = -pi/2;
pos = robot1.model.fkine(qr1);
XYplane = zeros(3,(370/5));
XYZ = pos(1:3,4);
XYplane(:,1) = XYZ;
index = 2;
for i=1:5:370
    
    
    pos = robot1.model.fkine(qr1);
    XYZ = pos(1:3,4);
    XYplane(:,index) = XYZ;
    qr1(1,1) = i*pi/180;
    index = index + 1;
    pause(0.01);
    
end
maxXY = plot3(XYplane(1,:),XYplane(2,:),XYplane(3,:));
%% Divi bricks between the 2 bots by proximity

distances = zeros(9,2);
bricknum = 1;
for i=1:9
    distances(i,:) = [(((base1(1) - brickposes{1,i}(1,1))^2 + (base1(2) - brickposes{1,i}(1,2))^2)^0.5) bricknum];
    bricknum = bricknum + 1;
end

distances = sortrows(distances,1);
%Assign 4 bricks closest to UR3 to UR3
UR3bricks = distances(1:4,2);
%Rest to UR5
UR5bricks = distances(5:9,2);





%% Move effector to brick
% try delete(maxXY); end;

% MovetoBrick(robot1,robot2,UR3bricks,UR5bricks,getbrickcoord);

q1r1 = robot1.model.getpos();
q1r2 = robot2.model.getpos();

T2 = transl(getbrickcoord{1,UR3bricks(1,1)}.x,getbrickcoord{1,UR3bricks(1,1)}.y,getbrickcoord{1,UR3bricks(1,1)}.z+0.2)*troty(pi);
q2r1 = robot1.model.ikcon(T2);
T22 = transl(getbrickcoord{1,UR5bricks(1,1)}.x,getbrickcoord{1,UR5bricks(1,1)}.y,getbrickcoord{1,UR5bricks(1,1)}.z+0.2)*troty(pi);
q2r2 = robot2.model.ikcon(T22);


for i = 1:steps
   q1Matrix(i,:) = (1-s(i))*q1r1 + s(i)*q2r1;
   q2Matrix(i,:) = (1-s(i))*q1r2 + s(i)*q2r2;
end


for i=1:steps
    robot1.model.animate(q1Matrix(i,:));
    robot2.model.animate(q2Matrix(i,:));
    pause(0.1)
end

%% Get end goals

A = base1(1:2);
B = base2(1:2);
B(1) = B(1) - 0.4;
V = B - A;
midV = A;
normal = [ V(2), -V(1)];
% normal2 = [-V(2),  V(1)];

point = [A(1)+normal(1) A(2)+normal(2)];

theta = atan((A(2)-point(2))/(A(1)-point(1)))

r = ((base1(1)-XYplane(1,1))^2 + (base1(2)-XYplane(2,1))^2)^0.5 %radius around UR3

x1 = r*0.8*cos(theta)+A(1) %distance from UR3 to 0.8r
y1 = r*0.8*sin(theta)+A(2)


plot([A(1), B(1)], [A(2), B(2)], 'r');
% axis([-5 5 -5 5]);
hold on
plot([midV(1), midV(1) + normal(1)], [midV(2), midV(2) + normal(2)], 'g');
plot([A(1), x1], [A(2), y1], 'b');

m = (B(2)-A(2))/(A(1)-B(1))

d = r*0.8 - brick.x/2


x = (d^2/(1+m^2))^0.5 + x1
if abs(m) == inf
    if base1(2) > base2(2)
        y = base1(2) - d
    
    else
        y = base1(2) + d
    end
elseif m == 0
        y = y1;
        if base1(1) < base2(1)
            x =  base1(1) + d
        else
            x = base1(1) - d
        end    
else
    y = (d^2-x^2)^0.5
    
end


plot([x1 x],[y1 y])

% m = 2.3;          %get a point (x,y) at a distance of d from another
%                       point (x1,y1) given a known gradient m
% d = 2.3;
% 
% x1 = 1.2;
% y1 = m*x1;
% 
% 
% x = (d^2/(1+m^2))^0.5 + x1
% 
% y = m*x
% 
% plot([0 x1],[0 y1])
% axis([0 6 0 6])
% hold on
% plot([x1 x],[y1 y])


%% Move brick to end goal

q1r1 = q2r1;

T3 = transl(getbrickcoord{1,UR3bricks(1,1)}.x,getbrickcoord{1,UR3bricks(1,1)}.y,getbrickcoord{1,UR3bricks(1,1)}.z+brick.z)*troty(pi);
q2r1 = robot1.model.ikcon(T3);

for i = 1:steps
   q1Matrix(i,:) = (1-s(i))*q1r1 + s(i)*q2r1;
end


for i=1:steps
    robot1.model.animate(q1Matrix(i,:));
    pause(0.1)
end
%%

q1r1 = q2r1;
brick1drop = transl(x1,y1,brick.z)*troty(pi);
T4 = brick1drop;
q2r1 = robot1.model.ikcon(T4);
for i = 1:steps
   q1Matrix(i,:) = (1-s(i))*q1r1 + s(i)*q2r1;
end
    transformedVertices = [vertices{1,1},ones(size(vertices{1,1},1),1)] * trotz(pi/2)';
    set(Bricks{9,1},'Vertices',transformedVertices(:,1:3));
%Animate movement along side brick animation
for i=1:steps
    robot1.model.animate(q1Matrix(i,:));
    tr = robot1.model.fkine(q1Matrix(i,:)); %* transl(0,0,0.08); %transl because the end of the effector is for some reason not where the visual end is???
    transformedVertices = [vertices{1,1},ones(size(vertices{1,1},1),1)] * tr';
    set(Bricks{UR3bricks(1,1),1},'Vertices',transformedVertices(:,1:3));
    drawnow();
    pause(0.1)
end
%'Drop' brick
transformedVertices = [vertices{1,1},ones(size(vertices{1,1},1),1)] * transl(brick1drop(1,4),brick1drop(2,4),0)';
set(Bricks{UR3bricks(1,1),1},'Vertices',transformedVertices(:,1:3));
drawnow();
%%











% %% Get 2nd Brick
% 
% 
% T1 = pos;
% q1 = robot2.model.getpos();
% 
% T2 = transl(getbrickcoord(1,2).x,getbrickcoord(1,2).y,getbrickcoord(1,2).z+0.25)*troty(pi);
% q2 = robot2.model.ikcon(T2);
% 
% 
% for i = 1:steps
%    q2Matrix(i,:) = (1-s(i))*q1 + s(i)*q2;
% end
% 
% 
% for i=1:steps
% robot2.model.animate(q2Matrix(i,:));
% pause(0.1)
% end
% 
% q1 = q2;
% 
% T3 = transl(getbrickcoord(1,2).x,getbrickcoord(1,2).y,getbrickcoord(1,2).z+brick.z+0.1)*troty(pi);
% q2 = robot2.model.ikcon(T3);
% 
% for i = 1:steps
%    q2Matrix(i,:) = (1-s(i))*q1 + s(i)*q2;
% end
% 
% 
% for i=1:steps
% robot2.model.animate(q2Matrix(i,:));
% pause(0.1)
% end
% 
% 
% q1 = q2;
% brick2drop = brick1drop;
% brick2drop(2,4) = brick2drop(2,4) - brick.y;
% brick2drop(3,4) = brick2drop(3,4) + brick.z;
% T4 = brick2drop;
% q2 = robot2.model.ikcon(T4);
% %%
% for i = 1:steps
%    q2Matrix(i,:) = (1-s(i))*q1 + s(i)*q2;
% end
% 
% 
% for i=1:steps
%     robot2.model.animate(q2Matrix(i,:));
%     tr = robot2.model.fkine(q2Matrix(i,:)) * transl(0,0,0.08);
%     transformedVertices = [vertices{1,2},ones(size(vertices{1,2},1),1)] * tr';
%     set(Bricks{2,1},'Vertices',transformedVertices(:,1:3));
%     drawnow();
%     pause(0.1)
% end
% %%
% transformedVertices = [vertices{1,1},ones(size(vertices{1,1},1),1)] * transl(brick2drop(1,4),brick2drop(2,4),0)';
% set(brick2,'Vertices',transformedVertices(:,1:3));
% drawnow();
% 
% 
% 
% 