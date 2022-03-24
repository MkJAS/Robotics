clear all
close all
load('brick.mat');

getbrickcoord = cell(1,9);
Bricks = cell(9,2);
vertices = cell(1,9);

base1 = [0,0.52,0];
base2 = [0.4,-0.6,0];

%Calculate location of UR5 base such that the centre of the rail is at
%base2 and the rail is rotated so that its normal is in line with the base
%of the UR3
local = base2 - base1                   %get UR5 in UR3 local coord frame
a = 90 - abs(atand(local(2)/local(1)))  %angle of needed rotation
base2x = 0.4*cosd(a) + base2(1)         %the LinUR5 base is at the end of the rail so in order to compensate
base2y = 0.4*sind(a) + base2(2)         %for the centre of the rail being at base2 I move the true base 0.4
base2r = [base2x base2y 0];         %update base2
% 
%                 
% a = 90 - abs(atand(local(2)/local(1)))  %angle of needed rotation
% base1x = 0.4*cosd(a) + base1(1)         %the LinUR5 base is at the end of the rail so in order to compensate
% base1y = 0.4*sind(a) + base1(2)         %for the centre of the rail being at base2 I move the true base 0.4
% base1r = [base1x base1y 0];
robot1 = LinearUR3(transl(base1));
robot2 = LinearUR5(transl(base2r));
%%
robot2.model.base = robot2.model.base * troty(deg2rad(a));  %rotate base by a
% robot1.model.base = robot1.model.base * troty(deg2rad(a));

qr2 = zeros(1,7);         %
qr2(1,1) = -0.4;
qr2(1,6) = -(pi/2);
robot2.model.animate(qr2) %Move UR5 so its starts at centre of rail

qr1 = zeros(1,7);         %
qr1(1,1) = -0.4;
robot1.model.animate(qr1) 


%%
hold on
% Get brick positions and plot them
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
%

%animate UR3 moving to standing position
qr1 = zeros(1,7);
for i=0:-1:-90
       
    robot1.model.animate(qr1);
%     robot2.model.animate(qr2);
    qr1(1,3) = i*pi/180;
    qr1(1,5) = i*pi/180;
    qr1(1,6) = -(i*pi/180);
    pause(0.01);
    
end
ogq1 = robot1.model.getpos;
ogT1 = robot1.model.fkine(ogq1);
ogq2 = robot2.model.getpos;
ogT2 = robot2.model.fkine(ogq2);

%get and plot max reach radius in XY plane around UR3
qr1 = zeros(1,7);
qr2 = zeros(1,7);
qr1(1,5) = -pi/2;
qr2(1,5) = -pi/2;
qr2(1,3) = -pi/2;
qr2(1,1) = -0.4;
pos = robot1.model.fkine(qr1);
pos2 = robot2.model.fkine(qr2);
XYplane = zeros(3,(370/5));
XYplane2 = zeros(3,(370/5));
XYZ = pos(1:3,4);
XYZ2 = pos2(1:3,4);
XYplane(:,1) = XYZ;
XYplane2(:,1) = XYZ2;
index = 2;
for i=1:5:370
    
    pos = robot1.model.fkine(qr1);
    XYZ = pos(1:3,4);
    XYplane(:,index) = XYZ;
    qr1(1,2) = i*pi/180;
    pos2 = robot2.model.fkine(qr2);
    XYZ2 = pos2(1:3,4);
    XYplane2(:,index) = XYZ2;
    qr2(1,2) = i*pi/180;
    index = index + 1;
    pause(0.01);
    
end
maxXY = plot3(XYplane(1,:),XYplane(2,:),XYplane(3,:));
maxXY2 = plot3(XYplane2(1,:),XYplane2(2,:),XYplane2(3,:));

% Divi bricks between the 2 bots by proximity

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

% Get end goals

r = ((base1(1)-XYplane(1,1))^2 + (base1(2)-XYplane(2,1))^2)^0.5; %radius around UR3
[droppoints,m] = GetGoals(base1,base2,r,brick)
UR3drops = [droppoints(1,:);droppoints(2,:);droppoints(4,:);droppoints(7,:)];
UR5drops = [droppoints(3,:);droppoints(5,:);droppoints(6,:);droppoints(8,:);droppoints(9,:)];


%%
 if abs(m) == inf
    brickrot = trotz(pi/2);
    elseif m == 0
        brickrot = 1;
    else
        brickrot = trotz(-atan(m));
 end
for i=1:4
    g1 = robot1.model.getpos();
    g2 = robot2.model.getpos();
    T1 = transl(getbrickcoord{1,UR3bricks(i,1)}.x,getbrickcoord{1,UR3bricks(i,1)}.y,getbrickcoord{1,UR3bricks(i,1)}.z+0.2)*troty(pi);
    T2 = transl(getbrickcoord{1,UR5bricks(i,1)}.x,getbrickcoord{1,UR5bricks(i,1)}.y,getbrickcoord{1,UR5bricks(i,1)}.z+0.2)*troty(pi);
    MovetoPoint(robot1,robot2,T1,T2);
    
    T1 = transl(getbrickcoord{1,UR3bricks(i,1)}.x,getbrickcoord{1,UR3bricks(i,1)}.y,getbrickcoord{1,UR3bricks(i,1)}.z+brick.z)*troty(pi);
    T2 = transl(getbrickcoord{1,UR5bricks(i,1)}.x,getbrickcoord{1,UR5bricks(i,1)}.y,getbrickcoord{1,UR5bricks(i,1)}.z+brick.z+0.08)*troty(pi);
%     if i==1
%         g1 = [4.2147   -1.0467   -2.6180   -3.1280   -1.5784   -3.6394];
%         g2 = robot2.model.getpos();
%     end
    MovetoPoint(robot1,robot2,T1,T2,g1,g2); 
    
    b1 = transl(UR3drops(i,:))*troty(pi) * brickrot;
    b2 = transl(UR5drops(i,:))*transl(0,0,brick.z+0.02)*troty(pi) * brickrot;
    [tr,tr2] = MoveBrick(robot1,robot2,b1,b2,Bricks,UR3bricks(i,1),UR5bricks(i,1),vertices);
    
    % 'Drop' bricks
    tr(3,4) = UR3drops(i,3);
    tr2(3,4) = UR5drops(i,3);
    transformedVertices = [vertices{1,UR3bricks(i,1)},ones(size(vertices{1,UR3bricks(i,1)},1),1)] * tr';
    transformedVertices2 = [vertices{1,UR5bricks(i,1)},ones(size(vertices{1,UR5bricks(i,1)},1),1)] * tr2';
    set(Bricks{UR3bricks(i,1),1},'Vertices',transformedVertices(:,1:3));
    set(Bricks{UR5bricks(i,1),1},'Vertices',transformedVertices2(:,1:3));
    drawnow();

end

    %UR5 has one more brick to move
    T2 = transl(getbrickcoord{1,UR5bricks(5,1)}.x,getbrickcoord{1,UR5bricks(5,1)}.y,getbrickcoord{1,UR5bricks(5,1)}.z+0.2)*troty(pi);
    MovetoPoint(robot1,robot2,ogT1,T2,ogq1,robot2.model.getpos());
    
    T2 = transl(getbrickcoord{1,UR5bricks(5,1)}.x,getbrickcoord{1,UR5bricks(5,1)}.y,getbrickcoord{1,UR5bricks(5,1)}.z+brick.z+0.08)*troty(pi);
    MovetoPoint(robot1,robot2,ogT1,T2,ogq1,robot2.model.getpos());  
    
    b2 = transl(UR5drops(5,:))*transl(0,0,brick.z+0.02)*troty(pi) * brickrot;
    [tr,tr2] = MoveBrick(robot1,robot2,0,b2,Bricks,0,UR5bricks(5,1),vertices);
    
   
    tr2(3,4) = UR5drops(5,3);
    transformedVertices2 = [vertices{1,UR5bricks(5,1)},ones(size(vertices{1,UR5bricks(5,1)},1),1)] * tr2';
    set(Bricks{UR5bricks(5,1),1},'Vertices',transformedVertices2(:,1:3));
    drawnow();
    
    %Move to original positions
    MovetoPoint(robot1,robot2,ogT1,ogT2,ogq1,ogq2); 
    
    %End
    
    
%% Move effector to brick
% % try delete(maxXY); end;
%  
% T2 = transl(getbrickcoord{1,UR3bricks(1,1)}.x,getbrickcoord{1,UR3bricks(1,1)}.y,getbrickcoord{1,UR3bricks(1,1)}.z+0.2)*troty(pi);
% T22 = transl(getbrickcoord{1,UR5bricks(1,1)}.x,getbrickcoord{1,UR5bricks(1,1)}.y,getbrickcoord{1,UR5bricks(1,1)}.z+0.2)*troty(pi);
% MovetoPoint(robot1,robot2,T2,T22);
% 
% 
% T3 = transl(getbrickcoord{1,UR3bricks(1,1)}.x,getbrickcoord{1,UR3bricks(1,1)}.y,getbrickcoord{1,UR3bricks(1,1)}.z+brick.z)*troty(pi);
% T33 = transl(getbrickcoord{1,UR5bricks(1,1)}.x,getbrickcoord{1,UR5bricks(1,1)}.y,getbrickcoord{1,UR5bricks(1,1)}.z+brick.z)*troty(pi);
% 
% MovetoPoint(robot1,robot2,T3,T33);
% %% Move brick to end goal
% 
% if abs(m) == inf
%     brickrot = trotz(pi/2);
% elseif m == 0
%     brickrot = 1;
% else
%     brickrot = trotz(-atan(m));
% end
% 
% 
% brick1drop = transl(droppoints(1,:))*troty(pi) * brickrot;
% MoveBrick(robot1,robot2,brick1drop,T22,Bricks,UR3bricks(1,1),UR5bricks(1,1),vertices,brick);
% 
% % 
% % T2 = transl(getbrickcoord{1,UR3bricks(1,1)}.x,getbrickcoord{1,UR3bricks(1,1)}.y,getbrickcoord{1,UR3bricks(1,1)}.z+0.2)*troty(pi);
% % T22 = transl(getbrickcoord{1,UR5bricks(1,1)}.x,getbrickcoord{1,UR5bricks(1,1)}.y,getbrickcoord{1,UR5bricks(1,1)}.z+0.2)*troty(pi);
% % MovetoPoint(robot1,robot2,T2,T22);
% 
% 
% %% Move brick to end goal
% 
% T3 = transl(getbrickcoord{1,UR3bricks(2,1)}.x,getbrickcoord{1,UR3bricks(2,1)}.y,getbrickcoord{1,UR3bricks(2,1)}.z+brick.z)*troty(pi);
% T33 = transl(getbrickcoord{1,UR5bricks(2,1)}.x,getbrickcoord{1,UR5bricks(2,1)}.y,getbrickcoord{1,UR5bricks(2,1)}.z+brick.z)*troty(pi);
% 
% MovetoPoint(robot1,robot2,T3,T33);
% 
% %%
% if abs(m) == inf
%     brickrot = trotz(pi/2);
% elseif m == 0
%     brickrot = 1;
% else
%     brickrot = trotz(-atan(m));
% end
% 
% 
% brick1drop = transl(droppoints(2,:))*troty(pi) * brickrot;
% MoveBrick(robot1,robot2,brick1drop,T33,Bricks,UR3bricks(2,1),UR5bricks(2,1),vertices,brick);
% 






