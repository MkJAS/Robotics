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
local = base2 - base1;                   %get UR5 in UR3 local coord frame
a = 90 - abs(atand(local(2)/local(1)));  %angle of needed rotation
base2x = 0.4*cosd(a) + base2(1);         %the LinUR5 base is at the end of the rail so in order to compensate
base2y = 0.4*sind(a) + base2(2);         %for the centre of the rail being at base2 I move the true base 0.4
base2r = [base2x base2y 0];         %update base2
% 
%                 
% a = 90 - abs(atand(local(2)/local(1)))  %angle of needed rotation
% base1x = base1(1)         %the LinUR5 base is at the end of the rail so in order to compensate
% base1y = 0.4 + base1(2)         %for the centre of the rail being at base2 I move the true base 0.4
% base1r = [base1x base1y 0];
robot1 = LinearUR3(transl(base1));
robot2 = LinearUR5(transl(base2r));

robot2.model.base = robot2.model.base * troty(deg2rad(a));  %rotate base by a
robot1.model.base = robot1.model.base * troty(deg2rad(a));

qr2 = zeros(1,7);         %
qr2(1,1) = -0.4;
qr2(1,6) = -(pi/2);
robot2.model.animate(qr2) %Move UR5 so its starts at centre of rail

qr1 = zeros(1,7);         %
qr1(1,1) = -0.4;
robot1.model.animate(qr1) 
hold on
PlaceObject('cage.ply')
axis([-2.5 2.5 -2 2 -0.01 2])
surf([-2,-2;2,2],[-2,2;-2,2],[0,0;0,0],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
% surf([-1.8,-1.8;-1.8,-1.8],[-1.8,1.8;-1.8,1.8],[0.01,0.01;1,1],'CData',imread('Screenshot_4.jpg'),'FaceColor','texturemap');
% surf([1.8,1.8;1.8,1.8],[-1.8,1.8;-1.8,1.8],[0.01,0.01;1,1],'CData',imread('Screenshot_4.jpg'),'FaceColor','texturemap');

%%

hold on
% Get brick positions and plot them
brick1pos = [0.35,0.8,0];
brick2pos = [0.4,0.2,0];
brick3pos = [-0.4,-0.2,0];
brick4pos = [0.4,-0.2,0];
brick5pos = [0,0,0];
brick6pos = [0.3,0,0];
brick7pos = [-0.3,0,0];
brick8pos = [0,-0.2,0];
brick9pos = [0.35,0.5,0];

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
% qr2(1,1) = -0.4;
pos = robot1.model.fkine(qr1);
pos2 = robot2.model.fkine(qr2);
XYplane = zeros(3,(370/5));
XYplane2 = zeros(3,(370/5));
XYZ = pos(1:3,4);
XYZ2 = pos2(1:3,4);
XYplane(:,1) = XYZ;
XYplane2(:,1) = XYZ2;
index = 2;
for j=-0.8:0.4:0
    qr1(1,1) = j;
    qr2(1,1) = j;
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
end
maxXY = plot3(XYplane(1,:),XYplane(2,:),XYplane(3,:));
maxXY2 = plot3(XYplane2(1,:),XYplane2(2,:),XYplane2(3,:));
r = ((base1(1)-XYplane(1,1))^2 + (base1(2)-XYplane(2,1))^2)^0.5; %radius around UR3
%% Get max reach and volume
steps = deg2rad(45);
rail_stps = 0.15;
qlim1 = robot1.model.qlim;
qlim2 = robot2.model.qlim;

pointCloud1size = prod(floor((qlim1(2:7,2)-qlim1(2:7,1)/steps + 1)));
pointCloud1size = pointCloud1size + prod(floor((qlim1(1,2)-qlim1(1,1)/rail_stps + 1)));

pointCloud2size = prod(floor((qlim2(2:7,2)-qlim2(2:7,1)/steps + 1)));
pointCloud2size = pointCloud2size + prod(floor((qlim2(1,2)-qlim2(1,1)/rail_stps + 1)));

pointCloud1 = zeros(pointCloud1size,3);
pointCloud2 = zeros(pointCloud2size,3);
counter = 1;



for q1 = qlim1(1,1):rail_stps:qlim1(1,2)
    for q2 = qlim1(2,1):steps:qlim1(2,2)
        for q3 = qlim1(3,1):steps:qlim1(3,2)
            for q4 = qlim1(4,1):steps:qlim1(4,2)
                for q5 = qlim1(5,1):steps:qlim1(5,2)
                  for q6 = qlim1(6,1):steps:qlim1(6,2)
                    % Don't need to worry about joint 7, just assume it=0
                    q7 = 0;
                        q = [q1,q2,q3,q4,q5,q6,q7];
                        tr = robot1.model.fkine(q);                        
                        pointCloud1(counter,:) = tr(1:3,4)';
                        counter = counter + 1; 
%                         if mod(counter/pointCloud1size * 100,1) == 0
%                             display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloud1size * 100),'% of poses']);
%                         end
                   end
                end
            end
        end
    end
end
counter = 1;
for q1 = qlim2(1,1):rail_stps:qlim2(1,2)
    for q2 = qlim2(2,1):steps:qlim2(2,2)
        for q3 = qlim2(3,1):steps:qlim2(3,2)
            for q4 = qlim2(4,1):steps:qlim2(4,2)
                for q5 = qlim2(5,1):steps:qlim2(5,2)
                  for q6 = qlim2(6,1):steps:qlim2(6,2)
                    % Don't need to worry about joint 7, just assume it=0
                    q7 = 0;
                        q = [q1,q2,q3,q4,q5,q6,q7];
                        tr = robot2.model.fkine(q);                        
                        pointCloud2(counter,:) = tr(1:3,4)';
                        counter = counter + 1; 
%                         if mod(counter/pointCloud2size * 100,1) == 0
%                             display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloud2size * 100),'% of poses']);
%                         end
                   end
                end
            end
        end
    end
end

 
r1cloud = plot3(pointCloud1(:,1),pointCloud1(:,2),pointCloud1(:,3),'r.');
r2cloud = plot3(pointCloud2(:,1),pointCloud2(:,2),pointCloud2(:,3),'b.');
%%
% Divi bricks between the 2 bots by proximity
distances = zeros(9,2); %contains the distances and brick number
bricknum = 1;
for i=1:9
    distances(i,:) = [(((base1(1) - brickposes{1,i}(1,1))^2 + (base1(2) - brickposes{1,i}(1,2))^2)^0.5) bricknum];
    bricknum = bricknum + 1;
end

distances = sortrows(distances,1);
UR3bricks = zeros(3,1);
count = 1;
for i=1:4                   %Assign at most 4 bricks closest to UR3 to UR3
    if distances(i,1)<r     %Check brick is within reach
       UR3bricks(i) = distances(i,2);
       count = count + 1;    
    end    
end
% UR3bricks = distances(1:4,2);
%Rest to UR5
UR5bricks = distances(count:9,2);

% Get end goals

distance = zeros((size(UR3bricks,1)),2);
[droppoints,m] = GetGoals(base1,base2,r,brick);
UR3drops = [droppoints(1,:);droppoints(2,:);droppoints(4,:);droppoints(7,:)];
UR5drops = [droppoints(3,:);droppoints(6,:);droppoints(5,:);droppoints(8,:);droppoints(9,:)];

%Sort brick order such that those closest to drop off point get moved first
for i=1:size(UR3bricks,1)
    distance(i,:) = [(((UR3drops(i,1) - brickposes{1,UR3bricks(i)}(1,1))^2 + (UR3drops(i,2) - brickposes{1,UR3bricks(i)}(1,2))^2)^0.5) UR3bricks(i)];
end

distance = sortrows(distance,1);
UR3bricks = distance(:,2);

for i=1:size(UR5bricks,1)
    distance(i,:) = [(((UR5drops(i,1) - brickposes{1,UR5bricks(i)}(1,1))^2 + (UR5drops(i,2) - brickposes{1,UR5bricks(i)}(1,2))^2)^0.5) UR5bricks(i)];
end

distance = sortrows(distance,1);
UR5bricks = distance(:,2);


pause(1);
try delete(maxXY); end;
try delete(maxXY2); end;
try delete(r1cloud); end;
try delete(r2cloud); end;


%%
 if abs(m) == inf
    brickrot = trotz(pi/2);
    elseif m == 0
        brickrot = 1;
    else
        brickrot = trotz(-atan(m));
 end
 one1 = cell(4,1);
 one2 = cell(4,1);
 two1 = cell(4,1);
 two2 = cell(4,1);
 three1 = cell(4,1);
 three2 = cell(4,1);
for i=1:4
    g1 = robot1.model.getpos();
    g2 = robot2.model.getpos();
    T1 = transl(getbrickcoord{1,UR3bricks(i,1)}.x,getbrickcoord{1,UR3bricks(i,1)}.y,getbrickcoord{1,UR3bricks(i,1)}.z+0.2)*troty(pi);
    T2 = transl(getbrickcoord{1,UR5bricks(i,1)}.x,getbrickcoord{1,UR5bricks(i,1)}.y,getbrickcoord{1,UR5bricks(i,1)}.z+0.2)*troty(pi);
    [one1{i},one2{i}] = MovetoPoint(robot1,robot2,T1,T2);
    
    T1 = transl(getbrickcoord{1,UR3bricks(i,1)}.x,getbrickcoord{1,UR3bricks(i,1)}.y,getbrickcoord{1,UR3bricks(i,1)}.z+brick.z)*troty(pi);
    T2 = transl(getbrickcoord{1,UR5bricks(i,1)}.x,getbrickcoord{1,UR5bricks(i,1)}.y,getbrickcoord{1,UR5bricks(i,1)}.z+brick.z+0.08)*troty(pi);
%     if i==1
%         g1 = [4.2147   -1.0467   -2.6180   -3.1280   -1.5784   -3.6394];
%         g2 = robot2.model.getpos();
%     end
    [two1{i},two2{i}] = MovetoPoint(robot1,robot2,T1,T2,g1,g2); 
    
    b1 = transl(UR3drops(i,:))*troty(pi) * brickrot;
    b2 = transl(UR5drops(i,:))*transl(0,0,brick.z+0.02)*troty(pi) * brickrot;
    [tr,tr2,three1{i},three2{i}] = MoveBrick(robot1,robot2,b1,b2,Bricks,UR3bricks(i,1),UR5bricks(i,1),vertices);
    
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
    [one1{5},one2{5}] = MovetoPoint(robot1,robot2,ogT1,T2,ogq1,robot2.model.getpos());
    
    T2 = transl(getbrickcoord{1,UR5bricks(5,1)}.x,getbrickcoord{1,UR5bricks(5,1)}.y,getbrickcoord{1,UR5bricks(5,1)}.z+brick.z+0.08)*troty(pi);
    [two1{5},two2{5}] = MovetoPoint(robot1,robot2,ogT1,T2,ogq1,robot2.model.getpos());  
    
    b2 = transl(UR5drops(5,:))*transl(0,0,brick.z+0.02)*troty(pi) * brickrot;
    [tr,tr2,three1{5},three2{5}] = MoveBrick(robot1,robot2,0,b2,Bricks,0,UR5bricks(5,1),vertices);
    
   
    tr2(3,4) = UR5drops(5,3);
    transformedVertices2 = [vertices{1,UR5bricks(5,1)},ones(size(vertices{1,UR5bricks(5,1)},1),1)] * tr2';
    set(Bricks{UR5bricks(5,1),1},'Vertices',transformedVertices2(:,1:3));
    drawnow();
    
    %Move to original positions
    MovetoPoint(robot1,robot2,ogT1,ogT2,ogq1,ogq2); 
    
    %End
    
    
%% Play ROS bag
close all
clear all

rosbag info 2018-03-20-18-34-46.bag

bag = rosbag('2018-03-20-18-34-46.bag');

sel = select(bag,'Topic','/joint_states');

qs = readMessages(sel);

bagrobot = UR3
for i=1:5:sel.NumMessages
    q = (qs{i,1}.Position)'
    bagrobot.model.animate(q);   
end





