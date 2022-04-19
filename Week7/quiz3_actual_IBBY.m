clear all
clc
close all

%% Questions for week 7 quiz
%% 1 Collision checking
%Parameters: pointOnPlane,planeNormal,point1OnLine,point2OnLine
mdl_puma560;
q = [pi/12,0,-pi/2,0,0,0];

planeNormal = [-1,0,0];
pointOnPlane = [1.8,0,0];
tr = p560.fkine(q);
startP = tr(1:3,4)';
display(startP)
endP = [tr(1,4),tr(2,4),tr(3,4)+10];
[intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,startP,endP)
% Check == 0 if there is no intersection
% Check == 1 if there is a line plane intersection between the two points
% Check == 2 if the segment lies in the plane (always intersecting)
% Check == 3 if there is intersection point which lies outside line segment

%% Collision checking with 3DoF robot
mdl_puma560;
q = [pi/12,0,-pi/2,0,0,0];
tr = zeros(4,4,R3.n+1);
tr(:,:,1) = R3.base;
L = p560.links;
for i = 1 : p560.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end
jointTr = tr(:,:,5);
effTr = p560.fkine(q);
planeNormal = [-1,0,0]; %for X = 5m
pointOnPlane = [1.8,0,0];%unless given
startP = jointTr(1:3,4)';
endP = Tr(1:3,4)';
[intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,startP,endP)

%this one doesnt work, i think Tr in line 37 is wrong

% Check == 0 if there is no intersection
% Check == 1 if there is a line plane intersection between the two points
% Check == 2 if the segment lies in the plane (always intersecting)
% Check == 3 if there is intersection point which lies outside line segment
%% Collision checking with 3DoF robot
%where would the robot R3 collide with flat wall x = 3m, unsure how to
%incorporate center of the second joint shit, 

mdl_3link3d;
q = [pi/12,0,0];
tr = zeros(4,4,R3.n+1);
tr(:,:,1) = R3.base;
L = R3.links;
for i = 1 : R3.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end
jointTr = tr(:,:,2); %is this the second joint???
effTr = R3.fkine(q);
planeNormal = [3,0,0]; %for X = 5m
pointOnPlane = [3,0,0];%unless given
startP = jointTr(1:3,4)';
endP = effTr(1:3,4)';
[intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,startP,endP)

% Check == 0 if there is no intersection
% Check == 1 if there is a line plane intersection between the two points
% Check == 2 if the segment lies in the plane (always intersecting)
% Check == 3 if there is intersection point which lies outside line segment

%% 2 Create 5DOF Planar
%All DH param. at 1m. End effectors pose?
close all;
clear;

% Joint angles
q = deg2rad([30,-60,45,-30,0]);

% Create robot
L1 = Link('d', 0, 'a', 1, 'alpha', 0, 'qlim', [-pi,pi]);
L2 = Link('d', 0, 'a', 1, 'alpha', 0, 'qlim', [-pi,pi]);
L3 = Link('d', 0, 'a', 1, 'alpha', 0, 'qlim', [-pi,pi]);
L4 = Link('d', 0, 'a', 1, 'alpha', 0, 'qlim', [-pi,pi]);
L5 = Link('d', 0, 'a', 1, 'alpha', 0, 'qlim', [-pi,pi]);
robot = SerialLink([L1, L2, L3, L4, L5]);

% calculate end effector pose
pose = robot.fkine(q);

% display position
disp(pose(1:3, 4)');

% ✓✓ IBBY TESTED AND VERIFIED ✓✓

%% 3 Distance Sense Distance to Puma End Effector
close all;
clear;

% specifiy these
q = deg2rad([0, 45, -80, 0, 45, 0]);
sensor_pose = transl(0.6304, -2.1490, 0.6223);

% create puma
mdl_puma560;

%end effector pose


% calculate displacement to sensor
robot_pose = p560.fkine(q)
distance_to_sensor = sqrt(sum(sum((sensor_pose(1:3, 4) - robot_pose(1:3, 4)).^2)));

% display result
disp(distance_to_sensor);

%% 4 Lab Assignment 1 Trajectories
clear
clc
close all
mdl_puma560
qlim = p560.qlim;                                                           
hold on;

q1 = [pi/10, pi/7, pi/5, pi/3, pi/4, pi/6]
q2 = [-pi/6, -pi/3, -pi/4, -pi/8, -pi/7, -pi/10]
steps = 35; 

        s = lspb(0,1,steps);
        qMatrix = nan(steps,6);
        for i = 1:steps
            qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
        end

figure(1)
p560.plot(qMatrix,'trail','r-')

velocity = zeros(steps,6);
for i = 2:steps
   velocity(i,:) = round((qMatrix(i,:) - qMatrix(i-1,:)),4);
end

a = 1
max(abs(velocity(:,:)))
%% 5 Point in Puma End Effector Coordinate Frame
%balls position with repsect to the end effectors coordinate frame
clear;
clc;
close all;

%specify these
ball =   transl(0.5,0.1,0.6) * trotx(pi/2);
q = deg2rad( [-90, 30, -80, 0, 45, 0] );

%create robot
mdl_puma560;

%determine end effector pose
ef = p560.fkine(q)

%get relative transform to ball
ef2ball = ef \ ball;

%display distance
xzy = ef2ball(1:3, 4)

% ✓✓ IBBY TESTED AND VERIFIED ✓✓
%% 6 Puma Ikine 
% 
clear;
clc;
close all;

%specify these
t = transl( 0.7,0.1,0.2);

%create robot
mdl_puma560;

%calculate end effector pose. orientation is masked off
q = p560.ikine(t, qn, [1 1 1 0 0 0]);
p560.plot(q);

%display result
disp(q);

% ✓✓ IBBY TESTED AND VERIFIED ✓✓
%% 7 Puma Distance to Wall along Z
%ray cast from the Z axis 

close all;
clear;
clc;

% plane definition
normal = [-1,0,0];
point = [1.2,0,0];

%robot joint angles
q = [pi/10,0,-pi/2,0,0,0];

%create robot
mdl_puma560;

%determine end effector pose
ee = p560.fkine(q);

v = ee(1:3, 1:3) * [0; 0; 1];

ee2 = ee;
ee2(1:3, 4) = ee2(1:3, 4) + v * 0.25;

p1 = ee(1:3, 4)';
p2 = ee2(1:3, 4)';

% determine intersection of line/plane from last robot link and specified
%plane
[intersectionPoint,check] = LinePlaneIntersection(normal, point, p1, p2)

%disp(intersectionPoint);
% 0 - no intersect
% 1 - line plane intersect
% 2 - parallel coincident
% 3 - intersect outside of segment

% ✓✓ IBBY TESTED AND VERIFIED ✓✓

%% TO DO
%Centre of the second joint when q is given and a flat wall at x=3m 
%% Safety (x2 questions)
% refer to module 7.4 
% Q In terms of safety what are benefits of colloborative robots? 
% ANSWERS:footprint, part time cycles, maintences= costs
% ANSall answers are incorrect
% which following safety measures could be used in the presence of robots
% ANSWERS:pressure plates, light curtain, cameras
% ANS: all are correct
%% Sawyer
% https://blog.robotiq.com/universal-robots-ur3-vs-rethink-robotics-sawyer
% previous sawyer model was Baxter
% made by rethink robotics


%%
%ROBOTICS TOOLBOX
%Does Ikine consider joint angles? ANS:NO BUT OTHER SOLVERS DO
%% 
% edit jtraj
% does not enforce joint limits

%%
clear all
clc
mdl_puma560
p560.base = p560.base * transl(0,0,0);
q = deg2rad([0, 50, -80, 0, 45, 0]);


