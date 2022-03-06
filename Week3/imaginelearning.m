close all;
clear all;

% Link('theta',__,'d',__,'a',__,'alpha',__,'offset',__,'qlim',[ ... ])

L1 = Link('d',1,'a',0,'alpha',pi/2,'offset',0);

L2 = Link('d',0,'a',1,'alpha',0,'offset',0);
% 
L3 = Link('d',0,'a',1,'alpha',-pi/2,'offset',0);
% 
L4 = Link('d',0,'a',1,'alpha',-pi/2,'offset',0);
% 
% L5 = Link('d',0,'a',0,'alpha',-pi/2,'offset',0);
% 
% L6 = Link('d',0,'a',0,'alpha',0,'offset',0);

robot = SerialLink([L1 L2 L3]);

q = zeros(1,3);

robot.plot(q)
%% 
close all;
clear all;

% Link('theta',__,'d',__,'a',__,'alpha',__,'offset',__,'qlim',[ ... ])

L1 = Link('d',0.1273,'a',0,'alpha',pi/2,'offset',0);

L2 = Link('d',0,'a',0.612,'alpha',0,'offset',0);
% 
L3 = Link('d',0,'a',0.5723,'alpha',0,'offset',0);
% 
L4 = Link('d',0,'a',0.16394,'alpha',-pi/2,'offset',0);
% 
L5 = Link('d',-0.1157,'a',0,'alpha',-pi/2,'offset',0);
% 
L6 = Link('d',0.0922,'a',0,'alpha',0,'offset',0);

robot = SerialLink([L1 L2 L3 L4 L5 L6]);

q = zeros(1,6);

robot.plot(q)
%% 

q = robot.getpos()

T = robot.fkine(q)

q = robot.ikine(T)

q = robot.getpos();

J = robot.jacob0(q)


