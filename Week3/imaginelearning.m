close all;
clear all;

% Link('theta',__,'d',__,'a',__,'alpha',__,'offset',__,'qlim',[ ... ])

 L(1) = Link('d', 0.138, 'a', 0, 'alpha', -pi/2, 'offset', 0);
            L(2) = Link('d', 0, 'a', 0.135, 'alpha',0,'offset', 0);
            L(3) = Link('d', 0, 'a', 0.147, 'alpha', pi, 'offset',0);
            L(4) = Link('d', 0.041, 'a', 0, 'alpha', -pi/2, 'offset', 0);
% 
% L5 = Link('d',0,'a',0,'alpha',-pi/2,'offset',0);
% 
% L6 = Link('d',0,'a',0,'alpha',0,'offset',0);

robot = SerialLink(L);

q = zeros(1,4);

robot.plot(q)
%% 
close all
clear all

% Link('theta',__,'d',__,'a',__,'alpha',__,'offset',__,'qlim',[ ... ])
  L(1) = Link('d', 0.138, 'a', 0, 'alpha', -pi/2, 'offset', 0,'qlim',[-135*pi/180 135*pi/180]);
            L(2) = Link('d', 0, 'a', 0.135, 'alpha',0,'offset', -pi/2,'qlim',[5*pi/180 80*pi/180]);
            L(3) = Link('d', 0, 'a', 0.147, 'alpha', pi, 'offset',0,'qlim',[-5*pi/180 85*pi/180]);
            L(4) = Link('d', 0, 'a', 0.041, 'alpha', -pi/2, 'offset', 0,'qlim',[-pi/2 pi/2]);
            L(5) = Link('d', 0.09, 'a', 0, 'alpha',0, 'offset', 0,'qlim',[-85*pi/180 85*pi/180]);

robot = SerialLink([L]);

q = zeros(1,5);

robot.plot(q)
%% 

q = robot.getpos()

T = robot.fkine(q)

q = robot.ikine(T)

q = robot.getpos();

J = robot.jacob0(q)


