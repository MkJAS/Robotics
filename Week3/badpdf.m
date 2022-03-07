close all;
clear all;

% Link('theta',__,'d',__,'a',__,'alpha',__,'offset',__,'qlim',[ ... ])

J1 = Link('d',0.475,'a',0.18,'alpha',pi/2,'offset',0,'qlim',[-170*pi/180 170*pi/180]);

J2 = Link('d',0,'a',0.385,'alpha',0,'offset',pi/2,'qlim',[-90*pi/180 135*pi/180]);
% 
J3 = Link('d',0,'a',0.116,'alpha',-pi/2,'offset',0,'qlim',[-80*pi/180 165*pi/180]);
% 
J4 = Link('d',0,'a',0.329,'alpha',0,'offset',0,'qlim',[-185*pi/180 185*pi/180]);
% 
J5 = Link('d',0,'a',0.084,'alpha',-pi/2,'offset',0,'qlim',[-120*pi/180 120*pi/180]);
% 
J6 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim',[-360*pi/180 360*pi/180]);

robot = SerialLink([J1 J2 J3 J4]);

q = zeros(1,4);

robot.plot(q)