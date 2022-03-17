clear all
close all
% Link('theta',__,'d',__,'a',__,'alpha',__,'offset',__,'qlim',[ ... ])

% 
% L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'offset',0,'qlim',[-2*pi 2*pi]);
% 
% L2 = Link('d',0,'a',-0.24365,'alpha',0,'offset',0,'qlim',[-2*pi 2*pi]);
% 
% L3 = Link('d',0,'a',-0.21325,'alpha',0,'offset',0,'qlim',[-2*pi 2*pi]);
% 
% L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'offset',0,'qlim',[-2*pi 2*pi]);
% 
% L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'offset',0,'qlim',[-2*pi 2*pi]);
% 
% L6 = Link('d',0.0819,'a',0,'alpha',0,'offset',0,'qlim',[-2*pi 2*pi]);

L(1) = Link('d', 0.1519, 'a', 0, 'alpha', -pi/2, 'offset', 0, 'qlim', [-2*pi 2*pi]);
            L(2) = Link('d', 0, 'a', -0.24365, 'alpha', -pi,'offset', pi, 'qlim', [-2*pi,2*pi]);
            L(3) = Link('d', 0, 'a', -0.21325, 'alpha', pi, 'offset', 0, 'qlim', [-150*pi/180,150*pi/180]);
            L(4) = Link('d', 0.11235, 'a', 0, 'alpha', -pi/2, 'offset', pi, 'qlim', [-2*pi 2*pi]);
            L(5) = Link('d', 0.08535, 'a', 0, 'alpha', pi/2, 'offset', 0, 'qlim', [-2*pi 2*pi]);
            L(6) = Link('d', 0.0819, 'a', 0, 'alpha', 0, 'offset', 0, 'qlim', [-2*pi 2*pi]);

myRobot = SerialLink(L, 'name', 'UR3')

q = zeros(1,6)
figure(2)
myRobot.plot(q)

myRobot.teach
