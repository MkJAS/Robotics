close all;
clear all;

L1 = Link('d',0,'a',2,'alpha',0,'offset',0,'qlim', [pi/2,-pi/2]);
... % More link definition in between (if required)
L2 = Link('d',0,'a',2,'alpha',pi/2,'offset',0,'qlim', [pi/2,-pi/2]);
Robot = SerialLink([L1 L2],'name','myRobot');
q = zeros(1,2); % This creates a vector of n joint angles at 0.
workspace = [-5 +5 -5 +5 -5 +5];
scale = 1;
Robot.plot(q,'workspace',workspace,'scale',scale); 

Robot.teach();