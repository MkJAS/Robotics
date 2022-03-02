clear all;
close all;

cowHerd = RobotCows(3);
hold on;
P1 = transl(0,0,0);
tranimate(P1);
T1 = cowHerd.cow{1}.base*P1^-1
T2 = cowHerd.cow{2}.base*P1^-1
T3 = cowHerd.cow{3}.base*P1^-1

%% 

for i=1:8
    switch i
        case 1
            P1 = se2(0,0,0);
            P1 = se3(P1);
            P2 = transl(0,0,10);            
        case 2
            P1 = P2;
            P2 = transl(0,0,10)* trotx(-10,'deg');
            roll = -10;
        case 3
            P1 = P2;
            P2 = transl(0,2,10)* trotx(-10,'deg');
        case 4
            P1 = P2;
            P2 = P2 * trotx(10,'deg');
            roll = 0;
        case 5
            P1 = P2;
            P2 = P2 * troty(20,'deg');
            pitch = 20;
        case 6
            P1 = P2;
            P2 = transl(2,2,10)*troty(20,'deg');
        case 7
            P1 = P2;
            P2 = P2 * troty(-20,'deg');
            pitch = 0;
        case 8
            P1 = P2;
            P2 = transl(2,2,0);
    end
%       axis([0,15,0,10,0,15],'square');
%       grid on;
%       hold on;
%       
%       q = rotm2quat(P2(1:3,1:3));
%       q(1,:)
      
%       try delete(texts); end;
%        try delete(tranimate); end;
%       message = sprintf([num2str(round(q(1,:),2,'significant')),'\n' ...
%                       ,num2str(roll),' ',num2str(pitch),' ',num2str(yaw)]);
%       texts = text(1, 10, message, 'FontSize', 10, 'Color', [.6 .2 .6]);
%       drawnow();
      tranimate(P1,P2,'fps',30);
      cowHerd.PlotSingleRandomStep();
      T1 = cowHerd.cow{1}.base*P1^-1
      T2 = cowHerd.cow{2}.base*P1^-1
      T3 = cowHerd.cow{3}.base*P1^-1
      
end

% steps = 100;
% delay = 0.1;
% 
% cowHerd.TestPlotManyStep(steps,delay);
% cowHerd.cow{2}.base

%% 
close all;
cowHerd = RobotCows(1);
hold on;
P1 = transl(0,0,10);
tranimate(P1);

for i=1:10
    cowHerd.PlotSingleRandomStep();
    cowpos = cowHerd.cow{1}.base;
    abovecow = cowpos;
    abovecow(3,4) = abovecow(3,4) + 5;
    tranimate(P1,abovecow);
    P1 = abovecow;
end







