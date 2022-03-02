clear all;
close all;

% P1 = se2(0,0,0);
% P1 = se3(P1);
% 
% P2 = transl(0,0,10);
% axis([0,15,0,10,0,15],'square');
% grid on;
% hold on
% tranimate(P1,P2);
% 
% 
% 
% 
% P6 = P4 * troty(20,'deg');
% clf;
% axis([0,15,0,10,0,15],'square');
% grid on;
% hold on
% 
% tranimate(P5,P6);
% 
% P7 =  transl(2,2,10)*troty(20,'deg');
% tranimate(P6,P7);
% 
% P8 = P7 * troty(-20,'deg');
% 
% 
% tranimate(P7,P8);
% 
% P9 = transl(2,2,0);
% tranimate(P8,P9);

P1 = eye(4);
P2 = eye(4);
roll = 0;
pitch = 0;
yaw = 0;
q = [0 0 0 0];
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
      axis([0,15,0,10,0,15],'square');
      grid on;
      hold on;
      
      q = rotm2quat(P2(1:3,1:3));
      q(1,:)
      
      try delete(texts); end;
       try delete(tranimate); end;
      message = sprintf([num2str(round(q(1,:),2,'significant')),'\n' ...
                      ,num2str(roll),' ',num2str(pitch),' ',num2str(yaw)]);
      texts = text(1, 10, message, 'FontSize', 10, 'Color', [.6 .2 .6]);
      drawnow();
      tranimate(P1,P2,'fps',30);
      
end

