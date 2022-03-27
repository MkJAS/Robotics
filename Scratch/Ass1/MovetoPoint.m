function MovetoPoint(robot1,robot2,point1,point2,g1,g2)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    if nargin < 5
          g1 = robot1.model.getpos();
          g2 = robot2.model.getpos();
    end
     
    steps = 50;
    s = lspb(0,1,steps);
    q1Matrix = nan(steps,7);
    q2Matrix = nan(steps,7);
    
    q1r1 = robot1.model.getpos();
    q1r2 = robot2.model.getpos();  
    g1(1,5) = deg2rad(-96);
    g1(1,6) = deg2rad(-96);


    q2r1 = robot1.model.ikcon(point1,g1); % 4.2147   -1.0467   -2.6180   -3.1280   -1.5784   -3.6394

    q2r2 = robot2.model.ikcon(point2,g2);


    for i = 1:steps
       q1Matrix(i,:) = (1-s(i))*q1r1 + s(i)*q2r1;
       q2Matrix(i,:) = (1-s(i))*q1r2 + s(i)*q2r2;
    end


    for i=1:steps
        robot1.model.animate(q1Matrix(i,:));
        robot2.model.animate(q2Matrix(i,:));
        pause(0.1)
    end

    
end

