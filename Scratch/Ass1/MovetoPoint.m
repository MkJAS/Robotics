function MovetoPoint(robot1,robot2,point1,point2)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    steps = 50;
    s = lspb(0,1,steps);
    q1Matrix = nan(steps,6);
    q2Matrix = nan(steps,7);
    
    q1r1 = robot1.model.getpos();
    q1r2 = robot2.model.getpos();


    q2r1 = robot1.model.ikcon(point1,q1r1);

    q2r2 = robot2.model.ikcon(point2,q1r2);


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

