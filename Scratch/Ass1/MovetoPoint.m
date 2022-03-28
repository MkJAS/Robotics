function [r1Ts,r2Ts] = MovetoPoint(robot1,robot2,point1,point2,g1,g2)
%Moves end effector to a point, typically a point above a brick then a
%point at the brick
    if nargin < 5 %if no q guess is supplied use the current positions
          g1 = robot1.model.getpos();
          g2 = robot2.model.getpos();
    end
     
    steps = 50;
    r1Ts = cell(steps,2);
    r2Ts = cell(steps,2);
    s = lspb(0,1,steps);
    q1Matrix = nan(steps,7);
    q2Matrix = nan(steps,7);
    
    q1r1 = robot1.model.getpos();
    q1r2 = robot2.model.getpos();  
    g1(1,5) = deg2rad(-96);
    g1(1,6) = deg2rad(-96);

    q2r1 = robot1.model.ikcon(point1,g1); 

    q2r2 = robot2.model.ikcon(point2,g2);

    for i = 1:steps
       q1Matrix(i,:) = (1-s(i))*q1r1 + s(i)*q2r1;
       q2Matrix(i,:) = (1-s(i))*q1r2 + s(i)*q2r2;
    end

    for i=1:steps
        robot1.model.animate(q1Matrix(i,:));
        robot2.model.animate(q2Matrix(i,:));
        %Display and store each transform and q matrix
        q1 = robot1.model.getpos()
        r1Ts{i,1} = q1;
        robot1.model.fkine(q1)
        r1Ts{i,2} = robot1.model.fkine(q1);
        q2 = robot2.model.getpos()
        r1Ts{i,1} = q2;
        robot2.model.fkine(q2)
        r2Ts{i,2} = robot2.model.fkine(q2);
        pause(0.1)
    end
end

