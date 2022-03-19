function MovetoBrick(robot1,robot2,UR3bricks,UR5bricks,getbrickcoord)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    steps = 50;
    s = lspb(0,1,steps);
    q1Matrix = nan(steps,6);
    q2Matrix = nan(steps,7);
    
    q1r1 = robot1.model.getpos();
    q1r2 = robot2.model.getpos();

    T2 = transl(getbrickcoord{1,UR3bricks(1,1)}.x,getbrickcoord{1,UR3bricks(1,1)}.y,getbrickcoord{1,UR3bricks(1,1)}.z+0.2)*troty(pi);
    q2r1 = robot1.model.ikcon(T2);
    T22 = transl(getbrickcoord{1,UR5bricks(1,1)}.x,getbrickcoord{1,UR5bricks(1,1)}.y,getbrickcoord{1,UR5bricks(1,1)}.z+0.2)*troty(pi);
    q2r2 = robot2.model.ikcon(T22);


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

