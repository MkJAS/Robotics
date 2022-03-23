function MoveBrick(robot1,robot2,point1,point2,Bricks,UR3bricks,UR5bricks,vertices,brick)
    steps = 50;
    s = lspb(0,1,steps);
    q1Matrix = nan(steps,6);
    q2Matrix = nan(steps,7);
    
    q1r1 = robot1.model.getpos();
    q1r2 = robot2.model.getpos();
    q = q1r1;
    q(1,4) = -pi/2;
    q2r1 = robot1.model.ikcon(point1,q);
    q2r2 = robot2.model.ikcon(point2,q1r2);
    
    
    for i = 1:steps
       q1Matrix(i,:) = (1-s(i))*q1r1 + s(i)*q2r1;
       q2Matrix(i,:) = (1-s(i))*q1r2 + s(i)*q2r2;
    end
    
    for i=1:steps
        robot1.model.animate(q1Matrix(i,:));
        robot2.model.animate(q2Matrix(i,:));
        tr = robot1.model.fkine(q1Matrix(i,:)); %* transl(0,0,0.08); %transl because the end of the effector is for some reason not where the visual end is???
        tr2 = robot2.model.fkine(q2Matrix(i,:))* transl(0,0,0.08); %transl because the end of the effector is for some reason not where the visual end is???
        transformedVertices = [vertices{1,UR3bricks(2,1)},ones(size(vertices{1,UR3bricks(2,1)},1),1)] * tr';
        transformedVertices2 = [vertices{1,UR5bricks(2,1)},ones(size(vertices{1,UR5bricks(2,1)},1),1)] * tr2';
        set(Bricks{UR3bricks(2,1),1},'Vertices',transformedVertices(:,1:3));
        set(Bricks{UR5bricks(2,1),1},'Vertices',transformedVertices2(:,1:3));
        drawnow();
        pause(0.1)
    end
    % 'Drop' brick
    tr(3,4) = brick.z;
    transformedVertices = [vertices{1,1},ones(size(vertices{1,1},1),1)] * tr';%transl(brick1drop(1,4),brick1drop(2,4),0)';
    set(Bricks{UR3bricks(1,1),1},'Vertices',transformedVertices(:,1:3));
    drawnow();
end

