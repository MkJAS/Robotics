function [tr,tr2,r1Ts,r2Ts] = MoveBrick(robot1,robot2,point1,point2,Bricks,UR3bricks,UR5bricks,vertices)
    steps = 50;
    s = lspb(0,1,steps);
    r1Ts = cell(steps,2);
    r2Ts = cell(steps,2);
    q1Matrix = nan(steps,7);
    q2Matrix = nan(steps,7);
    
    if size(point1,1) > 2
        q1r1 = robot1.model.getpos();
        q1r2 = robot2.model.getpos();
        g1 = q1r1;

        q2r1 = robot1.model.ikcon(point1,g1);
        q2r2 = robot2.model.ikcon(point2,q1r2);


        for i = 1:steps
           q1Matrix(i,:) = (1-s(i))*q1r1 + s(i)*q2r1;
           q2Matrix(i,:) = (1-s(i))*q1r2 + s(i)*q2r2;
        end

        for i=1:steps
            robot1.model.animate(q1Matrix(i,:));
            robot2.model.animate(q2Matrix(i,:));
            tr = robot1.model.fkine(q1Matrix(i,:));
            tr2 = robot2.model.fkine(q2Matrix(i,:))* transl(0,0,0.08); %transl because the end of the effector is for some reason not where the visual end is???
            transformedVertices = [vertices{1,UR3bricks},ones(size(vertices{1,UR3bricks},1),1)] * tr';
            transformedVertices2 = [vertices{1,UR5bricks},ones(size(vertices{1,UR5bricks},1),1)] * tr2';
            set(Bricks{UR3bricks,1},'Vertices',transformedVertices(:,1:3));
            set(Bricks{UR5bricks,1},'Vertices',transformedVertices2(:,1:3));
            drawnow();
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
    
    if size(point1,1) < 2
        q1r2 = robot2.model.getpos();
        q2r2 = robot2.model.ikcon(point2,q1r2);

        for i = 1:steps
           q2Matrix(i,:) = (1-s(i))*q1r2 + s(i)*q2r2;
        end

        for i=1:steps
            robot2.model.animate(q2Matrix(i,:));
            tr2 = robot2.model.fkine(q2Matrix(i,:))* transl(0,0,0.08); %transl because the end of the effector is for some reason not where the visual end is???
            transformedVertices2 = [vertices{1,UR5bricks},ones(size(vertices{1,UR5bricks},1),1)] * tr2';
            set(Bricks{UR5bricks,1},'Vertices',transformedVertices2(:,1:3));
            drawnow();
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
        tr = 0;            
    end    
end

