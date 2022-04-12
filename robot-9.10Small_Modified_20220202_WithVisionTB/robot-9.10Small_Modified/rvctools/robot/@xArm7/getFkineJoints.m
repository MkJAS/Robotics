function fkineJoints = getFkineJoints(robot, q)
    size = robot.model.n + 1;
    fkineJoints = cell(size, 1);
    fkineJoints{1, 1} = robot.model.base;
    for i = 2 : size
        link = robot.model.links(i - 1);
        theta = link.offset;
        offset = q(1, i - 1);
        a = link.a;
        d = link.d;
        alpha = link.alpha;
        tr = fkineJoints{i - 1, 1} * trotz(theta + offset) * transl(0, 0, d) * transl(a, 0, 0) * trotx(alpha);
        fkineJoints{i, 1} = tr;
    end

end