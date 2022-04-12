%% getPlanePointCloud
%returns a pointCloud for a given plane. Note that YZ is missing, as the
%radius for YZ and XZ are the same.
function planePointCloud = getPlanePointCloud(robot, plane)
    switch(plane)
        case 'XY'
            q = [0, pi/2, 0, pi, 0, 0, pi/2];
            counter = 1;
            planePointCloud = zeros(360, 3);
            for q1 = 0:pi/180:2*pi
                q(1, 1) = q1;
                tr = robot.model.fkine(q);
                planePointCloud(counter, :) = tr(1:3, 4)';
                counter = counter + 1; 
            end

        case 'XZ'
            q = [0, pi/2, 0, pi, 0, 0, pi/2];
            counter = 1;
            planePointCloud = zeros(360, 3);
            for q2 = 0:pi/180:2*pi
                q(2, 1) = q2;
                tr = robot.model.fkine(q);
                planePointCloud(counter, :) = tr(1:3, 4)';
                planePointCloud(counter, 3) = planePointCloud(counter, 3) - 0.1519;
                counter = counter + 1;
            end
    end
end
