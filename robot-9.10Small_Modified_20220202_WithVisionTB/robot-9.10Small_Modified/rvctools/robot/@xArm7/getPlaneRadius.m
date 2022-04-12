%% getPlaneRadius
% this function returns the average radius (or reach) of the robot in a
% given plane using an input of pointCloud data

function planeRadius = getPlaneRadius(robot, plane, pointCloud)
    switch(plane)
        case 'XY'
            centre = transl(robot.model.base);
            xValues = pointCloud(:, 1);
            yValues = pointCloud(:, 2);
            radiiList = zeros(360, 1);
            for i = 1:360
                radiiList(i, 1) = sqrt((centre(1, 1)- xValues(i, 1)).^2 + (centre(2, 1) - yValues(i, 1)).^2);
            end
            planeRadius = mean(radiiList);
         
        case 'XZ'
            centre = transl(robot.model.base);
            xValues = pointCloud(:, 1);
            zValues = pointCloud(:, 3);
            radiiList = zeros(360, 1);
            for i = 1:360
                radiiList(i, 1) = sqrt((centre(1, 1)- xValues(i, 1)).^2 + (centre(2, 1) - zValues(i, 1)).^2);
            end
            planeRadius = mean(radiiList);
    end
end