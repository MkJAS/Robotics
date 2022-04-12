%% showMaximumPlaneRadius
%Shows maximum Plane radius of the robot
function maximumReachAndVolume = getMaximumReachAndVolume(robot)
    maximumReachAndVolume = zeros(3, 1); %3 rows in order: XYRadius, XZRadius, Volume
    XYPointCloud = robot.getPlanePointCloud('XY');
    XZPointCloud = robot.getPlanePointCloud('XZ');
    %XY graph
    %figure('Name', 'XYCircle');
    XYRadius = robot.getPlaneRadius('XY', XYPointCloud);
    plot(XYPointCloud(:, 1), XYPointCloud(:, 2), 'r.');
    maximumReachAndVolume(1, 1) = XYRadius;
    XYRadiusDisp = ['XYRadius = ', num2str(XYRadius), 'm'];
    disp(XYRadiusDisp);
    
    %XZ graph
    %figure('Name', 'XZCircle');
    XZRadius = robot.getPlaneRadius('XZ', XZPointCloud);
    %plot(XZPointCloud(:, 1), XZPointCloud(:, 3), 'r.');
    maximumReachAndVolume(2, 1) = XZRadius;
    XZRadiusDisp = ['XZRadius = ', num2str(XZRadius), 'm'];
    disp(XZRadiusDisp);
    
    %Volume
    volume = 4/3*pi*XYRadius*XZRadius.^2; %formula for ellipse volume
    maximumReachAndVolume(3, 1) = volume;
    volumeDisp = ['volume = ', num2str(volume), 'm^3'];
    disp(volumeDisp);
end
