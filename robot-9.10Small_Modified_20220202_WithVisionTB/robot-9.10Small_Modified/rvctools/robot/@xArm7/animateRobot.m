%% animateRobot(robot, qMatrix)
% Args: 
%   Robot: a given robot
%   qMatrix: a matrix(for the xArm7) of n x 7 (where n = numSteps) which is
%   a list of all the joint angles between two end effector positions
% Description: 
%   this function animates the robot by iterating through the qMatrix and
%   individually animating each 

function animateRobot(robot, qMatrix)
    numStepsMtx = size(qMatrix);
    numSteps = numStepsMtx(1);
    for i = 1:numSteps
        drawnow()
        %animate robot motion
        animate(robot.model, qMatrix(i, :));
    end
end