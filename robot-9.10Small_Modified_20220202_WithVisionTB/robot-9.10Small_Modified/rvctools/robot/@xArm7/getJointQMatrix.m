%% jointQMatrix = getJointQMatrix(robot, currentTransform, goalTransform, numSteps)
%Args:
%   robot: the robot
%   currentTransform: the current transform of the robot end effector
%   goalTransform: the goal transform of the robot
%   numSteps: the number of steps between movements
%Output:
%   jointQMatrix: matrix of goalJoints MxN where M = numSteps and N = robot
%       joints
%Description:
%   This function takes, a robot's currentjoints, a robot's goal joints, and a
%   number of steps to calculate the qMatrix using the trapezoidal velocity
%   profile

function jointQMatrix = getJointQMatrix(robot, currentJoints, goalTransform, numSteps)
    goalJoints = robot.model.ikcon(goalTransform, currentJoints);
    s = lspb(0, 1, numSteps);
    jointQMatrix = nan(numSteps, 7);
    for i = 1 : numSteps
        jointQMatrix(i, :) = currentJoints + s(i) * (goalJoints - currentJoints);
    end 
    
end

