%% cartesianQMatrix = getCartesianQMatrix(currentTransform, transformMatrix, numSteps)
%Args:
%   robot: a given robot (in this case xArm7)
%   currentTransform: the current transform of the robot's end effector
%   transformMatrix
%   numSteps: the number of steps between movements
%Output:
%   cartesianQMatrix: a matrix(size MxN where M = numSteps and N = number 
%   of robot joints) of joint angles which utilises resolved motion rate
%   control
%Description:
%   This function takes the currentJoints and transformMatrix and returns a
%   cartesian qMatrix using resolved motion rate control. It calculates
%   this using differential kinematics equation xDot = Jacobian(q) * qDot
function cartesianQMatrix = getCartesianQMatrix(robot, currentTransform, currentJoints, goalTransform, numSteps)
    deltaT = 1/numSteps;
    cartesianQMatrix = nan(numSteps, 7);
    translationMatrix = robot.getTranslationMatrix(currentTransform, goalTransform, numSteps);
    cartesianQMatrix(1, :) = currentJoints;
    for i = 1 : numSteps - 1
        currentEndEffectorVelocity = (translationMatrix(i + 1, :) - translationMatrix(i, :)) / deltaT;
        currentJacobian = robot.model.jacob0(cartesianQMatrix(i, :));
        currentJointVelocity = pinv(currentJacobian) * currentEndEffectorVelocity'; %RMRC
        cartesianQMatrix(i + 1, :) = cartesianQMatrix(i, :) + deltaT * currentJointVelocity'; %update next joint state
    end

end

