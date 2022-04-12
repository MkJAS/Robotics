%% getTranslationMatrix(currentTransform, goalTransform, numSteps)
%Args:
%   currentTransform: the current 4x4 transform matrix of the end effector
%   goalTransform: the goal 4x4 transform matrix of the end effector
%   numSteps: the number of steps between movements
%Output:
%   transformMatrix: a matrix of transforms between two points
%Description:
%   This function takes the currentTransform and goalTransform of the end
%   effector and uses the trapezoidal velocity profile to plot a number N of
%   waypoints between the current and goal transform. N is equal to
%   numSteps
function translationMatrix = getTranslationMatrix(~, currentTransform, goalTransform, numSteps)
    s = lspb(0, 1, numSteps);
    translationMatrix = nan(numSteps, 6);
    currentTranslation = transl(currentTransform);
    goalTranslation = transl(goalTransform);
    currentRPY = tr2rpy(currentTransform);
    goalRPY = tr2rpy(goalTransform);
    for i = 1:numSteps
        translationMatrix(i, 1:3) = currentTranslation + s(i) * (goalTranslation - currentTranslation);
        translationMatrix(i, 4:6) =  currentRPY + s(i) * (goalRPY - currentRPY);
    end
end