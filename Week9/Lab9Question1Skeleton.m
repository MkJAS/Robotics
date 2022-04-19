function Lab9Question1Skeleton()
% 1.1) Set parameters for the simulation
mdl_puma560;        % Load robot model
t = ...             % Total time
steps = ...         % No. of steps
deltaT = t/steps;   % Discrete time step
delta = 2*pi/steps; % Small angle change
epsilon = ...       % Threshold value for manipulability/Damped Least Squares
W = diag([...]);    % Weighting matrix for the velocity vector

% 1.2) Allocate array data
m = zeros(steps,1);             % Array for Measure of Manipulability
qMatrix = zeros(steps,6);       % Array for joint angles
qdot = zeros(steps,6);          % Array for joint velocities
theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
x = zeros(3,steps);             % Array for x-y-z trajectory
positionError = zeros(3,steps); % For plotting trajectory error
angleError = zeros(3,steps);    % For plotting trajectory error

% 1.3) Set up trajectory, initial pose
s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
for i=1:steps
    x(1,i) = (1-s(i))*0.35 + s(i)*0.35; % Points in x
    x(2,i) = (1-s(i))*-0.55 + s(i)*0.55; % Points in y
    x(3,i) = 0.5 + 0.2*sin(i*delta); % Points in z
    theta(1,i) = 0;                 % Roll angle 
    theta(2,i) = 5*pi/9;            % Pitch angle
    theta(3,i) = 0;                 % Yaw angle
end
 
T = ...             % Create transformation of first point and angle
q0 = ...            % Initial guess for joint angles
qMatrix(1,:) = ...  % Solve joint angles to achieve first waypoint

% 1.4) Track the trajectory with RMRC
for i = 1:steps-1
    T = ...                 % Get forward transformation at current joint state
    deltaX = ...            % Get position error from next waypoint
    Rd = rpy2r(...);      % Get next RPY angles, convert to rotation matrix
    Ra = ...                % Current end-effector rotation matrix
    
	Rdot = (1/deltaT)*(Rd - Ra);       % Calculate rotation matrix error (see RMRC lectures)
    S = ....;                      % Skew symmetric! S(\omega)
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(?,?);S(?,?);S(?,?)];  % Check the structure of Skew Symmetric matrix! Extract the angular velocities. (see RMRC lectures)

    deltaR = ...        	% Calculate rotation matrix error
    deltaTheta = tr2rpy(...);% Convert rotation matrix to RPY angles
    xdot = ...              % Calculate end-effector velocity to reach next waypoint.
(Try using a weighting matrix to (de)emphasize certain dimensions)
    J = ...                 % Get Jacobian at current joint state
 
    if ...  % If manipulability is less than given threshold
        lambda = ... % Damping coefficient (try scaling it)
        invJ = ... % Apply Damped Least Squares pseudoinverse
    else
        invJ = ... % Don't use DLS
    end
    qdot(i,:) = ... % Solve the RMRC equation (you may need to transpose the         vector)
    for ... % Loop through joints 1 to 6
        if ... % If next joint angle is lower than joint limit...
            ... % Stop the motor
        elseif ... % If next joint angle is greater than joint limit ...
            ... % Stop the motor
        end
    end
    qMatrix(i+1,:) = ... % Update next joint state based on joint velocities
    m(i) = ...  % Record manipulability
    positionError(:,i) = deltaX;  % For plotting
    angleError(:,i) = deltaTheta; % For plotting
end

% 1.5) Plot the results
figure(1)
plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
p560.plot(qMatrix,'trail','r-')

for i = 1:6
    figure(2)
    subplot(3,2,i)
    plot(qMatrix(:,i),'k','LineWidth',1)
    title(['Joint ', num2str(i)])
    ylabel('Angle (rad)')
    refline(0,p560.qlim(i,1));
    refline(0,p560.qlim(i,2));
    
    figure(3)
    subplot(3,2,i)
    plot(qdot(:,i),'k','LineWidth',1)
    title(['Joint ',num2str(i)]);
    ylabel('Velocity (rad/s)')
    refline(0,0)
end

figure(4)
subplot(2,1,1)
plot(positionError'*1000,'LineWidth',1)
refline(0,0)
xlabel('Step')
ylabel('Position Error (mm)')
legend('X-Axis','Y-Axis','Z-Axis')

subplot(2,1,2)
plot(angleError','LineWidth',1)
refline(0,0)
xlabel('Step')
ylabel('Angle Error (rad)')
legend('Roll','Pitch','Yaw')
figure(5)
plot(m,'k','LineWidth',1)
refline(0,epsilon)
title('Manipulability')
