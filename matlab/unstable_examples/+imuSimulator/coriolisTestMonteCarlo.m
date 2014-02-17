clc
clear all
close all

% Flag to mark external configuration
externalCoriolisConfiguration = 1;

%% General configuration
navFrameRotating = 0;       % 0 = perform navigation in the fixed frame
                            % 1 = perform navigation in the rotating frame
IMU_type = 1;               % IMU type 1 or type 2
useRealisticValues = 1;     % use reaslist values for initial position and earth rotation
record_movie = 0;           % 0 = do not record movie
                            % 1 = record movie of the trajectories. One
                            % frame per time step (15 fps)

%% Scenario Configuration                            
deltaT = 0.01;              % timestep
timeElapsed = 5;            % Total elapsed time
times = 0:deltaT:timeElapsed;

% Initial Conditions
omegaEarthSeconds = [0;0;7.292115e-5]; % Earth Rotation rate (rad/s)
radiusEarth = 6378.1*1000;             % radius of Earth is 6,378.1 km

if useRealisticValues == 1
    omegaRotatingFrame = omegaEarthSeconds; % rotation of the moving frame wrt fixed frame
    omegaFixed = [0;0;0];       % constant rotation rate measurement
    accelFixed = [0.1;0;1];  % constant acceleration measurement
    g = [0;0;0];                % Gravity
    initialLongitude = -45;      % longitude in degrees
    initialLatitude = 30;       % latitude in degrees
    % initial position at some [longitude, latitude] location on earth's
    % surface (approximating Earth as a sphere)
    initialPosition = [radiusEarth*sind(initialLongitude);
                       radiusEarth*cosd(initialLongitude);
                       radiusEarth*sind(initialLatitude)];
    initialVelocity = [0; 0; 0];% initial velocity of the body in the rotating frame,
                                % (ignoring the velocity due to the earth's rotation)
else
    omegaRotatingFrame = [0;0;pi/300];  % rotation of the moving frame wrt fixed frame
    omegaFixed = [0;0;0];       % constant rotation rate measurement
    accelFixed = [1;0;0];     % constant acceleration measurement
    g = [0;0;0];                % Gravity
    initialPosition = [0; 1; 0];% initial position in both frames
    initialVelocity = [0;0;0];  % initial velocity in the rotating frame (ignoring the velocity due to the frame's rotation)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
numTests = 20
for testInd=1:numTests
  navFrameRotating = 0;
  accelFixed = 2*rand(3,1)-ones(3,1);
  imuSimulator.coriolisExample
  posFinErrorFixed(testInd) = norm(axisPositionsError(:,end))/trajectoryLengthFixedFrameGT*100
  rotFinErrorFixed(testInd) = norm(rotationsErrorVectors(:,end))
  velFinErrorFixed(testInd) =  norm(axisVelocityError(:,end))
  % Run the same initial conditions but navigating in the rotating frame
  navFrameRotating = 1;
  imuSimulator.coriolisExample
  posFinErrorRot(testInd) = norm(axisPositionsError(:,end))/trajLen*100
  rotFinErrorRot(testInd) = norm(rotationsErrorVectors(:,end))
  velFinErrorRot(testInd) =  norm(axisVelocityError(:,end))
end

mean(posFinErrorFixed)
max(posFinErrorFixed)

box(posFinErrorFixed)


