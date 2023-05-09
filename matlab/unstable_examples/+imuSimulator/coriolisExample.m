%% coriolisExample.m
% Author(s): David Jensen (david.jensen@gtri.gatech.edu)
% This script demonstrates the relationship between object motion in inertial and rotating reference frames.
% For this example, we consider a fixed (inertial) reference frame located at the center of the earth and initially
% coincident with the rotating ECEF reference frame (X towards 0 longitude, Y towards 90 longitude, Z towards 90 latitude),
% which rotates with the earth.
% A body is moving in the positive Z-direction and positive Y-direction with respect to the fixed reference frame.
% Its position is plotted in both the fixed and rotating reference frames to simulate how an observer in each frame would
% experience the body's motion.

import gtsam.*;

addpath(genpath('./Libraries'))

% Check for an external configuarion. This is useful for setting up
% multiple tests
if ~exist('externalCoriolisConfiguration', 'var')
    clc
    clear all
    close all
    %% General configuration
    navFrameRotating = 1;       % 0 = perform navigation in the fixed frame
                                % 1 = perform navigation in the rotating frame
    IMU_type = 1;               % IMU type 1 or type 2
    useRealisticValues = 1;     % use reaslist values for initial position and earth rotation
    record_movie = 0;           % 0 = do not record movie
                                % 1 = record movie of the trajectories. One
                                % frame per time step (15 fps)
    incrementalPlotting = 1;    % turn incremental plotting on and off. Turning plotting off increases
                                % speed for batch testing
    estimationEnabled = 1;

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
        accelFixed = [-0.5;0.5;2];  % constant acceleration measurement
        g = [0;0;0];                % Gravity
        % initial position at some [longitude, latitude] location on earth's
        % surface (approximating Earth as a sphere)
        initialLongitude = 45;      % longitude in degrees
        initialLatitude = 30;       % latitude in degrees
        initialAltitude = 0;        % Altitude above Earth's surface in meters
        [x, y, z] = sph2cart(initialLongitude * pi/180, initialLatitude * pi/180, radiusEarth + initialAltitude);
        initialPosition = [x; y; z];
        initialVelocity = [0; 0; 0];% initial velocity of the body in the rotating frame,
                                    % (ignoring the velocity due to the earth's rotation)
    else
        omegaRotatingFrame = [0;0;pi/5];  % rotation of the moving frame wrt fixed frame
        omegaFixed = [0;0;0];       % constant rotation rate measurement
        accelFixed = [0.5;0.5;0.5];     % constant acceleration measurement
        g = [0;0;0];                % Gravity
        initialPosition = [1; 0; 0];% initial position in both frames
        initialVelocity = [0;0;0];  % initial velocity in the rotating frame (ignoring the velocity due to the frame's rotation)
    end
    
    if navFrameRotating == 0
      omegaCoriolisIMU = [0;0;0];
    else
      omegaCoriolisIMU = omegaRotatingFrame;
    end
end


% From Wikipedia Angular Velocity page, dr/dt = W*r, where r is
% position vector and W is angular velocity tensor
% We add the initial velocity in the rotating frame because they
% are the same frame at t=0, so no transformation is needed
angularVelocityTensor = [         0             -omegaRotatingFrame(3)  omegaRotatingFrame(2);
                         omegaRotatingFrame(3)            0            -omegaRotatingFrame(1);
                         -omegaRotatingFrame(2) omegaRotatingFrame(1)             0         ];
initialVelocityFixedFrame = angularVelocityTensor * initialPosition + initialVelocity;
initialVelocityRotatingFrame = initialVelocity;
                     
%
currentRotatingFrame = Pose3;   % rotating frame initially coincides with fixed frame at t=0
currentPoseFixedGT = Pose3(Rot3, Point3(initialPosition));
currentPoseRotatingGT = currentPoseFixedGT; % frames coincide for t=0
currentVelocityFixedGT = initialVelocityFixedFrame;
currentVelocityRotatingGT = initialVelocityRotatingFrame;
%
epsBias = 1e-20;
sigma_init_x = noiseModel.Isotropic.Sigma(6, 1e-10);
sigma_init_v = noiseModel.Isotropic.Sigma(3, 1e-10);
sigma_init_b = noiseModel.Isotropic.Sigma(6, epsBias);

% Imu metadata
zeroBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1)); % bias is not of interest and is set to zero
IMU_metadata.AccelerometerSigma = 1e-5;
IMU_metadata.GyroscopeSigma = 1e-7;
IMU_metadata.IntegrationSigma = 1e-10;
IMU_metadata.BiasAccelerometerSigma = epsBias;
IMU_metadata.BiasGyroscopeSigma = epsBias;
IMU_metadata.BiasAccOmegaInit = epsBias;

%% Initialize storage variables
positionsInFixedGT = zeros(3, length(times));
velocityInFixedGT = zeros(3, length(times));

positionsInRotatingGT = zeros(3, length(times));
velocityInRotatingGT = zeros(3, length(times));

positionsEstimates = zeros(3,length(times));
velocitiesEstimates = zeros(3,length(times));

rotationsErrorVectors = zeros(3,length(times));   % Rotating/Fixed frame selected later

changePoseRotatingFrame = Pose3.Expmap([omegaRotatingFrame*deltaT; 0; 0; 0]); % rotation of the rotating frame at each time step
h = figure;

% Solver object
isamParams = ISAM2Params;
isamParams.setFactorization('CHOLESKY');
isamParams.setRelinearizeSkip(10);
isam = gtsam.ISAM2(isamParams);
newFactors = NonlinearFactorGraph;
newValues = Values;

% Video recording object
if record_movie == 1
    writerObj = VideoWriter('trajectories.avi');
    writerObj.Quality = 100; 
    writerObj.FrameRate = 15; %10; 
    open(writerObj);
    set(gca,'nextplot','replacechildren');
    set(gcf,'Renderer','zbuffer');
end

%% Print Info about the test
fprintf('\n-------------------------------------------------\n');
if navFrameRotating == 0
    fprintf('Performing navigation in the Fixed frame.\n');
else
    fprintf('Performing navigation in the Rotating frame.\n');
end
fprintf('Estimation Enabled = %d\n', estimationEnabled);
fprintf('IMU_type = %d\n', IMU_type);
fprintf('Record Movie = %d\n', record_movie);
fprintf('Realistic Values = %d\n', useRealisticValues);
fprintf('deltaT = %f\n', deltaT);
fprintf('timeElapsed = %f\n', timeElapsed);
fprintf('omegaRotatingFrame = [%f %f %f]\n', omegaRotatingFrame(1), omegaRotatingFrame(2), omegaRotatingFrame(3));
fprintf('omegaCoriolisIMU = [%f %f %f]\n', omegaCoriolisIMU(1), omegaCoriolisIMU(2), omegaCoriolisIMU(3));
fprintf('omegaFixed = [%f %f %f]\n', omegaFixed(1), omegaFixed(2), omegaFixed(3));
fprintf('accelFixed = [%f %f %f]\n', accelFixed(1), accelFixed(2), accelFixed(3));
fprintf('Initial Velocity = [%f %f %f]\n', initialVelocity(1), initialVelocity(2), initialVelocity(3));
if(exist('initialLatitude', 'var') && exist('initialLongitude', 'var'))
    fprintf('Initial Position\n\t[Long, Lat] = [%f %f] degrees\n\tEFEC = [%f %f %f]\n', ...
        initialLongitude, initialLatitude, initialPosition(1), initialPosition(2), initialPosition(3));
else
    fprintf('Initial Position = [%f %f %f]\n', initialPosition(1), initialPosition(2), initialPosition(3));
end
fprintf('\n');

%% Main loop: iterate through the ground truth trajectory, add factors
% and values to the factor graph, and perform inference
for i = 1:length(times)
    t = times(i);
    
    % Create keys for current state
    currentPoseKey = symbol('x', i);
    currentVelKey = symbol('v', i);
    currentBiasKey = symbol('b', i);
    
    %% Set priors on the first iteration
    if(i == 1)
        % known initial conditions
        currentPoseEstimate = currentPoseFixedGT;
        if navFrameRotating == 1
            currentVelocityEstimate = LieVector(currentVelocityRotatingGT);
        else
            currentVelocityEstimate = LieVector(currentVelocityFixedGT);
        end
        
        % Set Priors
        newValues.insert(currentPoseKey, currentPoseEstimate);
        newValues.insert(currentVelKey, currentVelocityEstimate);
        newValues.insert(currentBiasKey, zeroBias);
        % Initial values, same for IMU types 1 and 2
        newFactors.add(PriorFactorPose3(currentPoseKey, currentPoseEstimate, sigma_init_x));
        newFactors.add(PriorFactorLieVector(currentVelKey, currentVelocityEstimate, sigma_init_v));
        newFactors.add(PriorFactorConstantBias(currentBiasKey, zeroBias, sigma_init_b));
        
        % Store data
        positionsInFixedGT(:,1) = currentPoseFixedGT.translation;
        velocityInFixedGT(:,1) = currentVelocityFixedGT;
        positionsInRotatingGT(:,1) = currentPoseRotatingGT.translation;
        %velocityInRotatingGT(:,1) = currentPoseRotatingGT.velocity;
        positionsEstimates(:,i) = currentPoseEstimate.translation;
        velocitiesEstimates(:,i) = currentVelocityEstimate;
        currentRotatingFrameForPlot(1).p = currentRotatingFrame.translation;
        currentRotatingFrameForPlot(1).R = currentRotatingFrame.rotation.matrix;
    else
        
        %% Create ground truth trajectory
        % Update the position and velocity
        % x_t = x_0 + v_0*dt + 1/2*a_0*dt^2
        % v_t = v_0 + a_0*dt
        currentPositionFixedGT = Point3(currentPoseFixedGT.translation ...
            + currentVelocityFixedGT * deltaT + 0.5 * accelFixed * deltaT * deltaT);
        currentVelocityFixedGT = currentVelocityFixedGT + accelFixed * deltaT;
        
        currentPoseFixedGT = Pose3(Rot3, currentPositionFixedGT); % constant orientation
        
        % Rotate pose in fixed frame to get pose in rotating frame
        previousPositionRotatingGT = currentPoseRotatingGT.translation;
        currentRotatingFrame = currentRotatingFrame.compose(changePoseRotatingFrame);
        inverseCurrentRotatingFrame = (currentRotatingFrame.inverse);
        currentPoseRotatingGT = inverseCurrentRotatingFrame.compose(currentPoseFixedGT);
        currentPositionRotatingGT = currentPoseRotatingGT.translation;
        
        % Get velocity in rotating frame by treating it like a position and using compose 
        % Maybe Luca knows a better way to do this within GTSAM. 
        %currentVelocityPoseFixedGT = Pose3(Rot3, Point3(currentVelocityFixedGT-initialVelocityFixedFrame));
        %currentVelocityPoseRotatingGT = inverseCurrentRotatingFrame.compose(currentVelocityPoseFixedGT);
        
        %currentRot3RotatingGT = currentRotatingFrame.rotation();
        %currentVelocityRotatingGT = currentRot3RotatingGT.unrotate(Point3(currentVelocityFixedGT));
        % TODO: check if initial velocity in rotating frame is correct
        currentVelocityRotatingGT = (currentPositionRotatingGT-previousPositionRotatingGT)/deltaT;
        %currentVelocityRotatingGT = (currentPositionRotatingGT - previousPositionRotatingGT ...
        %    - 0.5 * accelFixed * deltaT * deltaT) / deltaT + accelFixed * deltaT;
        
        % Store GT (ground truth) poses
        positionsInFixedGT(:,i) = currentPoseFixedGT.translation;
        velocityInFixedGT(:,i) = currentVelocityFixedGT;
        positionsInRotatingGT(:,i) = currentPoseRotatingGT.translation;
        velocityInRotatingGT(:,i) = currentVelocityRotatingGT;
        currentRotatingFrameForPlot(i).p = currentRotatingFrame.translation;
        currentRotatingFrameForPlot(i).R = currentRotatingFrame.rotation.matrix;
        
        %% Estimate trajectory in rotating frame using GTSAM (ground truth measurements)
        % Instantiate preintegrated measurements class
        if IMU_type == 1
            currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
                zeroBias, ...
                IMU_metadata.AccelerometerSigma.^2 * eye(3), ...
                IMU_metadata.GyroscopeSigma.^2 * eye(3), ...
                IMU_metadata.IntegrationSigma.^2 * eye(3));
        elseif IMU_type == 2
            currentSummarizedMeasurement = gtsam.CombinedImuFactorPreintegratedMeasurements( ...
                zeroBias, ...
                IMU_metadata.AccelerometerSigma.^2 * eye(3), ...
                IMU_metadata.GyroscopeSigma.^2 * eye(3), ...
                IMU_metadata.IntegrationSigma.^2 * eye(3), ...
                IMU_metadata.BiasAccelerometerSigma.^2 * eye(3), ...
                IMU_metadata.BiasGyroscopeSigma.^2 * eye(3), ...
                IMU_metadata.BiasAccOmegaInit.^2 * eye(6));
        else
            error('imuSimulator:coriolisExample:IMU_typeNotFound', ...
                'IMU_type = %d does not exist.\nAvailable IMU types are 1 and 2\n', IMU_type);
        end
        
        % Add measurement
        currentSummarizedMeasurement.integrateMeasurement(accelFixed, omegaFixed, deltaT);
        
        % Add factors to graph
        if IMU_type == 1
            newFactors.add(ImuFactor( ...
                currentPoseKey-1, currentVelKey-1, ...
                currentPoseKey, currentVelKey, ...
                currentBiasKey-1, currentSummarizedMeasurement, g, omegaCoriolisIMU));
            newFactors.add(BetweenFactorConstantBias(currentBiasKey-1, currentBiasKey, zeroBias, ...
                noiseModel.Isotropic.Sigma(6, epsBias)));
            newFactors.add(PriorFactorConstantBias(currentBiasKey, zeroBias, ...
                noiseModel.Isotropic.Sigma(6, epsBias)));
        elseif IMU_type == 2
            newFactors.add(CombinedImuFactor( ...
                currentPoseKey-1, currentVelKey-1, ...
                currentPoseKey, currentVelKey, ...
                currentBiasKey-1, currentBiasKey, ...
                currentSummarizedMeasurement, g, omegaCoriolisIMU, ...
                noiseModel.Isotropic.Sigma(15, epsBias)));
            % TODO: prior on biases?
        end

        % Add values to the graph. Use the current pose and velocity
        % estimates as to values when interpreting the next pose and
        % velocity estimates
        %ImuFactor.Predict(currentPoseEstimate, currentVelocityEstimate, pose_j, vel_j, zeroBias, currentSummarizedMeasurement, g, omegaCoriolisIMU);
        newValues.insert(currentPoseKey, currentPoseEstimate);
        newValues.insert(currentVelKey, currentVelocityEstimate);
        newValues.insert(currentBiasKey, zeroBias);
        
        %newFactors.print(''); newValues.print('');
        
        %% Solve factor graph
        if(i > 1 && estimationEnabled)
            isam.update(newFactors, newValues);
            newFactors = NonlinearFactorGraph;
            newValues = Values;
            
            % Get the new pose, velocity, and bias estimates
            currentPoseEstimate = isam.calculateEstimate(currentPoseKey);
            currentVelocityEstimate = isam.calculateEstimate(currentVelKey);
            currentBias = isam.calculateEstimate(currentBiasKey);
            
            positionsEstimates(:,i) = currentPoseEstimate.translation;
            velocitiesEstimates(:,i) = currentVelocityEstimate;
            biasEstimates(:,i) = currentBias;
            
            % In matrix form: R_error = R_gt'*R_estimate
            % Perform Logmap on the rotation matrix to get a vector
            if navFrameRotating == 1
                rotationError = Rot3(currentPoseRotatingGT.rotation.matrix' * currentPoseEstimate.rotation.matrix);
            else
                rotationError = Rot3(currentPoseFixedGT.rotation.matrix' * currentPoseEstimate.rotation.matrix);
            end
            
            rotationsErrorVectors(:,i) = Rot3.Logmap(rotationError);
        end
    end
    
    %% incremental plotting for animation (ground truth)
    if incrementalPlotting == 1
        figure(h)
        %plot_trajectory(currentRotatingFrameForPlot(i),1, '-k', 'Rotating Frame',0.1,0.75,1)
        %hold on;
        plot3(positionsInFixedGT(1,1:i), positionsInFixedGT(2,1:i), positionsInFixedGT(3,1:i),'r');
        hold on;
        plot3(positionsInFixedGT(1,1), positionsInFixedGT(2,1), positionsInFixedGT(3,1), 'xr');
        plot3(positionsInFixedGT(1,i), positionsInFixedGT(2,i), positionsInFixedGT(3,i), 'or');

        plot3(positionsInRotatingGT(1,1:i), positionsInRotatingGT(2,1:i), positionsInRotatingGT(3,1:i), 'g');
        plot3(positionsInRotatingGT(1,1), positionsInRotatingGT(2,1), positionsInRotatingGT(3,1), 'xg');
        plot3(positionsInRotatingGT(1,i), positionsInRotatingGT(2,i), positionsInRotatingGT(3,i), 'og');

        if(estimationEnabled)
          plot3(positionsEstimates(1,1:i), positionsEstimates(2,1:i), positionsEstimates(3,1:i), 'b');
          plot3(positionsEstimates(1,1), positionsEstimates(2,1), positionsEstimates(3,1), 'xb');
          plot3(positionsEstimates(1,i), positionsEstimates(2,i), positionsEstimates(3,i), 'ob');
        end

        hold off;
        xlabel('X axis')
        ylabel('Y axis')
        zlabel('Z axis')
        axis equal
        grid on;
        %pause(0.1);

        % record frames for movie
        if record_movie == 1
            frame = getframe(gcf); 
            writeVideo(writerObj,frame);
        end
    end
end

if record_movie == 1
    close(writerObj);
end

% Calculate trajectory length
trajectoryLengthEstimated = 0;
trajectoryLengthFixedFrameGT = 0;
trajectoryLengthRotatingFrameGT = 0;
for i = 2:length(positionsEstimates)
    trajectoryLengthEstimated = trajectoryLengthEstimated + norm(positionsEstimates(:,i) - positionsEstimates(:,i-1));
    trajectoryLengthFixedFrameGT = trajectoryLengthFixedFrameGT + norm(positionsInFixedGT(:,i) - positionsInFixedGT(:,i-1));
    trajectoryLengthRotatingFrameGT = trajectoryLengthRotatingFrameGT + norm(positionsInRotatingGT(:,i) - positionsInRotatingGT(:,i-1));
end

%% Print and plot error results
% Calculate errors for appropriate navigation scheme
if navFrameRotating == 0
    axisPositionsError = positionsInFixedGT - positionsEstimates;
    axisVelocityError = velocityInFixedGT - velocitiesEstimates;
else
    axisPositionsError = positionsInRotatingGT - positionsEstimates;
    axisVelocityError = velocityInRotatingGT - velocitiesEstimates;
end

% Plot individual axis position errors
figure
plot(times, axisPositionsError);
plotTitle = sprintf('Axis Error in Estimated Position\n(IMU type = %d, omega = [%.2f; %.2f; %.2f])', ...
    IMU_type, omegaRotatingFrame(1), omegaRotatingFrame(2), omegaRotatingFrame(3));
title(plotTitle);
xlabel('Time [s]');
ylabel('Error (ground_truth - estimate) [m]');
legend('X axis', 'Y axis', 'Z axis', 'Location', 'NORTHWEST');

% Plot 3D position error
figure
positionError3D = sqrt(axisPositionsError(1,:).^2+axisPositionsError(2,:).^2 + axisPositionsError(3,:).^2);
plot(times, positionError3D);
plotTitle = sprintf('3D Error in Estimated Position\n(IMU type = %d, omega = [%.2f; %.2f; %.2f])', ...
    IMU_type, omegaRotatingFrame(1), omegaRotatingFrame(2), omegaRotatingFrame(3));
title(plotTitle);
xlabel('Time [s]');
ylabel('3D error [meters]');

% Plot 3D position error
if navFrameRotating == 0
    trajLen = trajectoryLengthFixedFrameGT;
else
    trajLen = trajectoryLengthRotatingFrameGT;
end

figure
plot(times, (positionError3D/trajLen)*100);
plotTitle = sprintf('3D Error normalized by distance in Estimated Position\n(IMU type = %d, omega = [%.2f; %.2f; %.2f])', ...
    IMU_type, omegaRotatingFrame(1), omegaRotatingFrame(2), omegaRotatingFrame(3));
title(plotTitle);
xlabel('Time [s]');
ylabel('normalized 3D error [% of distance traveled]');

% Plot individual axis velocity errors
figure
plot(times, axisVelocityError);
plotTitle = sprintf('Axis Error in Estimated Velocity\n(IMU type = %d, omega = [%.2f; %.2f; %.2f])', ...
    IMU_type, omegaRotatingFrame(1), omegaRotatingFrame(2), omegaRotatingFrame(3));
title(plotTitle);
xlabel('Time [s]');
ylabel('Error (ground_truth - estimate) [m/s]');
legend('X axis', 'Y axis', 'Z axis', 'Location', 'NORTHWEST');

% Plot 3D velocity error
figure
velocityError3D = sqrt(axisVelocityError(1,:).^2+axisVelocityError(2,:).^2 + axisVelocityError(3,:).^2);
plot(times, velocityError3D);
plotTitle = sprintf('3D Error in Estimated Velocity\n(IMU type = %d, omega = [%.2f; %.2f; %.2f])', ...
    IMU_type, omegaRotatingFrame(1), omegaRotatingFrame(2), omegaRotatingFrame(3));
title(plotTitle);
xlabel('Time [s]');
ylabel('3D error [meters/s]');

% Plot magnitude of rotation errors
figure
for i = 1:size(rotationsErrorVectors,2)
    rotationsErrorMagnitudes(i) = norm(rotationsErrorVectors(:,i));
end
plot(times,rotationsErrorMagnitudes);
title('Rotation Error');
xlabel('Time [s]');
ylabel('Error [rads]');


% Text output for errors
if navFrameRotating == 0
    fprintf('Fixed Frame ground truth trajectory length is %f [m]\n', trajectoryLengthFixedFrameGT);
    fprintf('Estimated trajectory length is %f [m]\n', trajectoryLengthEstimated);
    fprintf('Final position error = %f [m](%.4f%% of ground truth trajectory length)\n', ...
        norm(axisPositionsError(:,end)), norm(axisPositionsError(:,end))/trajectoryLengthFixedFrameGT*100);
else
    fprintf('Rotating Frame ground truth trajectory length is %f [m]\n', trajectoryLengthRotatingFrameGT);
    fprintf('Estimated trajectory length is %f [m]\n', trajectoryLengthEstimated);
    fprintf('Final position error = %f [m](%.4f%% of ground truth trajectory length)\n', ...
        norm(axisPositionsError(:,end)), norm(axisPositionsError(:,end))/trajectoryLengthRotatingFrameGT*100);
end
fprintf('Final velocity error = [%f %f %f] [m/s]\n', axisVelocityError(1,end), axisVelocityError(2,end), axisVelocityError(3,end));
fprintf('Final rotation error = %f [rads]\n', norm(rotationsErrorVectors(:,end)));

%% Plot final trajectories
figure
[x,y,z] = sphere(30);
if useRealisticValues
    mesh(radiusEarth*x,radiusEarth*y, radiusEarth*z) % where (a,b,c) is center of the sphere  % sphere for reference
else
    mesh(x,y,z);
end
hold on;
axis equal
% Ground truth trajectory in fixed reference frame
plot3(positionsInFixedGT(1,:), positionsInFixedGT(2,:), positionsInFixedGT(3,:),'r','lineWidth',4);
plot3(positionsInFixedGT(1,1), positionsInFixedGT(2,1), positionsInFixedGT(3,1), 'xr','lineWidth',4);
plot3(positionsInFixedGT(1,end), positionsInFixedGT(2,end), positionsInFixedGT(3,end), 'or','lineWidth',4);

% Ground truth trajectory in rotating reference frame
plot3(positionsInRotatingGT(1,:), positionsInRotatingGT(2,:), positionsInRotatingGT(3,:), '-g','lineWidth',4);
plot3(positionsInRotatingGT(1,1), positionsInRotatingGT(2,1), positionsInRotatingGT(3,1), 'xg','lineWidth',4);
plot3(positionsInRotatingGT(1,end), positionsInRotatingGT(2,end), positionsInRotatingGT(3,end), 'og','lineWidth',4);

% Estimates
if(estimationEnabled)
  plot3(positionsEstimates(1,:), positionsEstimates(2,:), positionsEstimates(3,:), '-b','lineWidth',4);
  plot3(positionsEstimates(1,1), positionsEstimates(2,1), positionsEstimates(3,1), 'xb','lineWidth',4);
  plot3(positionsEstimates(1,end), positionsEstimates(2,end), positionsEstimates(3,end), 'ob','lineWidth',4);
end

xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
axis equal
grid on;
hold off;

% TODO: logging rotation errors
%for all time steps
%    Rerror =  Rgt' * Restimated;
%    % transforming rotation matrix to axis-angle representation
%    vector_error = Rot3.Logmap(Rerror);
%    norm(vector_error)
%    
%   axis angle: [u,theta], with norm(u)=1
%   vector_error = u * theta;
   
% TODO: logging velocity errors
%velocities..
