%% coriolisExample.m
% Author(s): David Jensen (david.jensen@gtri.gatech.edu)
% This script demonstrates the relationship between object motion in inertial and rotating reference frames.
% For this example, we consider a fixed (inertial) reference frame located at the center of the earth and initially
% coincident with the rotating ECEF reference frame (X towards 0 longitude, Y towards 90 longitude, Z towards 90 latitude),
% which rotates with the earth.
% A body is moving in the positive Z-direction and positive Y-direction with respect to the fixed reference frame.
% Its position is plotted in both the fixed and rotating reference frames to simulate how an observer in each frame would
% experience the body's motion.

clc
clear all
close all

import gtsam.*;

addpath(genpath('./Libraries'))

%% General configuration
navFrameRotating = 1;       % 0 = perform navigation in the fixed frame
                            % 1 = perform navigation in the rotating frame
IMU_type = 1;               % IMU type 1 or type 2
useRealisticValues = 0;     % use reaslist values for initial position and earth rotation
record_movie = 0;           % 0 = do not record movie
                            % 1 = record movie of the trajectories. One
                            % frame per time step (15 fps)

%% Scenario Configuration                            
deltaT = 0.01;              % timestep
timeElapsed = 5;            % Total elapsed time
times = 0:deltaT:timeElapsed;

% Initial Conditions
omegaEarthSeconds = [0;0;7.292115e-5]; % Earth Rotation rate (rad/s)
if useRealisticValues == 1
    omegaRotatingFrame = omegaEarthSeconds; % rotation of the moving frame wrt fixed frame
    omegaFixed = [0;0;0];       % constant rotation rate measurement
    accelFixed = [0.5;-0.5;0];       % constant acceleration measurement
    g = [0;0;0];                % Gravity
    initialVelocity = [0;0;0];      % initial velocity
    initialPosition = [4509997.76107; 4509997.76107; 3189050];  % initial position
else
    omegaRotatingFrame = [0;0;pi/300];  % rotation of the moving frame wrt fixed frame
    omegaFixed = [0;0;0];       % constant rotation rate measurement
    accelFixed = [0.1;0;0];     % constant acceleration measurement
    g = [0;0;0];                % Gravity
    initialVelocity = [0;0;0];      % initial velocity
    initialPosition = [0; 1; 0];    % initial position, at 45 degrees longitude and 30 degrees latitude on earth surface
end

if navFrameRotating == 0
    omegaCoriolisIMU = [0;0;0];
else
    omegaCoriolisIMU = omegaRotatingFrame;
end

%
currentRotatingFrame = Pose3;   % rotating frame initially coincides with fixed frame at t=0
currentPoseFixedGT = Pose3(Rot3, Point3(initialPosition));
currentPoseRotatingGT = currentPoseFixedGT; % frames coincide for t=0
currentVelocityFixedGT = initialVelocity;
%
epsBias = 1e-15;
sigma_init_x = noiseModel.Isotropic.Sigma(6, 1e-10);
sigma_init_v = noiseModel.Isotropic.Sigma(3, 1e-10);
sigma_init_b = noiseModel.Isotropic.Sigma(6, epsBias);

% Imu metadata
zeroBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1)); % bias is not of interest and is set to zero
IMU_metadata.AccelerometerSigma = 1e-5;
IMU_metadata.GyroscopeSigma = 1e-7;
IMU_metadata.IntegrationSigma = 1e-10;
IMU_metadata.BiasAccelerometerSigma = 1e-5;
IMU_metadata.BiasGyroscopeSigma = 1e-7;
IMU_metadata.BiasAccOmegaInit = 1e-10;

%% Initialize storage variables
positionsInFixedGT = zeros(3, length(times));
velocityInFixedGT = zeros(3, length(times));

positionsInRotatingGT = zeros(3, length(times));
velocityInRotatingGT = zeros(3, length(times));

positionsEstimates = zeros(3,length(times));
velocityEstimates = zeros(3,length(times));
%rotationsEstimates = zeros(3,length(times));

changePoseRotatingFrame = Pose3.Expmap([omegaRotatingFrame*deltaT; 0; 0; 0]); % rotation of the rotating frame at each time step
h = figure(1);

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
fprintf('IMU_type = %d\n', IMU_type);
fprintf('Record Movie = %d\n', record_movie);
fprintf('Realistic Values = %d\n', useRealisticValues);
fprintf('deltaT = %f\n', deltaT);
fprintf('timeElapsed = %f\n', timeElapsed);
fprintf('omegaRotatingFrame = [%f %f %f]\n', omegaRotatingFrame(1), omegaRotatingFrame(2), omegaRotatingFrame(3));
fprintf('omegaFixed = [%f %f %f]\n', omegaFixed(1), omegaFixed(2), omegaFixed(3));
fprintf('accelFixed = [%f %f %f]\n', accelFixed(1), accelFixed(2), accelFixed(3));
fprintf('Initial Velocity = [%f %f %f]\n', initialVelocity(1), initialVelocity(2), initialVelocity(3));
fprintf('Initial Position = [%f %f %f]\n', initialPosition(1), initialPosition(2), initialPosition(3));
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
        currentPoseEstimate = currentPoseFixedGT; % known initial conditions
        currentVelocityEstimate = LieVector(currentVelocityFixedGT); % known initial conditions
        
        % Set Priors
        newValues.insert(currentPoseKey, currentPoseEstimate);
        newValues.insert(currentVelKey, currentVelocityEstimate);
        newValues.insert(currentBiasKey, zeroBias);
        % Initial values, same for IMU types 1 and 2
        newFactors.add(PriorFactorPose3(currentPoseKey, currentPoseEstimate, sigma_init_x));
        newFactors.add(PriorFactorLieVector(currentVelKey, currentVelocityEstimate, sigma_init_v));
        newFactors.add(PriorFactorConstantBias(currentBiasKey, zeroBias, sigma_init_b));
        
        % Store data
        positionsInFixedGT(:,1) = currentPoseFixedGT.translation.vector;
        velocityInFixedGT(:,1) = currentVelocityFixedGT;
        positionsInRotatingGT(:,1) = currentPoseRotatingGT.translation.vector;
        %velocityInRotatingGT(:,1) = currentPoseRotatingGT.velocity.vector;
        positionsEstimates(:,i) = currentPoseEstimate.translation.vector;
        currentRotatingFrameForPlot(1).p = currentRotatingFrame.translation.vector;
        currentRotatingFrameForPlot(1).R = currentRotatingFrame.rotation.matrix;
    else
        
        %% Create ground truth trajectory
        % Update the position and velocity
        % x_t = x_0 + v_0*dt + 1/2*a_0*dt^2
        % v_t = v_0 + a_0*dt
        currentPositionFixedGT = Point3(currentPoseFixedGT.translation.vector ...
            + currentVelocityFixedGT * deltaT + 0.5 * accelFixed * deltaT * deltaT);
        currentVelocityFixedGT = currentVelocityFixedGT + accelFixed * deltaT;
        
        currentPoseFixedGT = Pose3(Rot3, currentPositionFixedGT); % constant orientation
        
        % Rotate pose in fixed frame to get pose in rotating frame
        currentRotatingFrame = currentRotatingFrame.compose(changePoseRotatingFrame);
        inverseCurrentRotatingFrame = (currentRotatingFrame.inverse);
        currentPoseRotatingGT = inverseCurrentRotatingFrame.compose(currentPoseFixedGT);
        
        % Get velocity in rotating frame by treating it like a position and using compose 
        % Maybe Luca knows a better way to do this within GTSAM. 
        currentVelocityPoseFixedGT = Pose3(Rot3, Point3(currentVelocityFixedGT));
        currentVelocityPoseRotatingGT = inverseCurrentRotatingFrame.compose(currentVelocityPoseFixedGT);
        
        % Store GT (ground truth) poses
        positionsInFixedGT(:,i) = currentPoseFixedGT.translation.vector;
        velocityInFixedGT(:,i) = currentVelocityFixedGT;
        positionsInRotatingGT(:,i) = currentPoseRotatingGT.translation.vector;
        velocityInRotatingGT(:,i) = currentVelocityPoseRotatingGT.translation.vector;
        currentRotatingFrameForPlot(i).p = currentRotatingFrame.translation.vector;
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
        else
            error('imuSimulator:coriolisExample:IMU_typeNotFound', ...
                'IMU_type = %d does not exist.\nAvailable IMU types are 1 and 2\n', IMU_type);
        end

        % Add values to the graph. Use the current pose and velocity
        % estimates as to values when interpreting the next pose and
        % velocity estimates
        newValues.insert(currentPoseKey, currentPoseEstimate);
        newValues.insert(currentVelKey, currentVelocityEstimate);
        newValues.insert(currentBiasKey, zeroBias);
        
        %newFactors.print(''); newValues.print('');
        
        %% Solve factor graph
        if(i > 1)
            isam.update(newFactors, newValues);
            newFactors = NonlinearFactorGraph;
            newValues = Values;
            
            % Get the new pose, velocity, and bias estimates
            currentPoseEstimate = isam.calculateEstimate(currentPoseKey);
            currentVelocityEstimate = isam.calculateEstimate(currentVelKey);
            currentBias = isam.calculateEstimate(currentBiasKey);
            
            positionsEstimates(:,i) = currentPoseEstimate.translation.vector;
            %rotationsEstimates(:,i) = currentPoseEstimate.rotation.vector;
            velocitiesEstimates(:,i) = currentVelocityEstimate.vector;
            biasEstimates(:,i) = currentBias.vector;
        end
    end
    
    %% incremental plotting for animation (ground truth)
    figure(h)
    %plot_trajectory(currentRotatingFrameForPlot(i),1, '-k', 'Rotating Frame',0.1,0.75,1)
    %hold on;
    plot3(positionsInFixedGT(1,1:i), positionsInFixedGT(2,1:i), positionsInFixedGT(3,1:i),'r');
    hold on;
    %plot3(positionsInFixedGT(1,1), positionsInFixedGT(2,1), positionsInFixedGT(3,1), 'x');
    %plot3(positionsInFixedGT(1,i), positionsInFixedGT(2,i), positionsInFixedGT(3,i), 'o');
    
    plot3(positionsInRotatingGT(1,1:i), positionsInRotatingGT(2,1:i), positionsInRotatingGT(3,1:i), '-g');
    %plot3(positionsInRotatingGT(1,1), positionsInRotatingGT(2,1), positionsInRotatingGT(3,1), 'xg');
    %plot3(positionsInRotatingGT(1,i), positionsInRotatingGT(2,i), positionsInRotatingGT(3,i), 'og');
    
    plot3(positionsEstimates(1,1:i), positionsEstimates(2,1:i), positionsEstimates(3,1:i), '-b');
    %plot3(positionsEstimates(1,1), positionsEstimates(2,1), positionsEstimates(3,1), 'xb');
    %plot3(positionsEstimates(1,i), positionsEstimates(2,i), positionsEstimates(3,i), 'ob');
    
    hold off;
    xlabel('X axis')
    ylabel('Y axis')
    zlabel('Z axis')
    axis equal
    grid on;
    %pause(0.1);
    
    if record_movie == 1
        frame = getframe(gcf); 
        writeVideo(writerObj,frame);
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
if navFrameRotating == 0
    axisPositionsError = positionsInFixedGT - positionsEstimates;
    axisVelocityError = velocityInFixedGT - velocityEstimates;
else
    axisPositionsError = positionsInRotatingGT - positionsEstimates;
    axisVelocityError = velocityInRotatingGT - velocityEstimates;
end
figure
plot(times, axisPositionsError);
plotTitle = sprintf('Axis Error in Estimated Position\n(IMU type = %d, omega = [%.2f; %.2f; %.2f])', ...
    IMU_type, omegaRotatingFrame(1), omegaRotatingFrame(2), omegaRotatingFrame(3));
title(plotTitle);
xlabel('Time');
ylabel('Error (ground_truth - estimate) [m]');
legend('X axis', 'Y axis', 'Z axis', 'Location', 'NORTHWEST');

figure
positionError3D = sqrt(axisPositionsError(1,:).^2+axisPositionsError(2,:).^2 + axisPositionsError(3,:).^2);
plot(times, positionError3D);
plotTitle = sprintf('3D Error in Estimated Position\n(IMU type = %d, omega = [%.2f; %.2f; %.2f])', ...
    IMU_type, omegaRotatingFrame(1), omegaRotatingFrame(2), omegaRotatingFrame(3));
title(plotTitle);
xlabel('Time');
ylabel('3D error [meters]');

figure
plot(times, axisVelocityError);
plotTitle = sprintf('Axis Error in Estimated Velocity\n(IMU type = %d, omega = [%.2f; %.2f; %.2f])', ...
    IMU_type, omegaRotatingFrame(1), omegaRotatingFrame(2), omegaRotatingFrame(3));
title(plotTitle);
xlabel('Time');
ylabel('Error (ground_truth - estimate) [m/s]');
legend('X axis', 'Y axis', 'Z axis', 'Location', 'NORTHWEST');

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

%% Plot final trajectories
figure
sphere  % sphere for reference
hold on;
% Ground truth trajectory in fixed reference frame
plot3(positionsInFixedGT(1,:), positionsInFixedGT(2,:), positionsInFixedGT(3,:),'r');
plot3(positionsInFixedGT(1,1), positionsInFixedGT(2,1), positionsInFixedGT(3,1), 'xr');
plot3(positionsInFixedGT(1,end), positionsInFixedGT(2,end), positionsInFixedGT(3,end), 'or');

% Ground truth trajectory in rotating reference frame
plot3(positionsInRotatingGT(1,:), positionsInRotatingGT(2,:), positionsInRotatingGT(3,:), '-g');
plot3(positionsInRotatingGT(1,1), positionsInRotatingGT(2,1), positionsInRotatingGT(3,1), 'xg');
plot3(positionsInRotatingGT(1,end), positionsInRotatingGT(2,end), positionsInRotatingGT(3,end), 'og');

% Estimates
plot3(positionsEstimates(1,:), positionsEstimates(2,:), positionsEstimates(3,:), '-b');
plot3(positionsEstimates(1,1), positionsEstimates(2,1), positionsEstimates(3,1), 'xb');
plot3(positionsEstimates(1,end), positionsEstimates(2,end), positionsEstimates(3,end), 'ob');

xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
axis equal
grid on;
hold off;
