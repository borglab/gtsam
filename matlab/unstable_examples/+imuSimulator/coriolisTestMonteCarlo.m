clc
clear all
close all

% Flag to mark external configuration to the main test script
externalCoriolisConfiguration = 1;

%% General configuration
navFrameRotating = 0;       % 0 = perform navigation in the fixed frame
                            % 1 = perform navigation in the rotating frame
IMU_type = 1;               % IMU type 1 or type 2
useRealisticValues = 1;     % use reaslist values for initial position and earth rotation
record_movie = 0;           % 0 = do not record movie
                            % 1 = record movie of the trajectories. One
                            % frame per time step (15 fps)
incrementalPlotting = 0;
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
    accelFixed = [0.1;0;1];  % constant acceleration measurement
    g = [0;0;0];                % Gravity
    
    initialLongitude = 45;      % longitude in degrees
    initialLatitude = 30;       % latitude in degrees
    initialAltitude = 0;        % Altitude above Earth's surface in meters
    [x, y, z] = sph2cart(initialLongitude * pi/180, initialLatitude * pi/180, radiusEarth + initialAltitude);
    initialPosition = [x; y; z];
    
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

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Run tests with randomly generated initialPosition and accelFixed values
% For each initial position, a number of acceleration vectors are
% generated. For each (initialPosition, accelFixed) pair, the coriolis test
% is run for the following 3 scenarios
%   - Navigation performed in fixed frame
%   - Navigation performed in rotating frame, including the coriolis effect
%   - Navigation performed in rotating frame, ignoring coriolis effect
%

% Testing configuration
numPosTests = 1;
numAccelTests = 10;
totalNumTests = numPosTests * numAccelTests;

% Storage variables

posFinErrorFixed = zeros(numPosTests, numAccelTests);
rotFinErrorFixed = zeros(numPosTests, numAccelTests);
velFinErrorFixed = zeros(numPosTests, numAccelTests);

posFinErrorRotCoriolis = zeros(numPosTests, numAccelTests);
rotFinErrorRotCoriolis = zeros(numPosTests, numAccelTests);
velFinErrorRotCoriolis = zeros(numPosTests, numAccelTests);

posFinErrorRot = zeros(numPosTests, numAccelTests);
rotFinErrorRot = zeros(numPosTests, numAccelTests);
velFinErrorRot = zeros(numPosTests, numAccelTests);


numErrors = 0;
testIndPos = 1;
testIndAccel = 1;

while testIndPos <= numPosTests
    %generate a random initial position vector
    initialLongitude = rand()*360 - 180;    % longitude in degrees (-90 to 90)
    initialLatitude = rand()*180 - 90;      % latitude in degrees (-180 to 180)
    initialAltitude = rand()*150;           % Altitude above Earth's surface in meters (0-150)
    [x, y, z] = sph2cart(initialLongitude * pi/180, initialLatitude * pi/180, radiusEarth + initialAltitude);
    initialPosition = [x; y; z];
    
    while testIndAccel <= numAccelTests
        [testIndPos testIndAccel]
        % generate a random acceleration vector
        accelFixed = 2*rand(3,1)-ones(3,1);
        
        %lla = oldTestInfo(testIndPos,testIndAccel).initialPositionLLA;
        %initialLongitude = lla(1);
        %initialLatitude = lla(2);
        %initialAltitude = lla(3);
        %initialPosition = oldTestInfo(testIndPos, testIndAccel).initialPositionECEF;
        
        testInfo(testIndPos, testIndAccel).initialPositionLLA = [initialLongitude, initialLatitude, initialAltitude]; 
        testInfo(testIndPos, testIndAccel).initialPositionECEF = initialPosition;
        testInfo(testIndPos, testIndAccel).accelFixed = accelFixed;
        
        try
            omegaCoriolisIMU = [0;0;0];
            navFrameRotating = 0;
            imuSimulator.coriolisExample
            posFinErrorFixed(testIndPos, testIndAccel) = norm(axisPositionsError(:,end))/trajectoryLengthFixedFrameGT*100;
            rotFinErrorFixed(testIndPos, testIndAccel) = norm(rotationsErrorVectors(:,end));
            velFinErrorFixed(testIndPos, testIndAccel) =  norm(axisVelocityError(:,end));
            testInfo(testIndPos, testIndAccel).fixedPositionError = axisPositionsError(:,end);
            testInfo(testIndPos, testIndAccel).fixedVelocityError = axisVelocityError(:,end);
            testInfo(testIndPos, testIndAccel).fixedRotationError = rotationsErrorVectors(:,end);
            testInfo(testIndPos, testIndAccel).fixedEstTrajLength = trajectoryLengthEstimated;
            testInfo(testIndPos, testIndAccel).trajLengthFixedFrameGT = trajectoryLengthFixedFrameGT;
            testInfo(testIndPos, testIndAccel).trajLengthRotFrameGT = trajectoryLengthRotatingFrameGT;
            
            close all;
            
            % Run the same initial conditions but navigating in the rotating frame
            %  enable coriolis effect by setting:
            omegaCoriolisIMU = omegaRotatingFrame;
            navFrameRotating = 1;
            imuSimulator.coriolisExample
            posFinErrorRotCoriolis(testIndPos, testIndAccel) = norm(axisPositionsError(:,end))/trajLen*100;
            rotFinErrorRotCoriolis(testIndPos, testIndAccel) = norm(rotationsErrorVectors(:,end));
            velFinErrorRotCoriolis(testIndPos, testIndAccel) =  norm(axisVelocityError(:,end));
            testInfo(testIndPos, testIndAccel).rotCoriolisPositionError = axisPositionsError(:,end);
            testInfo(testIndPos, testIndAccel).rotCoriolisVelocityError = axisVelocityError(:,end);
            testInfo(testIndPos, testIndAccel).rotCoriolisRotationError = rotationsErrorVectors(:,end);
            testInfo(testIndPos, testIndAccel).rotCoriolisEstTrajLength = trajectoryLengthEstimated;
            
            close all;
            
            % Run the same initial conditions but navigating in the rotating frame
            % disable coriolis effect by setting:
            omegaCoriolisIMU = [0;0;0];
            navFrameRotating = 1;
            imuSimulator.coriolisExample
            posFinErrorRot(testIndPos, testIndAccel) = norm(axisPositionsError(:,end))/trajLen*100;
            rotFinErrorRot(testIndPos, testIndAccel) = norm(rotationsErrorVectors(:,end));
            velFinErrorRot(testIndPos, testIndAccel) =  norm(axisVelocityError(:,end));
            testInfo(testIndPos, testIndAccel).rotPositionError = axisPositionsError(:,end);
            testInfo(testIndPos, testIndAccel).rotVelocityError = axisVelocityError(:,end);
            testInfo(testIndPos, testIndAccel).rotRotationError = rotationsErrorVectors(:,end);
            testInfo(testIndPos, testIndAccel).rotEstTrajLength = trajectoryLengthEstimated;
            
            close all;
        catch
            numErrors = numErrors + 1;
            fprintf('*ERROR*');
            fprintf('%d tests cancelled due to errors\n', numErrors);
            fprintf('restarting test with new accelFixed');
            testIndAccel = testIndAccel - 1;
        end
        testIndAccel = testIndAccel + 1;
    end
    testIndAccel = 1;
    testIndPos = testIndPos + 1;
end
fprintf('\nTotal: %d tests cancelled due to errors\n', numErrors);

mean(posFinErrorFixed);
max(posFinErrorFixed);
% box(posFinErrorFixed);

mean(posFinErrorRotCoriolis);
max(posFinErrorRotCoriolis);
% box(posFinErrorRotCoriolis);

mean(posFinErrorRot);
max(posFinErrorRot);
% box(posFinErrorRot);


