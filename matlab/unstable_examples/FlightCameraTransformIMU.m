% Simulation for concurrent IMU, camera, IMU-camera transform estimation during flight with known landmarks
% author: Chris Beall
% date: July 2014

clear all;
clf;

import gtsam.*;

write_video = true;

use_camera = true;
use_camera_transform_noise = true;
gps_noise = 0.5;           % normally distributed (meters)
landmark_noise = 0.2;      % normally distributed (meters)
nrLandmarks = 1000;         % Number of randomly generated landmarks

% ground-truth IMU-camera transform
camera_transform = Pose3(Rot3.RzRyRx(-pi, 0, -pi/2),Point3());

% noise to compose onto the above for initialization
camera_transform_noise = Pose3(Rot3.RzRyRx(0.1,0.1,0.1),Point3(0,0.02,0));

if(write_video)
    videoObj = VideoWriter('FlightCameraIMU_transform_GPS0_5_lm0_2_robust.avi');
    videoObj.Quality = 100;
    videoObj.FrameRate = 10;
    open(videoObj);
end

%% IMU parameters
IMU_metadata.AccelerometerSigma = 1e-2;    
IMU_metadata.GyroscopeSigma = 1e-2;
IMU_metadata.AccelerometerBiasSigma = 1e-6;
IMU_metadata.GyroscopeBiasSigma = 1e-6;
IMU_metadata.IntegrationSigma = 1e-1;

n_gravity = [0;0;-9.8];
IMU_params = PreintegrationParams(n_gravity);
IMU_params.setAccelerometerCovariance(IMU_metadata.AccelerometerSigma.^2 * eye(3));
IMU_params.setGyroscopeCovariance(IMU_metadata.GyroscopeSigma.^2 * eye(3));
IMU_params.setIntegrationCovariance(IMU_metadata.IntegrationSigma.^2 * eye(3));

transformKey = 1000;
calibrationKey = 2000;

fg = NonlinearFactorGraph;
initial = Values;

%% some noise models
trans_cov = noiseModel.Diagonal.Sigmas([5*pi/180; 5*pi/180; 5*pi/180; 20; 20; 0.1]);
GPS_trans_cov = noiseModel.Diagonal.Sigmas([3; 3; 4]);
K_cov = noiseModel.Diagonal.Sigmas([20; 20; 0.001; 20; 20]);

l_cov = noiseModel.Diagonal.Sigmas([landmark_noise; landmark_noise; landmark_noise]);
z_cov = noiseModel.Diagonal.Sigmas([1.0;1.0]);
% z_cov = noiseModel.Robust(noiseModel.mEstimator.Huber(1.0), noiseModel.Diagonal.Sigmas([1.0;1.0]));

%% calibration initialization
K = Cal3_S2(20,1280,960);

% initialize K incorrectly
K_corrupt = Cal3_S2(K.fx()+10,K.fy()+10,0,K.px(),K.py());

isamParams = ISAM2Params;
isamParams.setFactorization('QR');
isam = ISAM2(isamParams);

%% Get initial conditions for the estimated trajectory
currentVelocityGlobal = [10;0;0];    % (This is slightly wrong!) Zhaoyang: Fixed
currentBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1));

sigma_init_v = noiseModel.Isotropic.Sigma(3, 1.0);
sigma_init_b = noiseModel.Isotropic.Sigmas([ 0.100; 0.100; 0.100; 5.00e-05; 5.00e-05; 5.00e-05 ]);
sigma_between_b = [ IMU_metadata.AccelerometerBiasSigma * ones(3,1); IMU_metadata.GyroscopeBiasSigma * ones(3,1) ];
w_coriolis = [0;0;0];

%% generate trajectory and landmarks
trajectory = flight_trajectory();
landmarks = ground_landmarks(nrLandmarks);

figure(1);
% 3D map subplot
a1 = subplot(2,2,1);
grid on;

plot3DTrajectory(trajectory,'-b',true,5);
plot3DPoints(landmarks,'*g');
axis([-800 800 -800 800 0 1600]);
axis equal;
hold on;
view(-37,40);

% camera subplot
a2 = subplot(2,2,2);
if ~use_camera
    title('Camera Off');
end

% IMU-cam transform subplot
a3 = subplot(2,2,3);
view(-37,40);
axis([-1 1 -1 1 -1 1]);
grid on;
xlabel('x');
ylabel('y');
zlabel('z');
title('Estimated vs. actual IMU-cam transform');
axis equal;

%% Main loop
for i=1:size(trajectory)-1
    %% Preliminaries
    xKey = symbol('x',i);
    pose = trajectory.atPose3(xKey);     % GT pose
    pose_t = pose.translation();    % GT pose-translation
    
    if exist('h_cursor','var')
        delete(h_cursor);
    end
    
    % current ground-truth position indicator
    h_cursor = plot3(a1, pose_t(1),pose_t(2),pose_t(3),'*');
 
    camera_pose = pose.compose(camera_transform);
    
    axes(a2);
    if use_camera
        % project (and plot 2D camera view inside)
        measurements = project_landmarks(camera_pose,landmarks, K);
        % plot red landmarks in 3D plot
        plot_projected_landmarks(a1, landmarks, measurements);
    else
        measurements = Values;
    end
    
    %% ISAM stuff
    currentVelKey =  symbol('v',i);
    currentBiasKey = symbol('b',i);
    
    initial.insert(currentVelKey, currentVelocityGlobal);
    initial.insert(currentBiasKey, currentBias);
    
    % prior on translation, sort of like GPS with noise!
    gps_pose = pose.retract([0; 0; 0; normrnd(0,gps_noise,3,1)]);
    fg.add(PoseTranslationPrior3D(xKey, gps_pose, GPS_trans_cov));
    
    %% First-time initialization
    if i==1
        % camera transform
        if use_camera_transform_noise
            camera_transform_init = camera_transform.compose(camera_transform_noise);
        else
            camera_transform_init = camera_transform;
        end
        initial.insert(transformKey,camera_transform_init);
        fg.add(PriorFactorPose3(transformKey,camera_transform_init,trans_cov));
        
        % calibration
        initial.insert(2000, K_corrupt);
        fg.add(PriorFactorCal3_S2(calibrationKey,K_corrupt,K_cov));
        
        initial.insert(xKey, pose);
        
        result = initial;
    end
    
    %% priors on first two poses
    if i < 3        
        % fg.add(PriorFactorVector(currentVelKey, currentVelocityGlobal, sigma_init_v));
        fg.add(PriorFactorConstantBias(currentBiasKey, currentBias, sigma_init_b));
    end
   
    %% the 'normal' case
    if i > 1
     
        xKey_prev = symbol('x',i-1);
        pose_prev = trajectory.atPose3(xKey_prev);
        
        step = pose_prev.between(pose);
                
        % insert estimate for current pose with some normal noise on
        % translation
        initial.insert(xKey,result.atPose3(xKey_prev).compose(step.retract([0; 0; 0; normrnd(0,0.2,3,1)])));
        
        % visual measurements
        if measurements.size > 0 && use_camera
            measurementKeys = KeyVector(measurements.keys);

            for zz = 0:measurementKeys.size-1
                zKey = measurementKeys.at(zz);
                lKey = symbol('l',symbolIndex(zKey));

                fg.add(ProjectionFactorPPPCCal3_S2(measurements.atPoint2(zKey), ...
                    z_cov, xKey, transformKey, lKey, calibrationKey, false, true));

                % only add landmark to values if doesn't exist yet
                if ~result.exists(lKey)
                    p = landmarks.atPoint3(lKey);
                    n = normrnd(0,landmark_noise,3,1);
                    noisy_landmark = p + n;
                    initial.insert(lKey, noisy_landmark);

                    % and add a prior since its position is known
                    fg.add(PriorFactorPoint3(lKey, noisy_landmark,l_cov));
                end
            end
        end % end landmark observations 
        
        %% IMU
        deltaT = 1;
        logmap = Pose3.Logmap(step);
        omega = logmap(1:3);
        velocity = logmap(4:6);
       
        % Simulate IMU measurements, considering Coriolis effect 
        % (in this simple example we neglect gravity and there are no other forces acting on the body)
        acc_omega = imuSimulator.calculateIMUMeas_coriolis( ...
        omega, omega, velocity, velocity, deltaT);
    
%         [ currentIMUPoseGlobal, currentVelocityGlobal ] = imuSimulator.integrateTrajectory( ...
%     currentIMUPoseGlobal, omega, velocity, velocity, deltaT);

        currentSummarizedMeasurement = PreintegratedImuMeasurements(IMU_params,currentBias);
    
        accMeas = acc_omega(1:3)-n_gravity;
        omegaMeas = acc_omega(4:6);
        currentSummarizedMeasurement.integrateMeasurement(accMeas, omegaMeas, deltaT);

        %% create IMU factor
        fg.add(ImuFactor( ...
        xKey_prev, currentVelKey-1, ...
        xKey, currentVelKey, ...
        currentBiasKey, currentSummarizedMeasurement));
    
        % Bias evolution as given in the IMU metadata
        fg.add(BetweenFactorConstantBias(currentBiasKey-1, currentBiasKey, imuBias.ConstantBias(zeros(3,1), zeros(3,1)), ...
        noiseModel.Diagonal.Sigmas(sqrt(10) * sigma_between_b)));
    
        %% ISAM update
        isam.update(fg, initial);
        result = isam.calculateEstimate();
        
        %% reset 
        initial = Values;
        fg = NonlinearFactorGraph;
        
        currentVelocityGlobal = result.atPoint3(currentVelKey);
        currentBias = result.atConstantBias(currentBiasKey);
        
        %% plot current pose result
        isam_pose = result.atPose3(xKey);
        pose_t = isam_pose.translation();

        if exist('h_result','var')
            delete(h_result);
        end

        h_result = plot3(a1, pose_t(1),pose_t(2),pose_t(3),'^b', 'MarkerSize', 10);
        title(a1, sprintf('Step %d', i));
        
        if exist('h_text1(1)', 'var')
            delete(h_text1(1));
%             delete(h_text2(1));
        end
        t = result.atPose3(transformKey).translation();
        ty = t(2);
        K_estimate = result.atCal3_S2(calibrationKey);
        K_errors = K.localCoordinates(K_estimate);
        
        camera_transform_estimate = result.atPose3(transformKey);
        
        fx = result.atCal3_S2(calibrationKey).fx();
        fy = result.atCal3_S2(calibrationKey).fy();
%         h_text1 = text(-600,0,0,sprintf('Y-Transform(0.0): %0.2f',ty));
        text(0,1300,0,sprintf('Calibration and IMU-cam transform errors:'));
        
        entries = [{' f_x', ' f_y', ' s', 'p_x', 'p_y'}; num2cell(K_errors')];
        h_text1 = text(0,1750,0,sprintf('%s = %0.1f\n', entries{:}));
        
        camera_transform_errors = camera_transform.localCoordinates(camera_transform_estimate);
        entries1 = [{'ax', 'ay', 'az', 'tx', 'ty', 'tz'}; num2cell(camera_transform_errors')];
        h_text2 = text(600,1700,0,sprintf('%s = %0.2f\n', entries1{:}));
        
        % marginal is really huge
%         marginal_camera_transform = isam.marginalCovariance(transformKey);
        % plot transform
        axes(a3);
        cla;
  
        plotPose3(camera_transform,[],1);
        plotPose3(camera_transform_estimate,[],0.5);

    end
    
    drawnow;
    if(write_video)
        currFrame = getframe(gcf);
        writeVideo(videoObj, currFrame)
    else
        pause(0.00001);
    end
  
    
end

%% print out final camera transform and write video
result.atPose3(transformKey);
if(write_video)
    close(videoObj);
end