%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Estimate trajectory, calibration, landmarks, body-camera offset,
% IMU
% @author Chris Beall
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
clc;
import gtsam.*

write_video = false;

if(write_video)
    videoObj = VideoWriter('test.avi');
    videoObj.Quality = 100;
    videoObj.FrameRate = 2;
    open(videoObj);
end

%% generate some landmarks
nrPoints = 8;
 landmarks = {Point3([20 15 1]'),...
        Point3([22 7 -1]'),...
        Point3([20 20 6]'),...
        Point3([24 19 -4]'),...
        Point3([26 17 -2]'),...
        Point3([12 15 4]'),...
        Point3([25 11 -6]'),...
        Point3([23 10 4]')};
    
IMU_metadata.AccelerometerSigma = 1e-2;    
IMU_metadata.GyroscopeSigma = 1e-2;
IMU_metadata.AccelerometerBiasSigma = 1e-6;
IMU_metadata.GyroscopeBiasSigma = 1e-6;
IMU_metadata.IntegrationSigma = 1e-1;

curvature = 5.0;
transformKey = 1000;
calibrationKey = 2000;
steps = 50;

fg = NonlinearFactorGraph;
initial = Values;

%% intial landmarks and camera trajectory shifted in + y-direction
y_shift = Point3(0,0.5,0);

% insert shifted points
for i=1:nrPoints
   initial.insert(100+i,landmarks{i}.compose(y_shift)); 
end

figure(1);
cla
hold on;

%% initial pose priors
pose_cov = noiseModel.Diagonal.Sigmas([0.1*pi/180; 0.1*pi/180; 0.1*pi/180; 1e-4; 1e-4; 1e-4]);

%% Actual camera translation coincides with odometry, but -90deg Z-X rotation
camera_transform = Pose3(Rot3.RzRyRx(-pi/2, 0, -pi/2),y_shift);
actual_transform = Pose3(Rot3.RzRyRx(-pi/2, 0, -pi/2),Point3());

trans_cov = noiseModel.Diagonal.Sigmas([1*pi/180; 1*pi/180; 1*pi/180; 20; 1e-6; 1e-6]);


move_forward = Pose3(Rot3(),Point3(1,0,0));
move_circle = Pose3(Rot3.RzRyRx(0.0,0.0,curvature*pi/180),Point3(1,0,0));
covariance = noiseModel.Diagonal.Sigmas([5*pi/180; 5*pi/180; 5*pi/180; 0.05; 0.05; 0.05]);
z_cov = noiseModel.Diagonal.Sigmas([1.0;1.0]);
    
%% calibration initialization
K = Cal3_S2(900,900,0,640,480);
K_corrupt = Cal3_S2(910,890,0,650,470);
K_cov = noiseModel.Diagonal.Sigmas([20; 20; 0.001; 20; 20]);

cheirality_exception_count = 0;

isamParams = gtsam.ISAM2Params;
isamParams.setFactorization('QR');
isam = ISAM2(isamParams);

currentIMUPoseGlobal = Pose3();

%% Get initial conditions for the estimated trajectory
currentVelocityGlobal = [1;0;0]; % the vehicle is stationary at the beginning
currentBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1));

sigma_init_v = noiseModel.Isotropic.Sigma(3, 1.0);
sigma_init_b = noiseModel.Isotropic.Sigmas([ 0.100; 0.100; 0.100; 5.00e-05; 5.00e-05; 5.00e-05 ]);
sigma_between_b = [ IMU_metadata.AccelerometerBiasSigma * ones(3,1); IMU_metadata.GyroscopeBiasSigma * ones(3,1) ];
g = [0;0;-9.8];
w_coriolis = [0;0;0];


for i=1:steps
    
    t = i-1;
    
    currentVelKey =  symbol('v',i);
    currentBiasKey = symbol('b',i);
    
    initial.insert(currentVelKey, currentVelocityGlobal);
    initial.insert(currentBiasKey, currentBias);
    
    if i==1
        
        % Pose Priors
        fg.add(PriorFactorPose3(1, Pose3(),pose_cov));
        fg.add(PriorFactorPose3(2, Pose3(Rot3(),Point3(1,0,0)),pose_cov));
        
        % insert first 
        initial.insert(1, Pose3());
        
        % camera transform
        initial.insert(transformKey,camera_transform);
        fg.add(PriorFactorPose3(transformKey,camera_transform,trans_cov));

        % calibration
        initial.insert(2000, K_corrupt);
        fg.add(PriorFactorCal3_S2(calibrationKey,K_corrupt,K_cov));
        
        % velocity and bias evolution
        fg.add(PriorFactorVector(currentVelKey, currentVelocityGlobal, sigma_init_v));
        fg.add(PriorFactorConstantBias(currentBiasKey, currentBias, sigma_init_b));
        
        result = initial;
    end
    if i == 2
        fg.add(PriorFactorPose3(2, Pose3(Rot3(),Point3(1,0,0)),pose_cov));
        fg.add(PriorFactorLieVector(currentVelKey, currentVelocityGlobal, sigma_init_v));
        fg.add(PriorFactorConstantBias(currentBiasKey, currentBias, sigma_init_b));
    end
    if i > 1
        if i < 11   
            step = move_forward;
        else
            step = move_circle;
        end
        
        initial.insert(i,result.at(i-1).compose(step));
        fg.add(BetweenFactorPose3(i-1,i, step, covariance));
        
        deltaT = 1;
        logmap = Pose3.Logmap(step);
        omega = logmap(1:3);
        velocity = logmap(4:6);
        %% Simulate IMU measurements, considering Coriolis effect 
        % (in this simple example we neglect gravity and there are no other forces acting on the body)
        acc_omega = imuSimulator.calculateIMUMeas_coriolis( ...
        omega, omega, velocity, velocity, deltaT);
    
        [ currentIMUPoseGlobal, currentVelocityGlobal ] = imuSimulator.integrateTrajectory( ...
    currentIMUPoseGlobal, omega, velocity, velocity, deltaT);

        currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
        currentBias, IMU_metadata.AccelerometerSigma.^2 * eye(3), ...
        IMU_metadata.GyroscopeSigma.^2 * eye(3), IMU_metadata.IntegrationSigma.^2 * eye(3));
    
        accMeas = acc_omega(1:3)-g;
        omegaMeas = acc_omega(4:6);
        currentSummarizedMeasurement.integrateMeasurement(accMeas, omegaMeas, deltaT);

        %% create IMU factor
        fg.add(ImuFactor( ...
        i-1, currentVelKey-1, ...
        i, currentVelKey, ...
        currentBiasKey, currentSummarizedMeasurement, g, w_coriolis));
    
        % Bias evolution as given in the IMU metadata
        fg.add(BetweenFactorConstantBias(currentBiasKey-1, currentBiasKey, imuBias.ConstantBias(zeros(3,1), zeros(3,1)), ...
        noiseModel.Diagonal.Sigmas(sqrt(steps) * sigma_between_b)));

    end
    
    % generate some camera measurements
    cam_pose = currentIMUPoseGlobal.compose(actual_transform);
%     gtsam.plotPose3(cam_pose);
    cam = PinholeCameraCal3_S2(cam_pose,K);
    i
%     result
    for j=1:nrPoints
        % All landmarks seen in every frame
        try
            z = cam.project(landmarks{j});
            fg.add(TransformCalProjectionFactorCal3_S2(z, z_cov, i, transformKey, 100+j, calibrationKey, false, true));
        catch
            cheirality_exception_count = cheirality_exception_count + 1;
        end % end try/catch
    end  
    
    if i > 1
        disp('ISAM Update');
        isam.update(fg, initial);
        result = isam.calculateEstimate();
        
        %% reset 
        initial = Values;
        fg = NonlinearFactorGraph;
        
        currentVelocityGlobal = isam.calculateEstimate(currentVelKey);
        currentBias = isam.calculateEstimate(currentBiasKey);
        
        %% Compute some marginals
        marginal = isam.marginalCovariance(calibrationKey);
        marginal_fx(i)=sqrt(marginal(1,1));
        marginal_fy(i)=sqrt(marginal(2,2));
        
        %% Compute condition number 
        isam_fg = isam.getFactorsUnsafe();
        isam_values = isam.getLinearizationPoint();
        gfg = isam_fg.linearize(isam_values);
        mat = gfg.jacobian();
        c(i) = cond(mat, 2);
        mat = gfg.augmentedJacobian();
        augmented_c(i)= cond(mat, 2);
        
        for f=0:isam_fg.size()-1
           nonlinear_factor = isam_fg.at(f);
           if strcmp(class(nonlinear_factor),'gtsam.TransformCalProjectionFactorCal3_S2')
               gaussian_factor = nonlinear_factor.linearize(isam_values);
               A = gaussian_factor.getA();
               b = gaussian_factor.getb();
               % Column 17 (fy) in jacobian
               A_col = A(:,17);  
               if A_col(2) == 0
%                    pause
                   disp('Cheirality Exception!');
               end
           end
        end
        
    end
     
    hold off;

    clf;
    figure(1);
    subplot(5,1,1:2);
    hold on;
    
    %% plot the integrated IMU frame (not from 
    gtsam.plotPose3(currentIMUPoseGlobal, [], 2);
    
    %% plot results
    result_camera_transform = result.at(transformKey);
    for j=1:i
      gtsam.plotPose3(result.at(j),[],0.5);
      gtsam.plotPose3(result.at(j).compose(result_camera_transform),[],0.5);
    end
    
    xlabel('x (m)');
    ylabel('y (m)');

    title(sprintf('Curvature %g deg, iteration %g', curvature, i));
    
    axis([0 20 0 20 -10 10]);
      view(-37,40);
%     axis equal
    
    for l=101:100+nrPoints
        plotPoint3(result.at(l),'g');
    end
    
    ty = result.at(transformKey).translation().y();
    fx = result.at(calibrationKey).fx();
    fy = result.at(calibrationKey).fy();
    px = result.at(calibrationKey).px();
    py = result.at(calibrationKey).py();
    text(1,5,5,sprintf('Y-Transform(0.0): %0.2f',ty));
    text(1,5,3,sprintf('fx(900): %.0f',fx));
    text(1,5,1,sprintf('fy(900): %.0f',fy));
    
    fxs(i) = fx;
    fys(i) = fy;
    pxs(i) = px;
    pys(i) = py;
    subplot(5,1,3);
    hold on;
    plot(1:steps,repmat(K.fx,1,steps),'r--');
    p(1) = plot(1:i,fxs,'r','LineWidth',2);
        
    plot(1:steps,repmat(K.fy,1,steps),'g--');
    p(2) = plot(1:i,fys,'g','LineWidth',2);
        
    if i > 1
        plot(2:i,fxs(2:i) + marginal_fx(2:i),'r-.');
        plot(2:i,fxs(2:i) - marginal_fx(2:i),'r-.');
        
        plot(2:i,fys(2:i) + marginal_fy(2:i),'g-.');
        plot(2:i,fys(2:i) - marginal_fy(2:i),'g-.');
        
        
        
        subplot(5,1,5);
        hold on;
        title('Condition Number');
        plot(2:i,c(2:i),'b-');
        plot(2:i,augmented_c(2:i),'r-');
        axis([0 steps 0 max(c(2:i))*1.1]);
        
        
%         figure(2);
%         plotBayesTree(isam);
        
    end
    legend(p, 'f_x', 'f_y', 'Location', 'SouthWest'); 
    
%     legend(p, 'f_x', 'f_x''', 'f_y', 'f_y''', 'Location', 'SouthWest'); 
    
    %% plot principal points
    subplot(5,1,4);
    hold on;
    plot(1:steps,repmat(K.px,1,steps),'r--');
    pp(1) = plot(1:i,pxs,'r','LineWidth',2);
        
    plot(1:steps,repmat(K.py,1,steps),'g--');
    pp(2) = plot(1:i,pys,'g','LineWidth',2);
    title('Principal Point');
    legend(pp, 'p_x', 'p_y', 'Location', 'SouthWest'); 
    
    if(write_video)
        currFrame = getframe(gcf);
        writeVideo(videoObj, currFrame)
    else
        pause(0.1);
    end
    
    
end

if(write_video)
    close(videoObj);
end

fprintf('Cheirality Exception count: %d\n', cheirality_exception_count);

disp('Transform after optimization');
result.at(transformKey)

disp('Calibration after optimization');
result.at(calibrationKey)

disp('Bias after optimization');
currentBias



