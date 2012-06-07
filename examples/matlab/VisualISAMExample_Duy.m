%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 3510, Georgia Tech Research Corporation, 
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
% 
% See LICENSE for the license information
%
% @brief A simple visual SLAM example for structure from motion
% @author Duy-Nguyen Ta
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Create a triangle target, just 3 points on a plane
nPoints = 3;
r = 10;
points = {};
for j=1:nPoints
    theta = (j-1)*2*pi/nPoints;
    points{j} = gtsamPoint3([r*cos(theta), r*sin(theta), 0]');
end

%% Create camera cameras on a circle around the triangle
nCameras = 30;
height = 10;
r = 30;
cameras = {};
K = gtsamCal3_S2(500,500,0,640/2,480/2);
for i=1:nCameras
    theta = (i-1)*2*pi/nCameras;
    t = gtsamPoint3([r*cos(theta), r*sin(theta), height]');
    cameras{i} = gtsamSimpleCamera_lookat(t, gtsamPoint3, gtsamPoint3([0,0,1]'), K);
end
odometry = cameras{1}.pose.between(cameras{2}.pose);

poseNoise = gtsamSharedNoiseModel_Sigmas([0.001 0.001 0.001 0.5 0.5 0.5]');
pointNoise = gtsamSharedNoiseModel_Sigma(3, 0.1);
measurementNoise = gtsamSharedNoiseModel_Sigma(2, 1.0);

%% Create an ISAM object for inference
isam = visualSLAMISAM(5);

%% Update ISAM
newFactors = visualSLAMGraph;
initialEstimates = visualSLAMValues;
figure(1); clf;
for i=1:nCameras
    
    % Prior for the first pose or odometry for subsequent cameras
    if (i==1)
        newFactors.addPosePrior(symbol('x',1), cameras{1}.pose, poseNoise);
        for j=1:nPoints
            newFactors.addPointPrior(symbol('l',j), points{j}, pointNoise);
        end
    else
        newFactors.addOdometry(symbol('x',i-1), symbol('x',i), odometry, poseNoise);
    end

    % Visual measurement factors
    for j=1:nPoints
        zij = cameras{i}.project(points{j});
        newFactors.addMeasurement(zij, measurementNoise, symbol('x',i), symbol('l',j), K);
    end
    
    % Initial estimates for the new pose. Also initialize points while in 
    % the first frame.
    if (i==1)
        initialEstimates.insertPose(symbol('x',i), cameras{i}.pose);
        for j=1:size(points,2)
            initialEstimates.insertPoint(symbol('l',j), points{j});
        end
    else
        %TODO: this might be suboptimal since "result" is not the fully
        %optimized result
        if (i==2), prevPose = cameras{1}.pose;
        else, prevPose = result.pose(symbol('x',i-1)); end
        initialEstimates.insertPose(symbol('x',i), prevPose.compose(odometry));
    end

    % Update ISAM, only update for the second frame onward
    % Update the first frame will cause error, since it's under constrained
    if (i>=2)
        isam.update(newFactors, initialEstimates);
        result = isam.estimate();

        % Plot results
        h=figure(1);
        hold on;
        for j=1:size(points,2)
            P = isam.marginalCovariance(symbol('l',j));
            point_j = result.point(symbol('l',j));
            plot3(point_j.x, point_j.y, point_j.z,'marker','o');
            covarianceEllipse3D([point_j.x;point_j.y;point_j.z],P);
        end

        for ii=i-1:i
            P = isam.marginalCovariance(symbol('x',ii));
            pose_ii = result.pose(symbol('x',ii));
            plotPose3(pose_ii,P,10);
        end
        axis([-35 35 -35 35 -35 35])
        view([36 34])
        colormap('hot')
%         print(h,'-dpng',sprintf('vISAM_%03d.png',i));
        
        % Reset newFactors and initialEstimates to prepare for the next 
        % update
        newFactors = visualSLAMGraph;
        initialEstimates = visualSLAMValues;
    end
end