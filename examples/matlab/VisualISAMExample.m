%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation, 
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

%% Create camera poses on a circle around the triangle
nCameras = 30;
height = 10;
r = 30;
poses = {};
for i=1:nCameras
    theta = (i-1)*2*pi/nCameras;
    t = gtsamPoint3([r*cos(theta), r*sin(theta), height]');
    camera = gtsamSimpleCamera_lookat(t, gtsamPoint3(), gtsamPoint3([0,0,1]'), gtsamCal3_S2())
    poses{i} = camera.pose();
end
odometry = poses{1}.between(poses{2});

poseNoise = gtsamSharedNoiseModel_Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
pointNoise = gtsamSharedNoiseModel_Sigma(3, 0.1);
measurementNoise = gtsamSharedNoiseModel_Sigma(2, 1.0);
K = gtsamCal3_S2(50,50,0,50,50);

%% Create an ISAM object for inference
isam = visualSLAMISAM;

%% Update ISAM
newFactors = visualSLAMGraph;
initialEstimates = visualSLAMValues;
for i=1:nCameras
    
    % Prior for the first pose or odometry for subsequent poses
    if (i==1)
        newFactors.addPosePrior(symbol('x',1), poses{1}, poseNoise);
    else
        newFactors.addOdometry(symbol('x',i-1), symbol('x',i), odometry, poseNoise);
    end

    % Visual measurement factors
    for j=1:nPoints
        camera = gtsamSimpleCamera(K,poses{i});
        zij = camera.project(points{j});
        newFactors.addMeasurement(zij, measurementNoise, symbol('x',i), symbol('l',j), K);
    end
    
    % Initial estimates for the new pose. Also initialize points while in 
    % the first frame.
    if (i==1)
        initialEstimates.insertPose(symbol('x',i), poses{i});
        for j=1:size(points,2)
            initialEstimates.insertPoint(symbol('l',j), points{j});
        end
    else
        %TODO: this might not be suboptimal since "result" is not the fully
        %optimized result
        prevPose = result.pose(symbol('x',i-1));
        initialEstimates.insertPose(symbol('x',i), prevPose.compose(odometry));
    end

    % Update ISAM, only update for the second frame onward
    % Update the first frame will cause error, since it's under constrained
    if (i>=2)
        isam.update(newFactors, initialEstimates);
        emptyFactors = visualSLAMGraph;
        emptyEstimates = visualSLAMValues;
        result = isam.estimate();

        % Plot first result
        figure(1);clf
        hold on;
        for j=1:size(points,2)
            P = isam.marginalCovariance(symbol('l',j));
            point_j = result.point(symbol('l',j));
            plot3(point_j.x, point_j.y, point_j.z,'marker','o');
            covarianceEllipse3D([point_j.x;point_j.y;point_j.z],P);
        end

        for ii=1:i
            P = isam.marginalCovariance(symbol('x',ii));
            pose_ii = result.pose(symbol('x',ii));
            plotPose3(pose_ii,P,10);
        end
        axis([-50 50 -50 50 -50 50])
        
        % Reset newFactors and initialEstimates to prepare for the next 
        % update
        newFactors = visualSLAMGraph;
        initialEstimates = visualSLAMValues;
    end
end