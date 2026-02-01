function [pts2dTracksStereo, initialEstimate] = points2DTrackStereo(K, cameraPoses, options, cylinders)
% Assess how accurately we can reconstruct points from a particular monocular camera setup. 
% After creation of the factor graph for each track, linearize it around ground truth. 
% There is no optimization
%
% @author: Zhaoyang Lv

import gtsam.*

%% create graph
graph = NonlinearFactorGraph;

%% create the noise factors
poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';
posePriorNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
stereoNoise = noiseModel.Isotropic.Sigma(3, 0.05);

cameraPosesNum = length(cameraPoses);

%% add measurements and initial camera & points values
pointsNum = 0;
cylinderNum = length(cylinders);
points3d = cell(0);
for i = 1:cylinderNum
    cylinderPointsNum = length(cylinders{i}.Points);
    pointsNum = pointsNum + length(cylinders{i}.Points);
    for j = 1:cylinderPointsNum
        points3d{end+1}.data = cylinders{i}.Points{j};
        points3d{end}.Z = cell(0);
        points3d{end}.cameraConstraint = cell(0);
        points3d{end}.visiblity = false;
        points3d{end}.cov = cell(cameraPosesNum);
    end
end

graph.add(PriorFactorPose3(symbol('x', 1), cameraPoses{1}, posePriorNoise));

%% initialize graph and values
initialEstimate = Values;
for i = 1:pointsNum
    point_j = points3d{i}.data + (0.05*randn(3,1));
    initialEstimate.insert(symbol('p', i), point_j);    
end

pts3d = cell(cameraPosesNum, 1);
cameraPosesCov = cell(cameraPosesNum, 1);
for i = 1:cameraPosesNum 
    pts3d{i} = cylinderSampleProjectionStereo(K, cameraPoses{i}, options.camera.resolution, cylinders);
    
    if isempty(pts3d{i}.Z)
        continue;
    end
    
    measurementNum = length(pts3d{i}.Z);
    for j = 1:measurementNum
        index = pts3d{i}.overallIdx{j};
        points3d{index}.Z{end+1} = pts3d{i}.Z{j};
        points3d{index}.cameraConstraint{end+1} = i;
        points3d{index}.visiblity = true;
  
        graph.add(GenericStereoFactor3D(StereoPoint2(pts3d{i}.Z{j}.uL, pts3d{i}.Z{j}.uR, pts3d{i}.Z{j}.v), ...
            stereoNoise, symbol('x', i), symbol('p', index), K));    
    end

    pose_i = cameraPoses{i}.retract(poseNoiseSigmas);
    initialEstimate.insert(symbol('x', i), pose_i);

    %% linearize the graph
    marginals = Marginals(graph, initialEstimate);

    for j = 1:pointsNum
        if points3d{j}.visiblity
            points3d{j}.cov{i} = marginals.marginalCovariance(symbol('p', j));
        end       
    end
    
    cameraPosesCov{i} = marginals.marginalCovariance(symbol('x', i));    
end

%% Plot the result
plotFlyingResults(points3d, cameraPoses, cameraPosesCov, cylinders, options);

%% get all the 2d points track information
pts2dTracksStereo.pt3d = cell(0);
pts2dTracksStereo.Z = cell(0);
pts2dTracksStereo.cov = cell(0);
for i = 1:pointsNum
    if ~points3d{i}.visiblity
        continue;
    end
    
    pts2dTracksStereo.pt3d{end+1} = points3d{i}.data;
    pts2dTracksStereo.Z{end+1} = points3d{i}.Z;
    pts2dTracksStereo.cov{end+1} = marginals.marginalCovariance(symbol('p', i));       
end

% 
% %% plot the result with covariance ellipses
% plotFlyingResults(pts2dTracksStereo.pt3d, pts2dTracksStereo.cov, cameraPoses, cameraPosesCov, cylinders, options);

end
