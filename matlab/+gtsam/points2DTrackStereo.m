function pts2dTracksStereo = points2DTrackStereo(K, cameraPoses, imageSize, cylinders)
% Assess how accurately we can reconstruct points from a particular monocular camera setup. 
% After creation of the factor graph for each track, linearize it around ground truth. 
% There is no optimization
% @author: Zhaoyang Lv

import gtsam.*

%% create graph
graph = NonlinearFactorGraph;

%% create the noise factors
poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';
posePriorNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
stereoNoise = noiseModel.Isotropic.Sigma(3,0.2);

cameraPosesNum = length(cameraPoses);

%% add measurements and initial camera & points values
pointsNum = 0;
cylinderNum = length(cylinders);
for i = 1:cylinderNum
    pointsNum = pointsNum + length(cylinders{i}.Points);
end

pts3d = cell(cameraPosesNum, 1);
initialEstimate = Values;
initialized = false;
for i = 1:cameraPosesNum 
    pts3d{i} = cylinderSampleProjectionStereo(K, cameraPoses{i}, imageSize, cylinders);
    
    if ~initialized
        graph.add(PriorFactorPose3(symbol('x', 1), cameraPoses{i}, posePriorNoise));
        initialized = true;
    end
    
    measurementNum = length(pts3d{i}.Z);
    if measurementNum < 1
        continue;
    end
    for j = 1:measurementNum
        graph.add(GenericStereoFactor3D(StereoPoint2(pts3d{i}.Z{j}.uL, pts3d{i}.Z{j}.uR, pts3d{i}.Z{j}.v), ...
            stereoNoise, symbol('x', i), symbol('p', pts3d{i}.overallIdx{j}), K));    
    end
end

%% initialize cameras and points close to ground truth 
for i = 1:cameraPosesNum
    pose_i = cameraPoses{i}.retract(0.1*randn(6,1));
    initialEstimate.insert(symbol('x', i), pose_i);    
end
ptsIdx = 0;
for i = 1:length(cylinders)
    for j = 1:length(cylinders{i}.Points)
        ptsIdx = ptsIdx + 1;
        point_j = cylinders{i}.Points{j}.retract(0.1*randn(3,1));
        initialEstimate.insert(symbol('p', ptsIdx), point_j);
    end
end

%% Print the graph
graph.print(sprintf('\nFactor graph:\n'));

marginals = Marginals(graph, initialEstimate);

%% get all the 2d points track information
% currently throws the Indeterminant linear system exception
for k = 1:cameraPosesNum
    num = length(pts3d{k}.data);
    if num < 1
        continue;
    end
    for i = 1:num
        pts2dTracksStereo.pt3d{i} = pts3d{k}.data{i};
        pts2dTracksStereo.Z{i} = pts3d{k}.Z{i};
        pts2dTracksStereo.cov{i} = marginals.marginalCovariance(symbol('p',pts3d{k}.overallIdx{i}));
    end
end

%% plot the result with covariance ellipses
hold on;
%plot3DPoints(initialEstimate, [], marginals);
%plot3DTrajectory(initialEstimate, '*', 1, 8, marginals);
plot3DTrajectory(initialEstimate, '*', 1, 8, marginals);
view(3);

end
