function pts2dTracksStereo = points2DTrackStereo(K, cameraPoses, imageSize, cylinders)
% Assess how accurately we can reconstruct points from a particular monocular camera setup. 
% After creation of the factor graph for each track, linearize it around ground truth. 
% There is no optimization
% @author: Zhaoyang Lv

import gtsam.*

%% create graph
graph = NonlinearFactorGraph;

%% create the noise factors
pointNoiseSigma = 0.1;
poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';
measurementNoiseSigma = 1.0;
posePriorNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
pointPriorNoise = noiseModel.Isotropic.Sigma(3, pointNoiseSigma);
measurementNoise = noiseModel.Isotropic.Sigma(2, measurementNoiseSigma);
stereoNoise = noiseModel.Isotropic.Sigma(3,1);

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
    pts3d{i} = cylinderSampleProjectionStereo(K, cameraPose, imageSize, cylinders);
    
    if ~initialized
        graph.add(PriorFactorPose3(symbol('x', 1), camera.pose, posePriorNoise));
        initialized = true;
    end
    
    for j = 1:length(pts3d.pts{i}.Z)
        if isempty(pts3d.pts{i}.Z{j})
            continue;
        end
        graph.add(GenericStereoFactor3D(StereoPoint2(pts3d{i}.Z{j}.uL, pts3d{i}.Z{j}.uR, pts3d{i}.Z{j}.v), ...
            stereoNoise, symbol('x', i), symbol('p', j), K));    
    end
end

%% initialize cameras and points close to ground truth 
for i = 1:cameraPosesNum
    pose_i = camera.pose.retract(0.1*randn(6,1));
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
ptx = 1;
for i = 1:length(cylinders)
    for j = 1:length(cylinders{i}.Points)
        if isempty(pts3d{k}.index{i}{j})
            continue;
        end
        idx = pts3d{k}.index{i}{j};
        pts2dTracksMono.pt3d{ptx} = pts3d{k}.data{idx};
        pts2dTracksMono.Z{ptx} = pts3d{k}.Z{idx};
        pts2dTracksMono.cov{ptx} = marginals.marginalCovariance(symbol('p',idx));

        ptx = ptx + 1;
    end
end

end
