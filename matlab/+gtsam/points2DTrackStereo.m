function pts2dTracksStereo = points2DTrackStereo(cameras, imageSize, cylinders)
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

cameraPosesNum = length(cameras);

%% add measurements and initial camera & points values
pointsNum = 0;
cylinderNum = length(cylinders);
for i = 1:cylinderNum
    pointsNum = pointsNum + length(cylinders{i}.Points);
end

pts3d = {};
initialEstimate = Values;
initialized = false;
for i = 1:cameraPosesNum
    % add a constraint on the starting pose    
    camera = cameras{i};
    
    pts3d.pts{i} = cylinderSampleProjection(camera, imageSize, cylinders);
    pts3d.camera{i} = camera;
   
    if ~initialized
        graph.add(PriorFactorPose3(symbol('x', 1), camera.pose, posePriorNoise));
        k = 0;
        if ~isempty(pts3d.pts{i}.data{1+k})
            graph.add(PriorFactorPoint3(symbol('p', 1), ...
                pts3d.pts{i}.data{1+k}, pointPriorNoise));
        else
            k = k+1;
        end
        initialized = true;
    end
    
    for j = 1:length(pts3d.pts{i}.Z)
        if isempty(pts3d.pts{i}.Z{j})
            continue;
        end
        graph.add(GenericProjectionFactorCal3_S2(pts3d.pts{i}.Z{j}, ...
            measurementNoise, symbol('x', i), symbol('p', j), camera.calibration) );    
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
ptIdx = 0;
for i = 1:pointsNum
   if isempty(pts3d.pts{i})
       continue;
   end
   pts2dTracksMono.cov{ptIdx} = marginals.marginalCovariance(symbol('p',i));
end

end
