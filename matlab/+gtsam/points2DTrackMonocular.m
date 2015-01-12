function pts2dTracksMono = points2DTrackMonocular(K, cameraPoses, imageSize, cylinders)
% Assess how accurately we can reconstruct points from a particular monocular camera setup. 
% After creation of the factor graph for each track, linearize it around ground truth. 
% There is no optimization
% @author: Zhaoyang Lv

import gtsam.*

%% create graph
graph = NonlinearFactorGraph;

pointNoiseSigma = 0.1;
poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';

%% add a constraint on the starting pose
posePriorNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
firstPose = cameraPoses{1};
graph.add(PriorFactorPose3(symbol('x', l), firstPose, posePriorNoise));

cameraPosesNum = length(cameraPoses);

%% add measurements and initial camera & points values
pointsNum = 0;
cylinderNum = length(cylinders);
for i = 1:cylinderNum
    pointsNum = pointsNum + length(cylinders{i}.Points);
end

measurementNoise = noiseModel.Isotropic.Sigma(2, measurementNoiseSigma);

pts3d = {};
initialEstimate = Values;
for i = 1:cameraPosesNum
    camera = SimpleCamera(K, cameraPoses{i});
    
    pts3d.pts{i} = cylinderSampleProjection(camera, imageSize, cylinders);
    pts3d.camera{i} = camera;
   
    for j = 1:length(pts3d.pts{i}.Z)
        graph.add(GenericProjectionFactorCal3_S2(pts3d.pts{i}.Z{j}, ...
            measurementNoise, symbol('x', i), symbol('p', j), camera.K) );
    
        point_j = pts3d.pts{i}.data{j}.retract(0.1*randn(3,1));
        initialEstimate.insert(symbol('p', j), point_j);
    end

    pose_i = camera.pose.retract(0.1*randn(6,1));
    initialEstimate.insert(symbole('x', i), pose_i);
    
end

%% Print the graph
graph.print(sprintf('\nFactor graph:\n'));

marginals = Marginals(graph, initialEstimate);

%% get all the 2d points track information
ptIdx = 0;
for i = 1:pointsNum
   if isempty(pts3d.pts{i})
       continue;
   end
   %pts2dTrackMono.pts2d = pts3d.pts{i}
   pts2dTracksMono.cov{ptIdx} = marginals.marginalCovariance(symbol('p',i));
end

end
