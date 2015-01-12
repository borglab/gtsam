function pts2dTracksmon = points2DTrackMonocular(K, cameraPoses, imageSize, cylinders)
% Assess how accurately we can reconstruct points from a particular monocular camera setup. 
% After creation of the factor graph for each track, linearize it around ground truth. 
% There is no optimization
% @author: Zhaoyang Lv

import gtsam.*

%% create graph
graph = NonlinearFactorGraph;

%% add a constraint on the starting pose
poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';
posePriorNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
firstPose = cameraPoses{1};
graph.add(PriorFactorPose3(symbol('x', l), firstPose, posePriorNoise));

cameraPosesNum = size(cameraPoses, 1);

%% add measurements
initialEstimate = Values;
for i = 1:cameraPosesNum
    [visiblePoints3, visiblePointsCylinderIdx] = cylinderSampleProjection(K, cameraPoses{i}, imageSize, cylinders);
    
    pointsNum = size(visiblePoints, 1);
    
    %% not finished
    %for j = 1:pointsNum
    %    graph.add();
    %end   
end

marginals = Marginals(graph, initialEstimate);

% should use all the points num to replace the num 100
for i = 1:100 
   marginals.marginalCovariance(symbol('p',i));
end

end
