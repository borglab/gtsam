function pts2dTracksMono = points2DTrackMonocular(K, cameraPoses, imageSize, cylinders)
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

measurementNoiseSigma = 1.0;
measurementNoise = noiseModel.Isotropic.Sigma(2, measurementNoiseSigma);

cameraPosesNum = length(cameraPoses);

%% add measurements and initial camera & points values
pointsNum = 0;
cylinderNum = length(cylinders);
points3d = cell(0);
for i = 1:cylinderNum
    cylinderPointsNum = length(cylinders{i}.Points);
    pointsNum = pointsNum + cylinderPointsNum; 
    for j = 1:cylinderPointsNum
        points3d{end+1}.data = cylinders{i}.Points{j};
        points3d{end}.Z = cell(0);
        points3d{end}.cameraConstraint = cell(0);
        points3d{end}.visiblity = false;
    end
end

graph.add(PriorFactorPose3(symbol('x', 1), cameraPoses{1}, posePriorNoise));

pts3d = cell(cameraPosesNum, 1);
initialEstimate = Values;
for i = 1:cameraPosesNum 
    
    cameraPose = cameraPoses{i};    
    pts3d{i} = cylinderSampleProjection(K, cameraPose, imageSize, cylinders);
    
    measurementNum = length(pts3d{i}.Z);
    for j = 1:measurementNum
        index = pts3d{i}.overallIdx{j};
        points3d{index}.Z{end+1} = pts3d{i}.Z{j};
        points3d{index}.cameraConstraint{end+1} = i;
        points3d{index}.visiblity = true;
    end
    
end

%% initialize graph and values
for i = 1:cameraPosesNum
    pose_i = cameraPoses{i}.retract(0.1*randn(6,1));
    initialEstimate.insert(symbol('x', i), pose_i);    
end

for i = 1:pointsNum
    % single measurement. not added to graph
    factorNum = length(points3d{i}.Z);
    if factorNum > 1
        for j = 1:factorNum
            cameraIdx = points3d{i}.cameraConstraint{j};
            graph.add(GenericProjectionFactorCal3_S2(points3d{i}.Z{j}, ...
                measurementNoise, symbol('x', cameraIdx), symbol('p', points3d{i}.cameraConstraint{j}), K) );    
        end
    end
   
    % add in values
    point_j = points3d{i}.data.retract(0.1*randn(3,1));
    initialEstimate.insert(symbol('p', i), point_j);
    
end

%% Print the graph
graph.print(sprintf('\nFactor graph:\n'));

marginals = Marginals(graph, initialEstimate);

%% get all the points track information
% currently throws the Indeterminant linear system exception
for i = 1:pointsNum
    if points3d{i}.visiblity
        pts2dTracksMono.pt3d{i} = points3d{i}.data;
        pts2dTracksMono.Z = points3d{i}.Z;

        if length(points3d{i}.Z) == 1
            %pts2dTracksMono.cov{i} singular matrix 
        else 
            pts2dTracksMono.cov{i} = marginals.marginalCovariance(symbol('p', i));    
        end
    end
end

for k = 1:cameraPosesNum
    num = length(pts3d{k}.data);
    for i = 1:num
        pts2dTracksMono.pt3d{i} = pts3d{k}.data{i};
        pts2dTracksMono.Z{i} = pts3d{k}.Z{i};
        pts2dTracksMono.cov{i} = marginals.marginalCovariance(symbol('p',pts3d{k}.overallIdx{visiblePointIdx}));
    end
end

%% plot the result with covariance ellipses
hold on;
%plot3DPoints(initialEstimate, [], marginals);
%plot3DTrajectory(initialEstimate, '*', 1, 8, marginals);
plot3DTrajectory(initialEstimate, '*', 1, 8);
view(3);


end
