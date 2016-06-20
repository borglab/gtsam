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
        points3d{end}.camConstraintIdx = cell(0);
        points3d{end}.added = cell(0);
        points3d{end}.visiblity = false;
        points3d{end}.cov = cell(cameraPosesNum);
    end
end

graph.add(PriorFactorPose3(symbol('x', 1), cameraPoses{1}, posePriorNoise));

%% initialize graph and values
initialEstimate = Values;
for i = 1:pointsNum
    point_j = points3d{i}.data.retract(0.1*randn(3,1));
    initialEstimate.insert(symbol('p', i), point_j); 
end

pts3d = cell(cameraPosesNum, 1);
cameraPosesCov = cell(cameraPosesNum, 1);
marginals = Values;
for i = 1:cameraPosesNum     
    cameraPose = cameraPoses{i};    
    pts3d{i} = cylinderSampleProjection(K, cameraPose, imageSize, cylinders);
    
    measurementNum = length(pts3d{i}.Z);
    for j = 1:measurementNum
        index = pts3d{i}.overallIdx{j};
        points3d{index}.Z{end+1} = pts3d{i}.Z{j};
        points3d{index}.camConstraintIdx{end+1} = i;
        points3d{index}.added{end+1} = false;
        
        if length(points3d{index}.Z) < 2
            continue;
        else
            for k = 1:length(points3d{index}.Z)
                if ~points3d{index}.added{k}                
                    graph.add(GenericProjectionFactorCal3_S2(points3d{index}.Z{k}, ...
                        measurementNoise, symbol('x', points3d{index}.camConstraintIdx{k}), ...
                        symbol('p', index), K) );
                    points3d{index}.added{k} = true;
                end
            end
        end 
        
        points3d{index}.visiblity = true;    
    end

    pose_i = cameraPoses{i}.retract(0.1*randn(6,1));
    initialEstimate.insert(symbol('x', i), pose_i);    

    marginals = Marginals(graph, initialEstimate);

    for j = 1:pointsNum
        if points3d{j}.visiblity
            points3d{j}.cov{i} = marginals.marginalCovariance(symbol('p',j));
        end
    end

    cameraPosesCov{i} = marginals.marginalCovariance(symbol('x',i));

end
  
%% Print the graph
graph.print(sprintf('\nFactor graph:\n'));

%% Plot the result
plotFlyingResults(points3d, cameraPoses, cameraPosesCov, cylinders, options);

%% get all the points track information
for i = 1:pointsNum
    if ~points3d{i}.visiblity
        continue;
    end
    
    pts2dTracksMono.pt3d{end+1} = points3d{i}.data;
    pts2dTracksMono.Z{end+1} = points3d{i}.Z;

    if length(points3d{i}.Z) == 1
        %pts2dTracksMono.cov{i} singular matrix 
    else 
        pts2dTracksMono.cov{end+1} = marginals.marginalCovariance(symbol('p', i));    
    end
end

end
