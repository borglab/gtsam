function VisualInitialize(options)
% VisualInitialize: initialize visualSLAM::iSAM object and noise parameters
% Authors: Duy Nguyen Ta and Frank Dellaert

% global variables, input
global data

% global variables, output
global isam frame_i result
global poseNoise odometryNoise pointNoise measurementNoise

%% Initialize iSAM
isam = visualSLAMISAM(options.reorderInterval);

%% Set Noise parameters
poseNoise = gtsamSharedNoiseModel_Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
odometryNoise = gtsamSharedNoiseModel_Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
pointNoise = gtsamSharedNoiseModel_Sigma(3, 0.1);
measurementNoise = gtsamSharedNoiseModel_Sigma(2, 1.0);

%% Add constraints/priors
newFactors = visualSLAMGraph;
initialEstimates = visualSLAMValues;
for frame_i=1:2
    ii = symbol('x',frame_i);
    if frame_i==1 & options.hardConstraint % add hard constraint
        newFactors.addPoseConstraint(ii,data.cameras{1}.pose);
    else
        newFactors.addPosePrior(ii,data.cameras{frame_i}.pose, poseNoise);
    end
    % TODO: init should not be from ground truth!
    initialEstimates.insertPose(ii,data.cameras{frame_i}.pose);
end

%% Add visual measurement factors from two first poses
for frame_i=1:2
    ii = symbol('x',frame_i);
    for j=1:size(data.points,2)
        jj = symbol('l',j);
        zij = data.cameras{frame_i}.project(data.points{j});
        newFactors.addMeasurement(zij, measurementNoise, ii, jj, data.K);
    end
end

%% Initialize points, possibly add priors on them
for j=1:size(data.points,2)
    jj = symbol('l',j);
    if options.pointPriors % add point priors
        newFactors.addPointPrior(jj, data.points{j}, pointNoise);
    end
    initialEstimates.insertPoint(jj, data.points{j});  % TODO: should not be from ground truth!
end

%% Update ISAM
if options.batchInitialization % Do a full optimize for first two poses
    fullyOptimized = newFactors.optimize(initialEstimates);
    isam.update(newFactors, fullyOptimized);
else
    isam.update(newFactors, initialEstimates);
end
% figure(1);tic;
% t=toc; plot(frame_i,t,'r.'); tic
result = isam.estimate();
% t=toc; plot(frame_i,t,'g.');

if options.alwaysRelinearize % re-linearize
    isam.reorder_relinearize();
end

cla;