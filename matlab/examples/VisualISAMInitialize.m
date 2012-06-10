function [noiseModels,isam,result] = VisualInitialize(data,truth,options)
% VisualInitialize: initialize visualSLAM::iSAM object and noise parameters
% Authors: Duy Nguyen Ta and Frank Dellaert

%% Initialize iSAM
isam = visualSLAMISAM(options.reorderInterval);

%% Set Noise parameters
noiseModels.pose = gtsamSharedNoiseModel_Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
noiseModels.odometry = gtsamSharedNoiseModel_Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
noiseModels.point = gtsamSharedNoiseModel_Sigma(3, 0.1);
noiseModels.measurement = gtsamSharedNoiseModel_Sigma(2, 1.0);

%% Add constraints/priors
% TODO: should not be from ground truth!
newFactors = visualSLAMGraph;
initialEstimates = visualSLAMValues;
for i=1:2
    ii = symbol('x',i);
    if i==1 & options.hardConstraint % add hard constraint
        newFactors.addPoseConstraint(ii,truth.cameras{1}.pose);
    else
        newFactors.addPosePrior(ii,truth.cameras{i}.pose, noiseModels.pose);
    end
    initialEstimates.insertPose(ii,truth.cameras{i}.pose);
end

%% Add visual measurement factors from two first poses
for i=1:2
    ii = symbol('x',i);
    for j=1:size(data.z,2)
        jj = symbol('l',j);
        newFactors.addMeasurement(data.z{i,j}, noiseModels.measurement, ii, jj, data.K);
    end
end

%% Initialize points, possibly add priors on them
% TODO: should not be from ground truth!
for j=1:size(data.z,2)
    jj = symbol('l',j);
    if options.pointPriors % add point priors
        newFactors.addPointPrior(jj, truth.points{j}, noiseModels.point);
    end
    initialEstimates.insertPoint(jj, truth.points{j});
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