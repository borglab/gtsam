function [noiseModels,isam,result] = VisualISAMInitialize(data,truth,options)
% VisualInitialize: initialize visualSLAM::iSAM object and noise parameters
% Authors: Duy Nguyen Ta, Frank Dellaert and Alex Cunningham

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
    if i==1
        if options.hardConstraint % add hard constraint
            newFactors.addPoseConstraint(ii,truth.cameras{1}.pose);
        else
            newFactors.addPosePrior(ii,truth.cameras{i}.pose, noiseModels.pose);
        end
    end
    initialEstimates.insertPose(ii,truth.cameras{i}.pose);
end

%% Add visual measurement factors from two first poses and initialize observed landmarks
for i=1:2
    ii = symbol('x',i);
    for k=1:length(data.Z{i})
        j = data.J{i}{k};
        jj = symbol('l',data.J{i}{k});
        newFactors.addMeasurement(data.Z{i}{k}, noiseModels.measurement, ii, jj, data.K);
        % TODO: initial estimates should not be from ground truth!
        if ~initialEstimates.exists(jj)
            initialEstimates.insertPoint(jj, truth.points{j});
        end
        if options.pointPriors % add point priors
            newFactors.addPointPrior(jj, truth.points{j}, noiseModels.point);
        end
    end
end

%% Add odometry between frames 1 and 2
newFactors.addOdometry(symbol('x',1), symbol('x',2), data.odometry{1}, noiseModels.odometry);

%% Update ISAM
if options.batchInitialization % Do a full optimize for first two poses
    fullyOptimized = newFactors.optimize(initialEstimates,0);
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