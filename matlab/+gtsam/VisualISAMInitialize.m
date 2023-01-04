function [noiseModels,isam,result,nextPoseIndex] = VisualISAMInitialize(data,truth,options)
% VisualISAMInitialize initializes visualSLAM::iSAM object and noise parameters
% Authors: Duy Nguyen Ta, Frank Dellaert and Alex Cunningham

import gtsam.*

%% Initialize iSAM
params = gtsam.ISAM2Params;
if options.alwaysRelinearize
    params.relinearizeSkip = 1;
end
isam = ISAM2(params);

%% Set Noise parameters
noiseModels.pose = noiseModel.Diagonal.Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]', true);
%noiseModels.odometry = noiseModel.Diagonal.Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
noiseModels.odometry = noiseModel.Diagonal.Sigmas([0.05 0.05 0.05 0.2 0.2 0.2]', true);
noiseModels.point = noiseModel.Isotropic.Sigma(3, 0.1, true);
noiseModels.measurement = noiseModel.Isotropic.Sigma(2, 1.0, true);

%% Add constraints/priors
% TODO: should not be from ground truth!
newFactors = NonlinearFactorGraph;
initialEstimates = Values;
for i=1:2
    ii = symbol('x',i);
    if i==1
        if options.hardConstraint % add hard constraint
            newFactors.add(NonlinearEqualityPose3(ii,truth.cameras{1}.pose));
        else
            newFactors.add(PriorFactorPose3(ii,truth.cameras{i}.pose, noiseModels.pose));
        end
    end
    initialEstimates.insert(ii,truth.cameras{i}.pose);
end

nextPoseIndex = 3;

%% Add visual measurement factors from two first poses and initialize observed landmarks
for i=1:2
    ii = symbol('x',i);
    for k=1:length(data.Z{i})
        j = data.J{i}{k};
        jj = symbol('l',data.J{i}{k});
        newFactors.add(GenericProjectionFactorCal3_S2(data.Z{i}{k}, noiseModels.measurement, ii, jj, data.K));
        % TODO: initial estimates should not be from ground truth!
        if ~initialEstimates.exists(jj)
            initialEstimates.insert(jj, truth.points{j});
        end
        if options.pointPriors % add point priors
            newFactors.add(PriorFactorPoint3(jj, truth.points{j}, noiseModels.point));
        end
    end
end

%% Add odometry between frames 1 and 2
newFactors.add(BetweenFactorPose3(symbol('x',1), symbol('x',2), data.odometry{1}, noiseModels.odometry));

%% Update ISAM
if options.batchInitialization % Do a full optimize for first two poses
    batchOptimizer = LevenbergMarquardtOptimizer(newFactors, initialEstimates);
    fullyOptimized = batchOptimizer.optimize();
    isam.update(newFactors, fullyOptimized);
else
    isam.update(newFactors, initialEstimates);
end
% figure(1);tic;
% t=toc; plot(frame_i,t,'r.'); tic
result = isam.calculateEstimate();
% t=toc; plot(frame_i,t,'g.');

