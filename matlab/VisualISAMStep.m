function [isam,result] = VisualISAMStep(data,noiseModels,isam,result,truth, options)
% VisualISAMStep: execute one update step of visualSLAM::iSAM object
% Authors: Duy Nguyen Ta and Frank Dellaert

% iSAM expects us to give it a new set of factors 
% along with initial estimates for any new variables introduced.
newFactors = visualSLAMGraph;
initialEstimates = visualSLAMValues;

%% Add odometry
i = result.nrPoses+1;
odometry = data.odometry{i-1};
newFactors.addOdometry(symbol('x',i-1), symbol('x',i), odometry, noiseModels.odometry);

%% Add visual measurement factors and initializations as necessary
for k=1:length(data.Z{i})
    zij = data.Z{i}{k};
    j = data.J{i}{k};
    jj = symbol('l', j);
    newFactors.addMeasurement(zij, noiseModels.measurement, symbol('x',i), jj, data.K);
    % TODO: initialize with something other than truth
    if ~result.exists(jj) && ~initialEstimates.exists(jj)
        lmInit = truth.points{j};
        initialEstimates.insertPoint(jj, lmInit);
    end
end

%% Initial estimates for the new pose.
prevPose = result.pose(symbol('x',i-1));
initialEstimates.insertPose(symbol('x',i), prevPose.compose(odometry));

%% Update ISAM
% figure(1);tic;
isam.update(newFactors, initialEstimates);
% t=toc; plot(frame_i,t,'r.'); tic
result = isam.estimate();
% t=toc; plot(frame_i,t,'g.');

if options.alwaysRelinearize % re-linearize
    isam.reorder_relinearize();
end
