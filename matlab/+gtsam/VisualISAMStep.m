function [isam,result,nextPoseIndex] = VisualISAMStep(data,noiseModels,isam,result,truth,nextPoseIndex)
% VisualISAMStep executes one update step of visualSLAM::iSAM object
% Authors: Duy Nguyen Ta and Frank Dellaert

% iSAM expects us to give it a new set of factors 
% along with initial estimates for any new variables introduced.
import gtsam.*
newFactors = NonlinearFactorGraph;
initialEstimates = Values;

%% Add odometry
import gtsam.*
odometry = data.odometry{nextPoseIndex-1};
newFactors.add(BetweenFactorPose3(symbol('x',nextPoseIndex-1), symbol('x',nextPoseIndex), odometry, noiseModels.odometry));

%% Add visual measurement factors and initializations as necessary
import gtsam.*
for k=1:length(data.Z{nextPoseIndex})
    zij = data.Z{nextPoseIndex}{k};
    j = data.J{nextPoseIndex}{k};
    jj = symbol('l', j);
    newFactors.add(GenericProjectionFactorCal3_S2(zij, noiseModels.measurement, symbol('x',nextPoseIndex), jj, data.K));
    % TODO: initialize with something other than truth
    if ~result.exists(jj) && ~initialEstimates.exists(jj)
        lmInit = truth.points{j};
        initialEstimates.insert(jj, lmInit);
    end
end

%% Initial estimates for the new pose.
import gtsam.*
prevPose = result.at(symbol('x',nextPoseIndex-1));
initialEstimates.insert(symbol('x',nextPoseIndex), prevPose.compose(odometry));

%% Update ISAM
% figure(1);tic;
isam.update(newFactors, initialEstimates);
% t=toc; plot(frame_i,t,'r.'); tic
result = isam.calculateEstimate();
% t=toc; plot(frame_i,t,'g.');

% Update nextPoseIndex to return
nextPoseIndex = nextPoseIndex + 1;

