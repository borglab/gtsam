function [isam,result] = VisualISAMStep(data,noiseModels,isam,result,options);
% VisualISAMStep: execute one update step of visualSLAM::iSAM object
% Authors: Duy Nguyen Ta and Frank Dellaert

% iSAM expects us to give it a new set of factors 
% along with initial estimates for any new variables introduced.
newFactors = visualSLAMGraph;
initialEstimates = visualSLAMValues;

%% Add odometry
i = double(result.nrPoses)+1;
odometry = data.odometry{i-1};
newFactors.addOdometry(symbol('x',i-1), symbol('x',i), odometry, noiseModels.odometry);

%% Add visual measurement factors
for j=1:size(data.z,2)
    newFactors.addMeasurement(data.z{i,j}, noiseModels.measurement, symbol('x',i), symbol('l',j), data.K);
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
