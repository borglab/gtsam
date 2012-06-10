function VisualISAMStep
% VisualISAMStep: execute one update step of visualSLAM::iSAM object
% Authors: Duy Nguyen Ta and Frank Dellaert

% global variables, input
global data pointNoise poseNoise measurementNoise

% global variables, output
global isam newFactors initialEstimates frame_i result
global poseNoise odometryNoise pointNoise measurementNoise

% options
global REORDER_INTERVAL HARD_CONSTRAINT POINT_PRIORS

%% Initialize iSAM
isam = visualSLAMISAM(REORDER_INTERVAL);

%% Set Noise parameters
poseNoise = gtsamSharedNoiseModel_Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
odometryNoise = gtsamSharedNoiseModel_Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
pointNoise = gtsamSharedNoiseModel_Sigma(3, 0.1);
measurementNoise = gtsamSharedNoiseModel_Sigma(2, 1.0);

%% Add constraints/priors
newFactors = visualSLAMGraph;
initialEstimates = visualSLAMValues;
i1 = symbol('x',1);
camera1 = data.cameras{1};
pose1 = camera1.pose;
if HARD_CONSTRAINT % add hard constraint
    newFactors.addPoseConstraint(i1,pose1);
else
    newFactors.addPosePrior(i1,pose1, poseNoise);
end
initialEstimates.insertPose(i1,pose1); % TODO: should not be from ground truth!

%% Add visual measurement factors from first pose
for j=1:size(data.points,2)
    jj = symbol('l',j);
    if POINT_PRIORS % add point priors
        newFactors.addPointPrior(jj, data.points{j}, pointNoise);
    end
    zij = camera1.project(data.points{j});
    newFactors.addMeasurement(zij, measurementNoise, i1, jj, data.K);
    initialEstimates.insertPoint(jj, data.points{j});  % TODO: should not be from ground truth!
end

frame_i = 1;
result = initialEstimates;
cla;