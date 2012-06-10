function VisualISAMStep
% VisualISAMStep: execute one update step of visualSLAM::iSAM object
% Authors: Duy Nguyen Ta and Frank Dellaert

% global variables, input
global frame_i odometryNoise measurementNoise
global data

% global variables, input/output
global isam

% global variables, output
global result

% options
global SHOW_TIMING ALWAYS_RELINEARIZE 

% iSAM expects us to give it a new set of factors 
% along with initial estimates for any new variables introduced.
newFactors = visualSLAMGraph;
initialEstimates = visualSLAMValues;

%% Add odometry
newFactors.addOdometry(symbol('x',frame_i-1), symbol('x',frame_i), data.odometry, odometryNoise);

%% Add visual measurement factors
for j=1:size(data.points,2)
    zij = data.cameras{frame_i}.project(data.points{j});
    newFactors.addMeasurement(zij, measurementNoise, symbol('x',frame_i), symbol('l',j), data.K);
end

%% Initial estimates for the new pose.
prevPose = result.pose(symbol('x',frame_i-1));
initialEstimates.insertPose(symbol('x',frame_i), prevPose.compose(data.odometry));

%% Update ISAM
% figure(1);tic;
isam.update(newFactors, initialEstimates);
% t=toc; plot(frame_i,t,'r.'); tic
result = isam.estimate();
% t=toc; plot(frame_i,t,'g.');

if ALWAYS_RELINEARIZE % re-linearize
    isam.reorder_relinearize();
end
