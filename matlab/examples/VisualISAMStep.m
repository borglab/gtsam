function VisualISAMStep
% VisualISAMStep: execute one update step of visualSLAM::iSAM object
% Authors: Duy Nguyen Ta and Frank Dellaert

% global variables, input
global frame_i odometry odometryNoise newFactors initialEstimates
global points cameras measurementNoise K

% global variables, input/output
global isam

% global variables, output
global result

% options
global BATCH_INIT SHOW_TIMING ALWAYS_RELINEARIZE 
global SAVE_GRAPH PRINT_STATS

% iSAM expects us to give it a new set of factors 
% along with initial estimates for any new variables introduced.
% We do not clear in frame 2 so we add to the factors added in Initialize
if frame_i > 2
    newFactors = visualSLAMGraph;
    initialEstimates = visualSLAMValues;
end

%% Add odometry
newFactors.addOdometry(symbol('x',frame_i-1), symbol('x',frame_i), odometry, odometryNoise);

%% Add visual measurement factors
for j=1:size(points,2)
    zij = cameras{frame_i}.project(points{j});
    newFactors.addMeasurement(zij, measurementNoise, symbol('x',frame_i), symbol('l',j), K);
end

%% Initial estimates for the new pose. Also initialize points while in the first frame.
if (frame_i==2), prevPose = cameras{1}.pose;
else, prevPose = result.pose(symbol('x',frame_i-1)); end
initialEstimates.insertPose(symbol('x',frame_i), prevPose.compose(odometry));

%% Update ISAM
if BATCH_INIT & (frame_i==2) % Do a full optimize for first two poses
    fullyOptimized = newFactors.optimize(initialEstimates);
    initialEstimates = fullyOptimized;
end
% figure(1);tic;
isam.update(newFactors, initialEstimates);
% t=toc; plot(frame_i,t,'r.'); tic
result = isam.estimate();
% t=toc; plot(frame_i,t,'g.');
if ALWAYS_RELINEARIZE % re-linearize
    isam.reorder_relinearize();
end

if SAVE_GRAPH
    isam.saveGraph(sprintf('VisualiSAM.dot',frame_i));
end

if PRINT_STATS
    isam.printStats();
end
