VisualISAMGlobalVars

%% Add odometry
newFactors.addOdometry(symbol('x',frame_i-1), symbol('x',frame_i), odometry, odometryNoise);

%% Add visual measurement factors
for j=1:nPoints
    zij = cameras{frame_i}.project(points{j});
    newFactors.addMeasurement(zij, measurementNoise, symbol('x',frame_i), symbol('l',j), K);
end

%% Initial estimates for the new pose. Also initialize points while in the first frame.
%TODO: this might be suboptimal since "result" is not the fully optimized result
if (frame_i==2), prevPose = cameras{1}.pose;
else, prevPose = result.pose(symbol('x',frame_i-1)); end
initialEstimates.insertPose(symbol('x',frame_i), prevPose.compose(odometry));

%% Update ISAM
if BATCH_INIT & (frame_i==2) % Do a full optimize for first two poses
    initialEstimates
    fullyOptimized = newFactors.optimize(initialEstimates)
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

%% Reset newFactors and initialEstimates to prepare for the next update
newFactors = visualSLAMGraph;
initialEstimates = visualSLAMValues;