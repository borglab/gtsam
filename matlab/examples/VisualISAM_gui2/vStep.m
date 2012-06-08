function [ handles ] = vStep( handles )
%VSTEP Summary of this function goes here
%   Detailed explanation goes here

    %% Add odometry
    handles.newFactors.addOdometry(symbol('x',handles.frame_i-1), symbol('x',handles.frame_i), handles.odometry, handles.odometryNoise);
    
    %% Add visual measurement factors
    for j=1:handles.nPoints
        zij = handles.cameras{handles.frame_i}.project(handles.points{j});
        handles.newFactors.addMeasurement(zij, handles.measurementNoise, symbol('x',handles.frame_i), symbol('l',j), handles.K);
    end
    
    %% Initial estimates for the new pose. Also initialize points while in the first frame.
    %TODO: this might be suboptimal since "result" is not the fully optimized result
    handles.frame_i
    if (handles.frame_i==2), prevPose = handles.cameras{1}.pose;
    else, prevPose = handles.result.pose(symbol('x',handles.frame_i-1)); end
    handles.initialEstimates.insertPose(symbol('x',handles.frame_i), prevPose.compose(handles.odometry));
    
    %% Update ISAM
    if handles.BATCH_INIT & (handles.frame_i==2) % Do a full optimize for first two poses
        handles.initialEstimates
        fullyOptimized = handles.newFactors.optimize(handles.initialEstimates)
        handles.initialEstimates = fullyOptimized;
    end
%     figure(1);tic;
    handles.isam.update(handles.newFactors, handles.initialEstimates);
%     t=toc; plot(handles.frame_i,t,'r.'); tic
    handles.result = handles.isam.estimate();
%     t=toc; plot(handles.frame_i,t,'g.');
    if handles.ALWAYS_RELINEARIZE % re-linearize
        handles.isam.reorder_relinearize();
    end
    
    if handles.SAVE_GRAPH
        handles.isam.saveGraph(sprintf('VisualiSAM.dot',handles.frame_i));
    end
    if handles.PRINT_STATS
        handles.isam.printStats();
    end
    handles.frame_i
    handles.DRAW_INTERVAL
    if mod(handles.frame_i,handles.DRAW_INTERVAL)==0
        vPlot(handles);
    end
    
    %% Reset newFactors and initialEstimates to prepare for the next update
    handles.newFactors = visualSLAMGraph;
    handles.initialEstimates = visualSLAMValues;
end

