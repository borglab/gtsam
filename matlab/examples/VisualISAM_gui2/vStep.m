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
        sprintf('Plotting')
        %% Plot results
        tic
%         h=figure(2);clf
%         set(1,'NumberTitle','off','Name','Visual iSAM');
        hold on;
        for j=1:handles.nPoints
            P = handles.isam.marginalCovariance(symbol('l',j));
            point_j = handles.result.point(symbol('l',j));
            plot3(point_j.x, point_j.y, point_j.z,'marker','o');
            covarianceEllipse3D([point_j.x;point_j.y;point_j.z],P);
        end
        for ii=1:handles.CAMERA_INTERVAL:handles.frame_i
            P = handles.isam.marginalCovariance(symbol('x',ii));
            pose_ii = handles.result.pose(symbol('x',ii));
            plotPose3(pose_ii,P,10);
            if handles.DRAW_TRUE_POSES % show ground truth
                plotPose3(handles.cameras{ii}.pose,0.001*eye(6),10);
            end
        end
        axis([-40 40 -40 40 -10 20]);axis equal
        view(3)
        colormap('hot')
%         figure(2);
        t=toc;
        if handles.DRAW_INTERVAL~=handles.NCAMERAS, plot(handles.frame_i,t,'b.'); end
        if handles.SAVE_FIGURES
            print(h,'-dpng',sprintf('VisualiSAM%03d.png',handles.frame_i));
        end
        if handles.SAVE_GRAPHS
            handles.isam.saveGraph(sprintf('VisualiSAM%03d.dot',handles.frame_i));
        end
        hold off;
    end
    
    %% Reset newFactors and initialEstimates to prepare for the next update
    handles.newFactors = visualSLAMGraph;
    handles.initialEstimates = visualSLAMValues;
	handles.frame_i = handles.frame_i+1;
end

