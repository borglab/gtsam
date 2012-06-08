function vPlot( handles )
%VPLOT Summary of this function goes here
%   Detailed explanation goes here
if (handles.frame_i<2)
    sprintf('Cannot plot the first frame')
    return
end
    
sprintf('Plotting to frame %d', handles.frame_i)
cla;
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

