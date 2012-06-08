VisualISAMGlobalVars

if (frame_i<2)
    sprintf('Cannot plot the first frame')
    return
end

%% Plot results
tic
%     h=figure(2);clf
%     set(1,'NumberTitle','off','Name','Visual iSAM');
cla;
hold on;
sprintf('Computing marginals and plotting. Please wait...')
for j=1:size(points,2)
    P = isam.marginalCovariance(symbol('l',j));
    point_j = result.point(symbol('l',j));
    plot3(point_j.x, point_j.y, point_j.z,'marker','o');
    covarianceEllipse3D([point_j.x;point_j.y;point_j.z],P);
end
for ii=1:CAMERA_INTERVAL:frame_i
    P = isam.marginalCovariance(symbol('x',ii));
    pose_ii = result.pose(symbol('x',ii));
    plotPose3(pose_ii,P,10);
    if DRAW_TRUE_POSES % show ground truth
        plotPose3(cameras{ii}.pose,0.001*eye(6),10);
    end
end
axis([-40 40 -40 40 -10 20]);axis equal
view(3)
colormap('hot')
sprintf('Done!')

%     figure(2);
t=toc;
if DRAW_INTERVAL~=NCAMERAS, plot(frame_i,t,'b.'); end
if SAVE_FIGURES
    print(h,'-dpng',sprintf('VisualiSAM%03d.png',frame_i));
end
if SAVE_GRAPHS
    isam.saveGraph(sprintf('VisualiSAM%03d.dot',frame_i));
end
