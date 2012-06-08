VisualISAMGlobalVars

%% Plot results
tic
%     h=figure(2);clf
%     set(1,'NumberTitle','off','Name','Visual iSAM');
h=gca;
cla(h);
hold on;
sprintf('Computing marginals and plotting. Please wait...')
for j=1:size(points,2)
    point_j = result.point(symbol('l',j));
    plot3(point_j.x, point_j.y, point_j.z,'marker','o');
    if (frame_i>1) 
        P = isam.marginalCovariance(symbol('l',j));
        covarianceEllipse3D([point_j.x;point_j.y;point_j.z],P); 
    end
end
for ii=1:CAMERA_INTERVAL:frame_i
    pose_ii = result.pose(symbol('x',ii));
    if (frame_i>1)
        P = isam.marginalCovariance(symbol('x',ii));
    else 
        P = []
    end
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
% if DRAW_INTERVAL~=NCAMERAS, plot(frame_i,t,'b.'); end
if SAVE_FIGURES
    print(h,'-dpng',sprintf('VisualiSAM%03d.png',frame_i));
end
if SAVE_GRAPHS
    isam.saveGraph(sprintf('VisualiSAM%03d.dot',frame_i));
end

drawnow