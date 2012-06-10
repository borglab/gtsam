function VisualISAMPlot(data, isam, result, options)
% VisualISAMPlot: plot current state of visualSLAM::iSAM object
% Authors: Duy Nguyen Ta and Frank Dellaert

M = double(result.nrPoses);
N = double(result.nrPoints);

h=gca;
cla(h);
hold on;

%% Plot points
for j=1:N
    jj = symbol('l',j);
    point_j = result.point(jj);
    plot3(point_j.x, point_j.y, point_j.z,'marker','o');
    P = isam.marginalCovariance(jj);
    covarianceEllipse3D([point_j.x;point_j.y;point_j.z],P);
end

%% Plot cameras
for i=1:options.cameraInterval:M
    ii = symbol('x',i);
    pose_i = result.pose(ii);
    if options.hardConstraint & (i==1)
        plotPose3(pose_i,[],10);
    else
        P = isam.marginalCovariance(ii);
        plotPose3(pose_i,P,10);
    end
    if options.drawTruePoses % show ground truth
        plotPose3(data.cameras{i}.pose,[],10);
    end
end

%% draw
axis([-40 40 -40 40 -10 20]);axis equal
view(3)
colormap('hot')
drawnow

%% do various optional things

if options.saveFigures
    fig2 = figure('visible','off');
    newax = copyobj(h,fig2);
    colormap(fig2,'hot');
    set(newax, 'units', 'normalized', 'position', [0.13 0.11 0.775 0.815]);
    print(fig2,'-dpng',sprintf('VisualiSAM%03d.png',M));
end

if options.saveDotFiles
    isam.saveGraph(sprintf('VisualiSAM%03d.dot',M));
end

if options.saveDotFile
    isam.saveGraph(sprintf('VisualiSAM.dot'));
end

if options.printStats
    isam.printStats();
end
