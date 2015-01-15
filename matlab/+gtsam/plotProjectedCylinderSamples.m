function plotProjectedCylinderSamples(pts3d, covariance, options, figID)
% plot the visible points on the cylinders
% author: Zhaoyang Lv

    import gtsam.*

    figure(figID);

    holdstate = ishold;
    hold on

    pointsNum = length(pts3d);
    for i = 1:pointsNum
        %gtsam.plotPoint3(p, 'g', covariance{i});
        plotPoint3(pts3d{i}, 'r', covariance{i}); 
        hold on
    end
    
%     for i=1:pointsNum
%         ray = pts2dTracksStereo{i}.between(cameraPose.translation()).vector();
%         dist = norm(ray);
%         
%         p = plot3(pts2dTracksStereo{i}.x, pts2dTracksStereo{i}.y, pts2dTracksStereo{i}.z, ...
%             'o', 'MarkerFaceColor', 'Green');
%         
%         for t=0:0.1:dist
%             marchingRay = ray * t;
%             p.XData = pts2dTracksStereo{i}.x + marchingRay(1);
%             p.YData = pts2dTracksStereo{i}.y + marchingRay(2);
%             p.ZData = pts2dTracksStereo{i}.z + marchingRay(3);
%             drawnow update
%         end
%         
%     end

    axis equal;
    axis([0, options.fieldSize.x, 0, options.fieldSize.y, 0, 20]);
    
    if ~holdstate
        hold off
    end
end