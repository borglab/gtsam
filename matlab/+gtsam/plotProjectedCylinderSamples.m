function plotProjectedCylinderSamples(visiblePoints3, cameraPose, figID)
% plot the visible projected points on the cylinders
% author: Zhaoyang Lv

    import gtsam.*

    figure(figID);

    holdstate = ishold;
    hold on

    %plotCamera(cameraPose, 5);
    
    pointsNum = size(visiblePoints3, 1)
    
    for i=1:pointsNum
        ray = visiblePoints3{i}.between(cameraPose.translation()).vector();
        dist = norm(ray);
        
        p = plot3(visiblePoints3{i}.x, visiblePoints3{i}.y, visiblePoints3{i}.z, ...
            'o', 'MarkerFaceColor', 'Green');
        
        for t=0:0.1:dist
            marchingRay = ray * t;
            p.XData = visiblePoints3{i}.x + marchingRay(1);
            p.YData = visiblePoints3{i}.y + marchingRay(2);
            p.ZData = visiblePoints3{i}.z + marchingRay(3);
            drawnow update
        end
        
    end
    
    if ~holdstate
        hold off
    end
end