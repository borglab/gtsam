function VisualISAMPlot( results, data )
%VISUALISAMPLOT Plot results of a step in visual ISAM
    hold on;
    
    for i=1:results.frame_i
        pose_ii = results.estimates.pose(symbol('x',i));
        plotPose3(pose_ii,results.Pposes{i},10);
    end
    
    for j=1:size(data.points,2)
        point_j = results.estimates.point(symbol('l',j));
        plot3(point_j.x, point_j.y, point_j.z,'marker','o');
        covarianceEllipse3D([point_j.x;point_j.y;point_j.z],results.Ppoints{j});
    end
    
    axis([-35 35 -35 35 -35 35])
    view([36 34])
    colormap('hot')
    
    hold off;
end

