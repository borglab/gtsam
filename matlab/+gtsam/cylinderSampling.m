function [cylinder] = cylinderSampling(baseCentroid, radius, height, density)
% 
% @author: Zhaoyang Lv
    import gtsam.*
    % calculate the cylinder area
    area = 2 * pi * radius * height;
    
    pointsNum = round(area * density);
    
    points3 = cell(pointsNum, 1);
    
    % sample the points
    for i = 1:pointsNum
        theta = 2 * pi * rand;
        x = radius * cos(theta) + baseCentroid(1);
        y = radius * sin(theta) + baseCentroid(2);
        z = height * rand;
        points3{i,1} = Point3([x,y,z]');
    end
   
    cylinder.area = area;
    cylinder.radius = radius;
    cylinder.height = height;
    cylinder.Points = points3;
    cylinder.centroid = Point3(baseCentroid(1), baseCentroid(2), height/2);
end