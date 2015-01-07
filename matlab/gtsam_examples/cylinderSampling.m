function [cylinder] = cylinderSampling(Point2, radius, height, density)
%
    import gtsam.*
    % calculate the cylinder area
    A = 2 * pi * radius * height;
    
    PointsNum = round(A * density);
    
    Points3 = cell(PointsNum, 1);
    
    % sample the points
    for i = 1:PointsNum
        theta = 2 * pi * rand;
        x = radius * cos(theta) + Point2.x;
        y = radius * sin(theta) + Point2.y;
        z = height * rand;
        Points3{i,1} = Point3([x,y,z]');
    end
   
    cylinder.area = A;
    cylinder.Points = Points3;
end