function plotCylinderSamples(cylinders, fieldSize)

    holdstate = ishold;
    hold on
    
    num = size(cylinders, 1);

    sampleDensity = 120;
    figure

    for i = 1:num        
        %base.z = cylinders{i}.centroid.z - cylinders{i}.height/2;        
        [X,Y,Z] = cylinder(cylinders{i}.radius, sampleDensity * cylinders{i}.radius * cylinders{i}.height);
        
        X = X + cylinders{i}.centroid.x;
        Y = Y + cylinders{i}.centroid.y;
        Z = Z * cylinders{i}.height;
        
        cylinderHandle = surf(X,Y,Z);
        set(cylinderHandle, 'FaceAlpha', 0.5);
        hold on
    end
    
    axis equal
    axis([0, fieldSize.x, 0, fieldSize.y, 0, 20]);
        
    if ~holdstate
        hold off
    end
    
end
