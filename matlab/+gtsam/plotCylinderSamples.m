function plotCylinderSamples(cylinders, fieldSize, figID)
% plot the cylinders on the given field
% @author: Zhaoyang Lv

    figure(figID);

    holdstate = ishold;
    hold on
    
    num = size(cylinders, 1);

    sampleDensity = 120;
    
    for i = 1:num                
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
        
    grid on
    
    if ~holdstate
        hold off
    end
    
end
