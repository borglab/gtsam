function [visiblePoints] = cylinderSampleProjectionStereo(K, pose, imageSize, cylinders)

import gtsam.*

%% memory allocation
cylinderNum = length(cylinders);
visiblePoints.index = cell(cylinderNum,1);

pointCloudNum = 0;
for i = 1:cylinderNum
    pointCloudNum = pointCloudNum + length(cylinders{i}.Points);
    visiblePoints.index{i} = cell(pointCloudNum,1);
end
visiblePoints.data = cell(pointCloudNum,1);
visiblePoints.Z = cell(pointCloudNum, 1);

%% check visiblity of points on each cylinder
pointCloudIndex = 0;
for i = 1:cylinderNum
    
    pointNum = length(cylinders{i}.Points);

    % to check point visibility        
    for j = 1:pointNum

        pointCloudIndex  = pointCloudIndex + 1;
                
        % For Cheirality Exception
        sampledPoint3 = cylinders{i}.Points{j};
        sampledPoint3local = pose.transform_to(sampledPoint3);
        if sampledPoint3local.z < 0
            continue; 
        end
        
        % measurements 
        Z.du = K.fx() * K.baseline() / samplePoint3.z;  
        Z.uL = K.fx() * samplePoint3.x / samplePoint3.z + K.px();
        Z.uR = uL + du;
        Z.v = K.fy() / samplePoint3.z + K.py();

        % ignore points not visible in the scene
        if Z.uL < 0 || Z.uL >= imageSize.x || ...
                Z.uR < 0 || Z.uR >= imageSize.x || ...
                Z.v < 0 || Z.v >= imageSize.y 
            continue;       
        end            

        % ignore points occluded
        % use a simple math hack to check occlusion:
        %   1. All points in front of cylinders' surfaces are visible
        %   2. For points behind the cylinders' surfaces, the cylinder
        for k = 1:cylinderNum

            rayCameraToPoint = pose.translation().between(sampledPoint3).vector();
            rayCameraToCylinder = pose.translation().between(cylinders{i}.centroid).vector();
            rayCylinderToPoint = cylinders{i}.centroid.between(sampledPoint3).vector();

            % Condition 1: all points in front of the cylinders'
            % surfaces are visible
            if dot(rayCylinderToPoint, rayCameraToCylinder) < 0
                visiblePoints.data{pointCloudIndex} = sampledPoint3;
                visiblePoints.Z{pointCloudIndex} = Z;
                visiblePoints.index{i}{j} = pointCloudIndex; 
                continue;
            end

            % Condition 2
            projectedRay = dot(rayCameraToCylinder, rayCameraToPoint);
            if projectedRay > 0
                rayCylinderToProjected = norm(projectedRay) / norm(rayCameraToPoint) * rayCameraToPoint;
                if rayCylinderToProjected(1) > cylinders{i}.radius && ...
                        rayCylinderToProjected(2) > cylinders{i}.radius
                    visiblePoints.data{pointCloudIndex} = sampledPoint3;
                    visiblePoints.Z{pointCloudIndex} = Z;
                    visiblePoints.index{i}{j} = pointCloudIndex;
                end
            end

        end
    end

end

end
