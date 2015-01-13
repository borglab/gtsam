function [visiblePoints] = cylinderSampleProjection(camera, imageSize, cylinders)

% Input: 
% Output:
%   visiblePoints:  data{k} 3D Point in overal point clouds with index k 
%                   Z{k}    2D measurements in overal point clouds with index k   
%                   index {i}{j}
%                   i: the cylinder index;
%                   j: the point index on the cylinder;
%                   
% @Description: Project sampled points on cylinder to camera frame
% @Authors: Zhaoyang Lv

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
                
        sampledPoint3 = cylinders{i}.Points{j};
        sampledPoint3local = camera.pose.transform_to(sampledPoint3);
        if sampledPoint3local.z < 0
            continue; % Cheirality Exception
        end
        Z2d = camera.project(sampledPoint3);

        % ignore points not visible in the scene
        if Z2d.x < 0 || Z2d.x >= imageSize.x ...
                || Z2d.y < 0 || Z2d.y >= imageSize.y 
            continue;       
        end            

        % ignore points occluded
        % use a simple math hack to check occlusion:
        %   1. All points in front of cylinders' surfaces are visible
        %   2. For points behind the cylinders' surfaces, the cylinder
        for k = 1:cylinderNum

            rayCameraToPoint = camera.pose.translation().between(sampledPoint3).vector();
            rayCameraToCylinder = camera.pose.translation().between(cylinders{i}.centroid).vector();
            rayCylinderToPoint = cylinders{i}.centroid.between(sampledPoint3).vector();

            % Condition 1: all points in front of the cylinders'
            % surfaces are visible
            if dot(rayCylinderToPoint, rayCameraToCylinder) < 0
                visiblePoints.data{pointCloudIndex} = sampledPoint3;
                visiblePoints.Z{pointCloudIndex} = Z2d;
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
                    visiblePoints.Z{pointCloudIndex} = Z2d;
                    visiblePoints.index{i}{j} = pointCloudIndex;
                end
            end

        end
    end

end
    
end
