function [visiblePoints] = cylinderSampleProjection(K, pose, imageSize, cylinders)

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

camera = PinholeCameraCal3_S2(pose, K);

%% memory allocation
cylinderNum = length(cylinders);

%% check visiblity of points on each cylinder
pointCloudIndex = 0;
visiblePointIdx = 1;
for i = 1:cylinderNum
    
    pointNum = length(cylinders{i}.Points);

    % to check point visibility        
    for j = 1:pointNum

        pointCloudIndex  = pointCloudIndex + 1;
                
        % Cheirality Exception
        sampledPoint3 = cylinders{i}.Points{j};
        sampledPoint3local = pose.transformTo(sampledPoint3);        
        if sampledPoint3local.z <= 0
            continue; 
        end
        Z2d = camera.project(sampledPoint3);

        % ignore points not visible in the scene
        if Z2d.x < 0 || Z2d.x >= imageSize.x || Z2d.y < 0 || Z2d.y >= imageSize.y 
            continue;       
        end            

        % ignore points occluded
        % use a simple math hack to check occlusion:
        %   1. All points in front of cylinders' surfaces are visible
        %   2. For points behind the cylinders' surfaces, the cylinder
        visible = true;
        for k = 1:cylinderNum

            rayCameraToPoint = pose.translation().between(sampledPoint3).vector();
            rayCameraToCylinder = pose.translation().between(cylinders{k}.centroid).vector();
            rayCylinderToPoint = cylinders{k}.centroid.between(sampledPoint3).vector();

            % Condition 1: all points in front of the cylinders'
            % surfaces are visible
            if dot(rayCylinderToPoint, rayCameraToCylinder) < 0
               continue;
            else 
                projectedRay = dot(rayCameraToCylinder, rayCameraToPoint) / norm(rayCameraToCylinder);
                if projectedRay > 0
                    %rayCylinderToProjected = rayCameraToCylinder - norm(projectedRay) / norm(rayCameraToPoint) * rayCameraToPoint;
                    if rayCylinderToPoint(1) > cylinders{k}.radius && ...
                            rayCylinderToPoint(2) > cylinders{k}.radius
                       continue;
                    else
                        visible = false;
                        break;
                    end
                end
            end
            
        end
        
        if visible
            visiblePoints.data{visiblePointIdx} = sampledPoint3;
            visiblePoints.Z{visiblePointIdx} = Z2d;
            visiblePoints.cylinderIdx{visiblePointIdx} = i;
            visiblePoints.overallIdx{visiblePointIdx} = pointCloudIndex;
            visiblePointIdx = visiblePointIdx + 1;
        end

    end
    
end
    
end
