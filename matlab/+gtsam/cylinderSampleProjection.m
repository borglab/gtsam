function [visiblePoints, visiblePointsCylinderIdx] = cylinderSampleProjection(K, cameraPose, imageSize, cylinders)
% Project sampled points on cylinder to camera frame
% Authors: Zhaoyang Lv

    import gtsam.*

    cylinder_num = size(cylinders, 1);
   
    %camera = SimpleCamera(cameraPose, K);
    camera = SimpleCamera.Lookat(cameraPose.translation(), cylinders{10}.centroid, Point3([0,0,1]'), K);

    visiblePoints = {};
    visiblePointsCylinderIdx = [];
    
    for i = 1:cylinder_num

        point_num = size( cylinders{i}.Points, 1);
        
        % to check point visibility        
        for j = 1:point_num
            sampledPoint3 = cylinders{i}.Points{j};
            measurements2d = camera.project(sampledPoint3);
                     
            % ignore points not visible in the scene
            if measurements2d.x < 0 || measurements2d.x >= imageSize.x ...
                    || measurements2d.y < 0 || measurements2d.y >= imageSize.y 
                continue;       
            end            
            
            % ignore points occluded
            % use a simple math hack to check occlusion:
            %   1. All points in front of cylinders' surfaces are visible
            %   2. For points behind the cylinders' surfaces, the cylinder
            for k = 1:cylinder_num
                
                rayCameraToPoint = cameraPose.translation().between(sampledPoint3).vector();
                rayCameraToCylinder = cameraPose.translation().between(cylinders{i}.centroid).vector();
                rayCylinderToPoint = cylinders{i}.centroid.between(sampledPoint3).vector();
                
                % Condition 1: all points in front of the cylinders'
                % surfaces are visible
                if dot(rayCylinderToPoint, rayCameraToCylinder) < 0
                    visiblePoints{end+1} = sampledPoint3;
                    visiblePointsCylinderIdx = [visiblePointsCylinderIdx, i];
                    continue;
                end

                % Condition 2
                projectedRay = dot(rayCameraToCylinder, rayCameraToPoint);
                if projectedRay > 0
                    rayCylinderToProjected = norm(projectedRay) / norm(rayCameraToPoint) * rayCameraToPoint;
                    if rayCylinderToProjected(1) > cylinders{i}.radius && ...
                            rayCylinderToProjected(2) > cylinders{i}.radius
                        visiblePoints{end+1} = sampledPoint3;
                        visiblePointsCylinderIdx = [visiblePointsCylinderIdx, i];
                    end
                end
                
            end
        end
        
    end
    
end
