function [] = cylinderSampleProjection(K, cameraPose, imageSize, cylinders)
% Project sampled points on cylinder to camera frame
% Authors: Zhaoyang Lv

    cylinder_num = size(cylinders, 1);
   
    camera = SimpleCamera(cameraPose, K);
        
    for i = 1:cylinder_num

        point_num = size( cylinders{i}.Points, 1);
        
        % to check point visibility        
        for j = 1:point_num
            sampledPoint3 = cylinders{i}.Poinsts{j};
            measurements2d = camera.project(sampledPoint3);
            
            % ignore points not visible in the scene
            if measurements2d.x < 0 || measurements.x >= imageSize.x ...
                    || measurements2d.y < 0 || measurements.y >= imageSize.y 
                continue;       
            end
            % ignore points occluded
            % use a simple math hack to check occlusion:
            %   1. All points in front of cylinders' surfaces are visible
            %   2. For points behind the cylinders' surfaces, the cylinder 
            for k = 1:cylinder_num
                
                rayCameraToPoint = sampledPoint3 - cameraPose.t;
                rayCameraToCylinder = cylinders{i} - cameraPose.t;
                
                projectedRay = dot(rayCameraToPoint, rayCameraToCylinder);
                distCameraToCylinder = norm(rayCameraToCylinder);
                
                
            end
        end
        
    end
    
end
