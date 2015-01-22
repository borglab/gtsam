function plotFlyingResults(pts3d, poses, posesCov, cylinders, options)
% plot the visible points on the cylinders and trajectories
% author: Zhaoyang Lv

import gtsam.*

holdstate = ishold;
hold on

if(options.writeVideo)
    videoObj = VideoWriter('Camera_Flying_Example.avi');
    videoObj.Quality = 100;
    videoObj.FrameRate = options.camera.fps;
    open(videoObj);
end

%% plot all the cylinders and sampled points
% now is plotting on a 100 * 100 field
figID = 1;
figure(figID);

view([30, 30]);

hlight = camlight('headlight'); 
lighting gouraud

sampleDensity = 120;
cylinderNum = length(cylinders);
for i = 1:cylinderNum                
    [X,Y,Z] = cylinder(cylinders{i}.radius, sampleDensity * cylinders{i}.radius * cylinders{i}.height);

    X = X + cylinders{i}.centroid.x;
    Y = Y + cylinders{i}.centroid.y;
    Z = Z * cylinders{i}.height;

    h_cylinder = surf(X,Y,Z);
    set(h_cylinder, 'FaceColor', [0 0 1], 'FaceAlpha', 0.2);
    h_cylinder.AmbientStrength = 1;
    hold on
end

drawnow;

if options.writeVideo
    currFrame = getframe(gcf);
    writeVideo(videoObj, currFrame);
end

%% plot trajectories and points 
posesSize = length(poses);
pointSize = length(pts3d);
for i = 1:posesSize
    if i > 1
        plot3([poses{i}.x; poses{i-1}.x], [poses{i}.y; poses{i-1}.y], [poses{i}.z; poses{i-1}.z], '-b');
    end
    
    if exist('h_cov', 'var')
        delete(h_pose_cov);
    end
       
    plotCamera(poses{i}, 2);
    
     gRp = poses{i}.rotation().matrix();  % rotation from pose to global
     C = poses{i}.translation().vector();
%     axisLength = 2;
%     
%     xAxis = C+gRp(:,1)*axisLength;
%     L = [C xAxis]';
%     line(L(:,1),L(:,2),L(:,3),'Color','r');
%     
%     yAxis = C+gRp(:,2)*axisLength;
%     L = [C yAxis]';
%     line(L(:,1),L(:,2),L(:,3),'Color','g');
%     
%     zAxis = C+gRp(:,3)*axisLength;
%     L = [C zAxis]';
%     line(L(:,1),L(:,2),L(:,3),'Color','b');
%     
    pPp = posesCov{i}(4:6,4:6); % covariance matrix in pose coordinate frame    
    gPp = gRp*pPp*gRp'; % convert the covariance matrix to global coordinate frame
    h_pose_cov = gtsam.covarianceEllipse3D(C,gPp); 
      
    
    if exist('h_point', 'var')
        for j = 1:pointSize
            delete(h_point{j});
        end
    end
    if exist('h_point_cov', 'var')
        for j = 1:pointSize
            delete(h_point_cov{j});
        end
    end
    
    h_point = cell(pointSize, 1);
    h_point_cov = cell(pointSize, 1);
    for j = 1:pointSize
        if ~isempty(pts3d{j}.cov{i})
            h_point{j} = plot3(pts3d{j}.data.x, pts3d{j}.data.y, pts3d{j}.data.z);
            h_point_cov{j} = gtsam.covarianceEllipse3D([pts3d{j}.data.x; pts3d{j}.data.y; pts3d{j}.data.z], pts3d{j}.cov{i});
        end
    end
    
    axis equal
    axis([0, options.fieldSize.x, 0, options.fieldSize.y, 0, 20]);
    
    drawnow;
    
    if options.writeVideo
        currFrame = getframe(gcf);
        writeVideo(videoObj, currFrame);
    end
end


if exist('h_pose_cov', 'var')
    delete(h_pose_cov);
end

% wait for two seconds
pause(2);

% change views angle
for i = 0 : 0.5 : 60
    view([i + 30, i]);
end

% camera flying through
for i = 1 : posesSize
    campos([poses{i}.x, poses{i}.y, poses{i}.z]);
    camtarget([options.fieldSize.x/2, options.fieldSize.y/2, 0]);
    camlight(hlight, 'headlight');
    
    drawnow
end


if ~holdstate
    hold off
end

%%close video
if(options.writeVideo)
    close(videoObj);
end

end