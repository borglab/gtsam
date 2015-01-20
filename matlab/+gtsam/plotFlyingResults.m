function plotFlyingResults(pts3d, pts3dCov, poses, posesCov, cylinders, options)
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

axis equal
axis([0, options.fieldSize.x, 0, options.fieldSize.y, 0, 20]);

view(3);

sampleDensity = 120;
cylinderNum = length(cylinders);
for i = 1:cylinderNum                
    [X,Y,Z] = cylinder(cylinders{i}.radius, sampleDensity * cylinders{i}.radius * cylinders{i}.height);

    X = X + cylinders{i}.centroid.x;
    Y = Y + cylinders{i}.centroid.y;
    Z = Z * cylinders{i}.height;

    h_cylinder = surf(X,Y,Z);
    set(h_cylinder, 'FaceAlpha', 0.5);
    hold on
end

drawnow;

if options.writeVideo
    currFrame = getframe(gcf);
    writeVideo(videoObj, currFrame);
end

%% plot trajectories
posesSize = length(poses);
for i = 1:posesSize
    if i > 1
        plot3([poses{i}.x; poses{i-1}.x], [poses{i}.y; poses{i-1}.y], [poses{i}.z; poses{i-1}.z], '-b');
    end
    
    if exist('h_cov', 'var')
        delete(h_cov);
    end
    
    gRp = poses{i}.rotation().matrix();  % rotation from pose to global
    C = poses{i}.translation().vector();
    axisLength = 2;
    
    xAxis = C+gRp(:,1)*axisLength;
    L = [C xAxis]';
    line(L(:,1),L(:,2),L(:,3),'Color','r');
    
    yAxis = C+gRp(:,2)*axisLength;
    L = [C yAxis]';
    line(L(:,1),L(:,2),L(:,3),'Color','g');
    
    zAxis = C+gRp(:,3)*axisLength;
    L = [C zAxis]';
    line(L(:,1),L(:,2),L(:,3),'Color','b');
    
    pPp = posesCov{i}(4:6,4:6); % covariance matrix in pose coordinate frame    
    gPp = gRp*pPp*gRp'; % convert the covariance matrix to global coordinate frame
    h_cov = gtsam.covarianceEllipse3D(C,gPp); 
    
    drawnow;
    
    if options.writeVideo
        currFrame = getframe(gcf);
        writeVideo(videoObj, currFrame);
    end
end


if exist('h_cov', 'var')
    delete(h_cov);
end

% wait for two seconds
pause(2);


%% plot point covariance

% if exist('h_cylinder', 'var')
%     delete(h_cylinder);
% end

pointSize = length(pts3d);
for i = 1:pointSize
    plot3(pts3d{i}.x, pts3d{i}.y, pts3d{i}.z);
    gtsam.covarianceEllipse3D([pts3d{i}.x; pts3d{i}.y; pts3d{i}.z], pts3dCov{i});
    %drawnow;
    
    if options.writeVideo
        currFrame = getframe(gcf);
        writeVideo(videoObj, currFrame);
    end
end

if ~holdstate
    hold off
end

%%close video
if(options.writeVideo)
    close(videoObj);
end

end