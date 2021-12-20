function plotFlyingResults(pts3d, poses, posesCov, cylinders, options)
% plot the visible points on the cylinders and trajectories
%
% author: Zhaoyang Lv

import gtsam.*

figID = 1;
figure(figID);
set(gcf, 'Position', [80,1,1800,1000]);


%% plot all the cylinders and sampled points

axis equal
axis([0, options.fieldSize(1), 0, options.fieldSize(2), 0, options.height + 30]);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Height (m)');

h = cameratoolbar('Show');

if options.camera.IS_MONO
    h_title = title('Quadrotor Flight Simulation with Monocular Camera');
else
    h_title = title('Quadrotor Flight Simulation with Stereo Camera');
end

text(100,1750,0, sprintf('Flying Speed: %0.1f\n', options.speed))

view([30, 30]);

hlight = camlight('headlight'); 
lighting gouraud

if(options.writeVideo)
    videoObj = VideoWriter('Camera_Flying_Example.avi');
    videoObj.Quality = 100;
    videoObj.FrameRate = options.camera.fps;
    open(videoObj);
end


sampleDensity = 120;
cylinderNum = length(cylinders);
h_cylinder = cell(cylinderNum);
for i = 1:cylinderNum
    
    hold on
    
    [X,Y,Z] = cylinder(cylinders{i}.radius, sampleDensity * cylinders{i}.radius * cylinders{i}.height);

    X = X + cylinders{i}.centroid(1);
    Y = Y + cylinders{i}.centroid(2);
    Z = Z * cylinders{i}.height;

    h_cylinder{i} = surf(X,Y,Z);
    set(h_cylinder{i}, 'FaceColor', [0 0 1], 'FaceAlpha', 0.2);
    h_cylinder{i}.AmbientStrength = 0.8;
    
end

%% plot trajectories and points 
posesSize = length(poses);
pointSize = length(pts3d);
for i = 1:posesSize
    if i > 1
        hold on
        plot3([poses{i}.x; poses{i-1}.x], [poses{i}.y; poses{i-1}.y], [poses{i}.z; poses{i-1}.z], '-b');
    end
    
    if exist('h_pose_cov', 'var')
        delete(h_pose_cov);
    end
       
    %plotCamera(poses{i}, 3);
    
    gRp = poses{i}.rotation().matrix();  % rotation from pose to global
    C = poses{i}.translation();
    axisLength = 3;
    
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
    h_pose_cov = gtsam.covarianceEllipse3D(C, gPp, options.plot.covarianceScale); 
    
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
            hold on
            h_point{j} = plot3(pts3d{j}.data(1), pts3d{j}.data(2), pts3d{j}.data(3));
            h_point_cov{j} = gtsam.covarianceEllipse3D([pts3d{j}.data(1); pts3d{j}.data(2); pts3d{j}.data(3)], ...
                pts3d{j}.cov{i}, options.plot.covarianceScale);
        end
    end 
    
    axis equal
    axis([0, options.fieldSize(1), 0, options.fieldSize(2), 0, options.height + 30]);

    drawnow
    
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

%% change views angle
for i = 30 : i : 90
    view([30, i]);

    if options.writeVideo
        currFrame = getframe(gcf);
        writeVideo(videoObj, currFrame);
    end
    
    drawnow
end

% changing perspective


%% camera flying through video
camzoom(0.8);
for i = 1 : posesSize
    
    hold on
    
    campos([poses{i}.x, poses{i}.y, poses{i}.z]);
    camtarget([options.fieldSize(1)/2, options.fieldSize(2)/2, 0]);
    camlight(hlight, 'headlight');
    
    if options.writeVideo
        currFrame = getframe(gcf);
        writeVideo(videoObj, currFrame);
    end
    
    drawnow
end

%%close video
if(options.writeVideo)
    close(videoObj);
end

end