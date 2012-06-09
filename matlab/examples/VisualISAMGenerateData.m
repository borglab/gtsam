VisualISAMGlobalVars

%% Generate simulated data
points = {};
if TRIANGLE % Create a triangle target, just 3 points on a plane
    nPoints = 3;
    r = 10;
    for j=1:nPoints
        theta = (j-1)*2*pi/nPoints;
        points{j} = gtsamPoint3([r*cos(theta), r*sin(theta), 0]');
    end
else % 3D landmarks as vertices of a cube
    nPoints = 8;
    points = {gtsamPoint3([10 10 10]'),...
        gtsamPoint3([-10 10 10]'),...
        gtsamPoint3([-10 -10 10]'),...
        gtsamPoint3([10 -10 10]'),...
        gtsamPoint3([10 10 -10]'),...
        gtsamPoint3([-10 10 -10]'),...
        gtsamPoint3([-10 -10 -10]'),...
        gtsamPoint3([10 -10 -10]')};
end

%% Create camera cameras on a circle around the triangle
height = 10; r = 40;
K = gtsamCal3_S2(500,500,0,640/2,480/2);
cameras = {};
gui = gcf;
for i=1:NCAMERAS
    theta = (i-1)*2*pi/NCAMERAS;
    t = gtsamPoint3([r*cos(theta), r*sin(theta), height]');
    cameras{i} = gtsamSimpleCamera_lookat(t, gtsamPoint3, gtsamPoint3([0,0,1]'), K);
    if SHOW_IMAGES % show images
        figure(2+i);clf;hold on
        set(2+i,'NumberTitle','off','Name',sprintf('Camera %d',i));
        for j=1:nPoints
            zij = cameras{i}.project(points{j});
            plot(zij.x,zij.y,'*');
            axis([1 640 1 480]);
        end
    end
end
figure(gui);
odometry = cameras{1}.pose.between(cameras{2}.pose);


%% Set Noise parameters
poseNoise = gtsamSharedNoiseModel_Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
odometryNoise = gtsamSharedNoiseModel_Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
pointNoise = gtsamSharedNoiseModel_Sigma(3, 0.1);
measurementNoise = gtsamSharedNoiseModel_Sigma(2, 1.0);