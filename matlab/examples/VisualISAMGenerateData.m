function data = VisualISAMGenerateData(options, showImages)
% VisualISAMGenerateData: create data for viusalSLAM::iSAM examples
% Authors: Duy Nguyen Ta and Frank Dellaert
if nargin<2, showImages=false; end

%% Generate simulated data
data.points = {};
if options.triangle % Create a triangle target, just 3 points on a plane
    nPoints = 3;
    r = 10;
    for j=1:nPoints
        theta = (j-1)*2*pi/nPoints;
        data.points{j} = gtsamPoint3([r*cos(theta), r*sin(theta), 0]');
    end
else % 3D landmarks as vertices of a cube
    nPoints = 8;
    data.points = {gtsamPoint3([10 10 10]'),...
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
data.K = gtsamCal3_S2(500,500,0,640/2,480/2);
data.cameras = {};
for i=1:options.nrCameras
    theta = (i-1)*2*pi/options.nrCameras;
    t = gtsamPoint3([r*cos(theta), r*sin(theta), height]');
    data.cameras{i} = gtsamSimpleCamera_lookat(t, gtsamPoint3, gtsamPoint3([0,0,1]'), data.K);
end

%% show images if asked
if showImages
    gui = gcf;
    for i=1:options.nrCameras
        figure(2+i);clf;hold on
        set(2+i,'NumberTitle','off','Name',sprintf('Camera %d',i));
        for j=1:nPoints
            zij = data.cameras{i}.project(data.points{j});
            plot(zij.x,zij.y,'*');
            axis([1 640 1 480]);
        end
    end
    figure(gui);
end

%% Calculate odometry between cameras
data.odometry = data.cameras{1}.pose.between(data.cameras{2}.pose);
