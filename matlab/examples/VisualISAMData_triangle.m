function [ data ] = VisualISAMData_triangle()
%VISUALISAMDATA_TRIANGLE Generate data for visual ISAM triangle example.
%   Landmarks include 3 points around the world's origin on the z=0 plane. 
%   Cameras are on a circle at a certain height, looking at the origin.
%% Create a triangle target, just 3 points on a plane
nPoints = 3;
r = 10;
data.points = {};
for j=1:nPoints
    theta = (j-1)*2*pi/nPoints;
    data.points{j} = gtsamPoint3([r*cos(theta), r*sin(theta), 0]');
end

%% Create camera cameras on a circle around the triangle
nCameras = 10;
height = 10;
r = 30;
data.cameras = {};
data.K = gtsamCal3_S2(500,500,0,640/2,480/2);
for i=1:nCameras
    theta = (i-1)*2*pi/nCameras;
    t = gtsamPoint3([r*cos(theta), r*sin(theta), height]');
    data.cameras{i} = gtsamSimpleCamera_lookat(t, gtsamPoint3, gtsamPoint3([0,0,1]'), data.K);
end

data.posePriorNoise = gtsamSharedNoiseModel_Sigmas([0.001 0.001 0.001 5.0 5.0 5.0]');
data.odometryNoise = gtsamSharedNoiseModel_Sigmas([0.001 0.001 0.001 2.0 2.0 2.0]');
data.pointPriorNoise = gtsamSharedNoiseModel_Sigma(3, 0.1);
data.measurementNoise = gtsamSharedNoiseModel_Sigma(2, 1.0);

end

