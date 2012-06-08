function [ handles ] = vData( handles )
%VDATA Summary of this function goes here
%   Detailed explanation goes here
%% Generate simulated data
if handles.TRIANGLE % Create a triangle target, just 3 points on a plane
    handles.nPoints = 3;
    r = 10;
    for j=1:handles.nPoints
        theta = (j-1)*2*pi/handles.nPoints;
        handles.points{j} = gtsamPoint3([r*cos(theta), r*sin(theta), 0]');
    end
else % 3D landmarks as vertices of a cube
    handles.nPoints = 8;
    handles.points = {gtsamPoint3([10 10 10]'),...
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
handles.K = gtsamCal3_S2(500,500,0,640/2,480/2);
for i=1:handles.NCAMERAS
    theta = (i-1)*2*pi/handles.NCAMERAS;
    t = gtsamPoint3([r*cos(theta), r*sin(theta), height]');
    handles.cameras{i} = gtsamSimpleCamera_lookat(t, gtsamPoint3, gtsamPoint3([0,0,1]'), handles.K);
%     if handles.SHOW_IMAGES % show images
%         figure(2+i);clf;hold on
%         set(2+i,'NumberTitle','off','Name',sprintf('Camera %d',i));
%         for j=1:nPoints
%             zij = handles.cameras{i}.project(handles.points{j});
%             plot(zij.x,zij.y,'*');
%             axis([1 640 1 480]);
%         end
%     end
end
handles.odometry = handles.cameras{1}.pose.between(handles.cameras{2}.pose);


%% Set Noise parameters
handles.poseNoise = gtsamSharedNoiseModel_Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
handles.odometryNoise = gtsamSharedNoiseModel_Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
handles.pointNoise = gtsamSharedNoiseModel_Sigma(3, 0.1);
handles.measurementNoise = gtsamSharedNoiseModel_Sigma(2, 1.0);

end

