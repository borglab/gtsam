%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Read graph from file and perform GraphSLAM
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
clc;
import gtsam.*

write_video = true;

if(write_video)
    videoObj = VideoWriter('test.avi');
    videoObj.Quality = 100;
    videoObj.FrameRate = 2;
    open(videoObj);
end

%% generate some landmarks
nrPoints = 8;
 landmarks = {Point3([20 15 1]'),...
        Point3([22 7 -1]'),...
        Point3([20 20 6]'),...
        Point3([24 19 -4]'),...
        Point3([26 17 -2]'),...
        Point3([12 15 4]'),...
        Point3([25 11 -6]'),...
        Point3([23 10 4]')};

fg = NonlinearFactorGraph;
pose_cov = noiseModel.Diagonal.Sigmas([1*pi/180; 1*pi/180; 1*pi/180; 0.1; 0.1; 0.1]);
fg.add(PriorFactorPose3(1, Pose3(),pose_cov));
fg.add(PriorFactorPose3(2, Pose3(Rot3(),Point3(1,0,0)),pose_cov));

curvature = 0.5;

initial = Values;

%% intial landmarks and camera trajectory shifted in + y-direction
y_shift = Point3(0,1,0);

% insert shifted points
for i=1:nrPoints
   initial.insert(100+i,landmarks{i}.compose(y_shift)); 
end

figure(1);
cla
hold on;


%% Actual camera translation coincides with odometry, but -90deg Z-X rotation
camera_transform = Pose3(Rot3.RzRyRx(-pi/2, 0, -pi/2),y_shift);
actual_transform = Pose3(Rot3.RzRyRx(-pi/2, 0, -pi/2),Point3());
initial.insert(1000,camera_transform);

trans_cov = noiseModel.Diagonal.Sigmas([5*pi/180; 5*pi/180; 5*pi/180; 20; 20; 20]);
fg.add(PriorFactorPose3(1000,camera_transform,trans_cov));


%% insert poses
initial.insert(1, Pose3());


move_forward = Pose3(Rot3(),Point3(1,0,0));
move_circle = Pose3(Rot3.RzRyRx(0.0,0.0,curvature*pi/180),Point3(1,0,0));
covariance = noiseModel.Diagonal.Sigmas([5*pi/180; 5*pi/180; 5*pi/180; 0.05; 0.05; 0.05]);
z_cov = noiseModel.Diagonal.Sigmas([1.0;1.0]);
    
K = Cal3_S2(900,900,0,640,480);
cheirality_exception_count = 0;

isamParams = gtsam.ISAM2Params;
isamParams.setFactorization('QR');
isam = ISAM2(isamParams);

result = initial

for i=1:20
    
    if i > 1
        if i < 11
            initial.insert(i,result.atPose3(i-1).compose(move_forward));
            fg.add(BetweenFactorPose3(i-1,i, move_forward, covariance));
        else
            initial.insert(i,result.atPose3(i-1).compose(move_circle));
            fg.add(BetweenFactorPose3(i-1,i, move_circle, covariance));
        end
        
    end
    
    % generate some camera measurements
    cam_pose = initial.atPose3(i).compose(actual_transform);
%     gtsam.plotPose3(cam_pose);
    cam = PinholeCameraCal3_S2(cam_pose,K);
    i
%     result
    for j=1:nrPoints
        % All landmarks seen in every frame
        try
            z = cam.project(landmarks{j});
            fg.add(TransformProjectionFactorCal3_S2(z, z_cov, i, 1000, 100+j, K));
        catch
            cheirality_exception_count = cheirality_exception_count + 1;
        end % end try/catch
    end  
    
    if i > 2
        disp('ISAM Update');
        isam.update(fg, initial);
        result = isam.calculateEstimate();
        
        %% reset 
        initial = Values;
        fg = NonlinearFactorGraph;
    end
    
    hold off;

    clf;
    hold on;
    
    %% plot results
    result_camera_transform = result.atPose3(1000);
    for j=1:i
      gtsam.plotPose3(result.atPose3(j));
      gtsam.plotPose3(result.atPose3(j).compose(result_camera_transform),[],0.5);
    end
    
    xlabel('x (m)');
    ylabel('y (m)');

    title(sprintf('Curvature %g deg, iteration %g', curvature, i));
    
    axis([0 20 0 20 -10 10]);
      view(-37,40);
%     axis equal
    
    for l=101:100+nrPoints
        plotPoint3(result.atPoint3(l),'g');
    end
    
    ty = result.atPose3(1000).translation().y();
    text(5,5,5,sprintf('Y-Transform: %0.2g',ty));
  
    if(write_video)
        currFrame = getframe(gcf);
        writeVideo(videoObj, currFrame)
    else
        pause(0.001);
    end
    
    
end

if(write_video)
    close(videoObj);
end

fprintf('Cheirality Exception count: %d\n', cheirality_exception_count);



disp('Transform after optimization');
result.atPose3(1000)



