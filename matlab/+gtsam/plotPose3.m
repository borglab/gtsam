function plotPose3(pose, P, axisLength)
% plotPose3 shows a Pose, possibly with covariance matrix
if nargin<3,axisLength=0.1;end

% get rotation and translation (center)
gRp = pose.rotation().matrix();  % rotation from pose to global
C = pose.translation();

if ~isempty(axisLength)
    % draw the camera axes
    xAxis = C+gRp(:,1)*axisLength;
    L = [C xAxis]';
    line(L(:,1),L(:,2),L(:,3),'Color','r');
    
    yAxis = C+gRp(:,2)*axisLength;
    L = [C yAxis]';
    line(L(:,1),L(:,2),L(:,3),'Color','g');
    
    zAxis = C+gRp(:,3)*axisLength;
    L = [C zAxis]';
    line(L(:,1),L(:,2),L(:,3),'Color','b');
end

% plot the covariance
if (nargin>2) && (~isempty(P))
    pPp = P(4:6,4:6); % covariance matrix in pose coordinate frame    
    gPp = gRp*pPp*gRp'; % convert the covariance matrix to global coordinate frame
    gtsam.covarianceEllipse3D(C,gPp);  
end
    
end
