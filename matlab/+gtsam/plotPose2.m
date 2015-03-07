function plotPose2(pose,color,P,axisLength)
% plotPose2 shows a Pose2, possibly with covariance matrix
if nargin<4,axisLength=0.1;end

plot(pose.x,pose.y,[color '*']);
c = cos(pose.theta);
s = sin(pose.theta);
quiver(pose.x,pose.y,c,s,axisLength,color);
if nargin>2
    pPp = P(1:2,1:2); % covariance matrix in pose coordinate frame    
    gRp = [c -s;s c]; % rotation from pose to global
    gtsam.covarianceEllipse([pose.x;pose.y],gRp*pPp*gRp',color);
end