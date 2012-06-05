function plotPose2(p,color,P)
% plotPose2: show a Pose2, possibly with covariance matrix
plot(p.x,p.y,[color '*']);
c = cos(p.theta);
s = sin(p.theta);
quiver(p.x,p.y,c,s,0.1,color);
if nargin>2
    pPp = P(1:2,1:2); % covariance matrix in pose coordinate frame    
    gRp = [c -s;s c]; % rotation from pose to global
    covarianceEllipse([p.x;p.y],gRp*pPp*gRp',color);
end