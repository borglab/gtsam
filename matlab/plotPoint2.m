function plotPoint2(p,color,P)
% plotPose2: show a Pose2, possibly with covariance matrix
if size(color,2)==1
    plot(p.x,p.y,[color '*']);
else
    plot(p.x,p.y,color);
end
if nargin>2
    pPp = P(1:2,1:2); % covariance matrix in pose coordinate frame
    covarianceEllipse([p.x;p.y],pPp,color(1));
end