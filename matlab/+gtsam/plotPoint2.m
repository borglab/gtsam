function plotPoint2(p,color,P)
% plotPoint2 shows a Point2, possibly with covariance matrix
if size(color,2)==1
    plot(p.x,p.y,[color '*']);
else
    plot(p.x,p.y,color);
end
if exist('P', 'var')
    gtsam.covarianceEllipse([p.x;p.y],P,color(1));
end