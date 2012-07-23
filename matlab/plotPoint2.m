function plotPoint2(p,color,P)
% plotPoint2: show a Point2, possibly with covariance matrix
if size(color,2)==1
    plot(p.x,p.y,[color '*']);
else
    plot(p.x,p.y,color);
end
if exist('P', 'var')
    covarianceEllipse([p.x;p.y],P,color(1));
end