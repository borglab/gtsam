function plotPoint2(p,color,P)
% plotPoint2 shows a Point2, possibly with covariance matrix
if size(color,2)==1
    plot(p(1),p(2),[color '*']);
else
    plot(p(1),p(2),color);
end
if exist('P', 'var') && (~isempty(P))
    gtsam.covarianceEllipse([p(1);p(2)],P,color(1));
end