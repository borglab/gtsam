function plotPoint3(p, color, P)
%PLOTPOINT3 Plot a Point3 with an optional covariance matrix
if size(color,2)==1
    plot3(p.x,p.y,p.z,[color '*']);
else
    plot3(p.x,p.y,p.z,color);
end
if exist('P', 'var')
    gtsam.covarianceEllipse3D([p.x;p.y;p.z],P);
end

end

