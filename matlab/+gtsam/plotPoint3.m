function plotPoint3(p, color, P)
%PLOTPOINT3 Plot a Point3 with an optional covariance matrix
if size(color,2)==1
    plot3(p(1),p(2),p(3),[color '*']);
else
    plot3(p(1),p(2),p(3),color);
end
if exist('P', 'var')
    gtsam.covarianceEllipse3D([p(1);p(2);p(3)],P);
end

end

