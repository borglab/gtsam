function plot3DPoints(values, linespec, marginals)
%PLOT3DPOINTS Plots the Point3's in a values, with optional covariances
%   Finds all the Point3 objects in the given Values object and plots them.
% If a Marginals object is given, this function will also plot marginal
% covariance ellipses for each point.

import gtsam.*

if ~exist('linespec', 'var') || isempty(linespec)
    linespec = 'g';
end
haveMarginals = exist('marginals', 'var');
keys = KeyVector(values.keys);

holdstate = ishold;
hold on

% Plot points and covariance matrices
for i = 0:keys.size-1
    key = keys.at(i);
    p = values.at(key);
    if isa(p, 'gtsam.Point3')
        if haveMarginals
            P = marginals.marginalCovariance(key);
            gtsam.plotPoint3(p, linespec, P);
        else
            gtsam.plotPoint3(p, linespec);
        end
    end
end

if ~holdstate
    hold off
end

end
