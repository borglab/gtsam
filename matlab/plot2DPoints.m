function plot2DPoints(values, marginals)
%PLOT2DPOINTS Plots the Point2's in a values, with optional covariances
%   Finds all the Point2 objects in the given Values object and plots them.
% If a Marginals object is given, this function will also plot marginal
% covariance ellipses for each point.

import gtsam.*

haveMarginals = exist('marginals', 'var');
keys = KeyVector(values.keys);

holdstate = ishold;
hold on

% Plot points and covariance matrices
for i = 0:keys.size-1
    key = keys.at(i);
    p = values.at(key);
    if isa(p, 'gtsam.Point2')
        if haveMarginals
            P = marginals.marginalCovariance(key);
            plotPoint2(p, 'g', P);
        else
            plotPoint2(p, 'g');
        end
    end
end

if ~holdstate
    hold off
end

end

