function plot3DPoints(values, linespec, marginals)
%PLOT3DPOINTS Plots the Point3's in a values, with optional covariances
%   Finds all the Point3 objects in the given Values object and plots them.
% If a Marginals object is given, this function will also plot marginal
% covariance ellipses for each point.

import gtsam.*

if ~exist('linespec', 'var') || isempty(linespec)
    linespec = 'g*';
end
haveMarginals = exist('marginals', 'var');
keys = KeyVector(values.keys);

holdstate = ishold;
hold on

if haveMarginals
    % Plot points and covariance matrices (slow)
    for i = 0:keys.size-1
        key = keys.at(i);
        try
            p = values.atPoint3(key)
            P = marginals.marginalCovariance(key);
            gtsam.plotPoint3(p, linespec, P);
        catch
            % I guess it's not a Point3
        end
    end
else
    % Extract all in C++ and plot all at once (fast)
    P = utilities.extractPoint3(values);
    if size(linespec,2)==1
        plot3(P(:,1),P(:,2),P(:,3),[linespec '*']);
    else
        plot3(P(:,1),P(:,2),P(:,3),linespec);
    end
end

if ~holdstate
    hold off
end

end
