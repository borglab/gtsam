function plot2DTrajectory(values, linespec, marginals)
%PLOT2DTRAJECTORY Plots the Pose2's in a values, with optional covariances
%   Finds all the Pose2 objects in the given Values object and plots them
% in increasing key order, connecting consecutive poses with a line.  If
% a Marginals object is given, this function will also plot marginal
% covariance ellipses for each pose.

import gtsam.*

if ~exist('linespec', 'var') || isempty(linespec)
    linespec = 'k*-';
end

haveMarginals = exist('marginals', 'var');

holdstate = ishold;
hold on

% Do something very efficient to draw trajectory
poses = utilities.extractPose2(values);
X = poses(:,1);
Y = poses(:,2);
theta = poses(:,3);
plot(X,Y,linespec);

% Quiver can also be vectorized if no marginals asked
if ~haveMarginals
    C = cos(theta);
    S = sin(theta);
    quiver(X,Y,C,S,0.1,linespec);
else
    % plotPose2 does both quiver and covariance matrix
    keys = KeyVector(values.keys);
    for i = 0:keys.size-1
        key = keys.at(i);
        try
            x = values.atPose2(key);
            P = marginals.marginalCovariance(key);
            gtsam.plotPose2(x,linespec(1), P);
        catch err
            % I guess it's not a Pose2
        end
    end
end

if ~holdstate
    hold off
end

end

