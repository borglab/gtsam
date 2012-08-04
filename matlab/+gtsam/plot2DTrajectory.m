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
keys = KeyVector(values.keys);

holdstate = ishold;
hold on

% Plot poses and covariance matrices
lastIndex = [];
for i = 0:keys.size-1
    key = keys.at(i);
    x = values.at(key);
    if isa(x, 'gtsam.Pose2')
        if ~isempty(lastIndex)
            % Draw line from last pose then covariance ellipse on top of
            % last pose.
            lastKey = keys.at(lastIndex);
            lastPose = values.at(lastKey);
            plot([ x.x; lastPose.x ], [ x.y; lastPose.y ], linespec);
            if haveMarginals
                P = marginals.marginalCovariance(lastKey);
                gtsam.plotPose2(lastPose, 'g', P);
            end
        end
        lastIndex = i;
    end
end

% Draw final covariance ellipse
if ~isempty(lastIndex) && haveMarginals
    lastKey = keys.at(lastIndex);
    lastPose = values.at(lastKey);
    P = marginals.marginalCovariance(lastKey);
    gtsam.plotPose2(lastPose, 'g', P);
end

if ~holdstate
    hold off
end

end

