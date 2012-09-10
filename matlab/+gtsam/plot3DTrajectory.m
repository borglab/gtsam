function plot3DTrajectory(values,linespec,frames,scale,marginals)
% plot3DTrajectory plots a 3D trajectory
% plot3DTrajectory(values,linespec,frames,scale,marginals)

if ~exist('scale','var') || isempty(scale), scale=1; end
if ~exist('frames','var'), scale=[]; end

import gtsam.*

haveMarginals = exist('marginals', 'var');
keys = KeyVector(values.keys);

holdstate = ishold;
hold on

% Plot poses and covariance matrices
lastIndex = [];
for i = 0:keys.size-1
    key = keys.at(i);
    x = values.at(key);
    if isa(x, 'gtsam.Pose3')
        if ~isempty(lastIndex)
            % Draw line from last pose then covariance ellipse on top of
            % last pose.
            lastKey = keys.at(lastIndex);
            lastPose = values.at(lastKey);
            plot3([ x.x; lastPose.x ], [ x.y; lastPose.y ], [ x.z; lastPose.z ], linespec);
            if haveMarginals
                P = marginals.marginalCovariance(lastKey);
            else
                P = [];
            end
            gtsam.plotPose3(lastPose, P, scale);
        end
        lastIndex = i;
    end
end

% Draw final pose
if ~isempty(lastIndex)
    lastKey = keys.at(lastIndex);
    lastPose = values.at(lastKey);
    if haveMarginals
        P = marginals.marginalCovariance(lastKey);
    else
        P = [];
    end
    gtsam.plotPose3(lastPose, P, scale);
end

if ~holdstate
    hold off
end

end