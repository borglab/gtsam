function plot3DTrajectory(values,linespec,frames,scale,marginals)
% plot3DTrajectory plots a 3D trajectory
% plot3DTrajectory(values,linespec,frames,scale,marginals)

if ~exist('scale','var') || isempty(scale), scale=1; end
if ~exist('frames','var'), scale=[]; end

import gtsam.*

Pose3Values = gtsam.utilities.allPose3s(values);

haveMarginals = exist('marginals', 'var');
keys = KeyVector(Pose3Values.keys);

holdstate = ishold;
hold on

% Plot poses and covariance matrices
lastIndex = [];
for i = 0:keys.size-1
  key = keys.at(i);
  try
    x = Pose3Values.atPose3(key);
    if ~isempty(lastIndex)
      % Draw line from last pose then covariance ellipse on top of
      % last pose.
      lastKey = keys.at(lastIndex);
      try
        lastPose = Pose3Values.atPose3(lastKey);
        plot3([ x.x; lastPose.x ], [ x.y; lastPose.y ], [ x.z; lastPose.z ], linespec);
        if haveMarginals
          P = marginals.marginalCovariance(lastKey);
        else
          P = [];
        end
        gtsam.plotPose3(lastPose, P, scale);
      catch err
        %                 warning(['no Pose3 at ' lastKey]);
      end
    end
    lastIndex = i;
  catch
    warning(['no Pose3 at ' key]);
  end
end

% Draw final pose
if ~isempty(lastIndex)
  lastKey = keys.at(lastIndex);
  try
    lastPose = Pose3Values.atPose3(lastKey);
    if haveMarginals
      P = marginals.marginalCovariance(lastKey);
    else
      P = [];
    end
    gtsam.plotPose3(lastPose, P, scale);
  catch
    % warning(['no Pose3 at ' lastIndex]);
  end
end

if ~holdstate
  hold off
end
