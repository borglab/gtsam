function plotFlyingResults(pts3d, covariance, values, marginals)
% plot the visible points on the cylinders and trajectories
% author: Zhaoyang Lv

import gtsam.*

haveMarginals = exist('marginals', 'var');
keys = KeyVector(values.keys);

holdstate = ishold;
hold on

keys = KeyVector(values.keys);

%% plot trajectories
for i = 0:keys.size - 1
    if exist('h_result', 'var')
        delete(h_result);
    end

    key = keys.at(i);
    pose = keys.at(key);
    
    P = marginals.marginalCovariance(key);
    
    h_result = gtsam.plotPose3(pose, P, 1);

end

%% plot point covariance



if exist('h_result', 'var')
    delete(h_result);
end

if ~holdstate
    hold off
end
    
end