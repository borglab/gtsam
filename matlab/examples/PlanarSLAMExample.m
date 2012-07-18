%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Simple robotics example using the pre-built planar SLAM domain
% @author Alex Cunningham
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Assumptions
%  - All values are axis aligned
%  - Robot poses are facing along the X axis (horizontal, to the right in images)
%  - We have bearing and range information for measurements
%  - We have full odometry for measurements
%  - The robot and landmarks are on a grid, moving 2 meters each step
%  - Landmarks are 2 meters away from the robot trajectory

%% Create keys for variables
import gtsam.*
i1 = symbol('x',1); i2 = symbol('x',2); i3 = symbol('x',3);
j1 = symbol('l',1); j2 = symbol('l',2);

%% Create graph container and add factors to it
graph = planarSLAM.Graph;

%% Add prior
import gtsam.*
priorMean = Pose2(0.0, 0.0, 0.0); % prior at origin
priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
graph.addPosePrior(i1, priorMean, priorNoise); % add directly to graph

%% Add odometry
import gtsam.*
odometry = Pose2(2.0, 0.0, 0.0);
odometryNoise = noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]);
graph.addRelativePose(i1, i2, odometry, odometryNoise);
graph.addRelativePose(i2, i3, odometry, odometryNoise);

%% Add bearing/range measurement factors
import gtsam.*
degrees = pi/180;
brNoise = noiseModel.Diagonal.Sigmas([0.1; 0.2]);
graph.addBearingRange(i1, j1, Rot2(45*degrees), sqrt(4+4), brNoise);
graph.addBearingRange(i2, j1, Rot2(90*degrees), 2, brNoise);
graph.addBearingRange(i3, j2, Rot2(90*degrees), 2, brNoise);

% print
graph.print(sprintf('\nFull graph:\n'));

%% Initialize to noisy points
import gtsam.*
initialEstimate = planarSLAM.Values;
initialEstimate.insertPose(i1, Pose2(0.5, 0.0, 0.2));
initialEstimate.insertPose(i2, Pose2(2.3, 0.1,-0.2));
initialEstimate.insertPose(i3, Pose2(4.1, 0.1, 0.1));
initialEstimate.insertPoint(j1, Point2(1.8, 2.1));
initialEstimate.insertPoint(j2, Point2(4.1, 1.8));

initialEstimate.print(sprintf('\nInitial estimate:\n'));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
result = graph.optimize(initialEstimate,1);
result.print(sprintf('\nFinal result:\n'));

%% Plot Covariance Ellipses
import gtsam.*
cla;hold on
marginals = graph.marginals(result);
for i=1:3
    key = symbol('x',i);
    pose{i} = result.pose(key);
    P{i}=marginals.marginalCovariance(key);
    if i>1
        plot([pose{i-1}.x;pose{i}.x],[pose{i-1}.y;pose{i}.y],'r-');
    end
end
for i=1:3
    plotPose2(pose{i},'g',P{i})
end
point = {};
for j=1:2
    key = symbol('l',j);
    point{j} = result.point(key);
    Q{j}=marginals.marginalCovariance(key);
    plotPoint2(point{j},'b',Q{j})
end
plot([pose{1}.x;point{1}.x],[pose{1}.y;point{1}.y],'c-');
plot([pose{2}.x;point{1}.x],[pose{2}.y;point{1}.y],'c-');
plot([pose{3}.x;point{2}.x],[pose{3}.y;point{2}.y],'c-');
axis([-0.6 4.8 -1 1])
axis equal
view(2)

