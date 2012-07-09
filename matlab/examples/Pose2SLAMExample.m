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
% @author Chris Beall
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Assumptions
%  - All values are axis aligned
%  - Robot poses are facing along the X axis (horizontal, to the right in images)
%  - We have full odometry for measurements
%  - The robot is on a grid, moving 2 meters each step

%% Create graph container and add factors to it
graph = pose2SLAMGraph;

%% Add prior
% gaussian for prior
priorMean = gtsamPose2(0.0, 0.0, 0.0); % prior at origin
priorNoise = gtsamnoiseModelDiagonal.Sigmas([0.3; 0.3; 0.1]);
graph.addPosePrior(1, priorMean, priorNoise); % add directly to graph

%% Add odometry
% general noisemodel for odometry
odometryNoise = gtsamnoiseModelDiagonal.Sigmas([0.2; 0.2; 0.1]);
graph.addRelativePose(1, 2, gtsamPose2(2.0, 0.0, 0.0 ), odometryNoise);
graph.addRelativePose(2, 3, gtsamPose2(2.0, 0.0, pi/2), odometryNoise);
graph.addRelativePose(3, 4, gtsamPose2(2.0, 0.0, pi/2), odometryNoise);
graph.addRelativePose(4, 5, gtsamPose2(2.0, 0.0, pi/2), odometryNoise);

%% Add pose constraint
model = gtsamnoiseModelDiagonal.Sigmas([0.2; 0.2; 0.1]);
graph.addRelativePose(5, 2, gtsamPose2(2.0, 0.0, pi/2), model);

% print
graph.print(sprintf('\nFactor graph:\n'));

%% Initialize to noisy points
initialEstimate = pose2SLAMValues;
initialEstimate.insertPose(1, gtsamPose2(0.5, 0.0, 0.2 ));
initialEstimate.insertPose(2, gtsamPose2(2.3, 0.1,-0.2 ));
initialEstimate.insertPose(3, gtsamPose2(4.1, 0.1, pi/2));
initialEstimate.insertPose(4, gtsamPose2(4.0, 2.0, pi  ));
initialEstimate.insertPose(5, gtsamPose2(2.1, 2.1,-pi/2));
initialEstimate.print(sprintf('\nInitial estimate:\n'));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
result = graph.optimize(initialEstimate,1);
result.print(sprintf('\nFinal result:\n'));

%% Plot Covariance Ellipses
cla;
X=result.poses();
plot(X(:,1),X(:,2),'k*-'); hold on
plot([result.pose(5).x;result.pose(2).x],[result.pose(5).y;result.pose(2).y],'r-');
marginals = graph.marginals(result);

for i=1:result.size()
    pose_i = result.pose(i);
    P = marginals.marginalCovariance(i)
	plotPose2(pose_i,'g',P);
end
axis([-0.6 4.8 -1 1])
axis equal
view(2)
