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
x1 = 1; x2 = 2; x3 = 3;
l1 = 1; l2 = 2;

%% Create graph container and add factors to it
graph = PlanarSLAMGraph;

%% Add prior
% gaussian for prior
prior_model = SharedDiagonal([0.3; 0.3; 0.1]);
prior_measurement = Pose2(0.0, 0.0, 0.0); % prior at origin
graph.addPrior(x1, prior_measurement, prior_model); % add directly to graph

%% Add odometry
% general noisemodel for odometry
odom_model = SharedDiagonal([0.2; 0.2; 0.1]);
odom_measurement = Pose2(2.0, 0.0, 0.0); % create a measurement for both factors (the same in this case)
graph.addOdometry(x1, x2, odom_measurement, odom_model);
graph.addOdometry(x2, x3, odom_measurement, odom_model);

%% Add measurements
% general noisemodel for measurements
meas_model = SharedDiagonal([0.1; 0.2]);

% create the measurement values - indices are (pose id, landmark id)
degrees = pi/180;
bearing11 = Rot2(45*degrees);
bearing21 = Rot2(90*degrees);
bearing32 = Rot2(90*degrees);
range11 = sqrt(4+4);
range21 = 2.0;
range32 = 2.0;

% create bearing/range factors and add them
graph.addBearingRange(x1, l1, bearing11, range11, meas_model);
graph.addBearingRange(x2, l1, bearing21, range21, meas_model);
graph.addBearingRange(x3, l2, bearing32, range32, meas_model);

% print
graph.print('full graph');

%% Initialize to noisy points
initialEstimate = PlanarSLAMValues;
initialEstimate.insertPose(x1, Pose2(0.5, 0.0, 0.2));
initialEstimate.insertPose(x2, Pose2(2.3, 0.1,-0.2));
initialEstimate.insertPose(x3, Pose2(4.1, 0.1, 0.1));
initialEstimate.insertPoint(l1, Landmark2(1.8, 2.1));
initialEstimate.insertPoint(l2, Landmark2(4.1, 1.8));

initialEstimate.print('initial estimate');

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
result = graph.optimize_(initialEstimate);
result.print('final result');

%% Print out the corresponding dense matrix
ord = graph.orderingCOLAMD(result);
gfg = graph.linearize(result,ord);
denseAb = gfg.denseJacobian

%% Get sparse matrix
IJS = gfg.sparse([12 7 4 10 1 14]');
Ab=sparse(IJS(1,:),IJS(2,:),IJS(3,:));
spy(Ab);
