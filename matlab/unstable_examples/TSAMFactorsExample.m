% TSAMFactorsExample
% Frank Dellaert, May 2014

import gtsam.*;

% noise models
noisePrior = noiseModel.Diagonal.Sigmas([0; 0; 0]);
noiseDelta = noiseModel.Isotropic.Sigma(2, 0.1);
noiseOdom = noiseModel.Diagonal.Sigmas([0.1; 0.1; 0.05]);

% Example is 1, landmark, 2 poses 10 and 20, 2 base nodes 100 and 200
%       +       +
% - b - p - l - p - b
%   +---+-------+---+
% Create a graph
graph = NonlinearFactorGraph
origin = Pose2;
graph.add(gtsam.PriorFactorPose2(100, origin, noisePrior))
graph.add(gtsam.PriorFactorPose2(10, origin, noisePrior))
graph.add(gtsam.PriorFactorPose2(20, origin, noisePrior))
graph.add(DeltaFactor(10, 1, Point2(1,0), noiseDelta))
graph.add(DeltaFactor(20, 1, Point2(-1,0), noiseDelta))
graph.add(OdometryFactorBase(100,10,200,20, Pose2(2,0,0), noiseOdom))
graph

% Initial values
initial = Values;
initial.insert(100,origin);
initial.insert(10,origin);
initial.insert(1,Point2(2,0));
initial.insert(20,origin);
initial.insert(200,origin);

graph.error(initial)
graph.at(0).error(initial)
graph.at(1).error(initial)
graph.at(2).error(initial)
graph.at(3).error(initial)

% optimize
params = LevenbergMarquardtParams;
params.setVerbosity('ERROR');
optimizer = LevenbergMarquardtOptimizer(graph, initial, params);
result = optimizer.optimize()