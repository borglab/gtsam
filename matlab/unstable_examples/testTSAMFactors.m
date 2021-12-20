% TSAMFactorsExample
% Frank Dellaert, May 2014

import gtsam.*;

% noise models
noisePrior = noiseModel.Diagonal.Sigmas([100; 100; 100]);
noiseDelta = noiseModel.Isotropic.Sigma(2, 0.1);
noiseOdom = noiseModel.Diagonal.Sigmas([0.1; 0.1; 0.05]);

% Example is 2 landmarks 1 and 2, 2 poses 10 and 20, 2 base nodes 100 and 200
%      l1       l2
%       +       +
% - b - p - p - b
%   +---+---+---+

% True values
b1 = Pose2(0,0,0);
b2 = Pose2(2,0,0);
l1 = Point2(0,1);
l2 = Point2(2,1);

% Create a graph
graph = NonlinearFactorGraph;
origin = Pose2;
graph.add(gtsam.PriorFactorPose2(10, origin, noisePrior))
graph.add(gtsam.PriorFactorPose2(20, origin, noisePrior))
graph.add(gtsam.PriorFactorPose2(100, origin, noisePrior))
graph.add(DeltaFactor(10, 1, b1.transformTo(l1), noiseDelta))
graph.add(DeltaFactor(20, 1, b2.transformTo(l2), noiseDelta))
graph.add(DeltaFactorBase(100,10, 200,2, b1.transformTo(l2), noiseDelta))
graph.add(DeltaFactorBase(200,20, 100,1, b2.transformTo(l1), noiseDelta))
graph.add(OdometryFactorBase(100,10,200,20, Pose2(2,0,0), noiseOdom))

% Initial values
initial = Values;
initial.insert(100,origin);
initial.insert(10,origin);
initial.insert(1,l1);
initial.insert(2,l2);
initial.insert(20,origin);
initial.insert(200,origin);

% optimize
params = LevenbergMarquardtParams;
% params.setVerbosity('ERROR');
optimizer = LevenbergMarquardtOptimizer(graph, initial, params);
result = optimizer.optimize();

% Check result
CHECK('error',result.atPose2(100).equals(b1,1e-5))
CHECK('error',result.atPose2(10).equals(origin,1e-5))
CHECK('error',result.atPoint2(1) - Point2(0,1) < 1e-5)
CHECK('error',result.atPoint2(2) - Point2(0,1) < 1e-5)
CHECK('error',result.atPose2(20).equals(origin,1e-5))
CHECK('error',result.atPose2(200).equals(b2,1e-5))

% Check error
CHECK('error',abs(graph.error(result))<1e-9)
for i=0:7
  CHECK('error',abs(graph.at(i).error(result))<1e-9)
end
