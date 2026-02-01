graph = NonlinearFactorGraph;
priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
graph.add(PriorFactorPose2(1, Pose2(0, 0, 0), priorNoise));

%% Add odometry factors
model = noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]);
graph.add(BetweenFactorPose2(1, 2, Pose2(2, 0, 0   ), model));
graph.add(BetweenFactorPose2(2, 3, Pose2(2, 0, pi/2), model));
graph.add(BetweenFactorPose2(3, 4, Pose2(2, 0, pi/2), model));
graph.add(BetweenFactorPose2(4, 5, Pose2(2, 0, pi/2), model));

%% Add pose constraint
graph.add(BetweenFactorPose2(5, 2, Pose2(2, 0, pi/2), model));

