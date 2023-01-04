// Create an empty nonlinear factor graph
NonlinearFactorGraph graph;

// Add a Gaussian prior on pose x_1
Pose2 priorMean(0.0, 0.0, 0.0);
auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));

// Add two odometry factors
Pose2 odometry(2.0, 0.0, 0.0);
auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
graph.add(BetweenFactor<Pose2>(1, 2, odometry, odometryNoise));
graph.add(BetweenFactor<Pose2>(2, 3, odometry, odometryNoise));
