NonlinearFactorGraph graph;
noiseModel::Diagonal::shared_ptr priorNoise =
  noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
graph.addPrior(1, Pose2(0, 0, 0), priorNoise);

// Add odometry factors
noiseModel::Diagonal::shared_ptr model =
  noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
graph.add(BetweenFactor<Pose2>(1, 2, Pose2(2, 0, 0     ), model));
graph.add(BetweenFactor<Pose2>(2, 3, Pose2(2, 0, M_PI_2), model));
graph.add(BetweenFactor<Pose2>(3, 4, Pose2(2, 0, M_PI_2), model));
graph.add(BetweenFactor<Pose2>(4, 5, Pose2(2, 0, M_PI_2), model));

// Add the loop closure constraint
graph.add(BetweenFactor<Pose2>(5, 2, Pose2(2, 0, M_PI_2), model));

