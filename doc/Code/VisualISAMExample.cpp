int relinearizeInterval = 3;
NonlinearISAM isam(relinearizeInterval);

// ... first frame initialization omitted ...

// Loop over the different poses, adding the observations to iSAM
for (size_t i = 1; i < poses.size(); ++i) {

  // Add factors for each landmark observation
  NonlinearFactorGraph graph;
  for (size_t j = 0; j < points.size(); ++j) {
    graph.add(
      GenericProjectionFactor<Pose3, Point3, Cal3_S2>
        (z[i][j], noise,Symbol('x', i), Symbol('l', j), K)
    );
  }

  // Add an initial guess for the current pose
  Values initialEstimate;
  initialEstimate.insert(Symbol('x', i), initial_x[i]);

  // Update iSAM with the new factors
  isam.update(graph, initialEstimate);
 }
