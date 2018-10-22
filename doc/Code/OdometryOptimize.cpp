// create (deliberatly inaccurate) initial estimate
Values initial;
initial.insert(1, Pose2(0.5, 0.0, 0.2));
initial.insert(2, Pose2(2.3, 0.1, -0.2));
initial.insert(3, Pose2(4.1, 0.1, 0.1));

// optimize using Levenberg-Marquardt optimization
Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();

