This directory contains a number of examples that illustrate the use of unstable components in GTSAM:

FixedLagSmootherExample: a 2D Pose SLAM example fusing measurements from multiple odometry-type sensors
ConcurrentFilteringAndSmoothingExample: a 2D Pose SLAM example demonstrating the Concurrent Filtering and Smoothing architecture
TGEqFScenarioExample, TFGInEKFScenarioExample, MEKFScenarioExample: biased inertial navigation with optional position, DVL and depth aiding on a simulated trajectory chosen with --scenario (Vertical, TypicalNavigation or WaypointSpline), using the TG-EqF, TFG-InEKF and MEKF filters respectively. They share the simulation and CSV logging in scenario_harness/.
