"""
GTSAM Copyright 2010, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

A structure-from-motion problem on a simulated dataset
"""

import matplotlib.pyplot as plt
import numpy as np

import gtsam
from gtsam import symbol_shorthand

L = symbol_shorthand.L
X = symbol_shorthand.X

from gtsam.examples import SFMdata
from gtsam.utils import plot

from gtsam import (Cal3_S2, DoglegOptimizer, GenericProjectionFactorCal3_S2,
                   Marginals, NonlinearFactorGraph, PinholeCameraCal3_S2,
                   PriorFactorPoint3, PriorFactorPose3, Values)


def main():
    """
    Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).

    Each variable in the system (poses and landmarks) must be identified with a unique key.
    We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
    Here we will use Symbols

    In GTSAM, measurement functions are represented as 'factors'. Several common factors
    have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
    Here we will use Projection factors to model the camera's landmark observations.
    Also, we will initialize the robot at some location using a Prior factor.

    When the factors are created, we will add them to a Factor Graph. As the factors we are using
    are nonlinear factors, we will need a Nonlinear Factor Graph.

    Finally, once all of the factors have been added to our factor graph, we will want to
    solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
    GTSAM includes several nonlinear optimizers to perform this step. Here we will use a
    trust-region method known as Powell's Dogleg

    The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
    nonlinear functions around an initial linearization point, then solve the linear system
    to update the linearization point. This happens repeatedly until the solver converges
    to a consistent set of variable values. This requires us to specify an initial guess
    for each variable, held in a Values container.
    """

    # Define the camera calibration parameters
    K = Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0)

    # Define the camera observation noise model
    measurement_noise = gtsam.noiseModel.Isotropic.Sigma(2, 1.0)  # one pixel in u and v

    # Create the set of ground-truth landmarks
    points = SFMdata.createPoints()

    # Create the set of ground-truth poses
    poses = SFMdata.createPoses()

    # Create a factor graph
    graph = NonlinearFactorGraph()

    # Add a prior on pose x1. This indirectly specifies where the origin is.
    # 0.3 rad std on roll,pitch,yaw and 0.1m on x,y,z
    pose_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.3, 0.1, 0.1, 0.1]))
    factor = PriorFactorPose3(X(0), poses[0], pose_noise)
    graph.push_back(factor)

    # Simulated measurements from each camera pose, adding them to the factor graph
    for i, pose in enumerate(poses):
        camera = PinholeCameraCal3_S2(pose, K)
        for j, point in enumerate(points):
            measurement = camera.project(point)
            factor = GenericProjectionFactorCal3_S2(measurement, measurement_noise, X(i), L(j), K)
            graph.push_back(factor)

    # Because the structure-from-motion problem has a scale ambiguity, the problem is still under-constrained
    # Here we add a prior on the position of the first landmark. This fixes the scale by indicating the distance
    # between the first camera and the first landmark. All other landmark positions are interpreted using this scale.
    point_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
    factor = PriorFactorPoint3(L(0), points[0], point_noise)
    graph.push_back(factor)
    graph.print("Factor Graph:\n")

    # Create the data structure to hold the initial estimate to the solution
    # Intentionally initialize the variables off from the ground truth
    initial_estimate = Values()
    rng = np.random.default_rng()
    for i, pose in enumerate(poses):
        transformed_pose = pose.retract(0.1 * rng.standard_normal(6).reshape(6, 1))
        initial_estimate.insert(X(i), transformed_pose)
    for j, point in enumerate(points):
        transformed_point = point + 0.1 * rng.standard_normal(3)
        initial_estimate.insert(L(j), transformed_point)
    initial_estimate.print("Initial Estimates:\n")

    # Optimize the graph and print results
    params = gtsam.DoglegParams()
    params.setVerbosity("TERMINATION")
    optimizer = DoglegOptimizer(graph, initial_estimate, params)
    print("Optimizing:")
    result = optimizer.optimize()
    result.print("Final results:\n")
    print("initial error = {}".format(graph.error(initial_estimate)))
    print("final error = {}".format(graph.error(result)))

    marginals = Marginals(graph, result)
    plot.plot_3d_points(1, result, marginals=marginals)
    plot.plot_trajectory(1, result, marginals=marginals, scale=8)
    plot.set_axes_equal(1)
    plt.show()


if __name__ == "__main__":
    main()
