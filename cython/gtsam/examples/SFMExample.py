"""
GTSAM Copyright 2010, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

A structure-from-motion problem on a simulated dataset
"""
from __future__ import print_function

import gtsam
import numpy as np
from gtsam.examples import SFMdata
from gtsam.gtsam import (Cal3_S2, DoglegOptimizer,
                         GenericProjectionFactorCal3_S2, NonlinearFactorGraph,
                         Point3, Pose3, PriorFactorPoint3, PriorFactorPose3,
                         Rot3, SimpleCamera, Values)


def symbol(name: str, index: int) -> int:
    """ helper for creating a symbol without explicitly casting 'name' from str to int """
    return gtsam.symbol(ord(name), index)


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
    trust-region method known as Powell's Degleg

    The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
    nonlinear functions around an initial linearization point, then solve the linear system
    to update the linearization point. This happens repeatedly until the solver converges
    to a consistent set of variable values. This requires us to specify an initial guess
    for each variable, held in a Values container.
    """

    # Define the camera calibration parameters
    K = Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0)

    # Define the camera observation noise model
    measurement_noise = gtsam.noiseModel_Isotropic.Sigma(2, 1.0)  # one pixel in u and v

    # Create the set of ground-truth landmarks
    points = SFMdata.createPoints()

    # Create the set of ground-truth poses
    poses = SFMdata.createPoses(K)

    # Create a factor graph
    graph = NonlinearFactorGraph()

    # Add a prior on pose x1. This indirectly specifies where the origin is.
    # 0.3 rad std on roll,pitch,yaw and 0.1m on x,y,z
    pose_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.3, 0.3, 0.3, 0.1, 0.1, 0.1]))
    factor = PriorFactorPose3(symbol('x', 0), poses[0], pose_noise)
    graph.push_back(factor)

    # Simulated measurements from each camera pose, adding them to the factor graph
    for i, pose in enumerate(poses):
        camera = SimpleCamera(pose, K)
        for j, point in enumerate(points):
            measurement = camera.project(point)
            factor = GenericProjectionFactorCal3_S2(
                measurement, measurement_noise, symbol('x', i), symbol('l', j), K)
            graph.push_back(factor)

    # Because the structure-from-motion problem has a scale ambiguity, the problem is still under-constrained
    # Here we add a prior on the position of the first landmark. This fixes the scale by indicating the distance
    # between the first camera and the first landmark. All other landmark positions are interpreted using this scale.
    point_noise = gtsam.noiseModel_Isotropic.Sigma(3, 0.1)
    factor = PriorFactorPoint3(symbol('l', 0), points[0], point_noise)
    graph.push_back(factor)
    graph.print_('Factor Graph:\n')

    # Create the data structure to hold the initial estimate to the solution
    # Intentionally initialize the variables off from the ground truth
    initial_estimate = Values()
    for i, pose in enumerate(poses):
        r = Rot3.Rodrigues(-0.1, 0.2, 0.25)
        t = Point3(0.05, -0.10, 0.20)
        transformed_pose = pose.compose(Pose3(r, t))
        initial_estimate.insert(symbol('x', i), transformed_pose)
    for j, point in enumerate(points):
        transformed_point = Point3(point.vector() + np.array([-0.25, 0.20, 0.15]))
        initial_estimate.insert(symbol('l', j), transformed_point)
    initial_estimate.print_('Initial Estimates:\n')

    # Optimize the graph and print results
    params = gtsam.DoglegParams()
    params.setVerbosity('TERMINATION')
    optimizer = DoglegOptimizer(graph, initial_estimate, params)
    print('Optimizing:')
    result = optimizer.optimize()
    result.print_('Final results:\n')
    print('initial error = {}'.format(graph.error(initial_estimate)))
    print('final error = {}'.format(graph.error(result)))


if __name__ == '__main__':
    main()
