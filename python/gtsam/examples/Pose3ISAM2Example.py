"""
GTSAM Copyright 2010-2018, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Pose SLAM example using iSAM2 in 3D space.
Author: Jerred Chen
Modelled after version by:
    - VisualISAM2Example by: Duy-Nguyen Ta (C++), Frank Dellaert (Python)
    - Pose2SLAMExample by: Alex Cunningham (C++), Kevin Deng & Frank Dellaert (Python)
"""
from __future__ import print_function
import gtsam
from gtsam import Pose3, Rot3
from gtsam.symbol_shorthand import X
import gtsam.utils.plot as gtsam_plot
import numpy as np
from numpy import cos, sin, pi
from numpy.random import multivariate_normal as mult_gauss
import matplotlib.pyplot as plt


def Pose3SLAM_ISAM2_plot(graph, current_estimate):
    """
    Plots incremental state of the robot for 3D Pose SLAM using iSAM2
    Author: Jerred Chen
    Based on version by:
        - Ellon Paiva (Python),
        - Duy Nguyen Ta and Frank Dellaert (MATLAB)
    """
    marginals = gtsam.Marginals(graph, current_estimate)

    fig = plt.figure(0)
    axes = fig.gca(projection='3d')
    plt.cla()

    i = 0
    while current_estimate.exists(X(i)):
        gtsam_plot.plot_pose3(0, current_estimate.atPose3(X(i)), 10,
                                marginals.marginalCovariance(X(i)))
        i += 1

    axes.set_xlim3d(-30, 45)
    axes.set_ylim3d(-30, 45)
    axes.set_zlim3d(-30, 45)
    plt.pause(1)

    return marginals


def createPoses():
    """
    Creates ground truth poses of the robot.
    """
    P0 = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    P1 = np.array([[0, -1, 0, 15],
                   [1, 0, 0, 15],
                   [0, 0, 1, 20],
                   [0, 0, 0, 1]])
    P2 = np.array([[cos(pi/4), 0, sin(pi/4), 30],
                   [0, 1, 0, 30],
                   [-sin(pi/4), 0, cos(pi/4), 30],
                   [0, 0, 0, 1]])
    P3 = np.array([[0, 1, 0, 30],
                   [0, 0, -1, 0],
                   [-1, 0, 0, -15],
                   [0, 0, 0, 1]])
    P4 = np.array([[-1, 0, 0, 0],
                   [0, -1, 0, -10],
                   [0, 0, 1, -10],
                   [0, 0, 0, 1]])
    P5 = P0[:]

    return [Pose3(P0), Pose3(P1), Pose3(P2), Pose3(P3), Pose3(P4), Pose3(P5)]

def Pose3_ISAM2_example():
    """
    Perform 3D SLAM given ground truth poses as well as simple
    loop closure detection.
    """
    plt.ion()

    def vector6(x, y, z, a, b, c):
        """Create 6d double numpy array."""
        return np.array([x, y, z, a, b, c], dtype=float)

    # Although this example only uses linear measurements and Gaussian noise models, it is important
    # to note that iSAM2 can be utilized to its full potential during nonlinear optimization. This example
    # simply showcases how iSAM2 may be applied to a Pose2 SLAM problem.
    PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(vector6(0.1, 0.1, 0.1, 0.3, 0.3, 0.3))
    ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(vector6(0.1, 0.1, 0.1, 0.2, 0.2, 0.2))

    # Create the ground truth poses of the robot trajectory.
    poses = createPoses()

    # iSAM2 parameters which can adjust the threshold necessary to force relinearization and how many
    # update calls are required to perform the relinearization.
    parameters = gtsam.ISAM2Params()
    parameters.setRelinearizeThreshold(0.1)
    parameters.setRelinearizeSkip(1)
    isam = gtsam.ISAM2(parameters)

    # Create a Nonlinear factor graph as well as the data structure to hold state estimates.
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()

    # Add prior factor to the first pose with intentionally poor initial estimate
    graph.push_back(gtsam.PriorFactorPose3(X(0), poses[0], PRIOR_NOISE))
    initial_estimate.insert(X(0), poses[0].compose(gtsam.Pose3(
        gtsam.Rot3.Rodrigues(-0.1, 0.2, 0.25), gtsam.Point3(0.05, -0.10, 0.20))))

    def determine_loop_closure(odom, current_estimate, xyz_tol=0.6, rot_tol=0.3):
        """
        Simple brute force approach which iterates through previous states
        and checks for loop closure.
        Author: Jerred Chen
        ### Parameters:
        odom: (numpy.ndarray) 1x6 vector representing noisy odometry transformation
        measurement in the body frame, [roll, pitch, yaw, x, y, z]
        current_estimate: (gtsam.Values) The current estimates computed by iSAM2.
        xyz_tol: (double) Optional argument for the translational tolerance.
        rot_tol: (double) Optional argument for the rotational tolerance.
        ### Returns:
        k: (int) The key of the state which is helping add the loop closure constraint.
        If loop closure is not found, then None is returned.
        """
        if current_estimate:
            rot = Rot3.RzRyRx(odom[:3])
            odom_tf = Pose3(rot, odom[3:6].reshape(-1,1))
            prev_est = current_estimate.atPose3(X(i-1))
            curr_est = prev_est.transformPoseFrom(odom_tf)
            for k in range(i):
                pose = current_estimate.atPose3(X(k))
                if (abs(pose.matrix()[:3,:3] - curr_est.matrix()[:3,:3]) <= rot_tol).all() and \
                    (abs(pose.matrix()[:3,3] - curr_est.matrix()[:3,3]) <= xyz_tol).all():
                        return k

    current_estimate = None
    for i in range(1, len(poses)):
        # The "ground truth" change between poses
        odom_tf = poses[i-1].transformPoseTo(poses[i])
        odom_xyz = odom_tf.x(), odom_tf.y(), odom_tf.z()
        odom_rpy = odom_tf.rotation().rpy()
        # Odometry measurement that is received by the robot and corrupted by gaussian noise
        measurement = mult_gauss(np.hstack((odom_rpy,odom_xyz)), ODOMETRY_NOISE.covariance())
        loop = determine_loop_closure(measurement, current_estimate)
        # If loop closure is detected, then adds a constraint between existing states in the factor graph
        if loop is not None:
            graph.push_back(gtsam.BetweenFactorPose3(X(i-1), X(loop), gtsam.Pose3(odom_tf), ODOMETRY_NOISE))
        else:
            graph.push_back(gtsam.BetweenFactorPose3(X(i-1), X(i), gtsam.Pose3(odom_tf), ODOMETRY_NOISE))
            # Intentionally insert poor initializations
            initial_estimate.insert(X(i), poses[i].compose(gtsam.Pose3(
                gtsam.Rot3.Rodrigues(-0.1, 0.2, 0.25), gtsam.Point3(0.05, -0.10, 0.20))))
        # Performs iSAM2 incremental update based on newly added factors
        isam.update(graph, initial_estimate)
        # Additional iSAM2 optimization
        isam.update()
        current_estimate = isam.calculateEstimate()
        print("*"*50)
        print(f"Inference after State {i}:")
        print(current_estimate)
        marginals = Pose3SLAM_ISAM2_plot(graph, current_estimate)
        initial_estimate.clear()
    # Print the final covariance matrix for each pose after completing inference
    i = 0
    while current_estimate.exists(X(i)):
        print(f"X{i} covariance:\n{marginals.marginalCovariance(X(i))}\n")
        i += 1

    plt.ioff()
    plt.show()

if __name__ == '__main__':
    Pose3_ISAM2_example()
