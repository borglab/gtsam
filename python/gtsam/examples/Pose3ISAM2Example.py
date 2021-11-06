"""
GTSAM Copyright 2010-2018, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Pose SLAM example using iSAM2 in 3D space.
Author: Jerred Chen
Modeled after:
    - VisualISAM2Example by: Duy-Nguyen Ta (C++), Frank Dellaert (Python)
    - Pose2SLAMExample by: Alex Cunningham (C++), Kevin Deng & Frank Dellaert (Python)
"""

from typing import List

import matplotlib.pyplot as plt
import numpy as np

import gtsam
import gtsam.utils.plot as gtsam_plot

def report_on_progress(graph: gtsam.NonlinearFactorGraph, current_estimate: gtsam.Values,
                        key: int):
    """Print and plot incremental progress of the robot for 2D Pose SLAM using iSAM2."""

    # Print the current estimates computed using iSAM2.
    print("*"*50 + f"\nInference after State {key+1}:\n")
    print(current_estimate)

    # Compute the marginals for all states in the graph.
    marginals = gtsam.Marginals(graph, current_estimate)

    # Plot the newly updated iSAM2 inference.
    fig = plt.figure(0)
    axes = fig.gca(projection='3d')
    plt.cla()

    i = 1
    while current_estimate.exists(i):
        gtsam_plot.plot_pose3(0, current_estimate.atPose3(i), 10,
                                marginals.marginalCovariance(i))
        i += 1

    axes.set_xlim3d(-30, 45)
    axes.set_ylim3d(-30, 45)
    axes.set_zlim3d(-30, 45)
    plt.pause(1)

def create_poses() -> List[gtsam.Pose3]:
    """Creates ground truth poses of the robot."""
    P0 = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    P1 = np.array([[0, -1, 0, 15],
                   [1, 0, 0, 15],
                   [0, 0, 1, 20],
                   [0, 0, 0, 1]])
    P2 = np.array([[np.cos(np.pi/4), 0, np.sin(np.pi/4), 30],
                   [0, 1, 0, 30],
                   [-np.sin(np.pi/4), 0, np.cos(np.pi/4), 30],
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

    return [gtsam.Pose3(P0), gtsam.Pose3(P1), gtsam.Pose3(P2),
            gtsam.Pose3(P3), gtsam.Pose3(P4), gtsam.Pose3(P5)]

def determine_loop_closure(odom_tf: gtsam.Pose3, current_estimate: gtsam.Values,
    key: int, xyz_tol=0.6, rot_tol=17) -> int:
    """Simple brute force approach which iterates through previous states
    and checks for loop closure.

    Args:
        odom_tf: The noisy odometry transformation measurement in the body frame.
        current_estimate: The current estimates computed by iSAM2.
        key: Key corresponding to the current state estimate of the robot.
        xyz_tol: Optional argument for the translational tolerance, in meters.
        rot_tol: Optional argument for the rotational tolerance, in degrees.
    Returns:
        k: The key of the state which is helping add the loop closure constraint.
            If loop closure is not found, then None is returned.
    """
    if current_estimate:
        prev_est = current_estimate.atPose3(key+1)
        curr_est = prev_est.compose(odom_tf)
        for k in range(1, key+1):
            pose = current_estimate.atPose3(k)
            if (abs(pose.matrix()[:3,:3] - curr_est.matrix()[:3,:3]) <= rot_tol*np.pi/180).all() and \
                (abs(pose.matrix()[:3,3] - curr_est.matrix()[:3,3]) <= xyz_tol).all():
                    return k

def Pose3_ISAM2_example():
    """Perform 3D SLAM given ground truth poses as well as simple
    loop closure detection."""
    plt.ion()

    # Declare the 3D translational standard deviations of the prior factor's Gaussian model, in meters.
    prior_xyz_sigma = 0.3

    # Declare the 3D rotational standard deviations of the prior factor's Gaussian model, in degrees.
    prior_rpy_sigma = 5

    # Declare the 3D translational standard deviations of the odometry factor's Gaussian model, in meters.
    odometry_xyz_sigma = 0.2

    # Declare the 3D rotational standard deviations of the odometry factor's Gaussian model, in degrees.
    odometry_rpy_sigma = 5

    # Although this example only uses linear measurements and Gaussian noise models, it is important
    # to note that iSAM2 can be utilized to its full potential during nonlinear optimization. This example
    # simply showcases how iSAM2 may be applied to a Pose2 SLAM problem.
    PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([prior_rpy_sigma*np.pi/180,
                                                                prior_rpy_sigma*np.pi/180,
                                                                prior_rpy_sigma*np.pi/180,
                                                                prior_xyz_sigma,
                                                                prior_xyz_sigma,
                                                                prior_xyz_sigma]))
    ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([odometry_rpy_sigma*np.pi/180,
                                                                odometry_rpy_sigma*np.pi/180,
                                                                odometry_rpy_sigma*np.pi/180,
                                                                odometry_xyz_sigma,
                                                                odometry_xyz_sigma,
                                                                odometry_xyz_sigma]))

    # Create a Nonlinear factor graph as well as the data structure to hold state estimates.
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()

    # Create iSAM2 parameters which can adjust the threshold necessary to force relinearization and how many
    # update calls are required to perform the relinearization.
    parameters = gtsam.ISAM2Params()
    parameters.setRelinearizeThreshold(0.1)
    parameters.setRelinearizeSkip(1)
    isam = gtsam.ISAM2(parameters)

    # Create the ground truth poses of the robot trajectory.
    true_poses = create_poses()

    # Create the ground truth odometry transformations, xyz translations, and roll-pitch-yaw rotations
    # between each robot pose in the trajectory.
    odometry_tf = [true_poses[i-1].transformPoseTo(true_poses[i]) for i in range(1, len(true_poses))]
    odometry_xyz = [(odometry_tf[i].x(), odometry_tf[i].y(), odometry_tf[i].z()) for i in range(len(odometry_tf))]
    odometry_rpy = [odometry_tf[i].rotation().rpy() for i in range(len(odometry_tf))]

    # Corrupt xyz translations and roll-pitch-yaw rotations with gaussian noise to create noisy odometry measurements.
    noisy_measurements = [np.random.multivariate_normal(np.hstack((odometry_rpy[i],odometry_xyz[i])), \
                                                        ODOMETRY_NOISE.covariance()) for i in range(len(odometry_tf))]

    # Add the prior factor to the factor graph, and poorly initialize the prior pose to demonstrate
    # iSAM2 incremental optimization.
    graph.push_back(gtsam.PriorFactorPose3(1, true_poses[0], PRIOR_NOISE))
    initial_estimate.insert(1, true_poses[0].compose(gtsam.Pose3(
        gtsam.Rot3.Rodrigues(-0.1, 0.2, 0.25), gtsam.Point3(0.05, -0.10, 0.20))))

    # Initialize the current estimate which is used during the incremental inference loop.
    current_estimate = initial_estimate
    for i in range(len(odometry_tf)):

        # Obtain the noisy translation and rotation that is received by the robot and corrupted by gaussian noise.
        noisy_odometry = noisy_measurements[i]

        # Compute the noisy odometry transformation according to the xyz translation and roll-pitch-yaw rotation.
        noisy_tf = gtsam.Pose3(gtsam.Rot3.RzRyRx(noisy_odometry[:3]), noisy_odometry[3:6].reshape(-1,1))

        # Determine if there is loop closure based on the odometry measurement and the previous estimate of the state.
        loop = determine_loop_closure(noisy_tf, current_estimate, i, xyz_tol=18, rot_tol=30)

        # Add a binary factor in between two existing states if loop closure is detected.
        # Otherwise, add a binary factor between a newly observed state and the previous state.
        if loop:
            graph.push_back(gtsam.BetweenFactorPose3(i + 1, loop, noisy_tf, ODOMETRY_NOISE))
        else:
            graph.push_back(gtsam.BetweenFactorPose3(i + 1, i + 2, noisy_tf, ODOMETRY_NOISE))

            # Compute and insert the initialization estimate for the current pose using a noisy odometry measurement.
            noisy_estimate = current_estimate.atPose3(i + 1).compose(noisy_tf)
            initial_estimate.insert(i + 2, noisy_estimate)

        # Perform incremental update to iSAM2's internal Bayes tree, optimizing only the affected variables.
        isam.update(graph, initial_estimate)
        current_estimate = isam.calculateEstimate()

        # Report all current state estimates from the iSAM2 optimization.
        report_on_progress(graph, current_estimate, i)
        initial_estimate.clear()

    # Print the final covariance matrix for each pose after completing inference.
    marginals = gtsam.Marginals(graph, current_estimate)
    i = 1
    while current_estimate.exists(i):
        print(f"X{i} covariance:\n{marginals.marginalCovariance(i)}\n")
        i += 1

    plt.ioff()
    plt.show()

if __name__ == '__main__':
    Pose3_ISAM2_example()
