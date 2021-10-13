"""
GTSAM Copyright 2010-2018, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Pose SLAM example using iSAM2 in the 2D plane.
Author: Jerred Chen, Yusuf Ali
Modelled after:
    - VisualISAM2Example by: Duy-Nguyen Ta (C++), Frank Dellaert (Python)
    - Pose2SLAMExample by: Alex Cunningham (C++), Kevin Deng & Frank Dellaert (Python)
"""

from __future__ import print_function
import math
import numpy as np
from numpy.random import multivariate_normal as mult_gauss
import gtsam
import matplotlib.pyplot as plt
import gtsam.utils.plot as gtsam_plot

def Pose2SLAM_ISAM2_plot(graph, current_estimate):
    """
    Plots incremental state of the robot for 2D Pose SLAM using iSAM2
    Author: Jerred Chen
    Based on version by:
        - Ellon Paiva (Python),
        - Duy Nguyen Ta and Frank Dellaert (MATLAB)
    """
    marginals = gtsam.Marginals(graph, current_estimate)

    fig = plt.figure(0)
    axes = fig.gca()
    plt.cla()

    i = 1
    while current_estimate.exists(i):
        gtsam_plot.plot_pose2(0, current_estimate.atPose2(i), 0.5, marginals.marginalCovariance(i))
        i += 1

    plt.axis('equal')
    axes.set_xlim(-1, 5)
    axes.set_ylim(-1, 3)
    plt.pause(1)
    return marginals


def Pose2SLAM_ISAM2_example():
    """
    """
    plt.ion()

    def vector3(x, y, z):
        """Create 3d double numpy array."""
        return np.array([x, y, z], dtype=float)

    # Although this example only uses linear measurements and Gaussian noise models, it is important
    # to note that iSAM2 can be utilized to its full potential during nonlinear optimization. This example
    # simply showcases how iSAM2 may be applied to a Pose2 SLAM problem.
    PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(vector3(0.3, 0.3, 0.1))
    ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(vector3(0.2, 0.2, 0.1))

    # Create a Nonlinear factor graph as well as the data structure to hold state estimates.
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()

    # iSAM2 parameters which can adjust the threshold necessary to force relinearization and how many
    # update calls are required to perform the relinearization.
    parameters = gtsam.ISAM2Params()
    parameters.setRelinearizeThreshold(0.1)
    parameters.setRelinearizeSkip(1)
    isam = gtsam.ISAM2(parameters)

    # The ground truth odometry measurements (without noise) of the robot during the trajectory.
    odom_arr = [(2, 0, 0),
                (2, 0, math.pi/2),
                (2, 0, math.pi/2),
                (2, 0, math.pi/2),
                (2, 0, math.pi/2)]

    # The initial estimates for robot poses 2-5. Pose 1 is initialized by the prior.
    # To demonstrate iSAM2 incremental optimization, poor initializations were intentionally inserted.
    pose_est = [gtsam.Pose2(2.3, 0.1, -0.2),
                gtsam.Pose2(4.1, 0.1, math.pi/2),
                gtsam.Pose2(4.0, 2.0, math.pi),
                gtsam.Pose2(2.1, 2.1, -math.pi/2),
                gtsam.Pose2(1.9, -0.2, 0.2)]

    graph.push_back(gtsam.PriorFactorPose2(1, gtsam.Pose2(0, 0, 0), PRIOR_NOISE))
    initial_estimate.insert(1, gtsam.Pose2(0.5, 0.0, 0.2))

    def determine_loop_closure(odom, current_estimate, xy_tol=0.6, theta_tol=0.3):
        """
        Simple brute force approach which iterates through previous states
        and checks for loop closure.
        Author: Jerred
        ### Parameters:
        odom: (numpy.ndarray) 1x3 vector representing noisy odometry (x, y, theta)
        measurement in the body frame.
        current_estimate: (gtsam.Values) The current estimates computed by iSAM2.
        xy_tol: (double) Optional argument for the x-y measurement tolerance.
        theta_tol: (double) Optional argument for the theta measurement tolerance.
        ### Returns:
        k: (int) The key of the state which is helping add the loop closure constraint.
        If loop closure is not found, then None is returned.
        """
        if current_estimate:
            prev_est = current_estimate.atPose2(i+1)
            rotated_odom = prev_est.rotation().matrix() @ odom[:2]
            curr_xy = np.array([prev_est.x() + rotated_odom[0],
                                prev_est.y() + rotated_odom[1]])
            curr_theta = prev_est.theta() + odom[2]
            for k in range(1, i+1):
                pose_xy = np.array([current_estimate.atPose2(k).x(),
                                    current_estimate.atPose2(k).y()])
                pose_theta = current_estimate.atPose2(k).theta()
                if (abs(pose_xy - curr_xy) <= xy_tol).all() and \
                    (abs(pose_theta - curr_theta) <= theta_tol):
                        return k

    current_estimate = None
    for i in range(len(odom_arr)):
        # The "ground truth" change between poses
        odom_x, odom_y, odom_theta = odom_arr[i]
        # Odometry measurement that is received by the robot and corrupted by gaussian noise
        odom = mult_gauss(odom_arr[i], ODOMETRY_NOISE.covariance())
        # Determine if there is loop closure based on the odometry measurement and the previous estimate of the state
        loop = determine_loop_closure(odom, current_estimate)
        if loop:
            graph.push_back(gtsam.BetweenFactorPose2(i + 1, loop, gtsam.Pose2(odom_x, odom_y, odom_theta), ODOMETRY_NOISE))
        else:
            graph.push_back(gtsam.BetweenFactorPose2(i + 1, i + 2, gtsam.Pose2(odom_x, odom_y, odom_theta), ODOMETRY_NOISE))
            initial_estimate.insert(i + 2, pose_est[i])
        # Incremental update to iSAM2's internal Baye's tree, which only optimizes upon the added variables
        # as well as any affected variables
        isam.update(graph, initial_estimate)
        # Another iSAM2 update can be performed for additional optimization
        isam.update()
        current_estimate = isam.calculateEstimate()
        print("*"*50)
        print(f"Inference after State {i+1}:")
        print(current_estimate)
        marginals = Pose2SLAM_ISAM2_plot(graph, current_estimate)
        initial_estimate.clear()
    # Print the final covariance matrix for each pose after completing inference
    for i in range(1, len(odom_arr)+1):
        print(f"X{i} covariance:\n{marginals.marginalCovariance(i)}\n")
    
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    Pose2SLAM_ISAM2_example()
