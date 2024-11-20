"""
GTSAM Copyright 2010, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

A visualSLAM example for the structure-from-motion problem on a simulated dataset
This version uses iSAM to solve the problem incrementally
"""

import numpy as np
import gtsam
from gtsam.examples import SFMdata
from gtsam import (Cal3_S2, GenericProjectionFactorCal3_S2,
                   NonlinearFactorGraph, NonlinearISAM, Pose3,
                   PriorFactorPoint3, PriorFactorPose3, Rot3,
                   PinholeCameraCal3_S2, Values, Point3)
from gtsam.symbol_shorthand import X, L

def main():
    """
    A structure-from-motion example with landmarks
    - The landmarks form a 10 meter cube
    - The robot rotates around the landmarks, always facing towards the cube
    """

    # Define the camera calibration parameters
    K = Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0)

    # Define the camera observation noise model
    camera_noise = gtsam.noiseModel.Isotropic.Sigma(
        2, 1.0)  # one pixel in u and v

    # Create the set of ground-truth landmarks
    points = SFMdata.createPoints()
    # Create the set of ground-truth poses
    poses = SFMdata.createPoses()

    # Create a NonlinearISAM object which will relinearize and reorder the variables
    # every "reorderInterval" updates
    isam = NonlinearISAM(reorderInterval=3)

    # Create a Factor Graph and Values to hold the new data
    graph = NonlinearFactorGraph()
    initial_estimate = Values()

    # Loop over the different poses, adding the observations to iSAM incrementally
    for i, pose in enumerate(poses):
        camera = PinholeCameraCal3_S2(pose, K)
        # Add factors for each landmark observation
        for j, point in enumerate(points):
            measurement = camera.project(point)
            factor = GenericProjectionFactorCal3_S2(
                measurement, camera_noise, X(i), L(j), K)
            graph.push_back(factor)

        # Intentionally initialize the variables off from the ground truth
        noise = Pose3(r=Rot3.Rodrigues(-0.1, 0.2, 0.25),
                      t=Point3(0.05, -0.10, 0.20))
        initial_xi = pose.compose(noise)

        # Add an initial guess for the current pose
        initial_estimate.insert(X(i), initial_xi)

        # If this is the first iteration, add a prior on the first pose to set the coordinate frame
        # and a prior on the first landmark to set the scale
        # Also, as iSAM solves incrementally, we must wait until each is observed at least twice before
        # adding it to iSAM.
        if i == 0:
            # Add a prior on pose x0, with 0.3 rad std on roll,pitch,yaw and 0.1m x,y,z
            pose_noise = gtsam.noiseModel.Diagonal.Sigmas(
                np.array([0.3, 0.3, 0.3, 0.1, 0.1, 0.1]))
            factor = PriorFactorPose3(X(0), poses[0], pose_noise)
            graph.push_back(factor)

            # Add a prior on landmark l0
            point_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
            factor = PriorFactorPoint3(L(0), points[0], point_noise)
            graph.push_back(factor)

            # Add initial guesses to all observed landmarks
            noise = np.array([-0.25, 0.20, 0.15])
            for j, point in enumerate(points):
                # Intentionally initialize the variables off from the ground truth
                initial_lj = points[j] + noise
                initial_estimate.insert(L(j), initial_lj)
        else:
            # Update iSAM with the new factors
            isam.update(graph, initial_estimate)
            current_estimate = isam.estimate()
            print('*' * 50)
            print('Frame {}:'.format(i))
            current_estimate.print('Current estimate: ')

            # Clear the factor graph and values for the next iteration
            graph.resize(0)
            initial_estimate.clear()


if __name__ == '__main__':
    main()
