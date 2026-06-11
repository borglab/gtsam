"""
GTSAM Copyright 2010, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

A structure-from-motion example using SphericalCamera.

SphericalCamera measures bearing vectors (Unit3) instead of pixel coordinates,
making it suitable for omnidirectional cameras, fisheye cameras with very wide
field of view, or any setup where direction measurements are more natural.
"""

import matplotlib.pyplot as plt
import numpy as np

import gtsam
from gtsam import symbol_shorthand

L = symbol_shorthand.L
X = symbol_shorthand.X

from gtsam.examples import SFMdata
from gtsam.utils import plot

from gtsam import (DoglegOptimizer, GeneralSFMFactorSphericalCamera,
                   Marginals, NonlinearFactorGraph, SphericalCamera,
                   PriorFactorPoint3, PriorFactorSphericalCamera, Values, Unit3)


def main():
    """
    Structure-from-Motion using SphericalCamera.

    Unlike PinholeCamera which measures pixel coordinates (Point2),
    SphericalCamera measures bearing vectors (Unit3) - directions on the
    unit sphere. This is the natural measurement model for omnidirectional
    cameras or when working with calibrated bearing measurements.

    Key differences from standard SFM:
    - No camera calibration needed (SphericalCamera uses EmptyCal)
    - Measurements are Unit3 (2 DOF direction) instead of Point2 (2 DOF pixel)
    - Uses GeneralSFMFactorSphericalCamera instead of GenericProjectionFactor
    """

    # Create the set of ground-truth landmarks (cube corners)
    points = SFMdata.createPoints()

    # Create the set of ground-truth poses (circular trajectory around landmarks)
    poses = SFMdata.createPoses()

    # Define the camera observation noise model
    # Unit3 has 2 DOF, so we use a 2D isotropic noise model
    # The sigma represents angular uncertainty in radians
    measurement_noise = gtsam.noiseModel.Isotropic.Sigma(2, 0.01)  # ~0.6 degrees

    # Create a factor graph
    graph = NonlinearFactorGraph()

    # Add a prior on camera x0. This indirectly specifies where the origin is.
    # SphericalCamera has 6 DOF (same as Pose3): 0.1 rad std on roll,pitch,yaw and 0.1m on x,y,z
    camera_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    )
    graph.push_back(PriorFactorSphericalCamera(X(0), SphericalCamera(poses[0]), camera_noise))

    # Simulated measurements from each camera pose, adding them to the factor graph
    for i, pose in enumerate(poses):
        camera = SphericalCamera(pose)
        for j, point in enumerate(points):
            # project2 returns Unit3 (bearing vector)
            measurement = camera.project2(point)
            factor = GeneralSFMFactorSphericalCamera(
                measurement, measurement_noise, X(i), L(j)
            )
            graph.push_back(factor)

    # Add a prior on the first landmark to fix the scale
    # (SFM has scale ambiguity, this fixes it)
    point_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
    graph.push_back(PriorFactorPoint3(L(0), points[0], point_noise))

    print("Factor Graph:")
    print(f"  Number of factors: {graph.size()}")
    print(f"  Number of poses: {len(poses)}")
    print(f"  Number of landmarks: {len(points)}")

    # Create initial estimates with some noise
    initial_estimate = Values()
    rng = np.random.default_rng(42)  # Fixed seed for reproducibility

    for i, pose in enumerate(poses):
        # Add noise to initial pose estimates
        noisy_pose = pose.retract(0.1 * rng.standard_normal(6))
        initial_estimate.insert(X(i), SphericalCamera(noisy_pose))

    for j, point in enumerate(points):
        # Add noise to initial landmark estimates
        noisy_point = point + 0.1 * rng.standard_normal(3)
        initial_estimate.insert(L(j), noisy_point)

    print(f"\nInitial error: {graph.error(initial_estimate):.6f}")

    # Optimize the graph using Dogleg optimizer
    params = gtsam.DoglegParams()
    params.setVerbosity("TERMINATION")
    optimizer = DoglegOptimizer(graph, initial_estimate, params)

    print("\nOptimizing...")
    result = optimizer.optimize()

    print(f"Final error: {graph.error(result):.6f}")
    print(f"Iterations: {optimizer.iterations()}")

    # Print some results
    print("\n=== Results ===")
    print("\nOptimized Landmark Positions:")
    for j in range(len(points)):
        estimated = result.atPoint3(L(j))
        ground_truth = points[j]
        error = np.linalg.norm(estimated - ground_truth)
        print(f"  L{j}: estimated={estimated}, error={error:.4f}m")

    print("\nOptimized Camera Poses (translation only):")
    for i in range(len(poses)):
        estimated_camera = result.atSphericalCamera(X(i))
        estimated_trans = estimated_camera.pose().translation()
        ground_truth_trans = poses[i].translation()
        error = np.linalg.norm(estimated_trans - ground_truth_trans)
        print(f"  X{i}: estimated={estimated_trans}, error={error:.4f}m")

    # Compute marginals for uncertainty visualization
    marginals = Marginals(graph, result)

    # Extract poses from SphericalCamera results for plotting
    result_poses = Values()
    for i in range(len(poses)):
        camera = result.atSphericalCamera(X(i))
        result_poses.insert(X(i), camera.pose())

    # Plot the results
    plot.plot_3d_points(1, result, marginals=marginals)
    plot.plot_trajectory(1, result_poses, marginals=None, scale=8)
    plot.set_axes_equal(1)
    plt.title("SphericalCamera SFM Result")
    plt.show()


if __name__ == "__main__":
    main()
