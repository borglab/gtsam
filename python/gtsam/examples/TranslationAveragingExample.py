"""
GTSAM Copyright 2010-2018, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

This example shows how 1dsfm uses outlier rejection (MFAS) and optimization (translation recovery)
together for estimating global translations from relative translation directions and global rotations.
The purpose of this example is to illustrate the connection between these two classes using a small SfM dataset.

Author: Akshay Krishnan
Date: September 2020
"""

import numpy as np

import gtsam
from gtsam.examples import SFMdata


def get_data():
    """"Returns data from SfMData.createPoses(). This contains global rotations and unit translations directions."""
    # Using toy dataset in SfMdata for example.
    poses = SFMdata.createPoses(gtsam.Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0))
    rotations = gtsam.Values()
    translation_directions = []
    for i in range(0, len(poses) - 2):
        # Add the rotation
        rotations.insert(i, poses[i].rotation())
        # Create unit translation measurements with next two poses
        for j in range(i + 1, i + 3):
            i_Z_j = gtsam.Unit3(poses[i].rotation().unrotate(
                poses[j].translation() - poses[i].translation()))
            translation_directions.append(gtsam.BinaryMeasurementUnit3(
                i, j, i_Z_j, gtsam.noiseModel.Isotropic.Sigma(3, 0.01)))
    # Add the last two rotations.
    rotations.insert(len(poses) - 1, poses[-1].rotation())
    rotations.insert(len(poses) - 2, poses[-2].rotation())
    return (rotations, translation_directions)


def estimate_poses_given_rot(measurements: gtsam.BinaryMeasurementsUnit3,
                             rotations: gtsam.Values):
    """Estimate poses given normalized translation directions and rotations between nodes.

    Arguments:
        measurements {BinaryMeasurementsUnit3}- List of translation direction from the first node to
        the second node in the coordinate frame of the first node.
        rotations {Values} -- Estimated rotations

    Returns:
        Values -- Estimated poses.
    """

    # Some hyperparameters.
    max_1dsfm_projection_directions = 50
    outlier_weight_threshold = 0.1

    # Convert the translation directions to global frame using the rotations.
    w_measurements = gtsam.BinaryMeasurementsUnit3()
    for measurement in measurements:
        w_measurements.append(gtsam.BinaryMeasurementUnit3(measurement.key1(), measurement.key2(), gtsam.Unit3(
            rotations.atRot3(measurement.key1()).rotate(measurement.measured().point3())), measurement.noiseModel()))

    # Indices of measurements that are to be used as projection directions. These are randomly chosen.
    indices = np.random.choice(len(w_measurements), min(
        max_1dsfm_projection_directions, len(w_measurements)), replace=False)
    # Sample projection directions from the measurements.
    projection_directions = [w_measurements[idx].measured() for idx in indices]

    outlier_weights = []
    # Find the outlier weights for each direction using MFAS.
    for direction in projection_directions:
        algorithm = gtsam.MFAS(w_measurements, direction)
        outlier_weights.append(algorithm.computeOutlierWeights())

    # Compute average of outlier weights.
    avg_outlier_weights = {}
    for outlier_weight_dict in outlier_weights:
        for k, v in outlier_weight_dict.items():
            if k in avg_outlier_weights:
                avg_outlier_weights[k] += v / len(outlier_weights)
            else:
                avg_outlier_weights[k] = v / len(outlier_weights)

    # Remove measurements that have weight greater than threshold.
    inlier_measurements = gtsam.BinaryMeasurementsUnit3()
    [inlier_measurements.append(m) for m in w_measurements if avg_outlier_weights[(
        m.key1(), m.key2())] < outlier_weight_threshold]

    # Run the optimizer to obtain translations for normalized directions.
    translations = gtsam.TranslationRecovery(inlier_measurements).run()

    poses = gtsam.Values()
    for key in rotations.keys():
        poses.insert(key, gtsam.Pose3(
            rotations.atRot3(key), translations.atPoint3(key)))
    return poses


def main():
    rotations, translation_directions = get_data()
    poses = estimate_poses_given_rot(translation_directions, rotations)
    print("**** Translation averaging output ****")
    print(poses)
    print("**************************************")


if __name__ == '__main__':
    main()
