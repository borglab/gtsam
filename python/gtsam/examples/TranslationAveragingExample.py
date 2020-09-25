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

from collections import defaultdict
from typing import Tuple, List

import numpy as np

import gtsam
from gtsam.examples import SFMdata


def get_data() -> Tuple[gtsam.Values, List[gtsam.BinaryMeasurementUnit3]]:
    """"Returns data from SfMData.createPoses(). This contains global rotations and unit translations directions."""
    # Using toy dataset in SfMdata for example.
    poses = SFMdata.createPoses(gtsam.Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0))
    # Rotations of the cameras in the world frame - wRc.
    rotations = gtsam.Values()
    # Normalized translation directions for pairs of cameras - from first camera to second,
    # in the coordinate frame of the first camera.
    translation_directions = []
    for i in range(0, len(poses) - 2):
        # Add the rotation.
        rotations.insert(i, poses[i].rotation())
        # Create unit translation measurements with next two poses.
        for j in range(i + 1, i + 3):
            i_Z_j = gtsam.Unit3(poses[i].rotation().unrotate(
                poses[j].translation() - poses[i].translation()))
            translation_directions.append(gtsam.BinaryMeasurementUnit3(
                i, j, i_Z_j, gtsam.noiseModel.Isotropic.Sigma(3, 0.01)))
    # Add the last two rotations.
    rotations.insert(len(poses) - 1, poses[-1].rotation())
    rotations.insert(len(poses) - 2, poses[-2].rotation())
    return (rotations, translation_directions)


def estimate_poses(relative_translations: gtsam.BinaryMeasurementsUnit3,
                   rotations: gtsam.Values) -> gtsam.Values:
    """Estimate poses given rotations normalized translation directions between cameras.

    Args:
        relative_translations -- List of normalized translation directions between camera pairs, each direction
                                 is from the first camera to the second, in the frame of the first camera.
        rotations -- Rotations of the cameras in the world frame.

    Returns:
        Values -- Estimated poses.
    """

    # Some hyperparameters, values used from 1dsfm.
    max_1dsfm_projection_directions = 48
    outlier_weight_threshold = 0.1

    # Convert the translation directions to global frame using the rotations.
    w_relative_translations = gtsam.BinaryMeasurementsUnit3()
    for relative_translation in relative_translations:
        w_relative_translation = gtsam.Unit3(rotations.atRot3(relative_translation.key1())
                                             .rotate(relative_translation.measured().point3()))
        w_relative_translations.append(gtsam.BinaryMeasurementUnit3(relative_translation.key1(),
                                                                    relative_translation.key2(),
                                                                    w_relative_translation,
                                                                    relative_translation.noiseModel()))

    # Indices of measurements that are to be used as projection directions. These are randomly chosen.
    sampled_indices = np.random.choice(len(w_relative_translations), min(
        max_1dsfm_projection_directions, len(w_relative_translations)), replace=False)
    # Sample projection directions from the measurements.
    projection_directions = [
        w_relative_translations[idx].measured() for idx in sampled_indices]

    outlier_weights = []
    # Find the outlier weights for each direction using MFAS.
    for direction in projection_directions:
        algorithm = gtsam.MFAS(w_relative_translations, direction)
        outlier_weights.append(algorithm.computeOutlierWeights())

    # Compute average of outlier weights. Each outlier weight is a map from a pair of Keys (camera IDs) to a weight,
    # where weights are proportional to the probability of the edge being an outlier.
    avg_outlier_weights = defaultdict(lambda: 0.0)
    for outlier_weight_dict in outlier_weights:
        for keypair, weight in outlier_weight_dict.items():
            avg_outlier_weights[keypair] += weight / len(outlier_weights)

    # Remove w_relative_tranlsations that have weight greater than threshold, these are outliers.
    inlier_w_relative_translations = gtsam.BinaryMeasurementsUnit3()
    [inlier_w_relative_translations.append(Z) for Z in w_relative_translations
        if avg_outlier_weights[(Z.key1(), Z.key2())] < outlier_weight_threshold]

    # Run the optimizer to obtain translations for normalized directions.
    w_translations = gtsam.TranslationRecovery(
        inlier_w_relative_translations).run()

    poses = gtsam.Values()
    for key in rotations.keys():
        poses.insert(key, gtsam.Pose3(
            rotations.atRot3(key), w_translations.atPoint3(key)))
    return poses


def main():
    rotations, translation_directions = get_data()
    poses = estimate_poses(translation_directions, rotations)
    print("**** Translation averaging output ****")
    print(poses)
    print("**************************************")


if __name__ == '__main__':
    main()
