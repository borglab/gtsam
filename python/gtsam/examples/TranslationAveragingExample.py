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

# Hyperparameters for 1dsfm, values used from Kyle Wilson's code.
MAX_1DSFM_PROJECTION_DIRECTIONS = 48
OUTLIER_WEIGHT_THRESHOLD = 0.1


def get_data() -> Tuple[gtsam.Values, List[gtsam.BinaryMeasurementUnit3]]:
    """"Returns global rotations and unit translation directions between 8 cameras 
    that lie on a circle and face the center. The poses of 8 cameras are obtained from SFMdata 
    and the unit translations directions between some camera pairs are computed from their 
    global translations. """
    # Using toy dataset in SfMdata for example.
    wTc = SFMdata.createPoses(gtsam.Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0))
    # Rotations of the cameras in the world frame.
    wRc_values = gtsam.Values()
    # Normalized translation directions from camera i to camera j
    # in the coordinate frame of camera i.
    i_iZj_list = []
    for i in range(0, len(wTc) - 2):
        # Add the rotation.
        wRc_values.insert(i, wTc[i].rotation())
        # Create unit translation measurements with next two poses.
        for j in range(i + 1, i + 3):
            i_iZj = gtsam.Unit3(wTc[i].rotation().unrotate(
                wTc[j].translation() - wTc[i].translation()))
            i_iZj_list.append(gtsam.BinaryMeasurementUnit3(
                i, j, i_iZj, gtsam.noiseModel.Isotropic.Sigma(3, 0.01)))
    # Add the last two rotations.
    wRc_values.insert(len(wTc) - 1, wTc[-1].rotation())
    wRc_values.insert(len(wTc) - 2, wTc[-2].rotation())
    return (wRc_values, i_iZj_list)


def estimate_poses(i_iZj_list: gtsam.BinaryMeasurementsUnit3,
                   wRc_values: gtsam.Values) -> gtsam.Values:
    """Estimate poses given rotations and normalized translation directions between cameras.

    Args:
        iZj_list -- List of normalized translation direction measurements between camera pairs, 
                    Z here refers to measurements. The measurements are of camera j with reference 
                    to camera i (iZj), in camera i's coordinate frame (i_). iZj represents a unit 
                    vector to j in i's frame and is not a transformation. 
        wRc_values -- Rotations of the cameras in the world frame.

    Returns:
        Values -- Estimated poses.
    """

    # Convert the translation direction measurements to world frame using the rotations.
    w_iZj_list = gtsam.BinaryMeasurementsUnit3()
    for i_iZj in i_iZj_list:
        w_iZj = gtsam.Unit3(wRc_values.atRot3(i_iZj.key1())
                                      .rotate(i_iZj.measured().point3()))
        w_iZj_list.append(gtsam.BinaryMeasurementUnit3(
            i_iZj.key1(), i_iZj.key2(), w_iZj, i_iZj.noiseModel()))

    # Indices of measurements that are to be used as projection directions. 
    # These are randomly chosen.
    sampled_indices = np.random.choice(len(w_iZj_list), min(
        MAX_1DSFM_PROJECTION_DIRECTIONS, len(w_iZj_list)), replace=False)
    # Sample projection directions from the measurements.
    projection_directions = [w_iZj_list[idx].measured()
                             for idx in sampled_indices]

    outlier_weights = []
    # Find the outlier weights for each direction using MFAS.
    for direction in projection_directions:
        algorithm = gtsam.MFAS(w_iZj_list, direction)
        outlier_weights.append(algorithm.computeOutlierWeights())

    # Compute average of outlier weights. Each outlier weight is a map from a pair of Keys 
    # (camera IDs) to a weight, where weights are proportional to the probability of the edge 
    # being an outlier.
    avg_outlier_weights = defaultdict(float)
    for outlier_weight_dict in outlier_weights:
        for keypair, weight in outlier_weight_dict.items():
            avg_outlier_weights[keypair] += weight / len(outlier_weights)

    # Remove w_relative_tranlsations that have weight greater than threshold, these are outliers.
    w_iZj_inliers = gtsam.BinaryMeasurementsUnit3()
    [w_iZj_inliers.append(Z) for Z in w_iZj_list
        if avg_outlier_weights[(Z.key1(), Z.key2())] < OUTLIER_WEIGHT_THRESHOLD]

    # Run the optimizer to obtain translations for normalized directions.
    wtc_values = gtsam.TranslationRecovery(w_iZj_inliers).run()

    wTc_values = gtsam.Values()
    for key in wRc_values.keys():
        wTc_values.insert(key, gtsam.Pose3(
            wRc_values.atRot3(key), wtc_values.atPoint3(key)))
    return wTc_values


def main():
    wRc_values, w_iZj_list = get_data()
    wTc_values = estimate_poses(w_iZj_list, wRc_values)
    print("**** Translation averaging output ****")
    print(wTc_values)
    print("**************************************")


if __name__ == '__main__':
    main()
