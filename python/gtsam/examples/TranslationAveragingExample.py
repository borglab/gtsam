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
from typing import List, Tuple

import gtsam
import numpy as np
from gtsam.examples import SFMdata

# Hyperparameters for 1dsfm, values used from Kyle Wilson's code.
MAX_1DSFM_PROJECTION_DIRECTIONS = 48
OUTLIER_WEIGHT_THRESHOLD = 0.1


def get_data() -> Tuple[gtsam.Values, List[gtsam.BinaryMeasurementUnit3]]:
    """"Returns global rotations and unit translation directions between 8 cameras
    that lie on a circle and face the center. The poses of 8 cameras are obtained from SFMdata
    and the unit translations directions between some camera pairs are computed from their
    global translations. """
    fx, fy, s, u0, v0 = 50.0, 50.0, 0.0, 50.0, 50.0
    wTc_list = SFMdata.createPoses(gtsam.Cal3_S2(fx, fy, s, u0, v0))
    # Rotations of the cameras in the world frame.
    wRc_values = gtsam.Values()
    # Normalized translation directions from camera i to camera j
    # in the coordinate frame of camera i.
    i_iZj_list = []
    for i in range(0, len(wTc_list) - 2):
        # Add the rotation.
        wRi = wTc_list[i].rotation()
        wRc_values.insert(i, wRi)
        # Create unit translation measurements with next two poses.
        for j in range(i + 1, i + 3):
            # Compute the translation from pose i to pose j, in the world reference frame.
            w_itj = wTc_list[j].translation() - wTc_list[i].translation()
            # Obtain the translation in the camera i's reference frame.
            i_itj = wRi.unrotate(w_itj)
            # Compute the normalized unit translation direction.
            i_iZj = gtsam.Unit3(i_itj)
            i_iZj_list.append(gtsam.BinaryMeasurementUnit3(
                i, j, i_iZj, gtsam.noiseModel.Isotropic.Sigma(3, 0.01)))
    # Add the last two rotations.
    wRc_values.insert(len(wTc_list) - 1, wTc_list[-1].rotation())
    wRc_values.insert(len(wTc_list) - 2, wTc_list[-2].rotation())
    return wRc_values, i_iZj_list


def filter_outliers(w_iZj_list: List[gtsam.BinaryMeasurementUnit3]) -> List[gtsam.BinaryMeasurementUnit3]:
    """Removes outliers from a list of Unit3 measurements that are the 
    translation directions from camera i to camera j in the world frame."""

    # Indices of measurements that are to be used as projection directions.
    # These are randomly chosen. All sampled directions must be unique.
    num_directions_to_sample = min(
        MAX_1DSFM_PROJECTION_DIRECTIONS, len(w_iZj_list))
    sampled_indices = np.random.choice(
        len(w_iZj_list), num_directions_to_sample, replace=False)

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

    # Remove w_iZj that have weight greater than threshold, these are outliers.
    w_iZj_inliers = []
    [w_iZj_inliers.append(w_iZj) for w_iZj in w_iZj_list if avg_outlier_weights[(
        w_iZj.key1(), w_iZj.key2())] < OUTLIER_WEIGHT_THRESHOLD]

    return w_iZj_inliers


def estimate_poses(i_iZj_list: List[gtsam.BinaryMeasurementUnit3],
                   wRc_values: gtsam.Values) -> gtsam.Values:
    """Estimate poses given rotations and normalized translation directions between cameras.

    Args:
        i_iZj_list: List of normalized translation direction measurements between camera pairs, 
                    Z here refers to measurements. The measurements are of camera j with reference 
                    to camera i (iZj), in camera i's coordinate frame (i_). iZj represents a unit 
                    vector to j in i's frame and is not a transformation. 
        wRc_values: Rotations of the cameras in the world frame.

    Returns:
        gtsam.Values: Estimated poses.
    """

    # Convert the translation direction measurements to world frame using the rotations.
    w_iZj_list = []
    for i_iZj in i_iZj_list:
        w_iZj = gtsam.Unit3(wRc_values.atRot3(i_iZj.key1())
                                      .rotate(i_iZj.measured().point3()))
        w_iZj_list.append(gtsam.BinaryMeasurementUnit3(
            i_iZj.key1(), i_iZj.key2(), w_iZj, i_iZj.noiseModel()))

    # Remove the outliers in the unit translation directions.
    w_iZj_inliers = filter_outliers(w_iZj_list)

    # Run the optimizer to obtain translations for normalized directions.
    wtc_values = gtsam.TranslationRecovery().run(w_iZj_inliers)

    wTc_values = gtsam.Values()
    for key in wRc_values.keys():
        wTc_values.insert(key, gtsam.Pose3(
            wRc_values.atRot3(key), wtc_values.atPoint3(key)))
    return wTc_values


def main():
    wRc_values, i_iZj_list = get_data()
    wTc_values = estimate_poses(i_iZj_list, wRc_values)
    print("**** Translation averaging output ****")
    print(wTc_values)
    print("**************************************")


if __name__ == '__main__':
    main()
