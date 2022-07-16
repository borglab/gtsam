"""Unit tests for track generation using a Disjoint Set Forest data structure.

Authors: John Lambert
"""

import unittest

import numpy as np

import gtsam
from gtsam import DsfTrackGenerator, Keypoints, KeypointsList, MatchIndicesMap
from gtsam.utils.test_case import GtsamTestCase


class TestDsfTrackGenerator(GtsamTestCase):
    """Tests for DsfTrackGenerator."""

    def test_track_generation(self) -> None:
        """Ensures that DSF generates two tracks from measurements in 3 images (H=200,W=400)."""
        kps_i0 = Keypoints(coordinates=np.array([[10.0, 20], [30, 40]]))
        kps_i1 = Keypoints(coordinates=np.array([[50.0, 60], [70, 80], [90, 100]]))
        kps_i2 = Keypoints(coordinates=np.array([[110.0, 120], [130, 140]]))

        keypoints_list = KeypointsList()
        keypoints_list.append(kps_i0)
        keypoints_list.append(kps_i1)
        keypoints_list.append(kps_i2)

        # For each image pair (i1,i2), we provide a (K,2) matrix of corresponding image indices (k1,k2).
        matches_dict = MatchIndicesMap()
        matches_dict[(0, 1)] = np.array([[0, 0], [1, 1]])
        matches_dict[(1, 2)] = np.array([[2, 0], [1, 1]])

        tracks = DsfTrackGenerator().generate_tracks_from_pairwise_matches(matches_dict, keypoints_list)
        print(tracks[0])
