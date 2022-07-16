"""

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
        """ """
        kps_i0 = Keypoints(coordinates=np.array([[0,0],[1,1]]))
        kps_i1 = Keypoints(coordinates=np.array([[2,2],[3,3],[4,4]]))
        kps_i2 = Keypoints(coordinates=np.array([[5,5],[6,6]]))

        keypoints_list = KeypointsList()
        keypoints_list.append(kps_i0)
        keypoints_list.append(kps_i1)
        keypoints_list.append(kps_i2)

        matches_dict = MatchIndicesMap()
        matches_dict[(0,0)] = np.array([[0,0],[1,1]])
        matches_dict[(1,1)] = np.array([[2,2],[3,3],[4,4]])
        import pdb; pdb.set_trace()

        tracks = DsfTrackGenerator.generate_tracks_from_pairwise_matches(matches_dict, keypoints_list)
