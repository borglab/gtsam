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
        """Ensures that DSF generates three tracks from measurements in 3 images (H=200,W=400)."""
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
        import pdb

        pdb.set_trace()
        assert len(tracks) == 3

        # Verify track 0.
        assert np.allclose(tracks[0].measurements()[0].uv, np.array([10.0, 20.0]))
        assert np.allclose(tracks[0].measurements()[1].uv, np.array([50.0, 60.0]))
        assert tracks[0].measurements()[0].i == 0
        assert tracks[0].measurements()[1].i == 1

        # Verify track 1.
        assert np.allclose(tracks[1].measurements()[0].uv, np.array([30.0, 40.0]))
        assert np.allclose(tracks[1].measurements()[1].uv, np.array([70.0, 80.0]))
        assert np.allclose(tracks[1].measurements()[2].uv, np.array([130.0, 140.0]))
        assert tracks[1].measurements()[0].i == 0
        assert tracks[1].measurements()[1].i == 1
        assert tracks[1].measurements()[2].i == 2

        # Verify track 2.
        assert np.allclose(tracks[2].measurements()[0].uv, np.array([90.0, 100.0]))
        assert np.allclose(tracks[2].measurements()[1].uv, np.array([110.0, 120.0]))
        assert tracks[2].measurements()[0].i == 1
        assert tracks[2].measurements()[1].i == 2
