"""Unit tests for track generation using a Disjoint Set Forest data structure.

Authors: John Lambert
"""

import unittest

import gtsam
import numpy as np
from gtsam import (IndexPair, KeypointsVector, MatchIndicesMap, Point2,
                   SfmMeasurementVector, SfmTrack2d)
from gtsam.gtsfm import Keypoints
from gtsam.utils.test_case import GtsamTestCase


class TestDsfTrackGenerator(GtsamTestCase):
    """Tests for DsfTrackGenerator."""

    def test_track_generation(self) -> None:
        """Ensures that DSF generates three tracks from measurements
        in 3 images (H=200,W=400)."""
        kps_i0 = Keypoints(np.array([[10.0, 20], [30, 40]]))
        kps_i1 = Keypoints(np.array([[50.0, 60], [70, 80], [90, 100]]))
        kps_i2 = Keypoints(np.array([[110.0, 120], [130, 140]]))

        keypoints_list = KeypointsVector()
        keypoints_list.append(kps_i0)
        keypoints_list.append(kps_i1)
        keypoints_list.append(kps_i2)

        # For each image pair (i1,i2), we provide a (K,2) matrix
        # of corresponding image indices (k1,k2).
        matches_dict = MatchIndicesMap()
        matches_dict[IndexPair(0, 1)] = np.array([[0, 0], [1, 1]])
        matches_dict[IndexPair(1, 2)] = np.array([[2, 0], [1, 1]])

        tracks = gtsam.gtsfm.tracksFromPairwiseMatches(
            matches_dict,
            keypoints_list,
            verbose=False,
        )
        assert len(tracks) == 3

        # Verify track 0.
        track0 = tracks[0]
        np.testing.assert_allclose(track0.measurements[0][1], Point2(10, 20))
        np.testing.assert_allclose(track0.measurements[1][1], Point2(50, 60))
        assert track0.measurements[0][0] == 0
        assert track0.measurements[1][0] == 1
        assert track0.numberMeasurements() == 2

        # Verify track 1.
        track1 = tracks[1]
        np.testing.assert_allclose(track1.measurements[0][1], Point2(30, 40))
        np.testing.assert_allclose(track1.measurements[1][1], Point2(70, 80))
        np.testing.assert_allclose(track1.measurements[2][1], Point2(130, 140))
        assert track1.measurements[0][0] == 0
        assert track1.measurements[1][0] == 1
        assert track1.measurements[2][0] == 2
        assert track1.numberMeasurements() == 3

        # Verify track 2.
        track2 = tracks[2]
        np.testing.assert_allclose(track2.measurements[0][1], Point2(90, 100))
        np.testing.assert_allclose(track2.measurements[1][1], Point2(110, 120))
        assert track2.measurements[0][0] == 1
        assert track2.measurements[1][0] == 2
        assert track2.numberMeasurements() == 2


class TestSfmTrack2d(GtsamTestCase):
    """Tests for SfmTrack2d."""

    def test_sfm_track_2d_constructor(self) -> None:
        """ """
        measurements = SfmMeasurementVector()
        measurements.append((0, Point2(10, 20)))
        track = SfmTrack2d(measurements=measurements)
        track.measurement(0)
        track.numberMeasurements() == 1


if __name__ == "__main__":
    unittest.main()
