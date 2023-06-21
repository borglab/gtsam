"""Unit tests for track generation using a Disjoint Set Forest data structure.

Authors: John Lambert
"""

import unittest

import numpy as np
from gtsam.gtsfm import Keypoints
from gtsam.utils.test_case import GtsamTestCase

import gtsam
from gtsam import IndexPair, Point2, SfmTrack2d


class TestDsfTrackGenerator(GtsamTestCase):
    """Tests for DsfTrackGenerator."""

    def test_track_generation(self) -> None:
        """Ensures that DSF generates three tracks from measurements
        in 3 images (H=200,W=400)."""
        kps_i0 = Keypoints(np.array([[10.0, 20], [30, 40]]))
        kps_i1 = Keypoints(np.array([[50.0, 60], [70, 80], [90, 100]]))
        kps_i2 = Keypoints(np.array([[110.0, 120], [130, 140]]))

        keypoints_list = []
        keypoints_list.append(kps_i0)
        keypoints_list.append(kps_i1)
        keypoints_list.append(kps_i2)

        # For each image pair (i1,i2), we provide a (K,2) matrix
        # of corresponding image indices (k1,k2).
        matches_dict = {}
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
        assert track0.numberMeasurements() == 2
        np.testing.assert_allclose(track0.measurements[0][1], Point2(10, 20))
        np.testing.assert_allclose(track0.measurements[1][1], Point2(50, 60))
        assert track0.measurements[0][0] == 0
        assert track0.measurements[1][0] == 1
        np.testing.assert_allclose(
            track0.measurementMatrix(),
            [
                [10, 20],
                [50, 60],
            ],
        )
        np.testing.assert_allclose(track0.indexVector(), [0, 1])

        # Verify track 1.
        track1 = tracks[1]
        np.testing.assert_allclose(
            track1.measurementMatrix(),
            [
                [30, 40],
                [70, 80],
                [130, 140],
            ],
        )
        np.testing.assert_allclose(track1.indexVector(), [0, 1, 2])

        # Verify track 2.
        track2 = tracks[2]
        np.testing.assert_allclose(
            track2.measurementMatrix(),
            [
                [90, 100],
                [110, 120],
            ],
        )
        np.testing.assert_allclose(track2.indexVector(), [1, 2])


class TestSfmTrack2d(GtsamTestCase):
    """Tests for SfmTrack2d."""

    def test_sfm_track_2d_constructor(self) -> None:
        """Test construction of 2D SfM track."""
        measurements = []
        measurements.append((0, Point2(10, 20)))
        track = SfmTrack2d(measurements=measurements)
        track.measurement(0)
        assert track.numberMeasurements() == 1


if __name__ == "__main__":
    unittest.main()
