"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for testing dataset access.
Author: Frank Dellaert (Python: Sushmita Warrier)
"""
# pylint: disable=invalid-name, no-name-in-module, no-member

from __future__ import print_function

import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestSfmData(GtsamTestCase):
    """Tests for SfmData and SfmTrack modules."""

    def setUp(self):
        """Initialize SfmData and SfmTrack"""
        self.data = gtsam.SfmData()
        # initialize SfmTrack with 3D point
        self.tracks = gtsam.SfmTrack()

    def test_tracks(self):
        """Test functions in SfmTrack"""
        # measurement is of format (camera_idx, imgPoint)
        # create arbitrary camera indices for two cameras
        i1, i2 = 4,5
        # create arbitrary image measurements for cameras i1 and i2
        uv_i1 = gtsam.Point2(12.6, 82)
        # translating point uv_i1 along X-axis
        uv_i2 = gtsam.Point2(24.88, 82)
        # add measurements to the track
        self.tracks.add_measurement(i1, uv_i1)
        self.tracks.add_measurement(i2, uv_i2)
        # Number of measurements in the track is 2
        self.assertEqual(self.tracks.number_measurements(), 2)
        # camera_idx in the first measurement of the track corresponds to i1
        cam_idx, img_measurement = self.tracks.measurement(0)
        self.assertEqual(cam_idx, i1)
        np.testing.assert_array_almost_equal(
            gtsam.Point3(0.,0.,0.), 
            self.tracks.point3()
        )


    def test_data(self):
        """Test functions in SfmData"""
        # Create new track with 3 measurements
        i1, i2, i3 = 3,5,6
        uv_i1 = gtsam.Point2(21.23, 45.64)
        # translating along X-axis
        uv_i2 = gtsam.Point2(45.7, 45.64)
        uv_i3 = gtsam.Point2(68.35, 45.64)
        # add measurements and arbitrary point to the track
        measurements = [(i1, uv_i1), (i2, uv_i2), (i3, uv_i3)]
        pt = gtsam.Point3(1.0, 6.0, 2.0)
        track2 = gtsam.SfmTrack(pt)
        track2.add_measurement(i1, uv_i1)
        track2.add_measurement(i2, uv_i2)
        track2.add_measurement(i3, uv_i3)
        self.data.add_track(self.tracks)
        self.data.add_track(track2)
        # Number of tracks in SfmData is 2
        self.assertEqual(self.data.number_tracks(), 2)
        # camera idx of first measurement of second track corresponds to i1
        cam_idx, img_measurement = self.data.track(1).measurement(0)
        self.assertEqual(cam_idx, i1)

if __name__ == '__main__':
    unittest.main()
