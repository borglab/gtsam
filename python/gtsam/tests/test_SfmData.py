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

import gtsam
import numpy as np
from gtsam import Point2, Point3, SfmData, SfmTrack
from gtsam.utils.test_case import GtsamTestCase


class TestSfmData(GtsamTestCase):
    """Tests for SfmData and SfmTrack modules."""

    def setUp(self):
        """Initialize SfmData and SfmTrack"""
        self.data = SfmData()
        # initialize SfmTrack with 3D point
        self.tracks = SfmTrack()

    def test_tracks(self):
        """Test functions in SfmTrack"""
        # measurement is of format (camera_idx, imgPoint)
        # create arbitrary camera indices for two cameras
        i1, i2 = 4, 5

        # create arbitrary image measurements for cameras i1 and i2
        uv_i1 = Point2(12.6, 82)

        # translating point uv_i1 along X-axis
        uv_i2 = Point2(24.88, 82)

        # add measurements to the track
        self.tracks.addMeasurement(i1, uv_i1)
        self.tracks.addMeasurement(i2, uv_i2)

        # Number of measurements in the track is 2
        self.assertEqual(self.tracks.numberMeasurements(), 2)

        # camera_idx in the first measurement of the track corresponds to i1
        cam_idx, img_measurement = self.tracks.measurement(0)
        self.assertEqual(cam_idx, i1)
        np.testing.assert_array_almost_equal(
            Point3(0., 0., 0.),
            self.tracks.point3()
        )

    def test_data(self):
        """Test functions in SfmData"""
        # Create new track with 3 measurements
        i1, i2, i3 = 3, 5, 6
        uv_i1 = Point2(21.23, 45.64)

        # translating along X-axis
        uv_i2 = Point2(45.7, 45.64)
        uv_i3 = Point2(68.35, 45.64)

        # add measurements and arbitrary point to the track
        measurements = [(i1, uv_i1), (i2, uv_i2), (i3, uv_i3)]
        pt = Point3(1.0, 6.0, 2.0)
        track2 = SfmTrack(pt)
        track2.addMeasurement(i1, uv_i1)
        track2.addMeasurement(i2, uv_i2)
        track2.addMeasurement(i3, uv_i3)
        self.data.addTrack(self.tracks)
        self.data.addTrack(track2)

        # Number of tracks in SfmData is 2
        self.assertEqual(self.data.numberTracks(), 2)

        # camera idx of first measurement of second track corresponds to i1
        cam_idx, img_measurement = self.data.track(1).measurement(0)
        self.assertEqual(cam_idx, i1)

    def test_Balbianello(self):
        """ Check that we can successfully read a bundler file and create a 
            factor graph from it
        """
        # The structure where we will save the SfM data
        filename = gtsam.findExampleDataFile("Balbianello.out")
        sfm_data = SfmData.FromBundlerFile(filename)

        # Check number of things
        self.assertEqual(5, sfm_data.numberCameras())
        self.assertEqual(544, sfm_data.numberTracks())
        track0 = sfm_data.track(0)
        self.assertEqual(3, track0.numberMeasurements())

        # Check projection of a given point
        self.assertEqual(0, track0.measurement(0)[0])
        camera0 = sfm_data.camera(0)
        expected = camera0.project(track0.point3())
        actual = track0.measurement(0)[1]
        self.gtsamAssertEquals(expected, actual, 1)

        # We share *one* noiseModel between all projection factors
        model = gtsam.noiseModel.Isotropic.Sigma(
            2, 1.0)  # one pixel in u and v

        # Convert to NonlinearFactorGraph
        graph = sfm_data.sfmFactorGraph(model)
        self.assertEqual(1419, graph.size())  # regression

        # Get initial estimate
        values = gtsam.initialCamerasAndPointsEstimate(sfm_data)
        self.assertEqual(549, values.size())  # regression


if __name__ == '__main__':
    unittest.main()
