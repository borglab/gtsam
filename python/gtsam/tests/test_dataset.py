"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for testing dataset access.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

from __future__ import print_function

from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import gtsam
import numpy as np
from gtsam import BetweenFactorPose3
from gtsam.utils.test_case import GtsamTestCase


class TestDataset(GtsamTestCase):
    """Tests for datasets.h wrapper."""

    def setUp(self):
        """Get some common paths."""
        self.pose3_example_g2o_file = gtsam.findExampleDataFile("pose3example.txt")

    def test_readG2o3D(self):
        """Test reading directly into factor graph."""
        is3D = True
        graph, initial = gtsam.readG2o(self.pose3_example_g2o_file, is3D)
        self.assertEqual(graph.size(), 6)
        self.assertEqual(initial.size(), 5)

    def test_read_write_g2o_3d_landmark(self):
        """Round-trip an EDGE_SE3_TRACKXYZ through the Python wrapper."""
        input_file = gtsam.findExampleDataFile(
            "edge_se3_trackxyz_example.g2o")
        graph, values = gtsam.readG2o(input_file, is3D=True)

        self.assertEqual(graph.size(), 1)
        factor = graph.at(0)
        self.assertIsInstance(factor, gtsam.BearingRangeFactor3D)
        measurement = factor.measured()
        kP = measurement.range() * measurement.bearing().unitVector()
        np.testing.assert_allclose(kP, [2.0, 1.0, 3.0], atol=1e-9)

        with TemporaryDirectory() as directory:
            output_file = str(Path(directory) / "roundtrip.g2o")
            gtsam.writeG2o(graph, values, output_file)
            actual_graph, actual_values = gtsam.readG2o(output_file, is3D=True)

        self.assertTrue(graph.equals(actual_graph, 1e-4))
        self.assertTrue(values.equals(actual_values, 1e-4))

    def test_parse3Dfactors(self):
        """Test parsing into data structure."""
        factors = gtsam.parse3DFactors(self.pose3_example_g2o_file)
        self.assertEqual(len(factors), 6)
        self.assertIsInstance(factors[0], BetweenFactorPose3)

    def test_find_nested_example_data_file(self):
        """Nested example paths should resolve against the example-data roots."""
        metadata_path = Path(gtsam.findExampleDataFile("legged_staircase/metadata.csv"))
        self.assertTrue(metadata_path.is_file())
        self.assertEqual(metadata_path.name, "metadata.csv")
        self.assertEqual(metadata_path.parent.name, "legged_staircase")


if __name__ == "__main__":
    unittest.main()
