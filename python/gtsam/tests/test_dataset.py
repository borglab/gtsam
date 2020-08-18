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

import unittest

import gtsam
from gtsam import BetweenFactorPose3
from gtsam.utils.test_case import GtsamTestCase


class TestDataset(GtsamTestCase):
    """Tests for datasets.h wrapper."""

    def setUp(self):
        """Get some common paths."""
        self.pose3_example_g2o_file = gtsam.findExampleDataFile(
            "pose3example.txt")

    def test_readG2o3D(self):
        """Test reading directly into factor graph."""
        is3D = True
        graph, initial = gtsam.readG2o(self.pose3_example_g2o_file, is3D)
        self.assertEqual(graph.size(), 6)
        self.assertEqual(initial.size(), 5)

    def test_parse3Dfactors(self):
        """Test parsing into data structure."""
        factors = gtsam.parse3DFactors(self.pose3_example_g2o_file)
        self.assertEqual(len(factors), 6)
        self.assertIsInstance(factors[0], BetweenFactorPose3)


if __name__ == '__main__':
    unittest.main()
