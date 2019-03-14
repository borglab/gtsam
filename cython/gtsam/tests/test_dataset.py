"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for testing dataset access.
Author: Frank Dellaert
"""
# pylint: disable=invalid-name, E1101

from __future__ import print_function

import unittest

import gtsam


class TestDataset(unittest.TestCase):
    def setUp(self):
        pass

    def test_3d_graph(self):
        is3D = True
        g2o_file = gtsam.findExampleDataFile("pose3example.txt")
        graph, initial = gtsam.readG2o(g2o_file, is3D)
        self.assertEqual(graph.size(), 6)
        self.assertEqual(initial.size(), 5)


if __name__ == '__main__':
    unittest.main()
