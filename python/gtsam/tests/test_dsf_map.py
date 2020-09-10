"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Disjoint Set Forest.
Author: Frank Dellaert & Varun Agrawal
"""
# pylint: disable=invalid-name, no-name-in-module, no-member

from __future__ import print_function

import unittest

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestDSFMap(GtsamTestCase):
    """Tests for DSFMap."""

    def test_all(self):
        """Test everything in DFSMap."""
        def key(index_pair):
            return index_pair.i(), index_pair.j()

        dsf = gtsam.DSFMapIndexPair()
        pair1 = gtsam.IndexPair(1, 18)
        self.assertEqual(key(dsf.find(pair1)), key(pair1))
        pair2 = gtsam.IndexPair(2, 2)
        
        # testing the merge feature of dsf
        dsf.merge(pair1, pair2)
        self.assertEqual(key(dsf.find(pair1)), key(dsf.find(pair2)))

    def test_sets(self):
        from gtsam import IndexPair
        dsf = gtsam.DSFMapIndexPair()
        dsf.merge(IndexPair(0, 1), IndexPair(1,2))
        dsf.merge(IndexPair(0, 1), IndexPair(3,4))
        dsf.merge(IndexPair(4,5), IndexPair(6,8))
        sets = dsf.sets()

        for i in sets:
            s = sets[i]
            for val in gtsam.IndexPairSetAsArray(s):
                val.i()
                val.j()


if __name__ == '__main__':
    unittest.main()
