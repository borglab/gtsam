"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Disjoint Set Forest.
Author: Frank Dellaert & Varun Agrawal & John Lambert
"""
# pylint: disable=invalid-name, no-name-in-module, no-member

from __future__ import print_function

import unittest
from typing import Tuple

from gtsam import DSFMapIndexPair, IndexPair, IndexPairSetAsArray
from gtsam.utils.test_case import GtsamTestCase


class TestDSFMap(GtsamTestCase):
    """Tests for DSFMap."""

    def test_all(self) -> None:
        """Test everything in DFSMap."""

        def key(index_pair) -> Tuple[int, int]:
            return index_pair.i(), index_pair.j()

        dsf = DSFMapIndexPair()
        pair1 = IndexPair(1, 18)
        self.assertEqual(key(dsf.find(pair1)), key(pair1))
        pair2 = IndexPair(2, 2)

        # testing the merge feature of dsf
        dsf.merge(pair1, pair2)
        self.assertEqual(key(dsf.find(pair1)), key(dsf.find(pair2)))

    def test_sets(self) -> None:
        """Ensure that pairs are merged correctly during Union-Find.

        An IndexPair (i,k) representing a unique key might represent the
        k'th detected keypoint in image i. For the data below, merging such
        measurements into feature tracks across frames should create 2 distinct sets.
        """
        dsf = DSFMapIndexPair()
        dsf.merge(IndexPair(0, 1), IndexPair(1, 2))
        dsf.merge(IndexPair(0, 1), IndexPair(3, 4))
        dsf.merge(IndexPair(4, 5), IndexPair(6, 8))
        sets = dsf.sets()

        merged_sets = set()

        for i in sets:
            set_keys = []
            s = sets[i]
            for val in IndexPairSetAsArray(s):
                set_keys.append((val.i(), val.j()))
            merged_sets.add(tuple(set_keys))

        # fmt: off
        expected_sets = {
            ((0, 1), (1, 2), (3, 4)), # set 1
            ((4, 5), (6, 8)) # set 2
        }
        # fmt: on
        assert expected_sets == merged_sets


if __name__ == "__main__":
    unittest.main()
