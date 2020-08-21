"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for the STL bindings in pybind11.
Author: Fan Jiang
"""
import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestStlBindings(GtsamTestCase):

    def test_Creation(self):
        key_set = gtsam.KeySet()
        key_vector = gtsam.KeyVector()
        isam2_threshold_map = gtsam.ISAM2ThresholdMap()
        pose2s = gtsam.BetweenFactorPose2s()
        pose3s = gtsam.BetweenFactorPose3s()


if __name__ == "__main__":
    unittest.main()
