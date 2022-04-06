"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

ProjectionFactorRollingShutter unit tests.
Author: Yotam Stern
"""
import unittest

import numpy as np

import gtsam
import gtsam_unstable
from gtsam.utils.test_case import GtsamTestCase


pose1 = gtsam.Pose3()
pose2 = gtsam.Pose3(np.array([[ 0.9999375 ,  0.00502487,  0.00998725,  0.1       ],
                              [-0.00497488,  0.999975  , -0.00502487,  0.02      ],
                              [-0.01001225,  0.00497488,  0.9999375 ,  1.        ],
                              [ 0.        ,  0.        ,  0.        ,  1.        ]]))
point = np.array([2, 0, 15])
point_noise = gtsam.noiseModel.Diagonal.Sigmas(np.ones(2))
cal = gtsam.Cal3_S2()
body_p_sensor = gtsam.Pose3()
alpha = 0.1
measured = np.array([0.13257015, 0.0004157])


class TestProjectionFactorRollingShutter(GtsamTestCase):

    def test_constructor(self):
        '''
        test constructor for the ProjectionFactorRollingShutter
        '''
        factor = gtsam_unstable.ProjectionFactorRollingShutter(measured, alpha, point_noise, 0, 1, 2, cal)
        factor = gtsam_unstable.ProjectionFactorRollingShutter(measured, alpha, point_noise, 0, 1, 2, cal,
                                                               body_p_sensor)
        factor = gtsam_unstable.ProjectionFactorRollingShutter(measured, alpha, point_noise, 0, 1, 2, cal, True, False)
        factor = gtsam_unstable.ProjectionFactorRollingShutter(measured, alpha, point_noise, 0, 1, 2, cal, True, False,
                                                               body_p_sensor)

    def test_error(self):
        '''
        test the factor error for a specific example
        '''
        values = gtsam.Values()
        values.insert(0, pose1)
        values.insert(1, pose2)
        values.insert(2, point)
        factor = gtsam_unstable.ProjectionFactorRollingShutter(measured, alpha, point_noise, 0, 1, 2, cal)
        self.gtsamAssertEquals(factor.error(values), np.array(0), tol=1e-9)


if __name__ == '__main__':
    unittest.main()
