"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information.

SmartStereoProjectionPoseFactor Python binding tests.
"""

# pylint: disable=no-member

import gc
import unittest

import gtsam
import gtsam_unstable
from gtsam.utils.test_case import GtsamTestCase


class TestSmartStereoProjectionPoseFactor(GtsamTestCase):
    """Tests the unstable smart stereo factor's container bindings."""

    def test_calibration_vector_round_trip(self):
        """The named vector preserves shared calibration ownership."""
        noise = gtsam.noiseModel.Isotropic.Sigma(3, 1.0)
        factor = gtsam_unstable.SmartStereoProjectionPoseFactor(noise)

        first = gtsam.Cal3_S2Stereo(500.0, 500.0, 0.0, 320.0, 240.0, 0.2)
        second = gtsam.Cal3_S2Stereo(510.0, 505.0, 0.0, 320.0, 240.0, 0.3)
        measurement_values = [
            gtsam.StereoPoint2(320.0, 300.0, 240.0),
            gtsam.StereoPoint2(330.0, 310.0, 245.0),
        ]
        measurements = gtsam_unstable.StereoPoint2Vector(measurement_values)
        calibrations = gtsam_unstable.Cal3_S2StereoVector([first, second])
        factor.add(measurements, [1, 2], calibrations)

        # Existing Python list inputs remain implicitly convertible.
        factor_from_lists = gtsam_unstable.SmartStereoProjectionPoseFactor(
            noise)
        factor_from_lists.add(measurement_values, [1, 2], [first, second])
        self.assertEqual(len(factor_from_lists.calibration()), 2)
        del factor_from_lists

        del calibrations, measurements, measurement_values, first, second
        gc.collect()
        recovered = factor.calibration()
        self.assertIsInstance(recovered,
                              gtsam_unstable.Cal3_S2StereoVector)
        self.assertEqual(len(recovered), 2)
        self.assertAlmostEqual(recovered[0].baseline(), 0.2)
        self.assertAlmostEqual(recovered[1].baseline(), 0.3)

        del factor
        gc.collect()
        self.assertAlmostEqual(recovered[0].baseline(), 0.2)


if __name__ == "__main__":
    unittest.main()
