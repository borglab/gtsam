"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Serialization and deep copy tests.

Author: Varun Agrawal
"""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from gtsam.utils.test_case import GtsamTestCase

import gtsam


@unittest.skipUnless(hasattr(gtsam.Pose3, "serialize"), "Serialization not enabled")
class TestDeepCopy(GtsamTestCase):
    """Tests for deep copy of various GTSAM objects."""

    def test_noise_model_factors_in_fresh_interpreter(self):
        """Polymorphic archives work without unstable's export registrations."""
        script = """
import pickle
import sys
sys.path.insert(0, sys.argv[1])
import gtsam
import numpy as np

models = [
    gtsam.noiseModel.Isotropic.Sigma(2, 0.5, False),
    gtsam.noiseModel.Gaussian.SqrtInformation(
        np.array([[1.0, 0.2], [0.0, 1.0]]), False),
]
for noise in models:
    factor = gtsam.PriorFactorPoint2(0, np.ones(2), noise)
    assert isinstance(factor.serialize(), str)
    assert factor.equals(pickle.loads(pickle.dumps(factor)), 1e-9)
"""
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(
                [sys.executable, "-I", "-c", script,
                 str(Path(gtsam.__file__).resolve().parents[1])],
                cwd=directory, capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_PreintegratedImuMeasurements(self):
        """
        Test the deep copy of `PreintegratedImuMeasurements`.
        """
        params = gtsam.PreintegrationParams.MakeSharedD(9.81)
        pim = gtsam.PreintegratedImuMeasurements(params)

        self.assertDeepCopyEquality(pim)

    def test_ImuFactor(self):
        """
        Test the deep copy of `ImuFactor`.
        """
        params = gtsam.PreintegrationParams.MakeSharedD(9.81)
        params.setAccelerometerCovariance(1e-7 * np.eye(3))
        params.setGyroscopeCovariance(1e-8 * np.eye(3))
        params.setIntegrationCovariance(1e-9 * np.eye(3))
        priorBias = gtsam.imuBias.ConstantBias(np.zeros(3), np.zeros(3))
        pim = gtsam.PreintegratedImuMeasurements(params, priorBias)

        # Preintegrate a single measurement for serialization to work.
        pim.integrateMeasurement(measuredAcc=np.zeros(3),
                                 measuredOmega=np.zeros(3),
                                 deltaT=0.005)

        factor = gtsam.ImuFactor(0, 1, 2, 3, 4, pim)

        self.assertDeepCopyEquality(factor)

    def test_PreintegratedCombinedMeasurements(self):
        """
        Test the deep copy of `PreintegratedCombinedMeasurements`.
        """
        params = gtsam.PreintegrationCombinedParams(np.asarray([0, 0, -9.81]))
        pim = gtsam.PreintegratedCombinedMeasurements(params)

        self.assertDeepCopyEquality(pim)


if __name__ == "__main__":
    unittest.main()
