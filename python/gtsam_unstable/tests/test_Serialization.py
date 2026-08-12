"""Regression tests for serialization in the unstable Python extension."""

import pickle
import unittest

import numpy as np

import gtsam
import gtsam_unstable
from gtsam.utils.test_case import GtsamTestCase


class TestSerialization(GtsamTestCase):

    @staticmethod
    def _factor(noise_model):
        return gtsam_unstable.ProjectionFactorPPPCal3_S2(
            np.ones(2), noise_model, 0, 1, 2, gtsam.Cal3_S2())

    @unittest.skipUnless(
        hasattr(gtsam_unstable.ProjectionFactorPPPCal3_S2, "serialize"),
        "Serialization not enabled")
    def test_projection_factor_noise_models(self):
        """Unstable factors serialize every supported noise-model family."""
        noise_models = {
            # This is the original issue #1173 reproduction. Smart construction
            # returns a Unit noise model for a vector of unit sigmas.
            "unit": gtsam.noiseModel.Isotropic.Sigmas(np.ones(2)),
            "isotropic": gtsam.noiseModel.Isotropic.Sigma(2, 0.5, False),
            "diagonal": gtsam.noiseModel.Diagonal.Sigmas(
                np.array([0.5, 1.5]), False),
            "gaussian": gtsam.noiseModel.Gaussian.SqrtInformation(
                np.array([[1.0, 0.2], [0.0, 1.0]]), False),
            "constrained": gtsam.noiseModel.Constrained.MixedSigmas(
                np.ones(2), np.array([0.0, 0.5])),
            "robust": gtsam.noiseModel.Robust.Create(
                gtsam.noiseModel.mEstimator.Huber.Create(1.345),
                gtsam.noiseModel.Isotropic.Sigma(2, 0.5, False)),
        }

        for name, noise_model in noise_models.items():
            with self.subTest(noise_model=name):
                factor = self._factor(noise_model)
                self.assertIsInstance(factor.serialize(), str)

                restored = pickle.loads(pickle.dumps(factor))
                self.assertTrue(factor.equals(restored, 1e-9))


if __name__ == "__main__":
    unittest.main()
