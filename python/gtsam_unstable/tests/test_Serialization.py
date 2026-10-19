"""Regression tests for serialization in the unstable Python extension."""

import pickle
from pathlib import Path
import subprocess
import sys
import tempfile
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
    def test_import_orders(self):
        """Each extension import order supports all noise-model round trips."""
        script = """
import importlib
import sys
import unittest
sys.path[:0] = sys.argv[1:3]
for module in sys.argv[3:]:
    importlib.import_module(module)
suite = unittest.defaultTestLoader.loadTestsFromName(
    'gtsam_unstable.tests.test_Serialization.TestSerialization.'
    'test_projection_factor_noise_models')
result = unittest.TextTestRunner().run(suite)
sys.exit(not result.wasSuccessful())
"""
        roots = [str(Path(module.__file__).resolve().parents[1])
                 for module in (gtsam, gtsam_unstable)]
        for order in (("gtsam", "gtsam_unstable"),
                      ("gtsam_unstable", "gtsam")):
            with self.subTest(import_order=order):
                with tempfile.TemporaryDirectory() as directory:
                    result = subprocess.run(
                        [sys.executable, "-I", "-c", script, *roots, *order],
                        cwd=directory, capture_output=True, text=True,
                        timeout=60)
                self.assertEqual(result.returncode, 0,
                                 result.stdout + result.stderr)

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
