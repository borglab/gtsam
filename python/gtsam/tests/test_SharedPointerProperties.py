"""Compatibility tests for shared-pointer-backed Python properties."""

import unittest

import gtsam


class TestSharedPointerProperties(unittest.TestCase):
    def test_triangulation_noise_model_property(self):
        params = gtsam.TriangulationParameters()
        model = gtsam.noiseModel.Isotropic.Sigma(2, 0.5)

        params.noiseModel = model

        self.assertIsInstance(params.noiseModel, gtsam.noiseModel.Base)

    def test_pcg_preconditioner_property(self):
        params = gtsam.PCGSolverParameters()
        preconditioner = gtsam.DummyPreconditionerParameters()

        params.preconditioner = preconditioner

        self.assertIsInstance(
            params.preconditioner, gtsam.DummyPreconditionerParameters
        )

    def test_legged_preintegration_params_property(self):
        params = gtsam.LeggedEstimatorParams()
        preintegration = gtsam.PreintegrationParams.MakeSharedU(9.81)

        params.preintegrationParams = preintegration

        self.assertIsInstance(
            params.preintegrationParams, gtsam.PreintegrationParams
        )


if __name__ == "__main__":
    unittest.main()
