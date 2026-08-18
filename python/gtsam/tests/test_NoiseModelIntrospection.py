"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for noise model introspection exposed on noiseModel::Base
and for the accessors on noiseModel::Robust.
"""

import unittest

import numpy as np
from gtsam.utils.test_case import GtsamTestCase

import gtsam


class TestNoiseModelIntrospection(GtsamTestCase):
    """Tests for the introspection methods declared on noiseModel::Base."""

    def test_dim_and_flags(self):
        """dim/isUnit/isConstrained are reachable from any noise model."""
        diagonal = gtsam.noiseModel.Diagonal.Sigmas(np.array([1.0, 2.0, 3.0]))
        self.assertEqual(diagonal.dim(), 3)
        self.assertFalse(diagonal.isUnit())
        self.assertFalse(diagonal.isConstrained())

        unit = gtsam.noiseModel.Unit.Create(2)
        self.assertEqual(unit.dim(), 2)
        self.assertTrue(unit.isUnit())

        constrained = gtsam.noiseModel.Constrained.All(2)
        self.assertTrue(constrained.isConstrained())

    def test_sigmas_on_base(self):
        """sigmas() is virtual on Base and resolves to the derived model."""
        sigmas = np.array([0.5, 4.0])
        model = gtsam.noiseModel.Diagonal.Sigmas(sigmas)
        np.testing.assert_allclose(model.sigmas(), sigmas)

    def test_mahalanobis_matches_manual(self):
        """squaredMahalanobisDistance agrees with the hand computation."""
        sigmas = np.array([1.0, 2.0])
        model = gtsam.noiseModel.Diagonal.Sigmas(sigmas)
        v = np.array([3.0, 4.0])
        expected = np.sum((v / sigmas) ** 2)
        self.assertAlmostEqual(model.squaredMahalanobisDistance(v), expected)
        self.assertAlmostEqual(model.mahalanobisDistance(v), np.sqrt(expected))

    def test_robust_is_not_reweighted(self):
        """A Robust model reports the *underlying* Mahalanobis distance.

        This is the property that makes chi-square style gating possible on
        robust factors: whiten() applies the m-estimator weight, but
        squaredMahalanobisDistance and unweightedWhiten do not.
        """
        sigmas = np.array([1.0])
        base = gtsam.noiseModel.Diagonal.Sigmas(sigmas)
        robust = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber.Create(1.345), base)

        outlier = np.array([50.0])

        # The un-reweighted quantities match the underlying Gaussian exactly.
        self.assertAlmostEqual(robust.squaredMahalanobisDistance(outlier),
                               base.squaredMahalanobisDistance(outlier))
        np.testing.assert_allclose(robust.unweightedWhiten(outlier),
                                   base.whiten(outlier))

        # whiten(), by contrast, is down-weighted by the m-estimator.
        self.assertLess(abs(robust.whiten(outlier)[0]), abs(base.whiten(outlier)[0]))

        # And the weight itself is now observable.
        self.assertLess(robust.weight(outlier), 1.0)
        self.assertAlmostEqual(base.weight(outlier), 1.0)

    def test_robust_dim_delegates(self):
        """Robust reports the dimension of the model it wraps."""
        base = gtsam.noiseModel.Isotropic.Sigma(3, 2.0)
        robust = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber.Create(1.345), base)
        self.assertEqual(robust.dim(), 3)

    def test_sigmas_unsupported_raises_cleanly(self):
        """Robust has no sigmas of its own, and says so intelligibly.

        Base::sigmas() is the fallback for models that cannot answer. It
        must surface as a Python exception carrying its message, not as
        pybind's unknown-exception catch-all.
        """
        robust = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber.Create(1.345),
            gtsam.noiseModel.Isotropic.Sigma(1, 1.0))

        with self.assertRaises(RuntimeError) as ctx:
            robust.sigmas()
        self.assertIn("not implemented", str(ctx.exception))


class TestRobustAccessors(GtsamTestCase):
    """Tests for noiseModel::Robust::robust() and ::noise()."""

    def test_noise_accessor_round_trip(self):
        """noise() returns the model that was passed to Create()."""
        sigmas = np.array([0.1, 0.2, 0.3])
        base = gtsam.noiseModel.Diagonal.Sigmas(sigmas)
        robust = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber.Create(1.345), base)

        recovered = robust.noise()
        self.assertEqual(recovered.dim(), 3)
        np.testing.assert_allclose(recovered.sigmas(), sigmas)

    def test_robust_accessor(self):
        """robust() returns the contained m-estimator."""
        loss = gtsam.noiseModel.mEstimator.Huber.Create(1.345)
        robust = gtsam.noiseModel.Robust.Create(
            loss, gtsam.noiseModel.Isotropic.Sigma(1, 1.0))
        self.assertAlmostEqual(robust.robust().weight(0.5), 1.0)

    def test_reachable_from_factor(self):
        """The chain factor -> noiseModel() -> noise() works end to end."""
        sigmas = np.array([0.5])
        base = gtsam.noiseModel.Diagonal.Sigmas(sigmas)
        robust = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber.Create(1.345), base)
        factor = gtsam.PriorFactorDouble(0, 1.0, robust)

        model = factor.noiseModel()
        self.assertEqual(model.dim(), 1)
        np.testing.assert_allclose(model.noise().sigmas(), sigmas)


if __name__ == "__main__":
    unittest.main()
