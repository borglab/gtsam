"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for IMU testing scenarios.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
"""
# pylint: disable=invalid-name, no-name-in-module

from __future__ import print_function

import unittest

from gtsam.utils.test_case import GtsamTestCase

import gtsam
from gtsam import (DoglegOptimizer, DoglegParams,
                   DummyPreconditionerParameters, GaussNewtonOptimizer,
                   GaussNewtonParams, GncLMOptimizer, GncLMParams, GncLossType,
                   LevenbergMarquardtOptimizer, LevenbergMarquardtParams,
                   NonlinearFactorGraph, Ordering, PCGSolverParameters, Point2,
                   PriorFactorPoint2, Values)

KEY1 = 1
KEY2 = 2


class TestScenario(GtsamTestCase):
    """Do trivial test with three optimizer variants."""
    def setUp(self):
        """Set up the optimization problem and ordering"""
        # create graph
        self.fg = NonlinearFactorGraph()
        model = gtsam.noiseModel.Unit.Create(2)
        self.fg.add(PriorFactorPoint2(KEY1, Point2(0, 0), model))

        # test error at minimum
        xstar = Point2(0, 0)
        self.optimal_values = Values()
        self.optimal_values.insert(KEY1, xstar)
        self.assertEqual(0.0, self.fg.error(self.optimal_values), 0.0)

        # test error at initial = [(1-cos(3))^2 + (sin(3))^2]*50 =
        x0 = Point2(3, 3)
        self.initial_values = Values()
        self.initial_values.insert(KEY1, x0)
        self.assertEqual(9.0, self.fg.error(self.initial_values), 1e-3)

        # optimize parameters
        self.ordering = Ordering()
        self.ordering.push_back(KEY1)

    def test_gauss_newton(self):
        gnParams = GaussNewtonParams()
        gnParams.setOrdering(self.ordering)
        actual = GaussNewtonOptimizer(self.fg, self.initial_values, gnParams).optimize()
        self.assertAlmostEqual(0, self.fg.error(actual))

    def test_levenberg_marquardt(self):
        lmParams = LevenbergMarquardtParams.CeresDefaults()
        lmParams.setOrdering(self.ordering)
        actual = LevenbergMarquardtOptimizer(self.fg, self.initial_values, lmParams).optimize()
        self.assertAlmostEqual(0, self.fg.error(actual))

    def test_levenberg_marquardt_pcg(self):
        lmParams = LevenbergMarquardtParams.CeresDefaults()
        lmParams.setLinearSolverType("ITERATIVE")
        cgParams = PCGSolverParameters()
        cgParams.preconditioner = DummyPreconditionerParameters()
        lmParams.setIterativeParams(cgParams)
        actual = LevenbergMarquardtOptimizer(self.fg, self.initial_values, lmParams).optimize()
        self.assertAlmostEqual(0, self.fg.error(actual))

    def test_dogleg(self):
        dlParams = DoglegParams()
        dlParams.setOrdering(self.ordering)
        actual = DoglegOptimizer(self.fg, self.initial_values, dlParams).optimize()
        self.assertAlmostEqual(0, self.fg.error(actual))

    def test_graduated_non_convexity(self):
        gncParams = GncLMParams()
        actual = GncLMOptimizer(self.fg, self.initial_values, gncParams).optimize()
        self.assertAlmostEqual(0, self.fg.error(actual))

    def test_gnc_params(self):
        base_params = LevenbergMarquardtParams()
        # Test base params
        for base_max_iters in (50, 100):
            base_params.setMaxIterations(base_max_iters)
            params = GncLMParams(base_params)
            self.assertEqual(params.baseOptimizerParams.getMaxIterations(), base_max_iters)

        # Test printing
        params_str = str(params)
        for s in (
            "lossType",
            "maxIterations",
            "muStep",
            "relativeCostTol",
            "weightsTol",
            "verbosity",
        ):
            self.assertTrue(s in params_str)

        # Test each parameter
        for loss_type in (GncLossType.TLS, GncLossType.GM):
            params.setLossType(loss_type)  # Default is TLS
            self.assertEqual(params.lossType, loss_type)
        for max_iter in (1, 10, 100):
            params.setMaxIterations(max_iter)
            self.assertEqual(params.maxIterations, max_iter)
        for mu_step in (1.1, 1.2, 1.5):
            params.setMuStep(mu_step)
            self.assertEqual(params.muStep, mu_step)
        for rel_cost_tol in (1e-5, 1e-6, 1e-7):
            params.setRelativeCostTol(rel_cost_tol)
            self.assertEqual(params.relativeCostTol, rel_cost_tol)
        for weights_tol in (1e-4, 1e-3, 1e-2):
            params.setWeightsTol(weights_tol)
            self.assertEqual(params.weightsTol, weights_tol)
        for i in (0, 1, 2):
            verb = GncLMParams.Verbosity(i)
            params.setVerbosityGNC(verb)
            self.assertEqual(params.verbosity, verb)
        for inl in ([], [10], [0, 100]):
            params.setKnownInliers(inl)
            self.assertEqual(params.knownInliers, inl)
            params.knownInliers = []
        for out in ([], [1], [0, 10]):
            params.setKnownInliers(out)
            self.assertEqual(params.knownInliers, out)
            params.knownInliers = []

        # Test optimizer params
        optimizer = GncLMOptimizer(self.fg, self.initial_values, params)
        for ict_factor in (0.9, 1.1):
            new_ict = ict_factor * optimizer.getInlierCostThresholds().item()
            optimizer.setInlierCostThresholds(new_ict)
            self.assertAlmostEqual(optimizer.getInlierCostThresholds(), new_ict)
        for w_factor in (0.8, 0.9):
            new_weights = w_factor * optimizer.getWeights()
            optimizer.setWeights(new_weights)
            self.assertAlmostEqual(optimizer.getWeights(), new_weights)
        optimizer.setInlierCostThresholdsAtProbability(0.9)
        w1 = optimizer.getInlierCostThresholds()
        optimizer.setInlierCostThresholdsAtProbability(0.8)
        w2 = optimizer.getInlierCostThresholds()
        self.assertLess(w2, w1)

    def test_iteration_hook(self):
        # set up iteration hook to track some testable values
        iteration_count = 0
        final_error = 0
        final_values = None

        def iteration_hook(iter, error_before, error_after):
            nonlocal iteration_count, final_error, final_values
            iteration_count = iter
            final_error = error_after
            final_values = optimizer.values()

        # optimize
        params = LevenbergMarquardtParams.CeresDefaults()
        params.setOrdering(self.ordering)
        params.iterationHook = iteration_hook
        optimizer = LevenbergMarquardtOptimizer(self.fg, self.initial_values, params)
        actual = optimizer.optimize()
        self.assertAlmostEqual(0, self.fg.error(actual))
        self.gtsamAssertEquals(final_values, actual)
        self.assertEqual(self.fg.error(actual), final_error)
        self.assertEqual(optimizer.iterations(), iteration_count)


if __name__ == "__main__":
    unittest.main()
