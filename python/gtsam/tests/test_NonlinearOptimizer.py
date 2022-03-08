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

import gtsam
from gtsam import (DoglegOptimizer, DoglegParams, DummyPreconditionerParameters,
                   GaussNewtonOptimizer, GaussNewtonParams, GncLMParams, GncLMOptimizer,
                   LevenbergMarquardtOptimizer, LevenbergMarquardtParams, NonlinearFactorGraph,
                   Ordering, PCGSolverParameters, Point2, PriorFactorPoint2, Values)
from gtsam.utils.test_case import GtsamTestCase

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
        cgParams.setPreconditionerParams(DummyPreconditionerParameters())
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
