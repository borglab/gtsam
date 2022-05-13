"""
Unit tests for optimization that logs to comet.ml.
Author: Jing Wu and Frank Dellaert
"""
# pylint: disable=invalid-name

import sys
if sys.version_info.major >= 3:
    from io import StringIO
else:
    from cStringIO import StringIO

import unittest
from datetime import datetime

import gtsam
import numpy as np
from gtsam import Rot3
from gtsam.utils.test_case import GtsamTestCase

from gtsam.utils.logging_optimizer import gtsam_optimize, optimize_using

KEY = 0
MODEL = gtsam.noiseModel.Unit.Create(3)


class TestOptimizeComet(GtsamTestCase):
    """Check correct logging to comet.ml."""

    def setUp(self):
        """Set up a small Karcher mean optimization example."""
        # Grabbed from KarcherMeanFactor unit tests.
        R = Rot3.Expmap(np.array([0.1, 0, 0]))
        rotations = {R, R.inverse()}  # mean is the identity
        self.expected = Rot3()

        def check(actual):
            # Check that optimizing yields the identity
            self.gtsamAssertEquals(actual.atRot3(KEY), self.expected, tol=1e-6)
            # Check that logging output prints out 3 lines (exact intermediate values differ by OS)
            self.assertEqual(self.capturedOutput.getvalue().count('\n'), 3)
            # reset stdout catcher
            self.capturedOutput.truncate(0)
        self.check = check

        self.graph = gtsam.NonlinearFactorGraph()
        for R in rotations:
            self.graph.add(gtsam.PriorFactorRot3(KEY, R, MODEL))
        self.initial = gtsam.Values()
        self.initial.insert(KEY, R)

        # setup output capture
        self.capturedOutput = StringIO()
        sys.stdout = self.capturedOutput

    def tearDown(self):
        """Reset print capture."""
        sys.stdout = sys.__stdout__

    def test_simple_printing(self):
        """Test with a simple hook."""

        # Provide a hook that just prints
        def hook(_, error):
            print(error)

        # Wrapper function sets the hook and calls optimizer.optimize() for us.
        params = gtsam.GaussNewtonParams()
        actual = optimize_using(gtsam.GaussNewtonOptimizer, hook, self.graph, self.initial)
        self.check(actual)
        actual = optimize_using(gtsam.GaussNewtonOptimizer, hook, self.graph, self.initial, params)
        self.check(actual)
        actual = gtsam_optimize(gtsam.GaussNewtonOptimizer(self.graph, self.initial, params),
                                params, hook)
        self.check(actual)

    def test_lm_simple_printing(self):
        """Make sure we are properly terminating LM"""
        def hook(_, error):
            print(error)

        params = gtsam.LevenbergMarquardtParams()
        actual = optimize_using(gtsam.LevenbergMarquardtOptimizer, hook, self.graph, self.initial)
        self.check(actual)
        actual = optimize_using(gtsam.LevenbergMarquardtOptimizer, hook, self.graph, self.initial,
                                params)
        self.check(actual)
        actual = gtsam_optimize(gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial, params),
                                params, hook)

    @unittest.skip("Not a test we want run every time, as needs comet.ml account")
    def test_comet(self):
        """Test with a comet hook."""
        from comet_ml import Experiment
        comet = Experiment(project_name="Testing",
                           auto_output_logging="native")
        comet.log_dataset_info(name="Karcher", path="shonan")
        comet.add_tag("GaussNewton")
        comet.log_parameter("method", "GaussNewton")
        time = datetime.now()
        comet.set_name("GaussNewton-" + str(time.month) + "/" + str(time.day) + " "
                       + str(time.hour)+":"+str(time.minute)+":"+str(time.second))

        # I want to do some comet thing here
        def hook(optimizer, error):
            comet.log_metric("Karcher error",
                             error, optimizer.iterations())

        gtsam_optimize(self.optimizer, self.params, hook)
        comet.end()

        actual = self.optimizer.values()
        self.gtsamAssertEquals(actual.atRot3(KEY), self.expected)

if __name__ == "__main__":
    unittest.main()
