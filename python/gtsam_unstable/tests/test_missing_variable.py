# pylint: disable=invalid-name,no-name-in-module,no-member,missing-class-docstring,missing-function-docstring

"""
Demonstrates incorrect pruning of old variables.

It seems like if you call isam.update() with two separate updates for the same
timestamp, eventually calculateEstimate() gets confused about which variables
still need to be marginalized.

joel@truher.org
"""

import unittest
import numpy as np
import gtsam  # type:ignore
import gtsam_unstable  # type:ignore
from gtsam.symbol_shorthand import X, L  # type:ignore


class TestMissingVariable(unittest.TestCase):

    def test_missing_variable(self):

        NOISE2 = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1]))
        NOISE3 = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))
        DX = gtsam.Pose2(1, 0, 0)
        ROT_180 = gtsam.Rot2.fromDegrees(180)

        isam = gtsam_unstable.IncrementalFixedLagSmoother(8)

        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()

        # ======= L0 has a prior, timestamp 0 =======
        graph.add(gtsam.PriorFactorPoint2(L(0), (0, 0), NOISE2))
        values.insert(L(0), gtsam.Point2(0, 0))
        timestamps.insert((L(0), 0))

        # ======= X0 has a prior, timestamp 0 =======
        graph.add(gtsam.PriorFactorPose2(X(0), gtsam.Pose2(0, 0, 0), NOISE3))
        values.insert(X(0), gtsam.Pose2(0, 0, 0))
        timestamps.insert((X(0), 0))

        isam.update(graph, values, timestamps)

        ###############################
        #
        # Set this to true to cause the failure
        #
        PLEASE_FAIL = True

        def odo(i):
            _graph = gtsam.NonlinearFactorGraph()
            _values = gtsam.Values()
            _timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()

            # odometry for time i
            _graph.add(gtsam.BetweenFactorPose2(X(i - 1), X(i), DX, NOISE3))
            _values.insert(X(i), gtsam.Pose2(i, 0, 0))
            _timestamps.insert((X(i), i))

            ###############################
            #
            # two separate updates for the same timestamp => eventual fail
            #
            if PLEASE_FAIL:
                isam.update(_graph, _values, _timestamps)
                _graph = gtsam.NonlinearFactorGraph()
                _values = gtsam.Values()
                _timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()

            # landmark sight for time i
            _graph.add(gtsam.BearingRangeFactor2D(X(i), L(0), ROT_180, i, NOISE2))
            _timestamps.insert((L(0), i))
            isam.update(_graph, _values, _timestamps)

            ###############################
            #
            # if PLEASE_FAIL is True, then this line fails with the message,
            # "IndexError: Requested variable 'x3' is not in this VectorValues."
            #
            isam.calculateEstimate()

        for ii in range(1, 20):
            odo(ii)


if __name__ == "__main__":
    unittest.main()
