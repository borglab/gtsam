# pylint: disable=invalid-name,no-name-in-module,no-member,missing-class-docstring,missing-function-docstring

"""
SEGV when unconnected old variable is pruned.

The L0 variable below has only a prior.  All the other variables are associated
with other factors ("Between" in this case).  When it's time to marginalize the
unconnected L0 variable, isam.update() segfaults.

joel@truher.org
"""

import unittest
import numpy as np
import gtsam  # type:ignore
import gtsam_unstable  # type:ignore
from gtsam.symbol_shorthand import X, L  # type:ignore


class TestSegvOldVariable(unittest.TestCase):

    def test_segv(self):
        NOISE2 = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1]))
        NOISE3 = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))
        DX = gtsam.Pose2(1, 0, 0)

        isam = gtsam_unstable.IncrementalFixedLagSmoother(2)

        # ======= L0 has only a prior, timestamp zero =======

        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()
        graph.add(gtsam.PriorFactorPoint2(L(0), (0, 0), NOISE2))
        values.insert(L(0), gtsam.Point2(0, 0))
        timestamps.insert((L(0), 0))
        isam.update(graph, values, timestamps)

        # ======= X0 has a prior, timestamp zero =======

        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()
        graph.add(gtsam.PriorFactorPose2(X(0), gtsam.Pose2(0, 0, 0), NOISE3))
        values.insert(X(0), gtsam.Pose2(0, 0, 0))
        timestamps.insert((X(0), 0))
        isam.update(graph, values, timestamps)

        # ======= X1 has odometry from X0, timestamp 1 =======

        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()
        graph.add(gtsam.BetweenFactorPose2(X(0), X(1), DX, NOISE3))
        values.insert(X(1), gtsam.Pose2(1, 0, 0))
        timestamps.insert((X(1), 1))
        isam.update(graph, values, timestamps)

        # ======= X2 has odometry from X1, timestamp 2 =======

        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()
        graph.add(gtsam.BetweenFactorPose2(X(1), X(2), DX, NOISE3))
        values.insert(X(2), gtsam.Pose2(2, 0, 0))
        timestamps.insert((X(2), 2))
        isam.update(graph, values, timestamps)

        # ======= X3 has odometry from X2, timestamp 3 =======

        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()
        graph.add(gtsam.BetweenFactorPose2(X(2), X(3), DX, NOISE3))
        values.insert(X(3), gtsam.Pose2(3, 0, 0))
        timestamps.insert((X(3), 3))

        ##################################
        #
        # this line segfaults:
        #
        isam.update(graph, values, timestamps)


if __name__ == "__main__":
    unittest.main()
