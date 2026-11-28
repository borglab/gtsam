"""Tests for manifold-generic ILQRTracking and DDPTracking Python bindings."""
import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestILQRAndDDPTracking(GtsamTestCase):
  """Verify ILQRTracking and DDPTracking across Vector, Pose2, and Unit3."""

  def test_vector_linear_quadratic(self):
    """One-step linear-quadratic problem matches closed-form control and gain."""
    x_keys = [gtsam.symbol('x', 0), gtsam.symbol('x', 1)]
    u_keys = [gtsam.symbol('u', 0)]
    graph = gtsam.NonlinearFactorGraph()
    graph.addPriorVector(u_keys[0], np.zeros(1), gtsam.noiseModel.Unit.Create(1))
    graph.addPriorVector(
        x_keys[1], np.array([2.0]), gtsam.noiseModel.Isotropic.Precision(1, 3.0))
    init = gtsam.Values()
    init.insert(x_keys[0], np.zeros(1))
    init.insert(u_keys[0], np.zeros(1))
    step = lambda x, u: x + u

    ilqr = gtsam.ILQRTrackingVector(x_keys, u_keys)
    plan_ilqr = ilqr.optimize(graph, init, step, 2)
    ddp = gtsam.DDPTrackingVector(x_keys, u_keys)
    plan_ddp = ddp.optimize(graph, init, step, 2)

    np.testing.assert_allclose(plan_ilqr.atVector(u_keys[0]), [1.5], atol=1e-6)
    np.testing.assert_allclose(
        plan_ilqr.atVector(u_keys[0]), plan_ddp.atVector(u_keys[0]), atol=1e-6)
    np.testing.assert_allclose(plan_ilqr.atVector(x_keys[1]), [1.5], atol=1e-6)
    np.testing.assert_allclose(ilqr.gain(0), [[-0.75]], atol=1e-6)
    self.assertEqual(ilqr.policy().size(), 1)
    self.assertEqual(ddp.policy().size(), 1)

  def test_pose2_tracking(self):
    """ILQRTrackingPose2 and DDPTrackingPose2 agree on a Pose2 factor graph."""
    horizon, dt = 10, 0.1
    x_keys = [gtsam.symbol('x', k) for k in range(horizon + 1)]
    u_keys = [gtsam.symbol('u', k) for k in range(horizon)]
    goal = gtsam.Pose2(1.0, 0.2, 0.3)
    field = lambda _x, u: np.array([u[0], 0.0, u[1]])

    graph = gtsam.NonlinearFactorGraph()
    init = gtsam.Values()
    init.insert(x_keys[0], gtsam.Pose2())
    q_noise = gtsam.noiseModel.Diagonal.Precisions(np.array([4.0, 4.0, 2.0]))
    r_noise = gtsam.noiseModel.Diagonal.Precisions(np.array([0.1, 0.1]))
    t_noise = gtsam.noiseModel.Diagonal.Precisions(np.array([100.0, 100.0, 50.0]))
    for k in range(horizon):
      init.insert(u_keys[k], np.zeros(2))
      graph.addPriorPose2(x_keys[k], goal, q_noise)
      graph.addPriorVector(u_keys[k], np.zeros(2), r_noise)
    graph.addPriorPose2(x_keys[-1], goal, t_noise)

    ilqr = gtsam.ILQRTrackingPose2(x_keys, u_keys)
    plan_ilqr = ilqr.optimize(graph, init, field, dt, 25)
    ddp = gtsam.DDPTrackingPose2(x_keys, u_keys)
    plan_ddp = ddp.optimize(graph, init, field, dt, 25)

    self.assertEqual(ilqr.policy().size(), horizon)
    self.assertEqual(ddp.policy().size(), horizon)
    for k in range(horizon):
      np.testing.assert_allclose(
          plan_ilqr.atVector(u_keys[k]), plan_ddp.atVector(u_keys[k]), atol=1e-3)
    err = np.linalg.norm(
        gtsam.Pose2.Logmap(goal.between(plan_ilqr.atPose2(x_keys[-1]))))
    self.assertLess(err, 0.05)

  def test_unit3_tracking(self):
    """ILQRTrackingUnit3 and DDPTrackingUnit3 agree on S^2."""
    horizon, dt = 10, 0.1
    x_keys = [gtsam.symbol('x', k) for k in range(horizon + 1)]
    u_keys = [gtsam.symbol('u', k) for k in range(horizon)]
    start = gtsam.Unit3(np.array([1.0, 0.0, 0.0]))
    goal = gtsam.Unit3(np.array([0.0, 0.0, 1.0]))
    field = lambda _x, u: u

    graph = gtsam.NonlinearFactorGraph()
    init = gtsam.Values()
    init.insert(x_keys[0], start)
    q_noise = gtsam.noiseModel.Unit.Create(2)
    r_noise = gtsam.noiseModel.Isotropic.Precision(2, 0.1)
    t_noise = gtsam.noiseModel.Isotropic.Precision(2, 50.0)
    for k in range(horizon):
      init.insert(u_keys[k], np.zeros(2))
      graph.add(gtsam.PriorFactorUnit3(x_keys[k], goal, q_noise))
      graph.addPriorVector(u_keys[k], np.zeros(2), r_noise)
    graph.add(gtsam.PriorFactorUnit3(x_keys[-1], goal, t_noise))

    ilqr = gtsam.ILQRTrackingUnit3(x_keys, u_keys)
    plan_ilqr = ilqr.optimize(graph, init, field, dt, 15)
    ddp = gtsam.DDPTrackingUnit3(x_keys, u_keys)
    plan_ddp = ddp.optimize(graph, init, field, dt, 15)

    for k in range(horizon):
      np.testing.assert_allclose(
          plan_ilqr.atVector(u_keys[k]), plan_ddp.atVector(u_keys[k]), atol=1e-5)
    self.assertLess(
        np.linalg.norm(goal.localCoordinates(plan_ilqr.atUnit3(x_keys[-1]))),
        0.02)


if __name__ == '__main__':
  unittest.main()
