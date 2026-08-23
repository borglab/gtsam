"""Tests for writable output parameters exposed by the pybind wrappers."""

import numpy as np

import gtsam
from gtsam.symbol_shorthand import X


def test_dynamic_matrix_outputs_and_omitted_defaults():
    """Dynamic Matrix pointers accept None defaults and copy results back."""
    factor = gtsam.NonlinearEquality2Point2(1, 2)
    x1 = np.array([1.0, 2.0])
    x2 = np.array([3.0, 5.0])
    expected = factor.evaluateError(x1, x2)
    H1 = np.zeros((2, 2))
    H2 = np.zeros((2, 2))

    actual = factor.evaluateError(x1, x2, H1, H2)

    np.testing.assert_allclose(actual, expected)
    np.testing.assert_allclose(H1, -np.eye(2))
    np.testing.assert_allclose(H2, np.eye(2))


def test_fixed_optional_jacobian_outputs_and_omitted_defaults():
    """Fixed OptionalJacobian arguments retain their C++ defaults."""
    pose = gtsam.Pose2(1.0, 2.0, 0.3)
    point = np.array([4.0, 6.0])
    expected = gtsam.BearingRange2D.Measure(pose, point)
    H1 = np.zeros((2, 3))
    H2 = np.zeros((2, 2))

    actual = gtsam.BearingRange2D.Measure(pose, point, H1, H2)

    assert actual.bearing().equals(expected.bearing(), 1e-12)
    assert actual.range() == expected.range()
    assert np.any(H1)
    assert np.any(H2)


def test_raw_matrix_output_and_omitted_default():
    """A raw Matrix output works without changing the old zero-argument call."""
    density = gtsam.ConcentratedGaussianPoint2(
        1, np.array([1.0, 2.0]), np.array([0.2, -0.1]), np.eye(2)
    )
    expected = density.retractMean()
    H = np.zeros((2, 2))

    actual = density.retractMean(H)

    np.testing.assert_allclose(actual, expected)
    np.testing.assert_allclose(H, np.eye(2))


def test_print_errors_callback_and_default():
    """The formerly hidden predicate is optional and accepts Python callables."""
    graph = gtsam.NonlinearFactorGraph()
    graph.add(
        gtsam.PriorFactorPose2(
            X(0), gtsam.Pose2(), gtsam.noiseModel.Isotropic.Sigma(3, 1.0)
        )
    )
    values = gtsam.Values()
    values.insert(X(0), gtsam.Pose2())
    seen = []

    graph.printErrors(values)
    graph.printErrors(
        values,
        "",
        gtsam.DefaultKeyFormatter,
        lambda factor, error, index: seen.append((factor, error, index)) or True,
    )

    assert len(seen) == 1
    assert isinstance(seen[0][0], gtsam.PriorFactorPose2)
    assert seen[0][1:] == (0.0, 0)
