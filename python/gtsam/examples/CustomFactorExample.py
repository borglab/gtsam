"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

CustomFactor demo that simulates a 1-D sensor fusion task.
Author: Fan Jiang, Frank Dellaert
"""

from functools import partial
from typing import List, Optional

import gtsam
import numpy as np

I = np.eye(1)


def simulate_car() -> List[float]:
    """Simulate a car for one second"""
    x0 = 0
    dt = 0.25  # 4 Hz, typical GPS
    v = 144 * 1000 / 3600  # 144 km/hour = 90mph, pretty fast
    x = [x0 + v * dt * i for i in range(5)]

    return x


def error_gps(measurement: np.ndarray, this: gtsam.CustomFactor,
              values: gtsam.Values,
              jacobians: Optional[List[np.ndarray]]) -> float:
    """GPS Factor error function
    :param measurement: GPS measurement, to be filled with `partial`
    :param this: gtsam.CustomFactor handle
    :param values: gtsam.Values
    :param jacobians: Optional list of Jacobians
    :return: the unwhitened error
    """
    key = this.keys()[0]
    estimate = values.atVector(key)
    error = estimate - measurement
    if jacobians is not None:
        jacobians[0] = I

    return error


def error_odom(measurement: np.ndarray, this: gtsam.CustomFactor,
               values: gtsam.Values,
               jacobians: Optional[List[np.ndarray]]) -> float:
    """Odometry Factor error function
    :param measurement: Odometry measurement, to be filled with `partial`
    :param this: gtsam.CustomFactor handle
    :param values: gtsam.Values
    :param jacobians: Optional list of Jacobians
    :return: the unwhitened error
    """
    key1 = this.keys()[0]
    key2 = this.keys()[1]
    pos1, pos2 = values.atVector(key1), values.atVector(key2)
    error = measurement - (pos1 - pos2)
    if jacobians is not None:
        jacobians[0] = I
        jacobians[1] = -I

    return error


def error_lm(measurement: np.ndarray, this: gtsam.CustomFactor,
             values: gtsam.Values,
             jacobians: Optional[List[np.ndarray]]) -> float:
    """Landmark Factor error function
    :param measurement: Landmark measurement, to be filled with `partial`
    :param this: gtsam.CustomFactor handle
    :param values: gtsam.Values
    :param jacobians: Optional list of Jacobians
    :return: the unwhitened error
    """
    key = this.keys()[0]
    pos = values.atVector(key)
    error = pos - measurement
    if jacobians is not None:
        jacobians[0] = I

    return error


def main():
    """Main runner."""

    x = simulate_car()
    print(f"Simulated car trajectory: {x}")

    add_noise = True  # set this to False to run with "perfect" measurements

    # GPS measurements
    sigma_gps = 3.0  # assume GPS is +/- 3m
    g = [
        x[k] + (np.random.normal(scale=sigma_gps) if add_noise else 0)
        for k in range(5)
    ]

    # Odometry measurements
    sigma_odo = 0.1  # assume Odometry is 10cm accurate at 4Hz
    o = [
        x[k + 1] - x[k] +
        (np.random.normal(scale=sigma_odo) if add_noise else 0)
        for k in range(4)
    ]

    # Landmark measurements:
    sigma_lm = 1  # assume landmark measurement is accurate up to 1m

    # Assume first landmark is at x=5, we measure it at time k=0
    lm_0 = 5.0
    z_0 = x[0] - lm_0 + (np.random.normal(scale=sigma_lm) if add_noise else 0)

    # Assume other landmark is at x=28, we measure it at time k=3
    lm_3 = 28.0
    z_3 = x[3] - lm_3 + (np.random.normal(scale=sigma_lm) if add_noise else 0)

    unknown = [gtsam.symbol('x', k) for k in range(5)]

    print("unknowns = ", list(map(gtsam.DefaultKeyFormatter, unknown)))

    # We now can use nonlinear factor graphs
    factor_graph = gtsam.NonlinearFactorGraph()

    # Add factors for GPS measurements
    gps_model = gtsam.noiseModel.Isotropic.Sigma(1, sigma_gps)

    # Add the GPS factors
    for k in range(5):
        gf = gtsam.CustomFactor(gps_model, [unknown[k]],
                                partial(error_gps, np.array([g[k]])))
        factor_graph.add(gf)

    # New Values container
    v = gtsam.Values()

    # Add initial estimates to the Values container
    for i in range(5):
        v.insert(unknown[i], np.array([0.0]))

    # Initialize optimizer
    params = gtsam.GaussNewtonParams()
    optimizer = gtsam.GaussNewtonOptimizer(factor_graph, v, params)

    # Optimize the factor graph
    result = optimizer.optimize()

    # calculate the error from ground truth
    error = np.array([(result.atVector(unknown[k]) - x[k])[0]
                      for k in range(5)])

    print("Result with only GPS")
    print(result, np.round(error, 2),
          f"\nJ(X)={0.5 * np.sum(np.square(error))}")

    # Adding odometry will improve things a lot
    odo_model = gtsam.noiseModel.Isotropic.Sigma(1, sigma_odo)

    for k in range(4):
        odof = gtsam.CustomFactor(odo_model, [unknown[k], unknown[k + 1]],
                                  partial(error_odom, np.array([o[k]])))
        factor_graph.add(odof)

    params = gtsam.GaussNewtonParams()
    optimizer = gtsam.GaussNewtonOptimizer(factor_graph, v, params)

    result = optimizer.optimize()

    error = np.array([(result.atVector(unknown[k]) - x[k])[0]
                      for k in range(5)])

    print("Result with GPS+Odometry")
    print(result, np.round(error, 2),
          f"\nJ(X)={0.5 * np.sum(np.square(error))}")

    # This is great, but GPS noise is still apparent, so now we add the two landmarks
    lm_model = gtsam.noiseModel.Isotropic.Sigma(1, sigma_lm)

    factor_graph.add(
        gtsam.CustomFactor(lm_model, [unknown[0]],
                           partial(error_lm, np.array([lm_0 + z_0]))))
    factor_graph.add(
        gtsam.CustomFactor(lm_model, [unknown[3]],
                           partial(error_lm, np.array([lm_3 + z_3]))))

    params = gtsam.GaussNewtonParams()
    optimizer = gtsam.GaussNewtonOptimizer(factor_graph, v, params)

    result = optimizer.optimize()

    error = np.array([(result.atVector(unknown[k]) - x[k])[0]
                      for k in range(5)])

    print("Result with GPS+Odometry+Landmark")
    print(result, np.round(error, 2),
          f"\nJ(X)={0.5 * np.sum(np.square(error))}")


if __name__ == "__main__":
    main()
