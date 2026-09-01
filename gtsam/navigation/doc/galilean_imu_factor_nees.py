"""Numerical helpers for the Galilean IMU factor NEES notebook.

This module deliberately contains the mechanical Monte Carlo, tabulation, and
Plotly code so that GalileanImuFactorNEES.ipynb can focus on assumptions,
scenarios, and conclusions. It is documentation support code, not public API.
"""

from __future__ import annotations

import gtsam
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from scipy.stats import chi2


BACKENDS = {
    "Manifold": gtsam.PreintegratedImuMeasurementsManifold,
    "Tangent": gtsam.PreintegratedImuMeasurementsTangent,
    "Lie group": gtsam.PreintegratedImuMeasurementsLieGroup,
    "Galilean": gtsam.PreintegratedImuMeasurementsG,
}

COMBINED_BACKENDS = {
    "Manifold": gtsam.PreintegratedCombinedMeasurementsManifold,
    "Tangent": gtsam.PreintegratedCombinedMeasurements,
    "Lie group": gtsam.PreintegratedCombinedMeasurementsLieGroup,
    "Galilean": gtsam.PreintegratedCombinedMeasurementsG,
}

BACKEND_STYLES = {
    "Manifold": ("#777777", "solid"),
    "Tangent": ("#999999", "dash"),
    "Lie group": ("#bbbbbb", "dot"),
    "Galilean": ("#14866d", "solid"),
}


def assert_logmap_default() -> None:
    """Verify that fresh standard and Combined parameters select Logmap."""
    standard = gtsam.PreintegrationParams(np.zeros(3))
    combined = gtsam.PreintegrationCombinedParams(np.zeros(3))
    assert standard.getImuFactorErrorMode() == gtsam.ImuFactorErrorMode.Logmap
    assert combined.getImuFactorErrorMode() == gtsam.ImuFactorErrorMode.Logmap


def make_params(accelerometer_sigmas, gyroscope_sigmas, gravity=None, omega=None):
    """Create standard preintegration parameters from continuous-time sigmas."""
    gravity = np.zeros(3) if gravity is None else np.asarray(gravity)
    params = gtsam.PreintegrationParams(gravity)
    params.setAccelerometerCovariance(np.diag(np.asarray(accelerometer_sigmas) ** 2))
    params.setGyroscopeCovariance(np.diag(np.asarray(gyroscope_sigmas) ** 2))
    params.setIntegrationCovariance(np.zeros((3, 3)))
    if omega is not None:
        params.setOmegaCoriolis(np.asarray(omega))
    return params


def make_combined_params(
    accelerometer_sigmas,
    gyroscope_sigmas,
    bias_accelerometer_rw_sigmas,
    bias_gyroscope_rw_sigmas,
):
    """Create Combined parameters from sensor and bias continuous-time sigmas."""
    params = gtsam.PreintegrationCombinedParams(np.zeros(3))
    params.setAccelerometerCovariance(np.diag(np.asarray(accelerometer_sigmas) ** 2))
    params.setGyroscopeCovariance(np.diag(np.asarray(gyroscope_sigmas) ** 2))
    params.setIntegrationCovariance(np.zeros((3, 3)))
    params.setBiasAccCovariance(np.diag(np.asarray(bias_accelerometer_rw_sigmas) ** 2))
    params.setBiasOmegaCovariance(np.diag(np.asarray(bias_gyroscope_rw_sigmas) ** 2))
    return params


def integrate(
    backend_type,
    accelerations,
    angular_velocities,
    dt,
    params,
    bias=None,
):
    """Integrate one paired sequence with a selected PIM backend."""
    bias = gtsam.imuBias.ConstantBias() if bias is None else bias
    pim = backend_type(params, bias)
    for acceleration, angular_velocity in zip(accelerations, angular_velocities):
        pim.integrateMeasurement(acceleration, angular_velocity, dt)
    return pim


def logmap_error(state_j, predicted_state_j):
    """Return the IMU factor convention: state_j.logmap(predictedState_j)."""
    return np.asarray(state_j.logmap(predicted_state_j))


def physical_error(state_j, predicted_state_j):
    """Return rotation-local and navigation-frame position/velocity errors."""
    rotation = gtsam.Rot3.Logmap(
        state_j.attitude().between(predicted_state_j.attitude())
    )
    return np.concatenate(
        (
            rotation,
            predicted_state_j.position() - state_j.position(),
            predicted_state_j.velocity() - state_j.velocity(),
        )
    )


def exact_held_input_state(acceleration, angular_velocity, duration):
    """Evaluate the exact Galilean exponential for one held IMU input."""
    tangent = np.zeros(10)
    tangent[:3] = np.asarray(angular_velocity) * duration
    tangent[3:6] = np.asarray(acceleration) * duration
    tangent[9] = duration
    delta = gtsam.Gal3.Expmap(tangent)
    return gtsam.NavState(delta.rotation(), delta.translation(), delta.velocity())


def _skew(vector):
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def exact_rotating_state(
    state_i, acceleration, angular_velocity, duration, gravity, omega
):
    """Evaluate the paper's exact constant-rate rotating-frame endpoint."""
    tangent = np.zeros(10)
    tangent[:3] = np.asarray(angular_velocity) * duration
    tangent[3:6] = np.asarray(acceleration) * duration
    tangent[9] = duration
    delta = gtsam.Gal3.Expmap(tangent)

    theta = -np.asarray(omega) * duration
    kernels = gtsam.so3.DexpFunctor(theta)
    earth_rotation = gtsam.Rot3.Expmap(theta).matrix()
    gamma_velocity = kernels.leftJacobian()
    gamma_position = gamma_velocity - kernels.Gamma().left()

    omega_cross = _skew(omega)
    rotation_i = state_i.attitude().matrix()
    position_i = state_i.position()
    transported_velocity_i = state_i.velocity() + omega_cross @ position_i
    position_j = gamma_position @ gravity * duration**2 + earth_rotation @ (
        position_i
        + transported_velocity_i * duration
        + rotation_i @ delta.translation()
    )
    transported_velocity_j = gamma_velocity @ gravity * duration + earth_rotation @ (
        transported_velocity_i + rotation_i @ delta.velocity()
    )
    velocity_j = transported_velocity_j - omega_cross @ position_j
    rotation_j = earth_rotation @ rotation_i @ delta.rotation().matrix()
    return gtsam.NavState(gtsam.Rot3(rotation_j), position_j, velocity_j)


def deterministic_convergence(
    acceleration, angular_velocity, duration, sample_periods, params, state_i
):
    """Measure deterministic endpoint errors as the sample period changes."""
    truth = exact_held_input_state(acceleration, angular_velocity, duration)
    result = {name: {"position": [], "velocity": []} for name in BACKENDS}
    for dt in sample_periods:
        steps = round(duration / dt)
        accelerations = np.tile(acceleration, (steps, 1))
        angular_velocities = np.tile(angular_velocity, (steps, 1))
        for name, backend_type in BACKENDS.items():
            predicted = integrate(
                backend_type, accelerations, angular_velocities, dt, params
            ).predict(state_i, gtsam.imuBias.ConstantBias())
            error = physical_error(truth, predicted)
            result[name]["position"].append(np.linalg.norm(error[3:6]))
            result[name]["velocity"].append(np.linalg.norm(error[6:9]))
    return truth, result


def _mean_interval(trials, dimension):
    return chi2.ppf([0.025, 0.975], trials * dimension) / trials


def _summary(errors, nees):
    result = {}
    for name in BACKENDS:
        values = nees[name]
        result[name] = {
            "mean_nees": values.mean(),
            "median_nees": np.median(values),
            "mean_error_norm": np.linalg.norm(errors[name].mean(axis=0)),
            "mean_half_width": 1.96 * values.std(ddof=1) / np.sqrt(len(values)),
        }
    return result


def _physical_summary(errors):
    return {
        name: {
            "position_rmse": np.sqrt(np.mean(np.sum(values[:, 3:6] ** 2, axis=1))),
            "velocity_rmse": np.sqrt(np.mean(np.sum(values[:, 6:9] ** 2, axis=1))),
        }
        for name, values in errors.items()
    }


def run_inertial_nees(
    acceleration,
    angular_velocity,
    duration,
    dt,
    trials,
    seed,
    accelerometer_sigmas,
    gyroscope_sigmas,
    params,
    state_i,
    truth,
):
    """Run paired Logmap NEES trials for one held inertial input."""
    steps = round(duration / dt)
    rng = np.random.default_rng(seed)
    errors = {name: np.empty((trials, 9)) for name in BACKENDS}
    physical = {name: np.empty((trials, 9)) for name in BACKENDS}
    nees = {name: np.empty(trials) for name in BACKENDS}
    bias = gtsam.imuBias.ConstantBias()

    for trial in range(trials):
        accelerations = acceleration + rng.normal(size=(steps, 3)) * (
            np.asarray(accelerometer_sigmas) / np.sqrt(dt)
        )
        angular_velocities = angular_velocity + rng.normal(size=(steps, 3)) * (
            np.asarray(gyroscope_sigmas) / np.sqrt(dt)
        )
        for name, backend_type in BACKENDS.items():
            pim = integrate(
                backend_type, accelerations, angular_velocities, dt, params, bias
            )
            predicted = pim.predict(state_i, bias)
            error = logmap_error(truth, predicted)
            covariance = np.asarray(pim.residualCovariance())
            errors[name][trial] = error
            physical[name][trial] = physical_error(truth, predicted)
            nees[name][trial] = error @ np.linalg.solve(covariance, error)

    return {
        "summary": _summary(errors, nees),
        "physical_summary": _physical_summary(physical),
        "interval": _mean_interval(trials, 9),
    }


def rotating_deterministic_samples(
    accelerations,
    angular_velocities,
    dt,
    params_by_condition,
    state_i,
    truth,
):
    """Measure deterministic rotating-frame errors for supplied samples."""
    result = {name: {} for name in BACKENDS}
    bias = gtsam.imuBias.ConstantBias()
    for name, backend_type in BACKENDS.items():
        for condition, params in params_by_condition.items():
            predicted = integrate(
                backend_type, accelerations, angular_velocities, dt, params, bias
            ).predict(state_i, bias)
            result[name][condition] = physical_error(truth, predicted)
    return result


def run_rotating_nees(
    acceleration,
    angular_velocity,
    duration,
    dt,
    trials,
    seed,
    accelerometer_sigmas,
    gyroscope_sigmas,
    params_by_condition,
    state_i,
    truth,
):
    """Run paired Logmap NEES trials with and without rotating-Earth input."""
    steps = round(duration / dt)
    rng = np.random.default_rng(seed)
    errors = {
        name: {condition: np.empty((trials, 9)) for condition in params_by_condition}
        for name in BACKENDS
    }
    physical = {
        name: {condition: np.empty((trials, 9)) for condition in params_by_condition}
        for name in BACKENDS
    }
    nees = {
        name: {condition: np.empty(trials) for condition in params_by_condition}
        for name in BACKENDS
    }
    bias = gtsam.imuBias.ConstantBias()

    for trial in range(trials):
        accelerations = acceleration + rng.normal(size=(steps, 3)) * (
            np.asarray(accelerometer_sigmas) / np.sqrt(dt)
        )
        angular_velocities = angular_velocity + rng.normal(size=(steps, 3)) * (
            np.asarray(gyroscope_sigmas) / np.sqrt(dt)
        )
        for name, backend_type in BACKENDS.items():
            for condition, params in params_by_condition.items():
                pim = integrate(
                    backend_type, accelerations, angular_velocities, dt, params, bias
                )
                predicted = pim.predict(state_i, bias)
                error = logmap_error(truth, predicted)
                covariance = np.asarray(pim.residualCovariance())
                errors[name][condition][trial] = error
                physical[name][condition][trial] = physical_error(truth, predicted)
                nees[name][condition][trial] = error @ np.linalg.solve(
                    covariance, error
                )

    summary = {name: {} for name in BACKENDS}
    physical_summary = {name: {} for name in BACKENDS}
    for name in BACKENDS:
        for condition in params_by_condition:
            values = nees[name][condition]
            summary[name][condition] = {
                "mean_nees": values.mean(),
                "mean_half_width": 1.96 * values.std(ddof=1) / np.sqrt(trials),
                "mean_error_norm": np.linalg.norm(errors[name][condition].mean(axis=0)),
            }
            block = physical[name][condition]
            physical_summary[name][condition] = {
                "position_rmse": np.sqrt(np.mean(np.sum(block[:, 3:6] ** 2, axis=1))),
                "velocity_rmse": np.sqrt(np.mean(np.sum(block[:, 6:9] ** 2, axis=1))),
            }
    return {
        "summary": summary,
        "physical_summary": physical_summary,
        "interval": _mean_interval(trials, 9),
    }


def run_fixed_bias_nees(
    accelerations,
    angular_velocities,
    dt,
    trials,
    seed,
    accelerometer_sigmas,
    gyroscope_sigmas,
    params,
    bias,
    state_i,
):
    """Run long-horizon sensor-noise NEES around a nonzero fixed bias."""
    nominal_accelerations = np.asarray(accelerations) + bias.accelerometer()
    nominal_angular_velocities = np.asarray(angular_velocities) + bias.gyroscope()
    means = {}
    for name, backend_type in BACKENDS.items():
        means[name] = integrate(
            backend_type,
            nominal_accelerations,
            nominal_angular_velocities,
            dt,
            params,
            bias,
        ).predict(state_i, bias)

    errors = {name: np.empty((trials, 9)) for name in BACKENDS}
    nees = {name: np.empty(trials) for name in BACKENDS}
    rng = np.random.default_rng(seed)
    for trial in range(trials):
        noisy_accelerations = nominal_accelerations + rng.normal(
            size=nominal_accelerations.shape
        ) * (np.asarray(accelerometer_sigmas) / np.sqrt(dt))
        noisy_angular_velocities = nominal_angular_velocities + rng.normal(
            size=nominal_angular_velocities.shape
        ) * (np.asarray(gyroscope_sigmas) / np.sqrt(dt))
        for name, backend_type in BACKENDS.items():
            pim = integrate(
                backend_type,
                noisy_accelerations,
                noisy_angular_velocities,
                dt,
                params,
                bias,
            )
            predicted = pim.predict(state_i, bias)
            error = logmap_error(means[name], predicted)
            covariance = np.asarray(pim.residualCovariance())
            errors[name][trial] = error
            nees[name][trial] = error @ np.linalg.solve(covariance, error)
    return {
        "summary": _summary(errors, nees),
        "interval": _mean_interval(trials, 9),
        "nominal_accelerations": nominal_accelerations,
        "nominal_angular_velocities": nominal_angular_velocities,
    }


def run_combined_bias_rw_nees(
    accelerations,
    angular_velocities,
    dt,
    trials,
    seed,
    accelerometer_sigmas,
    gyroscope_sigmas,
    bias_accelerometer_rw_sigmas,
    bias_gyroscope_rw_sigmas,
    params,
    initial_bias,
    state_i,
):
    """Run full 15D Combined NEES with paired bias random walks."""
    nominal_accelerations = np.asarray(accelerations) + initial_bias.accelerometer()
    nominal_angular_velocities = (
        np.asarray(angular_velocities) + initial_bias.gyroscope()
    )
    means = {}
    for name, backend_type in COMBINED_BACKENDS.items():
        means[name] = integrate(
            backend_type,
            nominal_accelerations,
            nominal_angular_velocities,
            dt,
            params,
            initial_bias,
        ).predict(state_i, initial_bias)

    errors = {name: np.empty((trials, 15)) for name in BACKENDS}
    nees = {name: np.empty(trials) for name in BACKENDS}
    rng = np.random.default_rng(seed)
    for trial in range(trials):
        pims = {
            name: backend_type(params, initial_bias)
            for name, backend_type in COMBINED_BACKENDS.items()
        }
        accelerometer_bias = initial_bias.accelerometer().copy()
        gyroscope_bias = initial_bias.gyroscope().copy()
        for acceleration, angular_velocity in zip(accelerations, angular_velocities):
            measured_acceleration = (
                acceleration
                + accelerometer_bias
                + rng.normal(size=3) * np.asarray(accelerometer_sigmas) / np.sqrt(dt)
            )
            measured_angular_velocity = (
                angular_velocity
                + gyroscope_bias
                + rng.normal(size=3) * np.asarray(gyroscope_sigmas) / np.sqrt(dt)
            )
            for pim in pims.values():
                pim.integrateMeasurement(
                    measured_acceleration, measured_angular_velocity, dt
                )
            accelerometer_bias += (
                rng.normal(size=3)
                * np.asarray(bias_accelerometer_rw_sigmas)
                * np.sqrt(dt)
            )
            gyroscope_bias += (
                rng.normal(size=3) * np.asarray(bias_gyroscope_rw_sigmas) * np.sqrt(dt)
            )

        final_bias = gtsam.imuBias.ConstantBias(accelerometer_bias, gyroscope_bias)
        bias_error = np.asarray(initial_bias.vector() - final_bias.vector())
        for name, pim in pims.items():
            predicted = pim.predict(state_i, initial_bias)
            navigation_error = logmap_error(means[name], predicted)
            error = np.concatenate((navigation_error, bias_error))
            covariance = np.asarray(pim.residualCovariance())
            errors[name][trial] = error
            nees[name][trial] = error @ np.linalg.solve(covariance, error)
    return {
        "summary": _summary(errors, nees),
        "interval": _mean_interval(trials, 15),
    }


def run_bias_update_sweep(
    accelerations,
    angular_velocities,
    dt,
    params,
    bias_hat,
    state_i,
    magnitudes,
    trials,
    seed,
):
    """Compare first-order bias correction with complete reintegration."""
    measured_accelerations = np.asarray(accelerations) + bias_hat.accelerometer()
    measured_angular_velocities = np.asarray(angular_velocities) + bias_hat.gyroscope()
    rng = np.random.default_rng(seed)
    directions_acc = rng.normal(size=(trials, 3))
    directions_acc /= np.linalg.norm(directions_acc, axis=1, keepdims=True)
    directions_gyro = rng.normal(size=(trials, 3))
    directions_gyro /= np.linalg.norm(directions_gyro, axis=1, keepdims=True)

    results = {
        name: {
            "rotation": np.empty((len(magnitudes), trials)),
            "position": np.empty((len(magnitudes), trials)),
            "velocity": np.empty((len(magnitudes), trials)),
        }
        for name in BACKENDS
    }
    baseline = {
        name: integrate(
            backend_type,
            measured_accelerations,
            measured_angular_velocities,
            dt,
            params,
            bias_hat,
        )
        for name, backend_type in BACKENDS.items()
    }

    # ||delta b_a|| = 30 ||delta b_omega|| while the joint six-vector has
    # the requested magnitude, matching the scaling used by Brossard et al.
    gyro_fraction = 1.0 / np.sqrt(30.0**2 + 1.0)
    for magnitude_index, magnitude in enumerate(magnitudes):
        for trial in range(trials):
            delta_acc = 30.0 * gyro_fraction * magnitude * directions_acc[trial]
            delta_gyro = gyro_fraction * magnitude * directions_gyro[trial]
            updated_bias = bias_hat + gtsam.imuBias.ConstantBias(delta_acc, delta_gyro)
            for name, backend_type in BACKENDS.items():
                corrected = baseline[name].predict(state_i, updated_bias)
                reintegrated = integrate(
                    backend_type,
                    measured_accelerations,
                    measured_angular_velocities,
                    dt,
                    params,
                    updated_bias,
                ).predict(state_i, updated_bias)
                error = logmap_error(reintegrated, corrected)
                results[name]["rotation"][magnitude_index, trial] = np.linalg.norm(
                    error[:3]
                )
                results[name]["position"][magnitude_index, trial] = np.linalg.norm(
                    error[3:6]
                )
                results[name]["velocity"][magnitude_index, trial] = np.linalg.norm(
                    error[6:9]
                )

    summary = {name: {} for name in BACKENDS}
    for name in BACKENDS:
        for component, values in results[name].items():
            summary[name][component] = {
                "median": np.median(values, axis=1),
                "lower": np.quantile(values, 0.33, axis=1),
                "upper": np.quantile(values, 0.67, axis=1),
            }
    return summary


def _table(headers, rows):
    align = ["---"] + ["---:" for _ in headers[1:]]
    return "\n".join(
        ["| " + " | ".join(headers) + " |", "|" + "|".join(align) + "|"]
        + ["| " + " | ".join(row) + " |" for row in rows]
    )


def _format_number(value, format_spec, best=False):
    """Format a number and optionally emphasize it in a Markdown table."""
    text = format(value, format_spec)
    return f"**{text}**" if best else text


def _is_best(value, best):
    """Treat numerically identical values as tied for best."""
    return bool(np.isclose(value, best, rtol=1e-12, atol=1e-15))


def inertial_table(result):
    physical_summaries = result["physical_summary"]
    statistical_summaries = result["summary"]
    best_position = min(
        values["position_rmse"] for values in physical_summaries.values()
    )
    best_velocity = min(
        values["velocity_rmse"] for values in physical_summaries.values()
    )
    best_error = min(
        values["mean_error_norm"] for values in statistical_summaries.values()
    )
    best_nees_distance = min(
        abs(values["mean_nees"] - 9.0) for values in statistical_summaries.values()
    )
    rows = []
    for name in BACKENDS:
        physical = physical_summaries[name]
        statistical = statistical_summaries[name]
        rows.append(
            [
                name,
                _format_number(
                    physical["position_rmse"],
                    ".4f",
                    _is_best(physical["position_rmse"], best_position),
                ),
                _format_number(
                    physical["velocity_rmse"],
                    ".4f",
                    _is_best(physical["velocity_rmse"], best_velocity),
                ),
                _format_number(
                    statistical["mean_error_norm"],
                    ".4f",
                    _is_best(statistical["mean_error_norm"], best_error),
                ),
                _format_number(
                    statistical["mean_nees"],
                    ".3f",
                    _is_best(abs(statistical["mean_nees"] - 9.0), best_nees_distance),
                ),
            ]
        )
    return _table(
        [
            "Backend",
            "Position RMS (m)",
            "Velocity RMS (m/s)",
            "Mean Logmap error norm",
            "Mean NEES",
        ],
        rows,
    )


def uncertainty_table(result, expected_nees):
    summaries = result["summary"]
    best_nees_distance = min(
        abs(values["mean_nees"] - expected_nees) for values in summaries.values()
    )
    best_error = min(values["mean_error_norm"] for values in summaries.values())
    rows = []
    for name in BACKENDS:
        values = summaries[name]
        rows.append(
            [
                name,
                _format_number(
                    values["mean_nees"],
                    ".3f",
                    _is_best(
                        abs(values["mean_nees"] - expected_nees),
                        best_nees_distance,
                    ),
                ),
                f"{values['median_nees']:.3f}",
                _format_number(
                    values["mean_error_norm"],
                    ".4f",
                    _is_best(values["mean_error_norm"], best_error),
                ),
            ]
        )
    return _table(
        ["Backend", "Mean NEES", "Median NEES", "Mean Logmap error norm"], rows
    )


def rotating_tables(result):
    conditions = tuple(next(iter(result["summary"].values())))
    best_position = {
        condition: min(
            result["physical_summary"][name][condition]["position_rmse"]
            for name in BACKENDS
        )
        for condition in conditions
    }
    best_velocity = {
        condition: min(
            result["physical_summary"][name][condition]["velocity_rmse"]
            for name in BACKENDS
        )
        for condition in conditions
    }
    best_nees_distance = min(
        abs(result["summary"][name][condition]["mean_nees"] - 9.0)
        for name in BACKENDS
        for condition in conditions
    )
    physical_rows, nees_rows = [], []
    for name in BACKENDS:
        for condition in result["summary"][name]:
            physical = result["physical_summary"][name][condition]
            statistical = result["summary"][name][condition]
            physical_rows.append(
                [
                    name,
                    condition,
                    _format_number(
                        physical["position_rmse"],
                        ".4f",
                        _is_best(physical["position_rmse"], best_position[condition]),
                    ),
                    _format_number(
                        physical["velocity_rmse"],
                        ".4f",
                        _is_best(physical["velocity_rmse"], best_velocity[condition]),
                    ),
                ]
            )
            nees_rows.append(
                [
                    name,
                    condition,
                    _format_number(
                        statistical["mean_nees"],
                        ".3f",
                        _is_best(
                            abs(statistical["mean_nees"] - 9.0),
                            best_nees_distance,
                        ),
                    ),
                ]
            )
    return (
        _table(
            ["Backend", "Earth rate", "Position RMS (m)", "Velocity RMS (m/s)"],
            physical_rows,
        ),
        _table(["Backend", "Earth rate", "Mean NEES"], nees_rows),
    )


def bias_update_table(magnitudes, summary):
    """Format median bias-correction errors at the largest update."""
    index = int(np.argmax(magnitudes))
    best_rotation = min(
        values["rotation"]["median"][index] for values in summary.values()
    )
    best_position = min(
        values["position"]["median"][index] for values in summary.values()
    )
    best_velocity = min(
        values["velocity"]["median"][index] for values in summary.values()
    )
    rows = []
    for name in BACKENDS:
        rotation = summary[name]["rotation"]["median"][index]
        position = summary[name]["position"]["median"][index]
        velocity = summary[name]["velocity"]["median"][index]
        rows.append(
            [
                name,
                _format_number(
                    rotation * 180 / np.pi * 1e6,
                    ".3f",
                    _is_best(rotation, best_rotation),
                ),
                _format_number(
                    position * 1e3,
                    ".4f",
                    _is_best(position, best_position),
                ),
                _format_number(
                    velocity * 1e2,
                    ".4f",
                    _is_best(velocity, best_velocity),
                ),
            ]
        )
    return _table(
        [
            "Backend",
            "Rotation (microdeg)",
            "Position (mm)",
            "Velocity (cm/s)",
        ],
        rows,
    )


def convergence_figure(sample_periods, deterministic):
    figure = go.Figure()
    for name in BACKENDS:
        color, dash = BACKEND_STYLES[name]
        figure.add_scatter(
            x=sample_periods,
            y=deterministic[name]["velocity"],
            mode="lines+markers",
            name=name,
            line=dict(color=color, dash=dash),
        )
    figure.update_xaxes(type="log", title="IMU sample period (s)")
    figure.update_yaxes(type="log", title="Velocity endpoint error (m/s)")
    figure.update_layout(
        title="Deterministic held-input discretization error",
        template="plotly_white",
    )
    return figure


def nees_figure(result, dimension, title):
    figure = go.Figure()
    interval = result["interval"]
    figure.add_hrect(
        y0=interval[0],
        y1=interval[1],
        fillcolor="#14866d",
        opacity=0.14,
        line_width=0,
        annotation_text="95% consistency band",
        annotation_position="top left",
    )
    names = list(BACKENDS)
    figure.add_scatter(
        x=names,
        y=[result["summary"][name]["mean_nees"] for name in names],
        mode="markers",
        name="Logmap",
        marker=dict(size=11, color="#3569a8", symbol="circle"),
        error_y=dict(
            type="data",
            array=[result["summary"][name]["mean_half_width"] for name in names],
            visible=True,
        ),
    )
    figure.add_hline(y=dimension, line_dash="dash", line_color="#333333")
    figure.update_layout(
        title=title,
        xaxis_title="Preintegration backend",
        yaxis_title="Mean NEES",
        template="plotly_white",
    )
    return figure


def rotating_nees_figure(result):
    figure = go.Figure()
    interval = result["interval"]
    figure.add_hrect(
        y0=interval[0],
        y1=interval[1],
        fillcolor="#14866d",
        opacity=0.14,
        line_width=0,
        annotation_text="95% consistency band",
        annotation_position="top left",
    )
    symbols = {"Not specified": "x", "Specified": "circle"}
    for condition in next(iter(result["summary"].values())):
        figure.add_scatter(
            x=list(BACKENDS),
            y=[result["summary"][name][condition]["mean_nees"] for name in BACKENDS],
            mode="markers",
            name=condition,
            marker=dict(size=11, color="#3569a8", symbol=symbols[condition]),
            error_y=dict(
                type="data",
                array=[
                    result["summary"][name][condition]["mean_half_width"]
                    for name in BACKENDS
                ],
                visible=True,
            ),
        )
    figure.add_hline(y=9, line_dash="dash", line_color="#333333")
    figure.update_layout(
        title="Powered-ascent Logmap NEES with and without Earth rate",
        xaxis_title="Preintegration backend",
        yaxis_title="Mean NEES",
        template="plotly_white",
        legend_title_text="omegaCoriolis",
    )
    return figure


def bias_update_figure(magnitudes, summary):
    """Create small multiples for first-order bias-update errors."""
    figure = make_subplots(
        rows=3,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.06,
        subplot_titles=(
            "Rotation error (microdegrees)",
            "Velocity error (cm/s)",
            "Position error (mm)",
        ),
    )
    scales = {"rotation": 180.0 / np.pi * 1e6, "position": 1e3, "velocity": 1e2}
    for row, component in enumerate(("rotation", "velocity", "position"), start=1):
        for name in BACKENDS:
            color, dash = BACKEND_STYLES[name]
            values = summary[name][component]
            scale = scales[component]
            figure.add_scatter(
                x=magnitudes,
                y=scale * values["median"],
                mode="lines",
                name=name,
                legendgroup=name,
                showlegend=row == 1,
                line=dict(color=color, dash=dash),
                customdata=np.column_stack(
                    (scale * values["lower"], scale * values["upper"])
                ),
                hovertemplate=(
                    "median %{y:.4g}"
                    "<br>33–67% [%{customdata[0]:.4g}, %{customdata[1]:.4g}]"
                    "<extra>%{fullData.name}</extra>"
                ),
                row=row,
                col=1,
            )
    figure.update_layout(
        title="First-order bias correction versus complete reintegration",
        template="plotly_white",
        height=760,
    )
    figure.update_xaxes(title_text="Joint bias-update magnitude", row=3, col=1)
    return figure


def interval_text(result):
    lower, upper = result["interval"]
    return f"Expected 95% interval for mean NEES: [{lower:.3f}, {upper:.3f}]"


def validate_finite(result):
    """Assert all summarized NEES values are finite."""
    for values in result["summary"].values():
        if "mean_nees" in values:
            assert np.isfinite(values["mean_nees"])
        else:
            for condition in values.values():
                assert np.isfinite(condition["mean_nees"])


def validate_bias_sweep(magnitudes, summary):
    """Check zero-update exactness and local second-order behavior."""
    magnitudes = np.asarray(magnitudes)
    zero_index = int(np.argmin(np.abs(magnitudes)))
    for backend in BACKENDS:
        for component in ("rotation", "position", "velocity"):
            median = summary[backend][component]["median"]
            assert median[zero_index] < 1e-12
            assert np.all(np.isfinite(median))

    positive = np.flatnonzero(magnitudes > 0.0)
    if len(positive) >= 2:
        first, second = positive[:2]
        scale = (magnitudes[second] / magnitudes[first]) ** 2
        for backend in BACKENDS:
            for component in ("rotation", "position", "velocity"):
                values = summary[backend][component]["median"]
                if values[first] > 1e-15:
                    ratio = values[second] / values[first]
                    assert 0.25 * scale < ratio < 4.0 * scale
