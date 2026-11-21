"""
Python translation of examples/AbcEquivariantFilterExample.cpp.

Runs the Attitude-Bias-Calibration EqF demo using the wrapped C++
EquivariantFilter (ABC-specific wrapper).
"""

from __future__ import annotations

from dataclasses import dataclass
import csv
import math
from typing import List

import numpy as np
import gtsam
from gtsam import Rot3, Unit3
from gtsam.utils import findExampleDataFile


@dataclass
class MeasurementRecord:
    y: np.ndarray
    d: np.ndarray
    R: np.ndarray
    cal_idx: int


@dataclass
class DataRecord:
    R: Rot3
    b: np.ndarray
    cal_rot: Rot3
    omega: np.ndarray
    input_covariance: np.ndarray
    measurements: List[MeasurementRecord]
    t: float
    dt: float


def _normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n == 0.0:
        return v
    return v / n


def load_data_from_csv(
    filename: str, start_row: int = 0, max_rows: int = -1, downsample: int = 1
) -> List[DataRecord]:
    data_list: List[DataRecord] = []
    with open(filename, newline="") as csvfile:
        reader = csv.reader(csvfile)
        header = next(reader, None)
        if header is None:
            return data_list

        line_number = 1
        row_count = 0
        prev_time = 0.0

        for row in reader:
            line_number += 1
            if line_number < start_row:
                continue
            if ((line_number - start_row - 1) % downsample) != 0:
                continue
            if max_rows != -1 and row_count >= max_rows:
                break
            if len(row) < 39:
                continue

            values = [float(x) if x else 0.0 for x in row]

            t = values[0]
            dt = 0.0 if row_count == 0 else t - prev_time
            prev_time = t

            R = Rot3.Quaternion(values[1], values[2], values[3], values[4])
            b = np.array([values[5], values[6], values[7]])

            cal_rot = Rot3.Quaternion(values[8], values[9], values[10], values[11])

            omega = np.array([values[12], values[13], values[14]])

            input_covariance = np.zeros((6, 6))
            input_covariance[0, 0] = values[15] ** 2
            input_covariance[1, 1] = values[16] ** 2
            input_covariance[2, 2] = values[17] ** 2
            input_covariance[3, 3] = values[18] ** 2
            input_covariance[4, 4] = values[19] ** 2
            input_covariance[5, 5] = values[20] ** 2

            measurements: List[MeasurementRecord] = []

            y0 = _normalize(np.array([values[21], values[22], values[23]]))
            d0 = _normalize(np.array([values[33], values[34], values[35]]))
            cov_y0 = np.diag([values[27] ** 2, values[28] ** 2, values[29] ** 2])
            measurements.append(MeasurementRecord(y0, d0, cov_y0, 0))

            y1 = _normalize(np.array([values[24], values[25], values[26]]))
            d1 = _normalize(np.array([values[36], values[37], values[38]]))
            cov_y1 = np.diag([values[30] ** 2, values[31] ** 2, values[32] ** 2])
            measurements.append(MeasurementRecord(y1, d1, cov_y1, -1))

            data_list.append(
                DataRecord(
                    R=R,
                    b=b,
                    cal_rot=cal_rot,
                    omega=omega,
                    input_covariance=input_covariance,
                    measurements=measurements,
                    t=t,
                    dt=dt,
                )
            )
            row_count += 1

    return data_list


def process_data_with_eqf(
    filter_eqf: gtsam.abc.AbcEquivariantFilter1,
    data_list: List[DataRecord],
) -> None:
    if not data_list:
        print("No data to process")
        return

    print(f"Processing {len(data_list)} data points with EqF...")
    att_errors: List[float] = []
    bias_errors: List[float] = []
    cal_errors: List[float] = []

    total_measurements = 0
    valid_measurements = 0

    rad_to_deg = 180.0 / math.pi
    progress_step = max(len(data_list) // 10, 1)
    print("Progress: ", end="", flush=True)

    for i, data in enumerate(data_list):
        filter_eqf.predict(data.omega, data.input_covariance, data.dt)

        for measurement in data.measurements:
            total_measurements += 1
            if np.any(np.isnan(measurement.y)) or np.any(np.isnan(measurement.d)):
                continue
            try:
                y_unit = Unit3(measurement.y)
                d_unit = Unit3(measurement.d)
                filter_eqf.update(y_unit, d_unit, measurement.R, measurement.cal_idx)
                valid_measurements += 1
            except Exception:
                continue

        estimate_R = filter_eqf.attitude()
        estimate_b = np.array(filter_eqf.bias()).reshape(3)
        estimate_cal = filter_eqf.calibration(0)

        att_error = Rot3.Logmap(data.R.between(estimate_R))
        bias_error = estimate_b - data.b
        cal_error = np.zeros(3)
        cal_error = Rot3.Logmap(data.cal_rot.between(estimate_cal))

        att_errors.append(np.linalg.norm(att_error))
        bias_errors.append(np.linalg.norm(bias_error))
        cal_errors.append(np.linalg.norm(cal_error))

        if i % progress_step == 0:
            print(".", end="", flush=True)

    print(" Done!")

    avg_att_error = float(np.mean(att_errors)) if att_errors else 0.0
    avg_bias_error = float(np.mean(bias_errors)) if bias_errors else 0.0
    avg_cal_error = float(np.mean(cal_errors)) if cal_errors else 0.0

    final_data = data_list[-1]
    final_R = filter_eqf.attitude()
    final_b = np.array(filter_eqf.bias()).reshape(3)
    final_cal = filter_eqf.calibration(0)
    final_att_error = Rot3.Logmap(final_data.R.between(final_R))
    final_bias_error = final_b - final_data.b
    final_cal_error = Rot3.Logmap(final_data.cal_rot.between(final_cal))

    print("\n=== Filter Performance Summary ===")
    print(f"Processed measurements: {total_measurements} (valid: {valid_measurements})")

    print("\n-- Average Errors --")
    print(f"Attitude: {avg_att_error * rad_to_deg}°")
    print(f"Bias: {avg_bias_error}")
    print(f"Calibration: {avg_cal_error * rad_to_deg}°")

    print("\n-- Final Errors --")
    print(f"Attitude: {np.linalg.norm(final_att_error) * rad_to_deg}°")
    print(f"Bias: {np.linalg.norm(final_bias_error)}")
    print(f"Calibration: {np.linalg.norm(final_cal_error) * rad_to_deg}°")

    print("\n-- Final State vs Ground Truth --")
    print(
        "Attitude (RPY) - Estimate:",
        (final_R.rpy() * rad_to_deg),
        "° | Truth:",
        (final_data.R.rpy() * rad_to_deg),
        "°",
    )
    print("Bias - Estimate:", final_b, "| Truth:", final_data.b)
    print(
        "Calibration (RPY) - Estimate:",
        (final_cal.rpy() * rad_to_deg),
        "° | Truth:",
        (final_data.cal_rot.rpy() * rad_to_deg),
        "°",
    )


def main() -> None:
    print("ABC-EqF: Attitude-Bias-Calibration Equivariant Filter Demo")
    print("==============================================================")

    try:
        csv_file_path = findExampleDataFile("EqFdata.csv")
    except Exception:
        print("Error: Could not find EqFdata.csv")
        return

    data = load_data_from_csv(csv_file_path)
    if not data:
        print("No data available to process. Exiting.")
        return

    n_cal = 1
    m_sensors = 2
    initial_sigma = np.eye(6 + 3 * n_cal)
    initial_sigma[0:3, 0:3] = 0.1 * np.eye(3)
    initial_sigma[3:6, 3:6] = 0.01 * np.eye(3)
    initial_sigma[6:9, 6:9] = 0.1 * np.eye(3)

    filter_eqf = gtsam.abc.AbcEquivariantFilter1(initial_sigma)
    process_data_with_eqf(filter_eqf, data)

    print("\nEqF demonstration completed successfully.")


if __name__ == "__main__":
    main()


