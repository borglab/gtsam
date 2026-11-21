"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Helper utilities for the staircase legged-estimation example.
"""

from __future__ import annotations

from dataclasses import dataclass
import csv
import io
from pathlib import Path
from typing import Dict, List, Sequence

import numpy as np

import gtsam


@dataclass(frozen=True)
class ImuSample:
    timestamp_s: float
    omega: np.ndarray
    specific_force: np.ndarray


@dataclass(frozen=True)
class ContactEvent:
    timestamp_s: float
    active_contacts: List[gtsam.ContactMeasurement]


SHOWCASE_VARIANTS = (
    "invariant_ekf",
    "invariant_graph",
    "fixed_lag_single_bias",
)

DEFAULT_SMOOTHER_LAG_SECONDS = 1.0
DEFAULT_FORCED_HEIGHT_PRIOR_UNTIL_S = 8.0
DEFAULT_FORCED_TERRAIN_HEIGHT = 0.0
FOOT_COLORS = ("#d62728", "#2ca02c", "#1f77b4", "#ff7f0e")


def legged_staircase_dataset_dir() -> Path:
    return Path(gtsam.findExampleDataFile("legged_staircase/metadata.csv")).parent


def load_legged_csv_dataset(dataset_dir: Path | None = None) -> Dict[str, object]:
    dataset_dir = (
        legged_staircase_dataset_dir() if dataset_dir is None else Path(dataset_dir)
    )

    metadata: Dict[str, object] = {}
    with (dataset_dir / "metadata.csv").open(newline="") as stream:
        for row in csv.reader(stream):
            if not row or row[0] == "key":
                continue
            if row[0] == "foot_names":
                metadata["foot_names"] = [value for value in row[1:] if value]
            elif row[0] == "dense_contact_stream":
                metadata["dense_contact_stream"] = row[1] == "1"
            elif row[0] == "contact_state_value":
                metadata["contact_state_value"] = int(row[1])
            elif row[0] == "timestamp_source":
                metadata["timestamp_source"] = row[1]

    imu_samples: List[ImuSample] = []
    with (dataset_dir / "imu.csv").open(newline="") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            imu_samples.append(
                ImuSample(
                    timestamp_s=float(row["timestamp_s"]),
                    omega=np.array(
                        [
                            float(row["omega_x"]),
                            float(row["omega_y"]),
                            float(row["omega_z"]),
                        ],
                        dtype=float,
                    ),
                    specific_force=np.array(
                        [
                            float(row["acc_x"]),
                            float(row["acc_y"]),
                            float(row["acc_z"]),
                        ],
                        dtype=float,
                    ),
                )
            )

    contact_events: List[ContactEvent] = []
    current_index = None
    current_time = 0.0
    active_contacts: List[gtsam.ContactMeasurement] = []
    with (dataset_dir / "contacts.csv").open(newline="") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            event_index = int(row["event_index"])
            if current_index is None or event_index != current_index:
                if active_contacts:
                    contact_events.append(ContactEvent(current_time, active_contacts))
                current_index = event_index
                current_time = float(row["timestamp_s"])
                active_contacts = []

            measurement = gtsam.ContactMeasurement()
            measurement.foot = int(row["foot_index"])
            measurement.bodyPoint = np.array(
                [
                    float(row["body_x"]),
                    float(row["body_y"]),
                    float(row["body_z"]),
                ],
                dtype=float,
            )
            measurement.touchdown = row["is_new_contact"] == "1"
            active_contacts.append(measurement)

    if active_contacts:
        contact_events.append(ContactEvent(current_time, active_contacts))

    metadata["imu_samples"] = imu_samples
    metadata["contact_events"] = contact_events
    return metadata


def first_full_contact_time(dataset: Dict[str, object]) -> float:
    foot_names: Sequence[str] = dataset["foot_names"]  # type: ignore[index]
    contact_events: List[ContactEvent] = dataset["contact_events"]  # type: ignore[index]
    for event in contact_events:
        if len(event.active_contacts) == len(foot_names):
            return event.timestamp_s
    raise ValueError("Dataset does not contain a full-contact event.")


def display_trajectory(
    trajectory: Sequence[Dict[str, object]],
    start_time_s: float,
    normalize_time: bool = True,
) -> List[Dict[str, object]]:
    displayed_rows: List[Dict[str, object]] = []
    for row in trajectory:
        timestamp_s = float(row["timestamp_s"])
        if timestamp_s < start_time_s:
            continue
        displayed_row = dict(row)
        if normalize_time:
            displayed_row["timestamp_s"] = timestamp_s - start_time_s
        displayed_rows.append(displayed_row)
    return displayed_rows


def make_legged_estimator_params() -> gtsam.LeggedEstimatorParams:
    params = gtsam.LeggedEstimatorParams()
    params.preintegrationParams = gtsam.PreintegrationParams.MakeSharedU(9.81)
    params.preintegrationParams.setGyroscopeCovariance(np.eye(3) * (8e-4 * 8e-4))
    params.preintegrationParams.setIntegrationCovariance(np.eye(3) * (1e-3 * 1e-3))
    params.preintegrationParams.setAccelerometerCovariance(np.eye(3) * (2e-2 * 2e-2))
    params.body_P_imu = gtsam.Pose3(gtsam.Rot3(), np.array([0.30, 0.0, 0.15]))
    params.footholdProcessSigma = 1e-4
    params.footholdInitSigma = 5e-1
    params.contactCovariance = np.diag([0.03 * 0.03, 0.03 * 0.03, 0.02 * 0.02])
    params.heightPriorSigma = 0.02
    params.useRobustContactNoise = False
    params.robustContactHuberK = 2.0
    params.imuBias = gtsam.imuBias.ConstantBias()
    params.biasAccRandomWalkSigma = 5e-3
    params.biasOmegaRandomWalkSigma = 1e-4
    params.useFullContactInitialization = True
    params.marginalizeLeavingFoot = True
    return params


def _initial_state() -> gtsam.NavState:
    return gtsam.NavState(gtsam.Rot3(), np.array([0.0, 0.0, 0.76]), np.zeros(3))


def _initial_footholds(num_feet: int) -> np.ndarray:
    return np.zeros((3, num_feet), dtype=float)


def _initial_covariance(num_feet: int) -> np.ndarray:
    covariance = np.zeros((9 + 3 * num_feet, 9 + 3 * num_feet), dtype=float)
    covariance[np.arange(9), np.arange(9)] = np.array(
        [1e-2, 1e-2, 1e-6, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
    )
    covariance[np.arange(9, covariance.shape[0]), np.arange(9, covariance.shape[0])] = (
        0.25
    )
    return covariance


def _initial_base_covariance() -> np.ndarray:
    covariance = np.zeros((9, 9), dtype=float)
    covariance[np.arange(9), np.arange(9)] = np.array(
        [1e-2, 1e-2, 1e-6, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
    )
    return covariance


def make_legged_estimator(
    variant_name: str,
    foot_names: Sequence[str],
    params: gtsam.LeggedEstimatorParams | None = None,
):
    params = make_legged_estimator_params() if params is None else params
    initial_state = _initial_state()
    footholds = _initial_footholds(len(foot_names))
    covariance = _initial_covariance(len(foot_names))
    base_covariance = _initial_base_covariance()
    foot_names = list(foot_names)

    if variant_name == "invariant_ekf":
        return gtsam.LeggedInvariantEKF(
            initial_state, footholds, covariance, params, foot_names
        )
    if variant_name == "invariant_graph":
        return gtsam.LeggedInvariantIEKF(
            initial_state, footholds, covariance, params, foot_names
        )
    if variant_name == "fixed_lag_single_bias":
        return gtsam.LeggedFixedLagSmoother(
            initial_state,
            footholds,
            base_covariance,
            params,
            DEFAULT_SMOOTHER_LAG_SECONDS,
            foot_names,
        )
    if variant_name == "fixed_lag_combined_bias":
        return gtsam.LeggedCombinedFixedLagSmoother(
            initial_state,
            footholds,
            base_covariance,
            params,
            DEFAULT_SMOOTHER_LAG_SECONDS,
            foot_names,
        )
    raise ValueError(f"Unknown legged estimator variant: {variant_name}")


def _trajectory_row(kind: str, timestamp_s: float, estimator) -> Dict[str, object]:
    estimate = estimator.estimate()
    bias = estimator.estimateBias()
    position = np.asarray(estimate.x(0), dtype=float)
    velocity = np.asarray(estimate.x(1), dtype=float)
    rpy = np.asarray(estimate.rotation().rpy(), dtype=float)
    bias_acc = np.asarray(bias.accelerometer(), dtype=float)
    bias_gyro = np.asarray(bias.gyroscope(), dtype=float)
    return {
        "kind": kind,
        "timestamp_s": timestamp_s,
        "x": float(position[0]),
        "y": float(position[1]),
        "z": float(position[2]),
        "roll": float(rpy[0]),
        "pitch": float(rpy[1]),
        "yaw": float(rpy[2]),
        "vx": float(velocity[0]),
        "vy": float(velocity[1]),
        "vz": float(velocity[2]),
        "bias_ax": float(bias_acc[0]),
        "bias_ay": float(bias_acc[1]),
        "bias_az": float(bias_acc[2]),
        "bias_gx": float(bias_gyro[0]),
        "bias_gy": float(bias_gyro[1]),
        "bias_gz": float(bias_gyro[2]),
        "bias_acc_norm": float(np.linalg.norm(bias_acc)),
        "bias_gyro_norm": float(np.linalg.norm(bias_gyro)),
    }


def _contact_frame_row(
    timestamp_s: float,
    estimator,
    active_contacts: Sequence[gtsam.ContactMeasurement],
    body_P_imu: gtsam.Pose3,
    breadcrumbs: Sequence[Dict[str, object]],
) -> Dict[str, object]:
    estimate = estimator.estimate()
    imu_rotation = estimate.rotation()
    imu_position = np.asarray(estimate.x(0), dtype=float)
    world_T_imu = gtsam.Pose3(imu_rotation, imu_position)
    world_T_body = world_T_imu.compose(body_P_imu.inverse())
    footholds = np.asarray(estimate.xMatrix(), dtype=float)[:, 2:]
    active_feet = [contact.foot for contact in active_contacts]
    active_footholds = footholds[:, active_feet] if active_feet else np.zeros((3, 0))
    return {
        "timestamp_s": float(timestamp_s),
        "imu_position": imu_position.tolist(),
        "imu_rotation": np.asarray(imu_rotation.matrix(), dtype=float).tolist(),
        "body_position": np.asarray(world_T_body.translation(), dtype=float).tolist(),
        "body_rotation": np.asarray(
            world_T_body.rotation().matrix(), dtype=float
        ).tolist(),
        "active_feet": active_feet,
        "active_footholds": active_footholds.T.tolist(),
        "active_foot_colors": [
            FOOT_COLORS[index % len(FOOT_COLORS)] for index in active_feet
        ],
        "breadcrumb_positions": [list(item["position"]) for item in breadcrumbs],
        "breadcrumb_colors": [
            FOOT_COLORS[int(item["foot"]) % len(FOOT_COLORS)] for item in breadcrumbs
        ],
    }


def _frame_axis_endpoints(
    origin: np.ndarray, rotation: np.ndarray, axis_length: float
) -> List[np.ndarray]:
    return [origin + axis_length * rotation[:, index] for index in range(3)]


def _axis_trace(origin: np.ndarray, endpoint: np.ndarray, color: str, name: str):
    import plotly.graph_objects as go

    return go.Scatter3d(
        x=[origin[0], endpoint[0]],
        y=[origin[1], endpoint[1]],
        z=[origin[2], endpoint[2]],
        mode="lines",
        line=dict(color=color, width=6),
        name=name,
        hoverinfo="skip",
    )


def _marker_trace(
    point: np.ndarray, color: str, name: str, size: int, symbol: str = "circle"
):
    import plotly.graph_objects as go

    return go.Scatter3d(
        x=[point[0]],
        y=[point[1]],
        z=[point[2]],
        mode="markers",
        marker=dict(color=color, size=size, symbol=symbol),
        name=name,
        hoverinfo="name+x+y+z",
    )


def _feet_trace(frame: Dict[str, object]):
    import plotly.graph_objects as go

    footholds = np.asarray(frame["active_footholds"], dtype=float)
    if footholds.size == 0:
        footholds = np.zeros((0, 3), dtype=float)
    return go.Scatter3d(
        x=footholds[:, 0] if len(footholds) else [],
        y=footholds[:, 1] if len(footholds) else [],
        z=footholds[:, 2] if len(footholds) else [],
        mode="markers",
        marker=dict(color=list(frame["active_foot_colors"]), size=6),
        name="Feet",
        hovertemplate="x=%{x:.3f}<br>y=%{y:.3f}<br>z=%{z:.3f}<extra></extra>",
    )


def _breadcrumbs_trace(frame: Dict[str, object]):
    import plotly.graph_objects as go

    footholds = np.asarray(frame["breadcrumb_positions"], dtype=float)
    if footholds.size == 0:
        footholds = np.zeros((0, 3), dtype=float)
    return go.Scatter3d(
        x=footholds[:, 0] if len(footholds) else [],
        y=footholds[:, 1] if len(footholds) else [],
        z=footholds[:, 2] if len(footholds) else [],
        mode="markers",
        marker=dict(color=list(frame["breadcrumb_colors"]), size=4, opacity=0.35),
        name="Footstep trail",
        hovertemplate="x=%{x:.3f}<br>y=%{y:.3f}<br>z=%{z:.3f}<extra></extra>",
    )


def _connector_trace(frame: Dict[str, object]):
    import plotly.graph_objects as go

    body = np.asarray(frame["body_position"], dtype=float)
    footholds = np.asarray(frame["active_footholds"], dtype=float)
    xs: List[float | None] = []
    ys: List[float | None] = []
    zs: List[float | None] = []
    for foothold in footholds:
        xs.extend([body[0], foothold[0], None])
        ys.extend([body[1], foothold[1], None])
        zs.extend([body[2], foothold[2], None])
    return go.Scatter3d(
        x=xs,
        y=ys,
        z=zs,
        mode="lines",
        line=dict(color="#777777", width=4, dash="dot"),
        name="Legs",
        hoverinfo="skip",
    )


def _animation_frame_traces(
    frame: Dict[str, object], axis_length: float, show_legend: bool
):
    imu_origin = np.asarray(frame["imu_position"], dtype=float)
    imu_rotation = np.asarray(frame["imu_rotation"], dtype=float)
    body_origin = np.asarray(frame["body_position"], dtype=float)
    body_rotation = np.asarray(frame["body_rotation"], dtype=float)
    imu_axes = _frame_axis_endpoints(imu_origin, imu_rotation, axis_length)
    body_axes = _frame_axis_endpoints(body_origin, body_rotation, axis_length)
    traces = [
        _marker_trace(imu_origin, "#8c564b", "IMU frame", 5, "diamond"),
        _marker_trace(body_origin, "#111111", "Body frame", 6),
        _feet_trace(frame),
        _breadcrumbs_trace(frame),
        _connector_trace(frame),
        _axis_trace(imu_origin, imu_axes[0], "#d62728", "IMU x"),
        _axis_trace(imu_origin, imu_axes[1], "#2ca02c", "IMU y"),
        _axis_trace(imu_origin, imu_axes[2], "#1f77b4", "IMU z"),
        _axis_trace(body_origin, body_axes[0], "#ff9896", "Body x"),
        _axis_trace(body_origin, body_axes[1], "#98df8a", "Body y"),
        _axis_trace(body_origin, body_axes[2], "#aec7e8", "Body z"),
    ]
    for trace in traces:
        trace.showlegend = show_legend
    return traces


def _side_axis_trace(
    origin: np.ndarray, endpoint: np.ndarray, color: str, name: str, dash: str = "solid"
):
    import plotly.graph_objects as go

    return go.Scatter(
        x=[origin[0], endpoint[0]],
        y=[origin[2], endpoint[2]],
        mode="lines",
        line=dict(color=color, width=4, dash=dash),
        name=name,
        hoverinfo="skip",
    )


def _side_marker_trace(
    point: np.ndarray, color: str, name: str, size: int, symbol: str = "circle"
):
    import plotly.graph_objects as go

    return go.Scatter(
        x=[point[0]],
        y=[point[2]],
        mode="markers",
        marker=dict(color=color, size=size, symbol=symbol),
        name=name,
        hoverinfo="name+x+y",
    )


def _side_feet_trace(frame: Dict[str, object]):
    import plotly.graph_objects as go

    footholds = np.asarray(frame["active_footholds"], dtype=float)
    if footholds.size == 0:
        footholds = np.zeros((0, 3), dtype=float)
    return go.Scatter(
        x=footholds[:, 0] if len(footholds) else [],
        y=footholds[:, 2] if len(footholds) else [],
        mode="markers",
        marker=dict(color=list(frame["active_foot_colors"]), size=9),
        name="Feet",
        hovertemplate="x=%{x:.3f}<br>z=%{y:.3f}<extra></extra>",
    )


def _side_breadcrumbs_trace(frame: Dict[str, object]):
    import plotly.graph_objects as go

    footholds = np.asarray(frame["breadcrumb_positions"], dtype=float)
    if footholds.size == 0:
        footholds = np.zeros((0, 3), dtype=float)
    return go.Scatter(
        x=footholds[:, 0] if len(footholds) else [],
        y=footholds[:, 2] if len(footholds) else [],
        mode="markers",
        marker=dict(color=list(frame["breadcrumb_colors"]), size=6, opacity=0.35),
        name="Footstep trail",
        hovertemplate="x=%{x:.3f}<br>z=%{y:.3f}<extra></extra>",
    )


def _side_connector_trace(frame: Dict[str, object]):
    import plotly.graph_objects as go

    body = np.asarray(frame["body_position"], dtype=float)
    footholds = np.asarray(frame["active_footholds"], dtype=float)
    xs: List[float | None] = []
    zs: List[float | None] = []
    for foothold in footholds:
        xs.extend([body[0], foothold[0], None])
        zs.extend([body[2], foothold[2], None])
    return go.Scatter(
        x=xs,
        y=zs,
        mode="lines",
        line=dict(color="#777777", width=3, dash="dot"),
        name="Legs",
        hoverinfo="skip",
    )


def _side_animation_frame_traces(
    frame: Dict[str, object], axis_length: float, show_legend: bool
):
    imu_origin = np.asarray(frame["imu_position"], dtype=float)
    imu_rotation = np.asarray(frame["imu_rotation"], dtype=float)
    body_origin = np.asarray(frame["body_position"], dtype=float)
    body_rotation = np.asarray(frame["body_rotation"], dtype=float)
    imu_axes = _frame_axis_endpoints(imu_origin, imu_rotation, axis_length)
    body_axes = _frame_axis_endpoints(body_origin, body_rotation, axis_length)
    traces = [
        _side_marker_trace(imu_origin, "#8c564b", "IMU frame", 9, "diamond"),
        _side_marker_trace(body_origin, "#111111", "Body frame", 10),
        _side_feet_trace(frame),
        _side_breadcrumbs_trace(frame),
        _side_connector_trace(frame),
        _side_axis_trace(imu_origin, imu_axes[0], "#d62728", "IMU x"),
        _side_axis_trace(imu_origin, imu_axes[2], "#1f77b4", "IMU z"),
        _side_axis_trace(body_origin, body_axes[0], "#ff9896", "Body x"),
        _side_axis_trace(body_origin, body_axes[2], "#aec7e8", "Body z"),
    ]
    for trace in traces:
        trace.showlegend = show_legend
    return traces


def _axis_range(values: np.ndarray, min_padding: float, frac_padding: float):
    low = float(np.min(values))
    high = float(np.max(values))
    span = max(high - low, 1e-6)
    padding = max(min_padding, frac_padding * span)
    return [low - padding, high + padding]


def _displayed_contact_frames(
    result: Dict[str, object], start_time_s: float
) -> List[Dict[str, object]]:
    contact_frames: List[Dict[str, object]] = result["contact_frames"]  # type: ignore[index]
    displayed_frames = [
        frame for frame in contact_frames if float(frame["timestamp_s"]) >= start_time_s
    ]
    if not displayed_frames:
        raise ValueError("Replay result does not contain any displayed contact frames.")
    return displayed_frames


def _frame_durations_ms(
    displayed_frames: Sequence[Dict[str, object]], start_time_s: float, speedup: float
) -> List[int]:
    if speedup <= 0.0:
        raise ValueError("speedup must be positive.")
    times = [float(frame["timestamp_s"]) - start_time_s for frame in displayed_frames]
    durations_ms: List[int] = []
    for index, time_s in enumerate(times):
        if index + 1 < len(times):
            dt_s = max(times[index + 1] - time_s, 1e-3)
        elif index > 0:
            dt_s = max(time_s - times[index - 1], 1e-3)
        else:
            dt_s = 0.2
        durations_ms.append(max(int(round(1000.0 * dt_s / speedup)), 20))
    return durations_ms


def _render_figure_frames_to_gif(
    figure, output_path: str | Path, durations_ms: Sequence[int]
):
    from PIL import Image
    import plotly.graph_objects as go
    import plotly.io as pio

    frames = [go.Figure(figure)]
    rendered_images: List[Image.Image] = []
    for frame in figure.frames:
        current = go.Figure(figure)
        current.frames = []
        current.update(data=frame.data)
        current.update_layout(frame.layout)
        png_bytes = pio.to_image(current, format="png")
        rendered_images.append(Image.open(io.BytesIO(png_bytes)).convert("P"))

    if not rendered_images:
        png_bytes = pio.to_image(frames[0], format="png")
        rendered_images.append(Image.open(io.BytesIO(png_bytes)).convert("P"))

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    rendered_images[0].save(
        output_path,
        save_all=True,
        append_images=rendered_images[1:],
        duration=list(durations_ms),
        loop=0,
        disposal=2,
    )


def make_contact_animation(
    result: Dict[str, object],
    variant_label: str,
    start_time_s: float = 0.0,
    axis_length: float = 0.08,
):
    import plotly.graph_objects as go

    displayed_frames = _displayed_contact_frames(result, start_time_s)

    plotted_frames = []
    all_points = []
    for index, frame in enumerate(displayed_frames):
        traces = _animation_frame_traces(frame, axis_length, show_legend=(index == 0))
        plotted_frames.append(
            go.Frame(
                name=str(index),
                data=traces,
                traces=list(range(len(traces))),
                layout=go.Layout(
                    title_text=(
                        f"{variant_label}: contact update at "
                        f"t={float(frame['timestamp_s']) - start_time_s:.2f} s"
                    )
                ),
            )
        )
        imu_origin = np.asarray(frame["imu_position"], dtype=float)
        body_origin = np.asarray(frame["body_position"], dtype=float)
        all_points.extend([imu_origin, body_origin])
        footholds = np.asarray(frame["active_footholds"], dtype=float)
        if footholds.size:
            all_points.extend(list(footholds))
        breadcrumbs = np.asarray(frame["breadcrumb_positions"], dtype=float)
        if breadcrumbs.size:
            all_points.extend(list(breadcrumbs))

    stacked_points = np.vstack(all_points)
    ranges = [
        _axis_range(stacked_points[:, 0], 0.04, 0.08),
        _axis_range(stacked_points[:, 1], 0.04, 0.08),
        _axis_range(stacked_points[:, 2], 0.03, 0.08),
    ]

    slider_steps = [
        {
            "args": [
                [frame.name],
                {
                    "frame": {"duration": 0, "redraw": True},
                    "mode": "immediate",
                    "transition": {"duration": 0},
                },
            ],
            "label": f"{float(displayed_frames[index]['timestamp_s']) - start_time_s:.2f}",
            "method": "animate",
        }
        for index, frame in enumerate(plotted_frames)
    ]

    figure = go.Figure(
        data=plotted_frames[0].data,
        frames=plotted_frames,
        layout=go.Layout(
            title=(
                f"{variant_label}: contact update at "
                f"t={float(displayed_frames[0]['timestamp_s']) - start_time_s:.2f} s"
            ),
            template="plotly_white",
            width=950,
            height=700,
            scene=dict(
                xaxis=dict(title="x [m]", range=ranges[0]),
                yaxis=dict(title="y [m]", range=ranges[1]),
                zaxis=dict(title="z [m]", range=ranges[2]),
                aspectmode="data",
            ),
            legend=dict(x=0.02, y=0.98),
            updatemenus=[
                {
                    "type": "buttons",
                    "direction": "left",
                    "x": 0.02,
                    "y": 1.08,
                    "showactive": False,
                    "buttons": [
                        {
                            "label": "Play",
                            "method": "animate",
                            "args": [
                                None,
                                {
                                    "frame": {"duration": 180, "redraw": True},
                                    "fromcurrent": True,
                                    "transition": {"duration": 0},
                                },
                            ],
                        },
                        {
                            "label": "Pause",
                            "method": "animate",
                            "args": [
                                [None],
                                {
                                    "frame": {"duration": 0, "redraw": False},
                                    "mode": "immediate",
                                    "transition": {"duration": 0},
                                },
                            ],
                        },
                    ],
                }
            ],
            sliders=[
                {
                    "x": 0.08,
                    "y": -0.02,
                    "len": 0.88,
                    "currentvalue": {"prefix": "time [s]: "},
                    "pad": {"t": 30},
                    "steps": slider_steps,
                }
            ],
        ),
    )
    return figure


def make_contact_side_animation(
    result: Dict[str, object],
    variant_label: str,
    start_time_s: float = 0.0,
    axis_length: float = 0.08,
):
    import plotly.graph_objects as go

    displayed_frames = _displayed_contact_frames(result, start_time_s)

    plotted_frames = []
    all_points = []
    for index, frame in enumerate(displayed_frames):
        traces = _side_animation_frame_traces(
            frame, axis_length, show_legend=(index == 0)
        )
        plotted_frames.append(
            go.Frame(
                name=str(index),
                data=traces,
                traces=list(range(len(traces))),
                layout=go.Layout(
                    title_text=(
                        f"{variant_label} side view: contact update at "
                        f"t={float(frame['timestamp_s']) - start_time_s:.2f} s"
                    )
                ),
            )
        )
        imu_origin = np.asarray(frame["imu_position"], dtype=float)
        body_origin = np.asarray(frame["body_position"], dtype=float)
        all_points.extend([imu_origin[[0, 2]], body_origin[[0, 2]]])
        footholds = np.asarray(frame["active_footholds"], dtype=float)
        if footholds.size:
            all_points.extend([point[[0, 2]] for point in footholds])
        breadcrumbs = np.asarray(frame["breadcrumb_positions"], dtype=float)
        if breadcrumbs.size:
            all_points.extend([point[[0, 2]] for point in breadcrumbs])

    stacked_points = np.vstack(all_points)
    x_range = _axis_range(stacked_points[:, 0], 0.04, 0.08)
    z_range = _axis_range(stacked_points[:, 1], 0.03, 0.08)

    slider_steps = [
        {
            "args": [
                [frame.name],
                {
                    "frame": {"duration": 0, "redraw": True},
                    "mode": "immediate",
                    "transition": {"duration": 0},
                },
            ],
            "label": f"{float(displayed_frames[index]['timestamp_s']) - start_time_s:.2f}",
            "method": "animate",
        }
        for index, frame in enumerate(plotted_frames)
    ]

    figure = go.Figure(
        data=plotted_frames[0].data,
        frames=plotted_frames,
        layout=go.Layout(
            title=(
                f"{variant_label} side view: contact update at "
                f"t={float(displayed_frames[0]['timestamp_s']) - start_time_s:.2f} s"
            ),
            template="plotly_white",
            width=950,
            height=560,
            xaxis=dict(title="x [m]", range=x_range),
            yaxis=dict(title="z [m]", range=z_range, scaleanchor="x", scaleratio=1.0),
            legend=dict(x=0.02, y=0.98),
            updatemenus=[
                {
                    "type": "buttons",
                    "direction": "left",
                    "x": 0.02,
                    "y": 1.08,
                    "showactive": False,
                    "buttons": [
                        {
                            "label": "Play",
                            "method": "animate",
                            "args": [
                                None,
                                {
                                    "frame": {"duration": 180, "redraw": True},
                                    "fromcurrent": True,
                                    "transition": {"duration": 0},
                                },
                            ],
                        },
                        {
                            "label": "Pause",
                            "method": "animate",
                            "args": [
                                [None],
                                {
                                    "frame": {"duration": 0, "redraw": False},
                                    "mode": "immediate",
                                    "transition": {"duration": 0},
                                },
                            ],
                        },
                    ],
                }
            ],
            sliders=[
                {
                    "x": 0.08,
                    "y": -0.08,
                    "len": 0.88,
                    "currentvalue": {"prefix": "time [s]: "},
                    "pad": {"t": 30},
                    "steps": slider_steps,
                }
            ],
        ),
    )
    return figure


def write_contact_animation_gif(
    result: Dict[str, object],
    variant_label: str,
    output_path: str | Path,
    start_time_s: float = 0.0,
    axis_length: float = 0.08,
    speedup: float = 5.0,
):
    displayed_frames = _displayed_contact_frames(result, start_time_s)
    figure = make_contact_animation(
        result, variant_label, start_time_s=start_time_s, axis_length=axis_length
    )
    _render_figure_frames_to_gif(
        figure,
        output_path,
        _frame_durations_ms(displayed_frames, start_time_s, speedup),
    )


def write_contact_side_animation_gif(
    result: Dict[str, object],
    variant_label: str,
    output_path: str | Path,
    start_time_s: float = 0.0,
    axis_length: float = 0.08,
    speedup: float = 5.0,
):
    displayed_frames = _displayed_contact_frames(result, start_time_s)
    figure = make_contact_side_animation(
        result, variant_label, start_time_s=start_time_s, axis_length=axis_length
    )
    _render_figure_frames_to_gif(
        figure,
        output_path,
        _frame_durations_ms(displayed_frames, start_time_s, speedup),
    )


def replay_legged_estimator(
    estimator,
    dataset: Dict[str, object],
    max_duration_seconds: float = float("inf"),
) -> Dict[str, object]:
    imu_samples: List[ImuSample] = dataset["imu_samples"]  # type: ignore[index]
    contact_events: List[ContactEvent] = dataset["contact_events"]  # type: ignore[index]
    body_P_imu = make_legged_estimator_params().body_P_imu

    trajectory: List[Dict[str, object]] = []
    contact_frames: List[Dict[str, object]] = []
    active_footholds_by_foot: Dict[int, np.ndarray] = {}
    breadcrumbs: List[Dict[str, object]] = []
    path_length = 0.0
    previous_position = None

    def log_state(kind: str, timestamp_s: float) -> None:
        nonlocal path_length, previous_position
        row = _trajectory_row(kind, timestamp_s, estimator)
        trajectory.append(row)
        position = np.array([row["x"], row["y"], row["z"]], dtype=float)
        if previous_position is not None:
            path_length += float(np.linalg.norm(position - previous_position))
        previous_position = position

    imu_index = 0
    contact_index = 0
    held_imu = imu_samples[0]
    current_time = held_imu.timestamp_s
    start_time = current_time

    def update_terrain_height(timestamp_s: float) -> None:
        elapsed_s = timestamp_s - start_time
        if elapsed_s <= DEFAULT_FORCED_HEIGHT_PRIOR_UNTIL_S:
            estimator.turnHeightPriorOn(DEFAULT_FORCED_TERRAIN_HEIGHT)
        else:
            estimator.turnHeightPriorOff()

    update_terrain_height(start_time)
    log_state("start", start_time)

    while True:
        next_imu_time = (
            imu_samples[imu_index + 1].timestamp_s
            if imu_index + 1 < len(imu_samples)
            else float("inf")
        )
        next_contact_time = (
            contact_events[contact_index].timestamp_s
            if contact_index < len(contact_events)
            else float("inf")
        )
        next_time = min(next_imu_time, next_contact_time)
        if not np.isfinite(next_time) or next_time - start_time > max_duration_seconds:
            break

        dt = next_time - current_time
        if dt > 0.0:
            estimator.predict(held_imu.omega, held_imu.specific_force, dt)
            current_time = next_time
            update_terrain_height(current_time)

        if abs(next_imu_time - next_time) < 1e-12:
            imu_index += 1
            held_imu = imu_samples[imu_index]
            log_state("imu", current_time)

        while (
            contact_index < len(contact_events)
            and abs(contact_events[contact_index].timestamp_s - current_time) < 1e-12
        ):
            event = contact_events[contact_index]
            active_feet = {contact.foot for contact in event.active_contacts}
            for foot, foothold in list(active_footholds_by_foot.items()):
                if foot not in active_feet:
                    breadcrumbs.append({"foot": foot, "position": foothold.tolist()})
                    del active_footholds_by_foot[foot]
            update_terrain_height(current_time)
            estimator.processContacts(event.active_contacts)
            log_state("contact", current_time)
            estimate_matrix = np.asarray(estimator.estimate().xMatrix(), dtype=float)
            footholds = estimate_matrix[:, 2:]
            for foot in active_feet:
                active_footholds_by_foot[foot] = np.asarray(
                    footholds[:, foot], dtype=float
                )
            contact_frames.append(
                _contact_frame_row(
                    current_time,
                    estimator,
                    event.active_contacts,
                    body_P_imu,
                    breadcrumbs,
                )
            )
            contact_index += 1

    final_row = _trajectory_row("final", current_time, estimator)
    return {
        "trajectory": trajectory,
        "contact_frames": contact_frames,
        "final": final_row,
        "path_length": path_length,
    }


def replay_legged_variants(
    dataset: Dict[str, object] | None = None,
    max_duration_seconds: float = float("inf"),
) -> Dict[str, Dict[str, object]]:
    dataset = load_legged_csv_dataset() if dataset is None else dataset
    foot_names: Sequence[str] = dataset["foot_names"]  # type: ignore[index]
    params = make_legged_estimator_params()
    results: Dict[str, Dict[str, object]] = {}
    for variant_name in SHOWCASE_VARIANTS:
        estimator = make_legged_estimator(variant_name, foot_names, params=params)
        results[variant_name] = replay_legged_estimator(
            estimator,
            dataset,
            max_duration_seconds=max_duration_seconds,
        )
    return results
