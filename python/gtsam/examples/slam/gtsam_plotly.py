# gtsam_plotly.py
import base64
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

import graphviz
import numpy as np
import plotly.graph_objects as go
from tqdm.notebook import tqdm  # Optional progress bar

import gtsam


# --- Dataclass for History ---
@dataclass
class SlamFrameData:
    """Holds all data needed for a single frame of the SLAM animation."""

    step_index: int
    results: gtsam.Values  # Estimates for variables active at this step
    marginals: Optional[gtsam.Marginals]  # Marginals for variables active at this step
    graph_dot_string: Optional[str] = None  # Graphviz DOT string for visualization


# --- Core Ellipse Calculation & Path Generation ---


def create_ellipse_path_from_cov(
    cx: float, cy: float, cov_matrix: np.ndarray, scale: float = 2.0, N: int = 60
) -> str:
    """Generates SVG path string for an ellipse from 2x2 covariance."""
    cov = cov_matrix[:2, :2] + np.eye(2) * 1e-9  # Ensure positive definite 2x2
    try:
        eigvals, eigvecs = np.linalg.eigh(cov)
        eigvals = np.maximum(eigvals, 1e-9)  # Ensure positive eigenvalues
        minor_std, major_std = np.sqrt(eigvals)  # eigh sorts ascending
        v_minor, v_major = eigvecs[:, 0], eigvecs[:, 1]
    except np.linalg.LinAlgError:
        # Fallback to a small circle if decomposition fails
        radius = 0.1 * scale
        t = np.linspace(0, 2 * np.pi, N)
        x_p = cx + radius * np.cos(t)
        y_p = cy + radius * np.sin(t)
    else:
        # Parametric equation using eigenvectors and eigenvalues
        t = np.linspace(0, 2 * np.pi, N)
        cos_t, sin_t = np.cos(t), np.sin(t)
        x_p = cx + scale * (
            major_std * cos_t * v_major[0] + minor_std * sin_t * v_minor[0]
        )
        y_p = cy + scale * (
            major_std * cos_t * v_major[1] + minor_std * sin_t * v_minor[1]
        )

    # Build SVG path string
    path = (
        f"M {x_p[0]},{y_p[0]} "
        + " ".join(f"L{x_},{y_}" for x_, y_ in zip(x_p[1:], y_p[1:]))
        + " Z"
    )
    return path


# --- Plotly Element Generators ---


def create_gt_landmarks_trace(
    landmarks_gt: Optional[np.ndarray],
) -> Optional[go.Scatter]:
    """Creates scatter trace for ground truth landmarks."""
    if landmarks_gt is None or landmarks_gt.size == 0:
        return None
    return go.Scatter(
        x=landmarks_gt[0, :],
        y=landmarks_gt[1, :],
        mode="markers",
        marker=dict(color="black", size=8, symbol="star"),
        name="Landmarks GT",
    )


def create_gt_path_trace(poses_gt: Optional[List[gtsam.Pose2]]) -> Optional[go.Scatter]:
    """Creates line trace for ground truth path."""
    if not poses_gt:
        return None
    return go.Scatter(
        x=[p.x() for p in poses_gt],
        y=[p.y() for p in poses_gt],
        mode="lines",
        line=dict(color="gray", width=1, dash="dash"),
        name="Path GT",
    )


def create_est_path_trace(
    est_path_x: List[float], est_path_y: List[float]
) -> go.Scatter:
    """Creates trace for the estimated path (all poses up to current)."""
    return go.Scatter(
        x=est_path_x,
        y=est_path_y,
        mode="lines+markers",
        line=dict(color="rgba(255, 0, 0, 0.3)", width=1),  # Fainter line for history
        marker=dict(size=4, color="red"),  # Keep markers prominent
        name="Path Est",
    )


def create_current_pose_trace(
    current_pose: Optional[gtsam.Pose2],
) -> Optional[go.Scatter]:
    """Creates a single marker trace for the current estimated pose."""
    if current_pose is None:
        return None
    return go.Scatter(
        x=[current_pose.x()],
        y=[current_pose.y()],
        mode="markers",
        marker=dict(color="magenta", size=10, symbol="cross"),
        name="Current Pose",
    )


def create_est_landmarks_trace(
    est_landmarks_x: List[float], est_landmarks_y: List[float]
) -> Optional[go.Scatter]:
    """Creates trace for currently estimated landmarks."""
    if not est_landmarks_x:
        return None
    return go.Scatter(
        x=est_landmarks_x,
        y=est_landmarks_y,
        mode="markers",
        marker=dict(color="blue", size=6, symbol="x"),
        name="Landmarks Est",
    )


def _create_ellipse_shape_dict(
    cx: float, cy: float, cov: np.ndarray, scale: float, fillcolor: str, line_color: str
) -> Dict[str, Any]:
    """Helper: Creates dict for a Plotly ellipse shape from covariance."""
    path = create_ellipse_path_from_cov(cx, cy, cov, scale)
    return dict(
        type="path",
        path=path,
        xref="x",
        yref="y",
        fillcolor=fillcolor,
        line_color=line_color,
        opacity=0.7,  # Make ellipses slightly transparent
    )


def create_pose_ellipse_shape(
    pose_mean_xy: np.ndarray, pose_cov: np.ndarray, scale: float
) -> Dict[str, Any]:
    """Creates shape dict for a pose covariance ellipse."""
    return _create_ellipse_shape_dict(
        cx=pose_mean_xy[0],
        cy=pose_mean_xy[1],
        cov=pose_cov,
        scale=scale,
        fillcolor="rgba(255,0,255,0.2)",  # Magenta fill
        line_color="rgba(255,0,255,0.5)",  # Magenta line
    )


def create_landmark_ellipse_shape(
    lm_mean_xy: np.ndarray, lm_cov: np.ndarray, scale: float
) -> Dict[str, Any]:
    """Creates shape dict for a landmark covariance ellipse."""
    return _create_ellipse_shape_dict(
        cx=lm_mean_xy[0],
        cy=lm_mean_xy[1],
        cov=lm_cov,
        scale=scale,
        fillcolor="rgba(0,0,255,0.1)",  # Blue fill
        line_color="rgba(0,0,255,0.3)",  # Blue line
    )


def dot_string_to_base64_svg(
    dot_string: Optional[str], engine: str = "neato"
) -> Optional[str]:
    """Renders a DOT string to a base64 encoded SVG using graphviz."""
    if not dot_string:
        return None
    try:
        source = graphviz.Source(dot_string, engine=engine)
        svg_bytes = source.pipe(format="svg")
        encoded_string = base64.b64encode(svg_bytes).decode("utf-8")
        return f"data:image/svg+xml;base64,{encoded_string}"
    except graphviz.backend.execute.CalledProcessError as e:
        print(f"Graphviz rendering error ({engine}): {e}")
        return None
    except Exception as e:
        print(f"Unexpected error during Graphviz SVG generation: {e}")
        return None


# --- Frame Content Generation ---
def generate_frame_content(
    frame_data: SlamFrameData,
    X: Callable[[int], int],
    L: Callable[[int], int],
    max_landmark_index: int,
    ellipse_scale: float = 2.0,
    graphviz_engine: str = "neato",
    verbose: bool = False,
) -> Tuple[List[go.Scatter], List[Dict[str, Any]], Optional[Dict[str, Any]]]:
    """Generates dynamic traces, shapes, and layout image for a single frame."""
    k = frame_data.step_index
    # Use the results specific to this frame for current elements
    step_results = frame_data.results
    step_marginals = frame_data.marginals

    frame_dynamic_traces: List[go.Scatter] = []
    frame_shapes: List[Dict[str, Any]] = []
    layout_image: Optional[Dict[str, Any]] = None

    # 1. Estimated Path (Full History or Partial)
    est_path_x = []
    est_path_y = []
    current_pose_est = None

    # Plot poses currently existing in the step_results (e.g., within lag)
    for i in range(k + 1):  # Check poses up to current step index
        pose_key = X(i)
        if step_results.exists(pose_key):
            pose = step_results.atPose2(pose_key)
            est_path_x.append(pose.x())
            est_path_y.append(pose.y())
            if i == k:
                current_pose_est = pose

    path_trace = create_est_path_trace(est_path_x, est_path_y)
    if path_trace:
        frame_dynamic_traces.append(path_trace)

    # Add a distinct marker for the current pose estimate
    current_pose_trace = create_current_pose_trace(current_pose_est)
    if current_pose_trace:
        frame_dynamic_traces.append(current_pose_trace)

    # 2. Estimated Landmarks (Only those present in step_results)
    est_landmarks_x, est_landmarks_y, landmark_keys = [], [], []
    for j in range(max_landmark_index + 1):
        lm_key = L(j)
        # Check existence in the results for the *current frame*
        if step_results.exists(lm_key):
            lm_point = step_results.atPoint2(lm_key)
            est_landmarks_x.append(lm_point[0])
            est_landmarks_y.append(lm_point[1])
            landmark_keys.append(lm_key)  # Store keys for covariance lookup

    lm_trace = create_est_landmarks_trace(est_landmarks_x, est_landmarks_y)
    if lm_trace:
        frame_dynamic_traces.append(lm_trace)

    # 3. Covariance Ellipses (Only for items in step_results AND step_marginals)
    if step_marginals:
        # Current Pose Ellipse
        pose_key = X(k)
        # Retrieve cov from marginals specific to this frame
        cov = step_marginals.marginalCovariance(pose_key)
        # Ensure mean comes from the pose in current results
        mean = step_results.atPose2(pose_key).translation()
        frame_shapes.append(create_pose_ellipse_shape(mean, cov, ellipse_scale))

        # Landmark Ellipses (Iterate over keys found in step_results)
        for lm_key in landmark_keys:
            try:
                # Retrieve cov from marginals specific to this frame
                cov = step_marginals.marginalCovariance(lm_key)
                # Ensure mean comes from the landmark in current results
                mean = step_results.atPoint2(lm_key)
                frame_shapes.append(
                    create_landmark_ellipse_shape(mean, cov, ellipse_scale)
                )
            except RuntimeError:  # Covariance might not be available
                if verbose:
                    print(
                        f"Warn: LM {gtsam.Symbol(lm_key).index()} cov not in marginals @ step {k}"
                    )
            except Exception as e:
                if verbose:
                    print(
                        f"Warn: LM {gtsam.Symbol(lm_key).index()} cov OTHER err @ step {k}: {e}"
                    )

    # 4. Graph Image for Layout
    img_src = dot_string_to_base64_svg(
        frame_data.graph_dot_string, engine=graphviz_engine
    )
    if img_src:
        layout_image = dict(
            source=img_src,
            xref="paper",
            yref="paper",
            x=0,
            y=1,
            sizex=0.48,
            sizey=1,
            xanchor="left",
            yanchor="top",
            layer="below",
            sizing="contain",
        )

    # Return dynamic elements for this frame
    return frame_dynamic_traces, frame_shapes, layout_image


# --- Figure Configuration ---


def configure_figure_layout(
    fig: go.Figure,
    num_steps: int,
    world_size: float,
    initial_shapes: List[Dict[str, Any]],
    initial_image: Optional[Dict[str, Any]],
) -> None:
    """Configures Plotly figure layout for side-by-side display."""
    steps = list(range(num_steps + 1))
    plot_domain = [0.52, 1.0]  # Right pane for the SLAM plot

    sliders = [
        dict(
            active=0,
            currentvalue={"prefix": "Step: "},
            pad={"t": 50},
            steps=[
                dict(
                    label=str(k),
                    method="animate",
                    args=[
                        [str(k)],
                        dict(
                            mode="immediate",
                            frame=dict(duration=100, redraw=True),
                            transition=dict(duration=0),
                        ),
                    ],
                )
                for k in steps
            ],
        )
    ]
    updatemenus = [
        dict(
            type="buttons",
            showactive=False,
            direction="left",
            pad={"r": 10, "t": 20},
            x=plot_domain[0],
            xanchor="left",
            y=0,
            yanchor="top",
            buttons=[
                dict(
                    label="Play",
                    method="animate",
                    args=[
                        None,
                        dict(
                            mode="immediate",
                            frame=dict(duration=100, redraw=True),
                            transition=dict(duration=0),
                            fromcurrent=True,
                        ),
                    ],
                ),
                dict(
                    label="Pause",
                    method="animate",
                    args=[
                        [None],
                        dict(
                            mode="immediate",
                            frame=dict(duration=0, redraw=False),
                            transition=dict(duration=0),
                        ),
                    ],
                ),
            ],
        )
    ]

    fig.update_layout(
        title="Factor Graph SLAM Animation (Graph Left, Results Right)",
        xaxis=dict(
            range=[-world_size / 2 - 2, world_size / 2 + 2],
            domain=plot_domain,
            constrain="domain",
        ),
        yaxis=dict(
            range=[-world_size / 2 - 2, world_size / 2 + 2],
            scaleanchor="x",
            scaleratio=1,
            domain=[0, 1],
        ),
        autosize=True,
        height=600,
        hovermode="closest",
        updatemenus=updatemenus,
        sliders=sliders,
        shapes=initial_shapes,  # Initial shapes (frame 0)
        images=([initial_image] if initial_image else []),  # Initial image (frame 0)
        legend=dict(
            x=plot_domain[0],
            y=1,
            traceorder="normal",  # Position legend
            bgcolor="rgba(255,255,255,0.5)",
        ),
        showlegend=False,
        margin=dict(l=0, r=0, t=50, b=50),
    )


# --- Main Animation Orchestrator ---


def create_slam_animation(
    history: List[SlamFrameData],
    X: Callable[[int], int],
    L: Callable[[int], int],
    max_landmark_index: int,
    landmarks_gt_array: Optional[np.ndarray] = None,
    poses_gt: Optional[List[gtsam.Pose2]] = None,
    world_size: float = 20.0,
    ellipse_scale: float = 2.0,
    graphviz_engine: str = "neato",
    verbose_cov_errors: bool = False,
) -> go.Figure:
    """Creates a side-by-side Plotly SLAM animation using a history of dataclasses."""
    if not history:
        raise ValueError("History cannot be empty.")
    print("Generating Plotly animation...")
    num_steps = history[-1].step_index
    fig = go.Figure()

    # 1. Create static GT traces ONCE
    gt_traces = []
    gt_lm_trace = create_gt_landmarks_trace(landmarks_gt_array)
    if gt_lm_trace:
        gt_traces.append(gt_lm_trace)
    gt_path_trace = create_gt_path_trace(poses_gt)
    if gt_path_trace:
        gt_traces.append(gt_path_trace)

    # 2. Generate content for the initial frame (k=0) to set up the figure
    initial_frame_data = next((item for item in history if item.step_index == 0), None)
    if initial_frame_data is None:
        raise ValueError("History must contain data for step 0.")

    (
        initial_dynamic_traces,
        initial_shapes,
        initial_image,
    ) = generate_frame_content(
        initial_frame_data,
        X,
        L,
        max_landmark_index,
        ellipse_scale,
        graphviz_engine,
        verbose_cov_errors,
    )

    # 3. Add initial traces (GT + dynamic frame 0)
    for trace in gt_traces:
        fig.add_trace(trace)
    for trace in initial_dynamic_traces:
        fig.add_trace(trace)

    # 4. Generate frames for the animation (k=0 to num_steps)
    frames = []
    steps_iterable = range(num_steps + 1)
    steps_iterable = tqdm(steps_iterable, desc="Creating Frames")

    for k in steps_iterable:
        frame_data = next((item for item in history if item.step_index == k), None)

        # Generate dynamic content specific to this frame
        frame_dynamic_traces, frame_shapes, layout_image = generate_frame_content(
            frame_data,
            X,
            L,
            max_landmark_index,
            ellipse_scale,
            graphviz_engine,
            verbose_cov_errors,
        )

        # Frame definition: includes static GT + dynamic traces for this step
        # Layout updates only include shapes and images for this step
        frames.append(
            go.Frame(
                data=gt_traces
                + frame_dynamic_traces,  # GT must be in each frame's data
                name=str(k),
                layout=go.Layout(
                    shapes=frame_shapes,  # Replaces shapes list for this frame
                    images=(
                        [layout_image] if layout_image else []
                    ),  # Replaces image list
                ),
            )
        )

    # 5. Assign frames to the figure
    fig.update(frames=frames)

    # 6. Configure overall layout (sliders, buttons, axes, etc.)
    configure_figure_layout(fig, num_steps, world_size, initial_shapes, initial_image)

    print("Plotly animation generated.")
    return fig
