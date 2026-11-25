"""Plotly visualization helpers for KalmanFilteringAndSmoothing.ipynb."""

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots


CONFIDENCE_95_2D = float(np.sqrt(5.991464547107979))  # sqrt(chi2_{2, 0.95})


def _ellipse_trace(means, covs, scale=CONFIDENCE_95_2D, points=60):
    angles = np.linspace(0.0, 2.0 * np.pi, points)
    circle = np.vstack((np.cos(angles), np.sin(angles)))
    xs, ys = [], []
    for mean, cov in zip(means, covs):
        pts = mean[:, None] + scale * (np.linalg.cholesky(cov) @ circle)
        xs.extend(pts[0].tolist() + [None])
        ys.extend(pts[1].tolist() + [None])
    return xs, ys


def _add_filter_traces(
    fig,
    truth,
    measurements,
    p_means,
    p_covs,
    post_means,
    post_covs,
    preview_steps=6,
    col=None,
    show_legend=True,
):
    kw = dict(row=1, col=col) if col is not None else {}
    fig.add_trace(
        go.Scatter(
            x=truth[:preview_steps, 0],
            y=truth[:preview_steps, 1],
            name="Ground truth",
            legendgroup="truth",
            showlegend=show_legend,
            mode="lines+markers",
            marker=dict(size=5),
            line=dict(color="#263544", dash="dot"),
        ),
        **kw,
    )
    fig.add_trace(
        go.Scatter(
            x=measurements[1:preview_steps, 0],
            y=measurements[1:preview_steps, 1],
            name="2D measurements",
            legendgroup="meas",
            showlegend=show_legend,
            mode="markers",
            marker=dict(color="#D62728", size=7, symbol="x"),
        ),
        **kw,
    )
    px, py = _ellipse_trace(p_means, p_covs)
    fig.add_trace(
        go.Scatter(
            x=px,
            y=py,
            name="Predictive density (95%)",
            legendgroup="pred",
            showlegend=show_legend,
            mode="lines",
            line=dict(color="#CA641E", dash="dash", width=1.6),
        ),
        **kw,
    )
    fig.add_trace(
        go.Scatter(
            x=[m[0] for m in p_means],
            y=[m[1] for m in p_means],
            name="Predictive mean",
            legendgroup="pred",
            showlegend=False,
            mode="markers",
            marker=dict(color="#CA641E", size=6, symbol="circle-open"),
        ),
        **kw,
    )
    qx, qy = _ellipse_trace(post_means, post_covs)
    fig.add_trace(
        go.Scatter(
            x=qx,
            y=qy,
            name="Updated posterior (95%)",
            legendgroup="post",
            showlegend=show_legend,
            mode="lines",
            fill="toself",
            fillcolor="rgba(51, 99, 170, 0.10)",
            line=dict(color="#3363AA", width=2.0),
        ),
        **kw,
    )
    fig.add_trace(
        go.Scatter(
            x=[m[0] for m in post_means],
            y=[m[1] for m in post_means],
            name="Posterior mean",
            legendgroup="post",
            showlegend=False,
            mode="lines+markers",
            marker=dict(color="#3363AA", size=6),
            line=dict(color="#3363AA", width=1.4),
        ),
        **kw,
    )


def plot_kf_ellipses(
    truth, measurements, pred_means, pred_covs, post_means, post_covs, preview_steps=6
):
    fig = go.Figure()
    _add_filter_traces(
        fig,
        truth,
        measurements,
        pred_means[1:preview_steps],
        pred_covs[1:preview_steps],
        post_means[:preview_steps],
        post_covs[:preview_steps],
        preview_steps=preview_steps,
    )
    fig.update_xaxes(title_text="x (m)")
    fig.update_yaxes(title_text="y (m)", scaleanchor="x", scaleratio=1)
    fig.update_layout(
        title="Kalman filter: predict (grow) and update (shrink)",
        template="plotly_white",
        height=440,
        legend=dict(orientation="h", y=-0.22),
    )
    return fig


def plot_kf_vs_elimination(
    truth,
    measurements,
    kf_pred_means,
    kf_pred_covs,
    kf_means,
    kf_covs,
    elim_pred_means,
    elim_pred_covs,
    elim_means,
    elim_covs,
    preview_steps=6,
):
    fig = make_subplots(
        rows=1,
        cols=2,
        subplot_titles=("Textbook Kalman filter", "Factor graph elimination"),
        horizontal_spacing=0.10,
    )
    _add_filter_traces(
        fig,
        truth,
        measurements,
        kf_pred_means[1:preview_steps],
        kf_pred_covs[1:preview_steps],
        kf_means[:preview_steps],
        kf_covs[:preview_steps],
        preview_steps=preview_steps,
        col=1,
        show_legend=True,
    )
    _add_filter_traces(
        fig,
        truth,
        measurements,
        elim_pred_means[1:preview_steps],
        elim_pred_covs[1:preview_steps],
        elim_means[:preview_steps],
        elim_covs[:preview_steps],
        preview_steps=preview_steps,
        col=2,
        show_legend=False,
    )
    fig.update_xaxes(title_text="x (m)", row=1, col=1)
    fig.update_yaxes(title_text="y (m)", scaleanchor="x", scaleratio=1, row=1, col=1)
    fig.update_xaxes(title_text="x (m)", matches="x", row=1, col=2)
    fig.update_yaxes(
        title_text="y (m)", matches="y", scaleanchor="x2", scaleratio=1, row=1, col=2
    )
    fig.update_layout(
        template="plotly_white",
        height=440,
        legend=dict(orientation="h", y=-0.22),
    )
    return fig


def plot_linear_smoothing(
    truth, measurements, filtered, filter_cov, lagged, lagged_cov, lag
):
    fig = make_subplots(
        rows=1,
        cols=2,
        subplot_titles=("2D trajectory estimates", "Posterior RMS position uncertainty"),
        horizontal_spacing=0.17,
    )
    fig.add_trace(
        go.Scatter(
            x=measurements[1:, 0],
            y=measurements[1:, 1],
            name="Measurements",
            mode="markers",
            marker=dict(color="#8A99A8", size=5, symbol="x"),
        ),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Scatter(
            x=truth[:, 0],
            y=truth[:, 1],
            name="Ground truth",
            mode="lines",
            line=dict(color="#263544", dash="dot"),
        ),
        row=1,
        col=1,
    )
    for label, states, covariances, color in (
        ("Filter (1 state)", filtered, filter_cov, "#CA641E"),
        (f"Fixed-lag ({lag + 1} states)", lagged, lagged_cov, "#3363AA"),
    ):
        fig.add_trace(
            go.Scatter(
                x=states[:, 0],
                y=states[:, 1],
                name=label,
                legendgroup=label,
                mode="lines+markers",
                marker=dict(size=4),
                line=dict(color=color),
            ),
            row=1,
            col=1,
        )
        uncertainty = np.sqrt(np.trace(covariances, axis1=1, axis2=2))
        fig.add_trace(
            go.Scatter(
                x=np.arange(len(truth)),
                y=uncertainty,
                name=label,
                legendgroup=label,
                showlegend=False,
                mode="lines",
                line=dict(color=color),
            ),
            row=1,
            col=2,
        )
    fig.update_xaxes(title_text="x position (m)", row=1, col=1)
    fig.update_yaxes(title_text="y position (m)", scaleanchor="x", row=1, col=1)
    fig.update_xaxes(title_text="State index", row=1, col=2)
    fig.update_yaxes(title_text="RMS uncertainty (m)", row=1, col=2)
    fig.update_layout(
        template="plotly_white",
        height=430,
        legend=dict(orientation="h", y=-0.27),
    )
    return fig


def plot_marginals_snake(truth, measurements, snapshots, lagged, lagged_cov):
    steps = sorted(snapshots)
    fig = make_subplots(
        rows=1,
        cols=len(steps),
        subplot_titles=[f"Step k = {k}" for k in steps],
        horizontal_spacing=0.06,
    )
    for col, k in enumerate(steps, start=1):
        show = col == 1
        indices, means, covs = snapshots[k]
        first = indices[0]
        end = steps[-1] + 1
        fig.add_trace(
            go.Scatter(
                x=truth[:end, 0],
                y=truth[:end, 1],
                name="Ground truth",
                legendgroup="truth",
                showlegend=show,
                mode="lines",
                line=dict(color="#263544", dash="dot", width=1.4),
            ),
            row=1,
            col=col,
        )
        fig.add_trace(
            go.Scatter(
                x=measurements[1 : k + 1, 0],
                y=measurements[1 : k + 1, 1],
                name="Measurements (≤ k)",
                legendgroup="meas",
                showlegend=show,
                mode="markers",
                marker=dict(color="#D62728", size=6, symbol="x"),
            ),
            row=1,
            col=col,
        )
        if first > 0:
            px, py = _ellipse_trace(lagged[:first], lagged_cov[:first])
            fig.add_trace(
                go.Scatter(
                    x=px,
                    y=py,
                    name="Marginalized (95%)",
                    legendgroup="old",
                    showlegend=show,
                    mode="lines",
                    line=dict(color="#A8B5C2", dash="dot", width=1.4),
                ),
                row=1,
                col=col,
            )
            fig.add_trace(
                go.Scatter(
                    x=lagged[:first, 0],
                    y=lagged[:first, 1],
                    name="Marginalized mean",
                    legendgroup="old",
                    showlegend=False,
                    mode="markers",
                    marker=dict(color="#8A99A8", size=5),
                ),
                row=1,
                col=col,
            )
        sx, sy = _ellipse_trace(means, covs)
        fig.add_trace(
            go.Scatter(
                x=sx,
                y=sy,
                name="Active window (95%)",
                legendgroup="snake",
                showlegend=show,
                mode="lines",
                fill="toself",
                fillcolor="rgba(51, 99, 170, 0.14)",
                line=dict(color="#3363AA", width=2.0),
            ),
            row=1,
            col=col,
        )
        fig.add_trace(
            go.Scatter(
                x=[m[0] for m in means],
                y=[m[1] for m in means],
                text=[f"x{j}" for j in indices],
                textposition="top center",
                name="Active window means",
                legendgroup="snake",
                showlegend=False,
                mode="lines+markers+text",
                marker=dict(color="#3363AA", size=6),
                line=dict(color="#3363AA", width=2.0),
            ),
            row=1,
            col=col,
        )
        # Highlight the newest state (head of the snake).
        hx, hy = _ellipse_trace([means[-1]], [covs[-1]])
        fig.add_trace(
            go.Scatter(
                x=hx,
                y=hy,
                name="Head xₖ (95%)",
                legendgroup="head",
                showlegend=show,
                mode="lines",
                line=dict(color="#CA641E", width=2.2),
            ),
            row=1,
            col=col,
        )
        fig.update_xaxes(title_text="x (m)", matches="x" if col > 1 else None, row=1, col=col)
        fig.update_yaxes(
            title_text="y (m)" if col == 1 else "",
            matches="y" if col > 1 else None,
            scaleanchor=f"x{col if col > 1 else ''}",
            scaleratio=1,
            row=1,
            col=col,
        )
    fig.update_layout(
        title="Fixed-lag marginals snake advancing across steps",
        template="plotly_white",
        height=400,
        legend=dict(orientation="h", y=-0.24),
    )
    return fig


def plot_circle_ekf(truth, estimates, beacons, first_trace, objective_fn):
    fig = make_subplots(
        rows=1,
        cols=2,
        subplot_titles=("Tracking a circle", "First range update"),
    )
    for label, poses in {"Truth": truth, **estimates}.items():
        fig.add_scatter(
            x=poses[:, 0],
            y=poses[:, 1],
            mode="lines",
            name=label,
            line=dict(dash="dash" if label == "Dead reckoning" else "solid"),
            row=1,
            col=1,
        )
    fig.add_scatter(
        x=beacons[:, 0],
        y=beacons[:, 1],
        mode="markers",
        name="Beacons",
        marker=dict(symbol="diamond", size=9, color="black"),
        row=1,
        col=1,
    )
    fig.add_scatter(
        x=np.arange(len(first_trace)),
        y=[objective_fn(x) for x in first_trace],
        mode="lines+markers",
        name="Same posterior objective",
        row=1,
        col=2,
    )
    fig.update_xaxes(title_text="x (m)", row=1, col=1)
    fig.update_yaxes(title_text="y (m)", scaleanchor="x", scaleratio=1, row=1, col=1)
    fig.update_xaxes(title_text="Gauss–Newton steps (EKF stops at 1)", row=1, col=2)
    fig.update_yaxes(
        title_text="Negative log posterior (up to a constant)",
        type="log",
        row=1,
        col=2,
    )
    fig.update_layout(
        template="plotly_white",
        height=440,
        legend=dict(orientation="h", y=-0.24),
    )
    return fig


def plot_nonlinear_smoothing(truth, ekf, iekf, smoothers, beacons):
    fig = go.Figure()
    fig.add_scatter(
        x=truth[:, 0],
        y=truth[:, 1],
        mode="lines",
        name="Truth",
        line=dict(color="#263544", dash="dot"),
    )
    colors = ["#3363AA", "#007E73"]
    traces = [
        ("EKF", ekf, "#CA641E"),
        ("Iterated EKF", iekf, "#E09F3E"),
        *[(label, pos, color) for (label, pos), color in zip(smoothers.items(), colors)],
    ]
    for label, pos, color in traces:
        fig.add_scatter(
            x=pos[:, 0],
            y=pos[:, 1],
            mode="lines",
            name=label,
            line=dict(color=color),
        )
    fig.add_scatter(
        x=beacons[:, 0],
        y=beacons[:, 1],
        mode="markers",
        name="Beacons",
        marker=dict(symbol="diamond", size=9, color="black"),
    )
    fig.update_xaxes(title_text="x (m)")
    fig.update_yaxes(title_text="y (m)", scaleanchor="x", scaleratio=1)
    fig.update_layout(
        title="Circular trajectory: EKF / IEKF vs. nonlinear fixed-lag smoothers",
        template="plotly_white",
        height=440,
        legend=dict(orientation="h", y=-0.22),
    )
    return fig
