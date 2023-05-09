"""Various plotting utlities."""

# pylint: disable=no-member, invalid-name

from typing import Iterable, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches
from mpl_toolkits.mplot3d import Axes3D  # pylint: disable=unused-import

import gtsam
from gtsam import Marginals, Point2, Point3, Pose2, Pose3, Values


# For translation between a scaling of the uncertainty ellipse and the 
# percentage of inliers see discussion in 
#   [PR 1067](https://github.com/borglab/gtsam/pull/1067)
# and the notebook python/gtsam/notebooks/ellipses.ipynb (needs scipy).
#
# In the following, the default scaling is chosen for 95% inliers, which
# translates to the following sigma values:
# 1D: 1.959963984540
# 2D: 2.447746830681
# 3D: 2.795483482915
#
# Further references are Stochastic Models, Estimation, and Control Vol 1 by Maybeck,
# page 366 and https://www.xarg.org/2018/04/how-to-plot-a-covariance-error-ellipse/
#
# For reference, here are the inlier percentages for some sigma values:
#   	    1    	    2    	    3    	    4    	    5
# 1D	68.26895	95.44997	99.73002	99.99367	99.99994
# 2D	39.34693	86.46647	98.88910	99.96645	99.99963
# 3D	19.87480	73.85359	97.07091	99.88660	99.99846

def set_axes_equal(fignum: int) -> None:
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Args:
      fignum: An integer representing the figure number for Matplotlib.
    """
    fig = plt.figure(fignum)
    if not fig.axes:
        ax = fig.add_subplot(projection='3d')
    else:
        ax = fig.axes[0]

    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])

    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))

    ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
    ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
    ax.set_zlim3d([origin[2] - radius, origin[2] + radius])


def ellipsoid(rx: float, ry: float, rz: float,
              n: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Numpy equivalent of Matlab's ellipsoid function.

    Args:
        rx: Radius of ellipsoid in X-axis.
        ry: Radius of ellipsoid in Y-axis.
        rz: Radius of ellipsoid in Z-axis.
        n: The granularity of the ellipsoid plotted.

    Returns:
        The points in the x, y and z axes to use for the surface plot.
    """
    u = np.linspace(0, 2 * np.pi, n + 1)
    v = np.linspace(0, np.pi, n + 1)
    x = -rx * np.outer(np.cos(u), np.sin(v)).T
    y = -ry * np.outer(np.sin(u), np.sin(v)).T
    z = -rz * np.outer(np.ones_like(u), np.cos(v)).T

    return x, y, z


def plot_covariance_ellipse_3d(axes,
                               origin: Point3,
                               P: np.ndarray,
                               scale: float = 1,
                               n: int = 8,
                               alpha: float = 0.5) -> None:
    """
    Plots a Gaussian as an uncertainty ellipse

    The ellipse is scaled in such a way that 95% of drawn samples are inliers.
    Derivation of the scaling factor is explained at the beginning of this file.

    Args:
        axes (matplotlib.axes.Axes): Matplotlib axes.
        origin: The origin in the world frame.
        P: The marginal covariance matrix of the 3D point
            which will be represented as an ellipse.
        scale: Scaling factor of the radii of the covariance ellipse.
        n: Defines the granularity of the ellipse. Higher values indicate finer ellipses.
        alpha: Transparency value for the plotted surface in the range [0, 1].
    """
    # this corresponds to 95%, see note above
    k = 2.795483482915
    U, S, _ = np.linalg.svd(P)

    radii = k * np.sqrt(S)
    radii = radii * scale
    rx, ry, rz = radii

    # generate data for "unrotated" ellipsoid
    xc, yc, zc = ellipsoid(rx, ry, rz, n)

    # rotate data with orientation matrix U and center c
    data = np.kron(U[:, 0:1], xc) + np.kron(U[:, 1:2], yc) + \
        np.kron(U[:, 2:3], zc)
    n = data.shape[1]
    x = data[0:n, :] + origin[0]
    y = data[n:2 * n, :] + origin[1]
    z = data[2 * n:, :] + origin[2]

    axes.plot_surface(x, y, z, alpha=alpha, cmap='hot')


def plot_covariance_ellipse_2d(axes,
                               origin: Point2,
                               covariance: np.ndarray) -> None:
    """
    Plots a Gaussian as an uncertainty ellipse

    The ellipse is scaled in such a way that 95% of drawn samples are inliers.
    Derivation of the scaling factor is explained at the beginning of this file.

    Args:
        axes (matplotlib.axes.Axes): Matplotlib axes.
        origin: The origin in the world frame.
        covariance: The marginal covariance matrix of the 2D point
                    which will be represented as an ellipse.
    """

    w, v = np.linalg.eigh(covariance)

    # this corresponds to 95%, see note above
    k = 2.447746830681

    angle = np.arctan2(v[1, 0], v[0, 0])
    # We multiply k by 2 since k corresponds to the radius but Ellipse uses
    # the diameter.
    e1 = patches.Ellipse(origin,
                         np.sqrt(w[0]) * 2 * k,
                         np.sqrt(w[1]) * 2 * k,
                         np.rad2deg(angle),
                         fill=False)
    axes.add_patch(e1)


def plot_point2_on_axes(axes,
                        point: Point2,
                        linespec: str,
                        P: Optional[np.ndarray] = None) -> None:
    """
    Plot a 2D point and its corresponding uncertainty ellipse on given axis
    `axes` with given `linespec`.

    The uncertainty ellipse (if covariance is given) is scaled in such a way
    that 95% of drawn samples are inliers, see `plot_covariance_ellipse_2d`.

    Args:
        axes (matplotlib.axes.Axes): Matplotlib axes.
        point: The point to be plotted.
        linespec: String representing formatting options for Matplotlib.
        P: Marginal covariance matrix to plot the uncertainty of the estimation.
    """
    axes.plot([point[0]], [point[1]], linespec, marker='.', markersize=10)
    if P is not None:
        plot_covariance_ellipse_2d(axes, point, P)

def plot_point2(
    fignum: int,
    point: Point2,
    linespec: str,
    P: np.ndarray = None,
    axis_labels: Iterable[str] = ("X axis", "Y axis"),
) -> plt.Figure:
    """
    Plot a 2D point on given figure with given `linespec`.

    The uncertainty ellipse (if covariance is given) is scaled in such a way
    that 95% of drawn samples are inliers, see `plot_covariance_ellipse_2d`.

    Args:
        fignum: Integer representing the figure number to use for plotting.
        point: The point to be plotted.
        linespec: String representing formatting options for Matplotlib.
        P: Marginal covariance matrix to plot the uncertainty of the estimation.
        axis_labels: List of axis labels to set.

    Returns:
        fig: The matplotlib figure.

    """
    fig = plt.figure(fignum)
    axes = fig.gca()
    plot_point2_on_axes(axes, point, linespec, P)

    axes.set_xlabel(axis_labels[0])
    axes.set_ylabel(axis_labels[1])

    return fig


def plot_pose2_on_axes(axes,
                       pose: Pose2,
                       axis_length: float = 0.1,
                       covariance: np.ndarray = None) -> None:
    """
    Plot a 2D pose on given axis `axes` with given `axis_length`.

    The ellipse is scaled in such a way that 95% of drawn samples are inliers,
    see `plot_covariance_ellipse_2d`.

    Args:
        axes (matplotlib.axes.Axes): Matplotlib axes.
        pose: The pose to be plotted.
        axis_length: The length of the camera axes.
        covariance (numpy.ndarray): Marginal covariance matrix to plot
            the uncertainty of the estimation.
    """
    # get rotation and translation (center)
    gRp = pose.rotation().matrix()  # rotation from pose to global
    t = pose.translation()
    origin = t

    # draw the camera axes
    x_axis = origin + gRp[:, 0] * axis_length
    line = np.append(origin[np.newaxis], x_axis[np.newaxis], axis=0)
    axes.plot(line[:, 0], line[:, 1], 'r-')

    y_axis = origin + gRp[:, 1] * axis_length
    line = np.append(origin[np.newaxis], y_axis[np.newaxis], axis=0)
    axes.plot(line[:, 0], line[:, 1], 'g-')

    if covariance is not None:
        pPp = covariance[0:2, 0:2]
        gPp = np.matmul(np.matmul(gRp, pPp), gRp.T)
        plot_covariance_ellipse_2d(axes, origin, gPp)


def plot_pose2(
        fignum: int,
        pose: Pose2,
        axis_length: float = 0.1,
        covariance: np.ndarray = None,
        axis_labels=("X axis", "Y axis", "Z axis"),
) -> plt.Figure:
    """
    Plot a 2D pose on given figure with given `axis_length`.

    The uncertainty ellipse (if covariance is given) is scaled in such a way
    that 95% of drawn samples are inliers, see `plot_covariance_ellipse_2d`.

    Args:
        fignum: Integer representing the figure number to use for plotting.
        pose: The pose to be plotted.
        axis_length: The length of the camera axes.
        covariance: Marginal covariance matrix to plot
            the uncertainty of the estimation.
        axis_labels (iterable[string]): List of axis labels to set.
    """
    # get figure object
    fig = plt.figure(fignum)
    axes = fig.gca()
    plot_pose2_on_axes(axes,
                       pose,
                       axis_length=axis_length,
                       covariance=covariance)

    axes.set_xlabel(axis_labels[0])
    axes.set_ylabel(axis_labels[1])

    return fig


def plot_point3_on_axes(axes,
                        point: Point3,
                        linespec: str,
                        P: Optional[np.ndarray] = None) -> None:
    """
    Plot a 3D point on given axis `axes` with given `linespec`.

    The uncertainty ellipse (if covariance is given) is scaled in such a way
    that 95% of drawn samples are inliers, see `plot_covariance_ellipse_3d`.

    Args:
        axes (matplotlib.axes.Axes): Matplotlib axes.
        point: The point to be plotted.
        linespec: String representing formatting options for Matplotlib.
        P: Marginal covariance matrix to plot the uncertainty of the estimation.
    """
    axes.plot([point[0]], [point[1]], [point[2]], linespec)
    if P is not None:
        plot_covariance_ellipse_3d(axes, point, P)


def plot_point3(
    fignum: int,
    point: Point3,
    linespec: str,
    P: np.ndarray = None,
    axis_labels: Iterable[str] = ("X axis", "Y axis", "Z axis"),
) -> plt.Figure:
    """
    Plot a 3D point on given figure with given `linespec`.

    The uncertainty ellipse (if covariance is given) is scaled in such a way
    that 95% of drawn samples are inliers, see `plot_covariance_ellipse_3d`.

    Args:
        fignum: Integer representing the figure number to use for plotting.
        point: The point to be plotted.
        linespec: String representing formatting options for Matplotlib.
        P: Marginal covariance matrix to plot the uncertainty of the estimation.
        axis_labels: List of axis labels to set.

    Returns:
        fig: The matplotlib figure.

    """
    fig = plt.figure(fignum)
    if not fig.axes:
        axes = fig.add_subplot(projection='3d')
    else:
        axes = fig.axes[0]
    plot_point3_on_axes(axes, point, linespec, P)

    axes.set_xlabel(axis_labels[0])
    axes.set_ylabel(axis_labels[1])
    axes.set_zlabel(axis_labels[2])

    return fig


def plot_3d_points(fignum,
                   values,
                   linespec="g*",
                   marginals=None,
                   title="3D Points",
                   axis_labels=('X axis', 'Y axis', 'Z axis')):
    """
    Plots the Point3s in `values`, with optional covariances.
    Finds all the Point3 objects in the given Values object and plots them.
    If a Marginals object is given, this function will also plot marginal
    covariance ellipses for each point.

    Args:
        fignum (int): Integer representing the figure number to use for plotting.
        values (gtsam.Values): Values dictionary consisting of points to be plotted.
        linespec (string): String representing formatting options for Matplotlib.
        marginals (numpy.ndarray): Marginal covariance matrix to plot the
            uncertainty of the estimation.
        title (string): The title of the plot.
        axis_labels (iterable[string]): List of axis labels to set.
    """

    keys = values.keys()

    # Plot points and covariance matrices
    for key in keys:
        try:
            point = values.atPoint3(key)
            if marginals is not None:
                covariance = marginals.marginalCovariance(key)
            else:
                covariance = None

            fig = plot_point3(fignum,
                              point,
                              linespec,
                              covariance,
                              axis_labels=axis_labels)

        except RuntimeError:
            continue
            # I guess it's not a Point3

    fig = plt.figure(fignum)
    fig.suptitle(title)
    fig.canvas.manager.set_window_title(title.lower())


def plot_pose3_on_axes(axes, pose, axis_length=0.1, P=None, scale=1):
    """
    Plot a 3D pose on given axis `axes` with given `axis_length`.

    The uncertainty ellipse (if covariance is given) is scaled in such a way
    that 95% of drawn samples are inliers, see `plot_covariance_ellipse_3d`.

    Args:
        axes (matplotlib.axes.Axes): Matplotlib axes.
        point (gtsam.Point3): The point to be plotted.
        linespec (string): String representing formatting options for Matplotlib.
        P (numpy.ndarray): Marginal covariance matrix to plot the uncertainty of the estimation.
    """
    # get rotation and translation (center)
    gRp = pose.rotation().matrix()  # rotation from pose to global
    origin = pose.translation()

    # draw the camera axes
    x_axis = origin + gRp[:, 0] * axis_length
    line = np.append(origin[np.newaxis], x_axis[np.newaxis], axis=0)
    axes.plot(line[:, 0], line[:, 1], line[:, 2], 'r-')

    y_axis = origin + gRp[:, 1] * axis_length
    line = np.append(origin[np.newaxis], y_axis[np.newaxis], axis=0)
    axes.plot(line[:, 0], line[:, 1], line[:, 2], 'g-')

    z_axis = origin + gRp[:, 2] * axis_length
    line = np.append(origin[np.newaxis], z_axis[np.newaxis], axis=0)
    axes.plot(line[:, 0], line[:, 1], line[:, 2], 'b-')

    # plot the covariance
    if P is not None:
        # covariance matrix in pose coordinate frame
        pPp = P[3:6, 3:6]
        # convert the covariance matrix to global coordinate frame
        gPp = gRp @ pPp @ gRp.T
        plot_covariance_ellipse_3d(axes, origin, gPp)


def plot_pose3(
    fignum: int,
    pose: Pose3,
    axis_length: float = 0.1,
    P: np.ndarray = None,
    axis_labels: Iterable[str] = ("X axis", "Y axis", "Z axis"),
) -> plt.Figure:
    """
    Plot a 3D pose on given figure with given `axis_length`.

    The uncertainty ellipse (if covariance is given) is scaled in such a way
    that 95% of drawn samples are inliers, see `plot_covariance_ellipse_3d`.

    Args:
        fignum: Integer representing the figure number to use for plotting.
        pose (gtsam.Pose3): 3D pose to be plotted.
        axis_length: The length of the camera axes.
        P: Marginal covariance matrix to plot the uncertainty of the estimation.
        axis_labels: List of axis labels to set.

    Returns:
        fig: The matplotlib figure.
    """
    # get figure object
    fig = plt.figure(fignum)
    if not fig.axes:
        axes = fig.add_subplot(projection='3d')
    else:
        axes = fig.axes[0]

    plot_pose3_on_axes(axes, pose, P=P, axis_length=axis_length)

    axes.set_xlabel(axis_labels[0])
    axes.set_ylabel(axis_labels[1])
    axes.set_zlabel(axis_labels[2])

    return fig


def plot_trajectory(
        fignum: int,
        values: Values,
        scale: float = 1,
        marginals: Marginals = None,
        title: str = "Plot Trajectory",
        axis_labels: Iterable[str] = ("X axis", "Y axis", "Z axis"),
) -> None:
    """
    Plot a complete 2D/3D trajectory using poses in `values`.

    Args:
        fignum: Integer representing the figure number to use for plotting.
        values: Values containing some Pose2 and/or Pose3 values.
        scale: Value to scale the poses by.
        marginals: Marginalized probability values of the estimation.
            Used to plot uncertainty bounds.
        title: The title of the plot.
        axis_labels (iterable[string]): List of axis labels to set.
    """
    fig = plt.figure(fignum)
    if not fig.axes:
        axes = fig.add_subplot(projection='3d')
    else:
        axes = fig.axes[0]

    axes.set_xlabel(axis_labels[0])
    axes.set_ylabel(axis_labels[1])
    axes.set_zlabel(axis_labels[2])

    # Plot 2D poses, if any
    poses = gtsam.utilities.allPose2s(values)
    for key in poses.keys():
        pose = poses.atPose2(key)
        if marginals:
            covariance = marginals.marginalCovariance(key)
        else:
            covariance = None

        plot_pose2_on_axes(axes,
                           pose,
                           covariance=covariance,
                           axis_length=scale)

    # Then 3D poses, if any
    poses = gtsam.utilities.allPose3s(values)
    for key in poses.keys():
        pose = poses.atPose3(key)
        if marginals:
            covariance = marginals.marginalCovariance(key)
        else:
            covariance = None

        plot_pose3_on_axes(axes, pose, P=covariance, axis_length=scale)

    fig.suptitle(title)
    fig.canvas.manager.set_window_title(title.lower())


def plot_incremental_trajectory(fignum: int,
                                values: Values,
                                start: int = 0,
                                scale: float = 1,
                                marginals: Optional[Marginals] = None,
                                time_interval: float = 0.0) -> None:
    """
    Incrementally plot a complete 3D trajectory using poses in `values`.

    Args:
        fignum: Integer representing the figure number to use for plotting.
        values: Values dict containing the poses.
        start: Starting index to start plotting from.
        scale: Value to scale the poses by.
        marginals: Marginalized probability values of the estimation.
            Used to plot uncertainty bounds.
        time_interval: Time in seconds to pause between each rendering.
            Used to create animation effect.
    """
    fig = plt.figure(fignum)
    if not fig.axes:
        axes = fig.add_subplot(projection='3d')
    else:
        axes = fig.axes[0]

    poses = gtsam.utilities.allPose3s(values)
    keys = gtsam.KeyVector(poses.keys())

    for key in keys[start:]:
        if values.exists(key):
            pose_i = values.atPose3(key)
            plot_pose3(fignum, pose_i, scale)

    # Update the plot space to encompass all plotted points
    axes.autoscale()

    # Set the 3 axes equal
    set_axes_equal(fignum)

    # Pause for a fixed amount of seconds
    plt.pause(time_interval)
