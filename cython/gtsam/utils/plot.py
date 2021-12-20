"""Various plotting utlities."""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from mpl_toolkits.mplot3d import Axes3D

import gtsam


def set_axes_equal(fignum):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Args:
      fignum (int): An integer representing the figure number for Matplotlib.
    """
    fig = plt.figure(fignum)
    ax = fig.gca(projection='3d')

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


def ellipsoid(xc, yc, zc, rx, ry, rz, n):
    """
    Numpy equivalent of Matlab's ellipsoid function.

    Args:
        xc (double): Center of ellipsoid in X-axis.
        yc (double): Center of ellipsoid in Y-axis.
        zc (double): Center of ellipsoid in Z-axis.
        rx (double): Radius of ellipsoid in X-axis.
        ry (double): Radius of ellipsoid in Y-axis.
        rz (double): Radius of ellipsoid in Z-axis.
        n (int): The granularity of the ellipsoid plotted. 

    Returns:
        tuple[numpy.ndarray]: The points in the x, y and z axes to use for the surface plot.
    """
    u = np.linspace(0, 2*np.pi, n+1)
    v = np.linspace(0, np.pi, n+1)
    x = -rx * np.outer(np.cos(u), np.sin(v)).T
    y = -ry * np.outer(np.sin(u), np.sin(v)).T
    z = -rz * np.outer(np.ones_like(u), np.cos(v)).T

    return x, y, z


def plot_covariance_ellipse_3d(axes, origin, P, scale=1, n=8, alpha=0.5):
    """
    Plots a Gaussian as an uncertainty ellipse

    Based on Maybeck Vol 1, page 366
    k=2.296 corresponds to 1 std, 68.26% of all probability
    k=11.82 corresponds to 3 std, 99.74% of all probability

    Args:
        axes (matplotlib.axes.Axes): Matplotlib axes.
        origin (gtsam.Point3): The origin in the world frame.
        P (numpy.ndarray): The marginal covariance matrix of the 3D point which will be represented as an ellipse.
        scale (float): Scaling factor of the radii of the covariance ellipse.
        n (int): Defines the granularity of the ellipse. Higher values indicate finer ellipses.
        alpha (float): Transparency value for the plotted surface in the range [0, 1].
    """
    k = 11.82
    U, S, _ = np.linalg.svd(P)

    radii = k * np.sqrt(S)
    radii = radii * scale
    rx, ry, rz = radii

    # generate data for "unrotated" ellipsoid
    xc, yc, zc = ellipsoid(0, 0, 0, rx, ry, rz, n)

    # rotate data with orientation matrix U and center c
    data = np.kron(U[:, 0:1], xc) + np.kron(U[:, 1:2], yc) + \
        np.kron(U[:, 2:3], zc)
    n = data.shape[1]
    x = data[0:n, :] + origin[0]
    y = data[n:2*n, :] + origin[1]
    z = data[2*n:, :] + origin[2]

    axes.plot_surface(x, y, z, alpha=alpha, cmap='hot')


def plot_pose2_on_axes(axes, pose, axis_length=0.1, covariance=None):
    """
    Plot a 2D pose on given axis `axes` with given `axis_length`.

    Args:
        axes (matplotlib.axes.Axes): Matplotlib axes.
        pose (gtsam.Pose2): The pose to be plotted.
        axis_length (float): The length of the camera axes.
        covariance (numpy.ndarray): Marginal covariance matrix to plot the uncertainty of the estimation.
    """
    # get rotation and translation (center)
    gRp = pose.rotation().matrix()  # rotation from pose to global
    t = pose.translation()
    origin = np.array([t.x(), t.y()])

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

        w, v = np.linalg.eig(gPp)

        # k = 2.296
        k = 5.0

        angle = np.arctan2(v[1, 0], v[0, 0])
        e1 = patches.Ellipse(origin, np.sqrt(w[0]*k), np.sqrt(w[1]*k),
                             np.rad2deg(angle), fill=False)
        axes.add_patch(e1)


def plot_pose2(fignum, pose, axis_length=0.1, covariance=None):
    """
    Plot a 2D pose on given figure with given `axis_length`.

    Args:
        fignum (int): Integer representing the figure number to use for plotting.
        pose (gtsam.Pose2): The pose to be plotted.
        axis_length (float): The length of the camera axes.
        covariance (numpy.ndarray): Marginal covariance matrix to plot the uncertainty of the estimation.
    """
    # get figure object
    fig = plt.figure(fignum)
    axes = fig.gca()
    plot_pose2_on_axes(axes, pose, axis_length=axis_length,
                       covariance=covariance)


def plot_point3_on_axes(axes, point, linespec, P=None):
    """
    Plot a 3D point on given axis `axes` with given `linespec`.

    Args:
        axes (matplotlib.axes.Axes): Matplotlib axes.
        point (gtsam.Point3): The point to be plotted.
        linespec (string): String representing formatting options for Matplotlib.
        P (numpy.ndarray): Marginal covariance matrix to plot the uncertainty of the estimation.
    """
    axes.plot([point.x()], [point.y()], [point.z()], linespec)
    if P is not None:
        plot_covariance_ellipse_3d(axes, point.vector(), P)


def plot_point3(fignum, point, linespec, P=None):
    """
    Plot a 3D point on given figure with given `linespec`.

    Args:
        fignum (int): Integer representing the figure number to use for plotting.
        point (gtsam.Point3): The point to be plotted.
        linespec (string): String representing formatting options for Matplotlib.
        P (numpy.ndarray): Marginal covariance matrix to plot the uncertainty of the estimation.
    """
    fig = plt.figure(fignum)
    axes = fig.gca(projection='3d')
    plot_point3_on_axes(axes, point, linespec, P)


def plot_3d_points(fignum, values, linespec="g*", marginals=None):
    """
    Plots the Point3s in `values`, with optional covariances.
    Finds all the Point3 objects in the given Values object and plots them.
    If a Marginals object is given, this function will also plot marginal
    covariance ellipses for each point.

    Args:
        fignum (int): Integer representing the figure number to use for plotting.
        values (gtsam.Values): Values dictionary consisting of points to be plotted.
        linespec (string): String representing formatting options for Matplotlib.
        covariance (numpy.ndarray): Marginal covariance matrix to plot the uncertainty of the estimation.
    """

    keys = values.keys()

    # Plot points and covariance matrices
    for i in range(keys.size()):
        try:
            key = keys.at(i)
            point = values.atPoint3(key)
            if marginals is not None:
                covariance = marginals.marginalCovariance(key)
            else:
                covariance = None

            plot_point3(fignum, point, linespec, covariance)

        except RuntimeError:
            continue
            # I guess it's not a Point3


def plot_pose3_on_axes(axes, pose, axis_length=0.1, P=None, scale=1):
    """
    Plot a 3D pose on given axis `axes` with given `axis_length`.

    Args:
        axes (matplotlib.axes.Axes): Matplotlib axes.
        point (gtsam.Point3): The point to be plotted.
        linespec (string): String representing formatting options for Matplotlib.
        P (numpy.ndarray): Marginal covariance matrix to plot the uncertainty of the estimation.
    """
    # get rotation and translation (center)
    gRp = pose.rotation().matrix()  # rotation from pose to global
    origin = pose.translation().vector()

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


def plot_pose3(fignum, pose, axis_length=0.1, P=None):
    """
    Plot a 3D pose on given figure with given `axis_length`.

    Args:
        fignum (int): Integer representing the figure number to use for plotting.
        pose (gtsam.Pose3): 3D pose to be plotted.
        linespec (string): String representing formatting options for Matplotlib.
        P (numpy.ndarray): Marginal covariance matrix to plot the uncertainty of the estimation.
    """
    # get figure object
    fig = plt.figure(fignum)
    axes = fig.gca(projection='3d')
    plot_pose3_on_axes(axes, pose, P=P,
                       axis_length=axis_length)


def plot_trajectory(fignum, values, scale=1, marginals=None):
    """
    Plot a complete 3D trajectory using poses in `values`.

    Args:
        fignum (int): Integer representing the figure number to use for plotting.
        values (gtsam.Values): Values dict containing the poses.
        scale (float): Value to scale the poses by.
        marginals (gtsam.Marginals): Marginalized probability values of the estimation.
            Used to plot uncertainty bounds.
    """
    pose3Values = gtsam.utilities_allPose3s(values)
    keys = gtsam.KeyVector(pose3Values.keys())
    lastIndex = None

    for i in range(keys.size()):
        key = keys.at(i)
        try:
            pose = pose3Values.atPose3(key)
        except:
            print("Warning: no Pose3 at key: {0}".format(key))

        if lastIndex is not None:
            lastKey = keys.at(lastIndex)
            try:
                lastPose = pose3Values.atPose3(lastKey)
            except:
                print("Warning: no Pose3 at key: {0}".format(lastKey))
                pass

            if marginals:
                covariance = marginals.marginalCovariance(lastKey)
            else:
                covariance = None

            plot_pose3(fignum, lastPose,  P=covariance,
                       axis_length=scale)

        lastIndex = i

    # Draw final pose
    if lastIndex is not None:
        lastKey = keys.at(lastIndex)
        try:
            lastPose = pose3Values.atPose3(lastKey)
            if marginals:
                covariance = marginals.marginalCovariance(lastKey)
            else:
                covariance = None

            plot_pose3(fignum, lastPose, P=covariance,
                       axis_length=scale)

        except:
            pass
