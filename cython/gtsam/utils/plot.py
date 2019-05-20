"""Various plotting utlities."""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches


def plot_pose2_on_axes(axes, pose, axis_length=0.1, covariance=None):
    """Plot a 2D pose on given axis 'axes' with given 'axis_length'."""
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
    """Plot a 2D pose on given figure with given 'axis_length'."""
    # get figure object
    fig = plt.figure(fignum)
    axes = fig.gca()
    plot_pose2_on_axes(axes, pose, axis_length, covariance)


def plot_point3_on_axes(axes, point, linespec):
    """Plot a 3D point on given axis 'axes' with given 'linespec'."""
    axes.plot([point.x()], [point.y()], [point.z()], linespec)


def plot_point3(fignum, point, linespec):
    """Plot a 3D point on given figure with given 'linespec'."""
    fig = plt.figure(fignum)
    axes = fig.gca(projection='3d')
    plot_point3_on_axes(axes, point, linespec)


def plot_3d_points(fignum, values, linespec, marginals=None):
    """
    Plots the Point3s in 'values', with optional covariances.
    Finds all the Point3 objects in the given Values object and plots them.
    If a Marginals object is given, this function will also plot marginal
    covariance ellipses for each point.
    """

    keys = values.keys()

    # Plot points and covariance matrices
    for i in range(keys.size()):
        try:
            p = values.atPoint3(keys.at(i))
            # if haveMarginals
            #     P = marginals.marginalCovariance(key);
            #     gtsam.plot_point3(p, linespec, P);
            # else
            plot_point3(fignum, p, linespec)
        except RuntimeError:
            continue
            # I guess it's not a Point3


def plot_pose3_on_axes(axes, pose, axis_length=0.1):
    """Plot a 3D pose on given axis 'axes' with given 'axis_length'."""
    # get rotation and translation (center)
    gRp = pose.rotation().matrix()  # rotation from pose to global
    t = pose.translation()
    origin = np.array([t.x(), t.y(), t.z()])

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
    # TODO (dellaert): make this work
    # if (nargin>2) && (~isempty(P))
    #     pPp = P(4:6,4:6); % covariance matrix in pose coordinate frame
    #     gPp = gRp*pPp*gRp'; % convert the covariance matrix to global coordinate frame
    #     gtsam.covarianceEllipse3D(origin,gPp);
    # end


def plot_pose3(fignum, pose, axis_length=0.1):
    """Plot a 3D pose on given figure with given 'axis_length'."""
    # get figure object
    fig = plt.figure(fignum)
    axes = fig.gca(projection='3d')
    plot_pose3_on_axes(axes, pose, axis_length)
