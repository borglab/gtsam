import numpy as _np
import matplotlib.pyplot as _plt
from mpl_toolkits.mplot3d import Axes3D as _Axes3D

def plotPoint3(fignum, point, linespec):
    fig = _plt.figure(fignum)
    ax = fig.gca(projection='3d')
    ax.plot([point.x()],[point.y()],[point.z()], linespec)


def plot3DPoints(fignum, values, linespec, marginals=None):
    # PLOT3DPOINTS Plots the Point3's in a values, with optional covariances
    #    Finds all the Point3 objects in the given Values object and plots them.
    #  If a Marginals object is given, this function will also plot marginal
    #  covariance ellipses for each point.

    keys = values.keys()

    # Plot points and covariance matrices
    for key in keys:
        try:
            p = values.point3_at(key);
            # if haveMarginals
            #     P = marginals.marginalCovariance(key);
            #     gtsam.plotPoint3(p, linespec, P);
            # else
            plotPoint3(fignum, p, linespec);
        except RuntimeError:
            continue
            #I guess it's not a Point3

def plotPose3(fignum, pose, axisLength=0.1):
    # get figure object
    fig = _plt.figure(fignum)
    ax = fig.gca(projection='3d')

    # get rotation and translation (center)
    gRp = pose.rotation().matrix()  # rotation from pose to global
    C = pose.translation().vector()

    # draw the camera axes
    xAxis = C+gRp[:,0]*axisLength
    L = _np.append(C[_np.newaxis], xAxis[_np.newaxis], axis=0)
    ax.plot(L[:,0],L[:,1],L[:,2],'r-')
    
    yAxis = C+gRp[:,1]*axisLength
    L = _np.append(C[_np.newaxis], yAxis[_np.newaxis], axis=0)
    ax.plot(L[:,0],L[:,1],L[:,2],'g-')
    
    zAxis = C+gRp[:,2]*axisLength
    L = _np.append(C[_np.newaxis], zAxis[_np.newaxis], axis=0)
    ax.plot(L[:,0],L[:,1],L[:,2],'b-')

    # # plot the covariance
    # if (nargin>2) && (~isempty(P))
    #     pPp = P(4:6,4:6); % covariance matrix in pose coordinate frame    
    #     gPp = gRp*pPp*gRp'; % convert the covariance matrix to global coordinate frame
    #     gtsam.covarianceEllipse3D(C,gPp);  
    # end