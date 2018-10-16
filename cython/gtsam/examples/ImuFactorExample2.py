"""
iSAM2 example with ImuFactor.
Author: Robert Truax (C++), Frank Dellaert (Python)
"""
# pylint: disable=invalid-name, E1101

from __future__ import print_function

import math

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # pylint: disable=W0611

import gtsam
import gtsam.utils.plot as gtsam_plot


def X(key):
    """Create symbol for pose key."""
    return gtsam.symbol(ord('x'), key)


def V(key):
    """Create symbol for velocity key."""
    return gtsam.symbol(ord('v'), key)


def vector3(x, y, z):
    """Create 3d double numpy array."""
    return np.array([x, y, z], dtype=np.float)


def create_poses(angular_velocity=np.pi,
                 delta_t=0.01, radius=30.0):
    # Create the set of ground-truth poses
    poses = []
    theta = 0.0
    up = gtsam.Point3(0, 0, 1)
    target = gtsam.Point3(0, 0, 0)
    for i in range(80):
        position = gtsam.Point3(radius * math.cos(theta),
                                radius * math.sin(theta), 0.0)
        camera = gtsam.SimpleCamera.Lookat(
            position, target, up, gtsam.Cal3_S2())
        poses.append(camera.pose())
        theta += delta_t * angular_velocity

    return poses


def ISAM2_plot(values, fignum=0):
    """Plot poses."""
    fig = plt.figure(fignum)
    axes = fig.gca(projection='3d')
    plt.cla()

    i = 0
    min_bounds = 0, 0, 0
    max_bounds = 0, 0, 0
    while values.exists(X(i)):
        pose_i = values.atPose3(X(i))
        gtsam_plot.plot_pose3(fignum, pose_i, 10)
        position = pose_i.translation().vector()
        min_bounds = [min(v1, v2) for (v1, v2) in zip(position, min_bounds)]
        max_bounds = [max(v1, v2) for (v1, v2) in zip(position, max_bounds)]
        # max_bounds = min(pose_i.x(), max_bounds[0]), 0, 0
        i += 1

    # draw
    axes.set_xlim3d(min_bounds[0]-1, max_bounds[0]+1)
    axes.set_ylim3d(min_bounds[1]-1, max_bounds[1]+1)
    axes.set_zlim3d(min_bounds[2]-1, max_bounds[2]+1)
    plt.pause(1)


I = np.eye(3)
accCov = I * 0.1
gyroCov = I * 0.1
intCov = I * 0.1
secOrder = False

# IMU preintegration parameters
# Default Params for a Z-up navigation frame, such as ENU: gravity points along negative Z-axis
g = 9.81
PARAMS = gtsam.PreintegrationParams.MakeSharedU()
PARAMS.setAccelerometerCovariance(accCov)
PARAMS.setGyroscopeCovariance(gyroCov)
PARAMS.setIntegrationCovariance(intCov)
PARAMS.setUse2ndOrderCoriolis(secOrder)
PARAMS.setOmegaCoriolis(vector3(0, 0, 0))


def IMU_example():

    # Create the set of ground-truth landmarks and poses
    angular_velocity = math.radians(180)  # rad/sec
    delta_t = 1.0/18  # makes for 10 degrees per step
    radius = 30
    acceleration = radius * angular_velocity**2
    poses = create_poses(angular_velocity, delta_t, radius)

    # Create a factor graph
    newgraph = gtsam.NonlinearFactorGraph()
    totalgraph = gtsam.NonlinearFactorGraph()

    # Create (incremental) ISAM2 solver
    isam = gtsam.ISAM2()

    # Create the initial estimate to the solution
    # Intentionally initialize the variables off from the ground truth
    initialEstimate = gtsam.Values()
    totalEstimate = gtsam.Values()

    # Add a prior on pose x0. This indirectly specifies where the origin is.
    # 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    noise = gtsam.noiseModel_Diagonal.Sigmas(
        np.array([0.3, 0.3, 0.3, 0.1, 0.1, 0.1]))
    newgraph.push_back(gtsam.PriorFactorPose3(X(0), poses[0], noise))
    totalgraph.push_back(gtsam.PriorFactorPose3(X(0), poses[0], noise))

    # Add imu priors
    biasKey = gtsam.symbol(ord('b'), 0)
    biasnoise = gtsam.noiseModel_Isotropic.Sigma(6, 0.1)
    biasprior = gtsam.PriorFactorConstantBias(biasKey, gtsam.imuBias_ConstantBias(),
                                              biasnoise)
    newgraph.push_back(biasprior)
    totalgraph.push_back(biasprior)
    initialEstimate.insert(biasKey, gtsam.imuBias_ConstantBias())
    totalEstimate.insert(biasKey, gtsam.imuBias_ConstantBias())
    velnoise = gtsam.noiseModel_Isotropic.Sigma(3, 0.1)

    # Calculate with correct initial velocity
    velocity = vector3(0, angular_velocity * radius, 0)
    velprior = gtsam.PriorFactorVector(V(0), velocity, velnoise)
    newgraph.push_back(velprior)
    totalgraph.push_back(velprior)
    initialEstimate.insert(V(0), velocity)
    totalEstimate.insert(V(0), velocity)

    accum = gtsam.PreintegratedImuMeasurements(PARAMS)

    # Simulate poses and imu measurements, adding them to the factor graph
    for i, pose_i in enumerate(poses):
        delta = gtsam.Pose3(gtsam.Rot3.Rodrigues(0, 0, 0),
                            gtsam.Point3(0.05, -0.10, 0.20))
        if i == 0:  # First time add two poses
            initialEstimate.insert(X(0), poses[0].compose(delta))
            initialEstimate.insert(X(1), poses[1].compose(delta))
            totalEstimate.insert(X(0), poses[0].compose(delta))
            totalEstimate.insert(X(1), poses[1].compose(delta))
        elif i >= 2:  # Add more poses as necessary
            initialEstimate.insert(X(i), pose_i.compose(delta))
            totalEstimate.insert(X(i), pose_i.compose(delta))

        if i > 0:
            # Add Bias variables periodically
            if i % 5 == 0:
                biasKey += 1
                b1 = biasKey - 1
                b2 = biasKey
                cov = gtsam.noiseModel_Isotropic.Variance(6, 0.1)
                f = gtsam.BetweenFactorConstantBias(
                    b1, b2, gtsam.imuBias_ConstantBias(), cov)
                newgraph.add(f)
                totalgraph.add(f)
                initialEstimate.insert(biasKey, gtsam.imuBias_ConstantBias())
                totalEstimate.insert(biasKey, gtsam.imuBias_ConstantBias())

            # Predict acceleration and gyro measurements in (actual) body frame
            nRb = pose_i.rotation().matrix()
            bRn = np.transpose(nRb)
            measuredAcc = - np.dot(bRn, vector3(0, 0, -g)) + \
                vector3(0, 0, acceleration)  # in body
            measuredOmega = np.dot(bRn, vector3(0, 0, angular_velocity))
            accum.integrateMeasurement(measuredAcc, measuredOmega, delta_t)

            # Add Imu Factor
            imufac = gtsam.ImuFactor(
                X(i - 1), V(i - 1), X(i), V(i), biasKey, accum)
            newgraph.add(imufac)
            totalgraph.add(imufac)

            # insert new velocity
            initialEstimate.insert(V(i), velocity)
            totalEstimate.insert(V(i), velocity)
            accum.resetIntegration()

        # Batch solution
        isam_full = gtsam.ISAM2()
        isam_full.update(totalgraph, totalEstimate)
        result = isam_full.calculateEstimate()

        ISAM2_plot(totalEstimate,0)
        ISAM2_plot(result,1)

        # Incremental solution
        isam.update(newgraph, initialEstimate)
        result = isam.calculateEstimate()
        newgraph = gtsam.NonlinearFactorGraph()
        initialEstimate.clear()

        ISAM2_plot(result,2)


if __name__ == '__main__':
    IMU_example()
