"""
iSAM2 example with ImuFactor.
Author: Robert Truax (C++), Frank Dellaert (Python)
"""
# pylint: disable=invalid-name, E1101

from __future__ import print_function

import math

import gtsam
import gtsam.utils.plot as gtsam_plot
import matplotlib.pyplot as plt
import numpy as np
from gtsam import (ISAM2, BetweenFactorConstantBias, Cal3_S2,
                   ConstantTwistScenario, ImuFactor, NonlinearFactorGraph,
                   PinholeCameraCal3_S2, Point3, Pose3,
                   PriorFactorConstantBias, PriorFactorPose3,
                   PriorFactorVector, Rot3, Values)
from gtsam import symbol_shorthand_B as B
from gtsam import symbol_shorthand_V as V
from gtsam import symbol_shorthand_X as X
from mpl_toolkits.mplot3d import Axes3D  # pylint: disable=W0611


def vector3(x, y, z):
    """Create 3d double numpy array."""
    return np.array([x, y, z], dtype=np.float)


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


# IMU preintegration parameters
# Default Params for a Z-up navigation frame, such as ENU: gravity points along negative Z-axis
g = 9.81
n_gravity = vector3(0, 0, -g)
PARAMS = gtsam.PreintegrationParams.MakeSharedU(g)
I = np.eye(3)
PARAMS.setAccelerometerCovariance(I * 0.1)
PARAMS.setGyroscopeCovariance(I * 0.1)
PARAMS.setIntegrationCovariance(I * 0.1)
PARAMS.setUse2ndOrderCoriolis(False)
PARAMS.setOmegaCoriolis(vector3(0, 0, 0))

BIAS_COVARIANCE = gtsam.noiseModel_Isotropic.Variance(6, 0.1)
DELTA = Pose3(Rot3.Rodrigues(0, 0, 0),
              Point3(0.05, -0.10, 0.20))


def IMU_example():
    """Run iSAM 2 example with IMU factor."""

    # Start with a camera on x-axis looking at origin
    radius = 30
    up = Point3(0, 0, 1)
    target = Point3(0, 0, 0)
    position = Point3(radius, 0, 0)
    camera = PinholeCameraCal3_S2.Lookat(position, target, up, Cal3_S2())
    pose_0 = camera.pose()

    # Create the set of ground-truth landmarks and poses
    angular_velocity = math.radians(180)  # rad/sec
    delta_t = 1.0/18  # makes for 10 degrees per step

    angular_velocity_vector = vector3(0, -angular_velocity, 0)
    linear_velocity_vector = vector3(radius * angular_velocity, 0, 0)
    scenario = ConstantTwistScenario(
        angular_velocity_vector, linear_velocity_vector, pose_0)

    # Create a factor graph
    newgraph = NonlinearFactorGraph()

    # Create (incremental) ISAM2 solver
    isam = ISAM2()

    # Create the initial estimate to the solution
    # Intentionally initialize the variables off from the ground truth
    initialEstimate = Values()

    # Add a prior on pose x0. This indirectly specifies where the origin is.
    # 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    noise = gtsam.noiseModel_Diagonal.Sigmas(
        np.array([0.1, 0.1, 0.1, 0.3, 0.3, 0.3]))
    newgraph.push_back(PriorFactorPose3(X(0), pose_0, noise))

    # Add imu priors
    biasKey = B(0)
    biasnoise = gtsam.noiseModel_Isotropic.Sigma(6, 0.1)
    biasprior = PriorFactorConstantBias(biasKey, gtsam.imuBias_ConstantBias(),
                                        biasnoise)
    newgraph.push_back(biasprior)
    initialEstimate.insert(biasKey, gtsam.imuBias_ConstantBias())
    velnoise = gtsam.noiseModel_Isotropic.Sigma(3, 0.1)

    # Calculate with correct initial velocity
    n_velocity = vector3(0, angular_velocity * radius, 0)
    velprior = PriorFactorVector(V(0), n_velocity, velnoise)
    newgraph.push_back(velprior)
    initialEstimate.insert(V(0), n_velocity)

    accum = gtsam.PreintegratedImuMeasurements(PARAMS)

    # Simulate poses and imu measurements, adding them to the factor graph
    for i in range(80):
        t = i * delta_t  # simulation time
        if i == 0:  # First time add two poses
            pose_1 = scenario.pose(delta_t)
            initialEstimate.insert(X(0), pose_0.compose(DELTA))
            initialEstimate.insert(X(1), pose_1.compose(DELTA))
        elif i >= 2:  # Add more poses as necessary
            pose_i = scenario.pose(t)
            initialEstimate.insert(X(i), pose_i.compose(DELTA))

        if i > 0:
            # Add Bias variables periodically
            if i % 5 == 0:
                biasKey += 1
                factor = BetweenFactorConstantBias(
                    biasKey - 1, biasKey, gtsam.imuBias_ConstantBias(), BIAS_COVARIANCE)
                newgraph.add(factor)
                initialEstimate.insert(biasKey, gtsam.imuBias_ConstantBias())

            # Predict acceleration and gyro measurements in (actual) body frame
            nRb = scenario.rotation(t).matrix()
            bRn = np.transpose(nRb)
            measuredAcc = scenario.acceleration_b(t) - np.dot(bRn, n_gravity)
            measuredOmega = scenario.omega_b(t)
            accum.integrateMeasurement(measuredAcc, measuredOmega, delta_t)

            # Add Imu Factor
            imufac = ImuFactor(X(i - 1), V(i - 1), X(i), V(i), biasKey, accum)
            newgraph.add(imufac)

            # insert new velocity, which is wrong
            initialEstimate.insert(V(i), n_velocity)
            accum.resetIntegration()

        # Incremental solution
        isam.update(newgraph, initialEstimate)
        result = isam.calculateEstimate()
        ISAM2_plot(result)

        # reset
        newgraph = NonlinearFactorGraph()
        initialEstimate.clear()


if __name__ == '__main__':
    IMU_example()
