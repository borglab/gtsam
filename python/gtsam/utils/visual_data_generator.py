from __future__ import print_function
from typing import Tuple

import math
import numpy as np
from math import pi

import gtsam
from gtsam import Point3, Pose3, PinholeCameraCal3_S2, Cal3_S2


class Options:
    """
    Options to generate test scenario
    """

    def __init__(self, triangle: bool = False, nrCameras: int = 3, K=Cal3_S2()) -> None:
        """
        Options to generate test scenario
        @param triangle: generate a triangle scene with 3 points if True, otherwise
                  a cube with 8 points
        @param nrCameras: number of cameras to generate
        @param K: camera calibration object
        """
        self.triangle = triangle
        self.nrCameras = nrCameras


class GroundTruth:
    """
    Object holding generated ground-truth data
    """

    def __init__(self, K=Cal3_S2(), nrCameras: int = 3, nrPoints: int = 4) -> None:
        self.K = K
        self.cameras = [Pose3()] * nrCameras
        self.points = [Point3(0, 0, 0)] * nrPoints

    def print_(self, s="") -> None:
        print(s)
        print("K = ", self.K)
        print("Cameras: ", len(self.cameras))
        for camera in self.cameras:
            print("\t", camera)
        print("Points: ", len(self.points))
        for point in self.points:
            print("\t", point)
        pass


class Data:
    """
    Object holding generated measurement data
    """

    class NoiseModels:
        pass

    def __init__(self, K=Cal3_S2(), nrCameras: int = 3, nrPoints: int = 4) -> None:
        self.K = K
        self.Z = [x[:] for x in [[gtsam.Point2()] * nrPoints] * nrCameras]
        self.J = [x[:] for x in [[0] * nrPoints] * nrCameras]
        self.odometry = [Pose3()] * nrCameras

        # Set Noise parameters
        self.noiseModels = Data.NoiseModels()
        self.noiseModels.posePrior = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.001, 0.001, 0.001, 0.1, 0.1, 0.1]))
        # noiseModels.odometry = gtsam.noiseModel.Diagonal.Sigmas(
        #    np.array([0.001,0.001,0.001,0.1,0.1,0.1]))
        self.noiseModels.odometry = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.05, 0.05, 0.05, 0.2, 0.2, 0.2]))
        self.noiseModels.pointPrior = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
        self.noiseModels.measurement = gtsam.noiseModel.Isotropic.Sigma(2, 1.0)


def generate_data(options) -> Tuple[Data, GroundTruth]:
    """ Generate ground-truth and measurement data. """

    K = Cal3_S2(500, 500, 0, 640. / 2., 480. / 2.)
    nrPoints = 3 if options.triangle else 8

    truth = GroundTruth(K=K, nrCameras=options.nrCameras, nrPoints=nrPoints)
    data = Data(K, nrCameras=options.nrCameras, nrPoints=nrPoints)

    # Generate simulated data
    if options.triangle:  # Create a triangle target, just 3 points on a plane
        r = 10
        for j in range(len(truth.points)):
            theta = j * 2 * pi / nrPoints
            truth.points[j] = Point3(r * math.cos(theta), r * math.sin(theta), 0)
    else:  # 3D landmarks as vertices of a cube
        truth.points = [
            Point3(10, 10, 10), Point3(-10, 10, 10),
            Point3(-10, -10, 10), Point3(10, -10, 10),
            Point3(10, 10, -10), Point3(-10, 10, -10),
            Point3(-10, -10, -10), Point3(10, -10, -10)
        ]

    # Create camera cameras on a circle around the triangle
    height = 10
    r = 40
    for i in range(options.nrCameras):
        theta = i * 2 * pi / options.nrCameras
        t = Point3(r * math.cos(theta), r * math.sin(theta), height)
        truth.cameras[i] = PinholeCameraCal3_S2.Lookat(t,
                                                       Point3(0, 0, 0),
                                                       Point3(0, 0, 1),
                                                       truth.K)
        # Create measurements
        for j in range(nrPoints):
            # All landmarks seen in every frame
            data.Z[i][j] = truth.cameras[i].project(truth.points[j])
            data.J[i][j] = j

    # Calculate odometry between cameras
    for i in range(1, options.nrCameras):
        data.odometry[i] = truth.cameras[i - 1].pose().between(
            truth.cameras[i].pose())

    return data, truth
