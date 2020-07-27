"""
A structure-from-motion example with landmarks
 - The landmarks form a 10 meter cube
 - The robot rotates around the landmarks, always facing towards the cube
"""
# pylint: disable=invalid-name, E1101

import numpy as np

import gtsam


def createPoints():
    # Create the set of ground-truth landmarks
    points = [gtsam.Point3(10.0, 10.0, 10.0),
              gtsam.Point3(-10.0, 10.0, 10.0),
              gtsam.Point3(-10.0, -10.0, 10.0),
              gtsam.Point3(10.0, -10.0, 10.0),
              gtsam.Point3(10.0, 10.0, -10.0),
              gtsam.Point3(-10.0, 10.0, -10.0),
              gtsam.Point3(-10.0, -10.0, -10.0),
              gtsam.Point3(10.0, -10.0, -10.0)]
    return points


def createPoses(K):
    # Create the set of ground-truth poses
    radius = 40.0
    height = 10.0
    angles = np.linspace(0, 2*np.pi, 8, endpoint=False)
    up = gtsam.Point3(0, 0, 1)
    target = gtsam.Point3(0, 0, 0)
    poses = []
    for theta in angles:
        position = gtsam.Point3(radius*np.cos(theta),
                                radius*np.sin(theta), height)
        camera = gtsam.PinholeCameraCal3_S2.Lookat(position, target, up, K)
        poses.append(camera.pose())
    return poses
