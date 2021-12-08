"""
A structure-from-motion example with landmarks
 - The landmarks form a 10 meter cube
 - The robot rotates around the landmarks, always facing towards the cube
"""
# pylint: disable=invalid-name, E1101

from typing import List

import numpy as np

import gtsam
from gtsam import Cal3_S2, Point3, Pose3


def createPoints() -> List[Point3]:
    # Create the set of ground-truth landmarks
    points = [
        Point3(10.0, 10.0, 10.0),
        Point3(-10.0, 10.0, 10.0),
        Point3(-10.0, -10.0, 10.0),
        Point3(10.0, -10.0, 10.0),
        Point3(10.0, 10.0, -10.0),
        Point3(-10.0, 10.0, -10.0),
        Point3(-10.0, -10.0, -10.0),
        Point3(10.0, -10.0, -10.0),
    ]
    return points


def createPoses(K: Cal3_S2) -> List[Pose3]:
    """Generate a set of ground-truth camera poses arranged in a circle about the origin."""
    radius = 40.0
    height = 10.0
    angles = np.linspace(0, 2 * np.pi, 8, endpoint=False)
    up = gtsam.Point3(0, 0, 1)
    target = gtsam.Point3(0, 0, 0)
    poses = []
    for theta in angles:
        position = gtsam.Point3(radius * np.cos(theta), radius * np.sin(theta), height)
        camera = gtsam.PinholeCameraCal3_S2.Lookat(position, target, up, K)
        poses.append(camera.pose())
    return poses
