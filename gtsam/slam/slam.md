# SLAM

The `slam` module provides a collection of factors, constraints, utilities, and initialization algorithms commonly used in Simultaneous Localization and Mapping (SLAM) and Structure from Motion (SfM) applications. It builds upon the core GTSAM inference engine (`gtsam/inference`) and geometric types (`gtsam/geometry`).

## Core Factors

These are fundamental factor types often used as building blocks in SLAM.
-   [PriorFactor](doc/PriorFactor.ipynb) : A prior factor acting only on the rotation component of a pose variable.
-   [BetweenFactor](doc/BetweenFactor.ipynb) : Represents relative measurements between two poses or other Lie group variables (e.g., derived from [odometry](https://en.wikipedia.org/wiki/Odometry)).

## Visual SLAM/SfM Factors

Factors specifically designed for visual data (camera measurements).

-   [GenericProjectionFactor](doc/GenericProjectionFactor.ipynb) : Standard monocular projection factor relating a 3D landmark, camera pose, and fixed calibration to a 2D measurement.
-   [GeneralSFMFactor](doc/GeneralSFMFactor.ipynb) : Projection factors used when camera calibration is unknown or optimized alongside poses and landmarks.
-   [StereoFactor](doc/StereoFactor.ipynb) : Standard stereo projection factor relating a 3D landmark, camera pose, and fixed stereo calibration to a `StereoPoint2` measurement.
-   [EssentialMatrixFactor](doc/EssentialMatrixFactor.ipynb) : Factors constraining poses or calibration based on the Essential matrix derived from calibrated cameras.
-   [EssentialMatrixConstraint](doc/EssentialMatrixConstraint.ipynb) : Factor constraining the relative pose between two cameras based on a measured Essential matrix.
-   [TriangulationFactor](doc/TriangulationFactor.ipynb) : Factor constraining a 3D point based on a measurement from a single known camera view, useful for triangulation.
-   [PlanarProjectionFactor](doc/PlanarProjectionFactor.ipynb) : Projection factors specialized for robots moving on a 2D plane.

## Smart Factors

Factors that implicitly manage landmark variables, marginalizing them out during optimization.

-   [SmartFactorParams](doc/SmartFactorParams.ipynb) : Configuration parameters controlling the behavior of smart factors (linearization, degeneracy handling, etc.).
-   [SmartProjectionFactor](doc/SmartProjectionFactor.ipynb) : Smart factor for monocular measurements where both camera pose and calibration are optimized.
-   [SmartProjectionPoseFactor](doc/SmartProjectionPoseFactor.ipynb) : Smart factor for monocular measurements where camera calibration is fixed, optimizing only poses.
-   [SmartProjectionRigFactor](doc/SmartProjectionRigFactor.ipynb) : Smart factor for calibrated multi-camera rigs, optimizing only the rig's body pose.
-   [SmartFactorBase](https://github.com/borglab/gtsam/blob/develop/gtsam/slam/SmartFactorBase.h) : Abstract base class for smart factors (internal use).

## Other Geometric Factors & Constraints

Factors representing various geometric relationships or constraints.

-   [PoseRotationPrior](doc/PoseRotationPrior.ipynb) : A prior factor acting only on the rotation component of a pose variable.
-   [PoseTranslationPrior](doc/PoseTranslationPrior.ipynb) : A prior factor acting only on the translation component of a pose variable.
-   [OrientedPlane3Factor](doc/OrientedPlane3Factor.ipynb) : Factors for estimating and constraining 3D planar landmarks (`OrientedPlane3`).
-   [RotateFactor](doc/RotateFactor.ipynb) : Factors constraining an unknown rotation based on how it transforms measured rotations or directions.
-   [KarcherMeanFactor](doc/KarcherMeanFactor.ipynb) : Factor for constraining the Karcher mean (geometric average) of a set of rotations or other manifold values.
-   [FrobeniusFactor](doc/FrobeniusFactor.ipynb) : Factors operating directly on rotation matrix entries using the Frobenius norm, an alternative to Lie algebra-based factors.
-   [ReferenceFrameFactor](doc/ReferenceFrameFactor.ipynb) : Factor relating the same landmark observed in two different coordinate frames via an unknown transformation, useful for map merging.
-   [BoundingConstraint](doc/BoundingConstraint.ipynb) : Abstract base class for creating inequality constraints (e.g., keeping a variable within certain bounds). Requires C++ derivation.
-   [AntiFactor](doc/AntiFactor.ipynb) : A factor designed to negate the effect of another factor, useful for dynamically removing constraints.

## Initialization & Utilities

Helper functions and classes for SLAM tasks.

-   [lago](doc/lago.ipynb) : Linear Approximation for Graph Optimization (LAGO) for initializing `Pose2` graphs.
-   [InitializePose3](doc/InitializePose3.ipynb) : Methods for initializing `Pose3` graphs by first solving for rotations, then translations.
-   [dataset](doc/dataset.ipynb) : Utility functions for loading/saving common SLAM dataset formats (g2o, TORO).
-   [expressions](https://github.com/borglab/gtsam/blob/develop/gtsam/slam/expressions.h) : Pre-defined Expression trees for common SLAM factor types (internal use for Expression-based factors).