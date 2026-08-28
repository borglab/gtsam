# SLAM

The `slam` module provides a collection of factors, constraints, utilities, and initialization algorithms commonly used in Simultaneous Localization and Mapping (SLAM) and Structure from Motion (SfM) applications. It builds upon the core GTSAM inference engine (`gtsam/inference`) and geometric types (`gtsam/geometry`).

## Core Factors

These are fundamental factor types often used as building blocks in SLAM.
-   [PriorFactor](doc/PriorFactor.ipynb) : A prior factor acting only on the rotation component of a pose variable.
-   [BetweenFactor](doc/BetweenFactor.ipynb) : Represents relative measurements between two poses or other Lie group variables (e.g., derived from [odometry](https://en.wikipedia.org/wiki/Odometry)).

## Visual SLAM/SfM Factors

Factors specifically designed for visual data (camera measurements).

-   [GenericProjectionFactor](doc/ProjectionFactor.ipynb) : Standard monocular projection factor relating a 3D landmark, camera pose, and fixed calibration to a 2D measurement.
-   [GeneralSFMFactor](doc/GeneralSFMFactor.ipynb) : Projection factors used when camera calibration is unknown or optimized alongside poses and landmarks.
-   [StereoFactor](doc/StereoFactor.ipynb) : Standard stereo projection factor relating a 3D landmark, camera pose, and fixed stereo calibration to a `StereoPoint2` measurement.
-   [TriangulationFactor](doc/TriangulationFactor.ipynb) : Factor constraining a 3D point based on a measurement from a single known camera view, useful for triangulation.
-   [PlanarProjectionFactor](doc/PlanarProjectionFactor.ipynb) : Projection factors specialized for robots moving on a 2D plane.

## Smart Factors

Factors that implicitly manage landmark variables, marginalizing them out during optimization.

-   [SmartFactorParams](doc/SmartProjectionParams.ipynb) : Configuration parameters controlling the behavior of smart factors (linearization, degeneracy handling, etc.).
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
-   [WahbaFactor](doc/WahbaFactor.ipynb) : A `Rot3` direction-correspondence factor with a three-dimensional chordal residual and exact D=1 QCQP conversion.
-   [KnownLandmarkFactor](doc/KnownLandmarkFactor.ipynb) : Conventional `wTk` and certifiable inverse-state `kTw` factors for observations of fixed Pose2/Pose3 landmarks.
-   [KarcherMeanFactor](doc/KarcherMeanFactor.ipynb) : Factor for constraining the Karcher mean (geometric average) of a set of rotations or other manifold values.
-   [FrobeniusFactor](doc/FrobeniusFactor.ipynb) : Factors operating directly on rotation matrix entries using the Frobenius norm, an alternative to Lie algebra-based factors.
-   [ReferenceFrameFactor](doc/ReferenceFrameFactor.ipynb) : Factor relating the same landmark observed in two different coordinate frames via an unknown transformation, useful for map merging.

## Initialization & Utilities

Helper functions and classes for SLAM tasks.

-   [lago](doc/lago.ipynb) : Linear Approximation for Graph Optimization (LAGO) for initializing `Pose2` graphs.
-   [InitializePose3](doc/InitializePose3.ipynb) : Methods for initializing `Pose3` graphs by first solving for rotations, then translations.
-   [FAST-Sync](doc/FastSync.ipynb) : A sparse chordal initializer for matrix Lie-group synchronization. In C++, call `fastSync<T>(graph)` for `Rot2`, `Rot3`, `Pose2`, `Pose3`, `Similarity2`, `Similarity3`, or `SL4`. Generated Python and MATLAB entry points are `fastSyncRot2`, `fastSyncRot3`, `fastSyncPose2`, `fastSyncPose3`, `fastSyncSimilarity2`, `fastSyncSimilarity3`, and `fastSyncSL4`; see the runnable [Python](../../python/gtsam/examples/FastSyncExample.ipynb) and [MATLAB](../../matlab/gtsam_examples/FastSyncExample.m) examples.
-   [dataset](doc/dataset.ipynb) : Utility functions for loading/saving common SLAM dataset formats (g2o, TORO).
-   [expressions](https://github.com/borglab/gtsam/blob/develop/gtsam/slam/expressions.h) : Pre-defined Expression trees for common SLAM factor types (internal use for Expression-based factors).

### FAST-Sync input and gauge behavior

`fastSync<T>` reads matching `BetweenFactor<T>` measurements and accepts only finite, positive, isotropic Gaussian noise. Anisotropic, constrained, and robust between-factor models are rejected. The measurement graph must be non-empty and connected; disconnected graphs are detected during QR elimination and raise `IndeterminateSystemException`. The graph may contain at most one matching `PriorFactor<T>`.

The relaxed problem uses fixed-size `N`-by-`N` matrices for measurements, reduced-system blocks, back-substitution, and projection, where `N` is obtained from the matrix representation returned by `T::matrix()`. The complete Gaussian graph retains dynamic sparse storage because its topology is only known at runtime. FAST-Sync defaults to a METIS nested-dissection ordering, accepts another supported `OrderingType` such as COLAMD or a caller-supplied complete `Ordering`, and uses an exact identity gauge at the ordering's final key. Projection to the target group occurs only after the complete ambient-space solve. If a matching prior is present, the rounded solution is subsequently left-aligned to that prior; without a prior, the selected ordering's gauge is retained. Selecting METIS in a build without METIS support reports the nested-dissection error. New fixed-size matrix Lie groups can opt in by specializing `FastSyncProjection<T>`.
