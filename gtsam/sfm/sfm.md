# Structure from Motion

The `sfm` module provides dataset containers and algorithms for global structure from motion: feature-track assembly, three-view transfer, focal-length self-calibration, rotation averaging, translation and location recovery, trajectory alignment, and bundle-adjustment setup.

## Fastest CPU bundle-adjustment path

For the fastest currently merged **GTSAM-only CPU path without Ceres or CHOLMOD**, use the point-batched multifrontal configuration (see [issue #2299](https://github.com/borglab/gtsam/issues/2299)):

1. Group every landmark's reprojection measurements into one `BatchFactor<SfmFactor, 2>`.
2. Supply an explicit Schur ordering with **all landmark keys first, followed by all camera keys**.
3. Select `NonlinearOptimizerParams::MULTIFRONTAL_SOLVER`.
4. Compile GTSAM in Release mode with TBB enabled.

Example:

```cpp
using Camera = gtsam::PinholeCamera<gtsam::Cal3Bundler>;
using SfmFactor = gtsam::GeneralSFMFactor<Camera, gtsam::Point3>;

gtsam::NonlinearFactorGraph graph;
for (size_t j = 0; j < data.numberTracks(); ++j) {
  const auto& track = data.tracks[j];
  if (track.measurements.size() < 2) continue;

  std::map<gtsam::Key, gtsam::Point2> measurements;
  for (const auto& [cameraIndex, imagePoint] : track.measurements) {
    measurements[gtsam::Symbol('c', cameraIndex)] = imagePoint;
  }
  graph.add(std::make_shared<gtsam::BatchFactor<SfmFactor, 2>>(
      measurements, gtsam::Symbol('p', j), projectionNoise));
}

gtsam::Ordering pointFirstOrdering;
for (size_t j = 0; j < data.numberTracks(); ++j) {
  pointFirstOrdering.push_back(gtsam::Symbol('p', j));
}
for (size_t i = 0; i < data.numberCameras(); ++i) {
  pointFirstOrdering.push_back(gtsam::Symbol('c', i));
}

gtsam::LevenbergMarquardtParams params;
gtsam::LevenbergMarquardtParams::SetCeresDefaults(&params);
params.linearSolverType =
    gtsam::NonlinearOptimizerParams::MULTIFRONTAL_SOLVER;
params.multifrontalParams.qrMode =
    gtsam::MultifrontalParameters::QRMode::Allow;
params.setOrdering(pointFirstOrdering);

gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
const gtsam::Values result = optimizer.optimize();
```

Add the camera and landmark gauge priors required by the reconstruction before optimizing. The complete reference construction is in [`timing/internal/SfmBalBenchmark.cpp`](../../timing/internal/SfmBalBenchmark.cpp), and the matching solver configuration is in [`timing/timeSFMBAL.cpp`](../../timing/timeSFMBAL.cpp).

The arm64/TBB measurements reported in issue #2299 are benchmark guidance, not performance guarantees:

| BAL dataset | Point-batch / `MultifrontalSolver` total optimization time |
| --- | ---: |
| BAL-16 | 0.247 s |
| BAL-88 | 0.975 s |
| BAL-135 | 1.349 s |

Why this is fast: point batching reduces factor and allocation overhead, point-first elimination forms the camera Schur complement instead of dense landmark-camera fill, and `MULTIFRONTAL_SOLVER` reuses the symbolic structure throughout the nonlinear iterations. Do not substitute unconstrained COLAMD or METIS and assume it will discover the same ordering; ordering is heuristic and has a large effect on bundle adjustment.

The point-batch factor and `MultifrontalParameters` tuning interface are currently C++ APIs. Python can set a point-first `Ordering` and select `MULTIFRONTAL_SOLVER`, but it cannot reproduce the full recommended point-batched configuration yet.

If GTSAM is built with CHOLMOD, the timing program also exposes the explicit point-batch sparse-Schur comparison:

```sh
./timing/timeSFMBAL --point-batch-schur-cholmod-only \
  /path/to/problem-135-90642-pre.txt
```

Issue #2299 reports this benchmark path within roughly 13–16% of Ceres total solver time on matched BAL-88 and BAL-135 solve budgets. It is a specialized timing path, whereas the point-batch multifrontal recipe above uses the ordinary GTSAM nonlinear optimizer.

## CUDA acceleration

The SFM CUDA implementation requires a CUDA-enabled build and lives in `gtsam/sfm/cuda/`. The public C++ API remains in the `gtsam::cuda` namespace, and Python exposes it as `gtsam.cuda`.

- [SfmLevenbergMarquardtOptimizer](doc/SfmLevenbergMarquardtOptimizer.ipynb): Fully GPU-resident Levenberg-Marquardt for BAL-style bundle adjustment using dense-Schur, cuDSS, or PCG linear solvers.
- [GNC with the CUDA SFM optimizer](doc/CudaSfmGncOptimizer.ipynb): Robust bundle adjustment using the CUDA SFM optimizer as the GNC inner solver.

## Data and feature tracks

- [Keypoints](doc/Keypoints.ipynb): Image feature coordinates and pairwise-match track generation.
- [SfmTrack2d](doc/SfmTrack2d.ipynb): Camera-indexed two-dimensional observations of one feature track.
- [SfmTrack](doc/SfmTrack.ipynb): A 2D track with its reconstructed 3D point and optional color.
- [SfmData](doc/SfmData.ipynb): BAL/Bundler cameras and tracks, graph construction, and initialization.

## Measurement records

- [UnaryMeasurement](doc/UnaryMeasurement.ipynb): A typed measurement and noise model attached to one key.
- [BinaryMeasurement](doc/BinaryMeasurement.ipynb): A typed, directed measurement and noise model between two keys.

## Three-view transfer and self-calibration

- [TransferEdges](doc/TransferEdges.ipynb): C++ edge-orientation helper shared by transfer factors.
- [TransferFactor](doc/TransferFactor.ipynb): Three-view transfer using fundamental matrices.
- [EssentialTransferFactor](doc/EssentialTransferFactor.ipynb): Three-view transfer using essential matrices and fixed shared calibration.
- [EssentialTransferFactorK](doc/EssentialTransferFactorK.ipynb): Three-view transfer with calibration variables.
- [SelfCalibrationFactor](doc/SelfCalibrationFactor.ipynb): Two-camera focal-length constraints from a fundamental matrix.

## Rotation averaging

- [ShonanFactor](doc/ShonanFactor.ipynb): One relative-rotation factor at a lifted relaxation level.
- [ShonanGaugeFactor](doc/ShonanGaugeFactor.ipynb): C++ stabilizer-gauge constraint for lifted Shonan variables.
- [ShonanAveragingParameters](doc/ShonanAveragingParameters.ipynb): Anchoring, robustness, optimization, and certification settings.
- [ShonanAveraging](doc/ShonanAveraging.ipynb): Common C++ staircase and certificate implementation.
- [ShonanAveraging2](doc/ShonanAveraging2.ipynb): Planar rotation averaging.
- [ShonanAveraging3](doc/ShonanAveraging3.ipynb): Spatial rotation averaging.

## Translation and position recovery

- [TranslationFactor](doc/TranslationFactor.ipynb): C++ chordal translation-direction factor.
- [BilinearAngleTranslationFactor](doc/BilinearAngleTranslationFactor.ipynb): C++ BATA factor with a per-edge scale variable.
- [LocationRecovery](doc/LocationRecovery.ipynb): Generic graph construction for direction-based position recovery.
- [TranslationRecovery](doc/TranslationRecovery.ipynb): Translation averaging with gauge fixing and optional metric edges.
- [GlobalPositioner](doc/GlobalPositioner.ipynb): GLOMAP-style joint camera and landmark positioning.
- [MFAS](doc/MFAS.ipynb): 1DSfM ordering and translation-edge outlier scoring.

## Trajectory alignment

- [TrajectoryAlignerSim3](doc/TrajectoryAlignerSim3.ipynb): Similarity alignment of one or more child pose trajectories to a parent trajectory.

## Python wrapper support types

Template classes appear in Python under concrete names. `BinaryMeasurementUnit3`, `BinaryMeasurementRot3`, and `BinaryMeasurementPoint3` are covered by the `BinaryMeasurement` guide; the corresponding `UnaryMeasurement` variants are covered by its guide. Transfer and Shonan template instantiations are likewise listed in their source-class notebooks.

The wrapper-only containers `BinaryMeasurementsUnit3`, `BinaryMeasurementsRot3`, `BinaryMeasurementsPoint3`, `KeyPairDoubleMap`, `MatchIndicesMap`, and `KeypointsVector` only adapt C++ containers at language boundaries. Python users should normally pass and receive native lists and dictionaries, as demonstrated in the related class guides.
