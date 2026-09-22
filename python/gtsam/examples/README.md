# Python Examples

Notebook examples are grouped by the GTSAM module they demonstrate:

| Folder | Topics |
|--------|--------|
| [geometry](geometry/) | Rotations |
| [basis](basis/) | Basis functions and spline trajectories |
| [linear](linear/) | Gaussian factor graphs and linear-quadratic control |
| [discrete](discrete/) | Discrete inference and hidden Markov models |
| [hybrid](hybrid/) | Hybrid inference |
| [nonlinear](nonlinear/) | Custom factors, optimization, filtering, and smoothing |
| [constrained](constrained/) | Linear, quadratic, and quadratically constrained programs |
| [certifiable](certifiable/) | Certifiable estimation and semidefinite relaxations |
| [navigation](navigation/) | IMU, GNSS, and continuous-time state estimation |
| [sfm](sfm/) | Cameras, structure from motion, and visual odometry |
| [slam](slam/) | Localization, pose graphs, and landmark SLAM |

Standalone Python scripts and shared helper modules live in this directory.
Import shared helpers through `gtsam.examples` so notebooks can run from their
module folders. Add new notebooks to the appropriate folder; the corresponding
MyST subsection discovers them automatically.

## Porting Progress

| C++ Example Name                                      | Ported |
|-------------------------------------------------------|--------|
| [CameraResectioning](sfm/CameraResectioning.ipynb) | :heavy_check_mark: |
| CombinedImuFactorsExample                             | :heavy_check_mark: |
| [CreateSFMExampleData](sfm/CreateSFMExampleData.ipynb) | :heavy_check_mark: |
| [DiscreteBayesNetExample](discrete/DiscreteBayesNetExample.ipynb) | :heavy_check_mark: |
| DiscreteBayesNet_FG                                   | none of the required discrete functionality is exposed through Python |
| [easyPoint2KalmanFilter](nonlinear/easyPoint2KalmanFilter.ipynb) | ExtendedKalmanFilter not yet exposed through Python |
| [elaboratePoint2KalmanFilter](nonlinear/elaboratePoint2KalmanFilter.ipynb) | GaussianSequentialSolver not yet exposed through Python |
| [FisheyeExample](sfm/FisheyeExample.ipynb) | :heavy_check_mark: |
| [FixedLagSmootherExample](nonlinear/FixedLagSmootherExample.ipynb) | :heavy_check_mark: |
| [HMMExample](discrete/HMMExample.ipynb) | :heavy_check_mark: |
| ImuFactorsExample2                                    | :heavy_check_mark: |
| ImuFactorsExample                                     |        |
| IMUKittiExampleGPS                                    | :heavy_check_mark: |
| InverseKinematicsExampleExpressions.cpp               |        |
| ISAM2Example_SmartFactor                              |        |
| ISAM2_SmartFactorStereo_IMU                           |        |
| LocalizationExample                                   | :heavy_check_mark: |
| METISOrderingExample                                  |        |
| [OdometryExample](slam/OdometryExample.ipynb) | :heavy_check_mark: |
| [PlanarSLAMExample](slam/PlanarSLAMExample.ipynb) | :heavy_check_mark: |
| [Pose2SLAMExample](slam/Pose2SLAMExample.ipynb) | :heavy_check_mark: |
| Pose2SLAMExampleExpressions                           | ExpressionFactorGraph not yet exposed through Python |
| Pose2SLAMExample_g2o                                  | :heavy_check_mark: |
| Pose2SLAMExample_graph                                | :heavy_check_mark: |
| Pose2SLAMExample_graphviz                             | :heavy_check_mark: |
| Pose2SLAMExample_lago                                 | lago not yet exposed through Python |
| [Pose2SLAMStressTest](slam/Pose2SLAMStressTest.ipynb) | :heavy_check_mark: |
| [Pose2SLAMwSPCG](slam/Pose2SLAMwSPCG.ipynb) | :heavy_check_mark: |
| Pose3Localization                                     |        |
| Pose3SLAMExample_changeKeys                           |        |
| Pose3SLAMExampleExpressions_BearingRangeWithTransform |        |
| Pose3SLAMExample_g2o                                  | :heavy_check_mark: |
| Pose3SLAMExample_initializePose3Chordal               | :heavy_check_mark: |
| Pose3SLAMExample_initializePose3Gradient              |        |
| [RangeISAMExample_plaza2](slam/RangeISAMExample_plaza2.ipynb) | :heavy_check_mark: |
| [SelfCalibrationExample](sfm/SelfCalibrationExample.ipynb) | :heavy_check_mark: |
| SFMdata                                               | :heavy_check_mark: |     
| SFMExample_bal_COLAMD_METIS                           |        |
| SFMExample_bal                                        | :heavy_check_mark: |
| [SFMExample](sfm/SFMExample.ipynb) | :heavy_check_mark: |
| SFMExampleExpressions_bal                             |        |
| SFMExampleExpressions                                 |        |
| SFMExample_SmartFactor                                |        |
| SFMExample_SmartFactorPCG                             |        |
| ShonanAveragingCLI                                    | :heavy_check_mark: |
| [SimpleRotation](geometry/SimpleRotation.ipynb) | :heavy_check_mark: |
| SolverComparer                                        |        |
| [StereoVOExample](sfm/StereoVOExample.ipynb) | :heavy_check_mark: |
| [StereoVOExample_large](sfm/StereoVOExample_large.ipynb) | :heavy_check_mark: |
| TimeTBB                                               |        |
| UGM_chain                                             | discrete functionality not yet exposed |
| UGM_small                                             | discrete functionality not yet exposed |
| VisualISAM2Example                                    | :heavy_check_mark: |
| [VisualISAMExample](sfm/VisualISAMExample.ipynb) | :heavy_check_mark: |

Extra Examples (with no C++ equivalent)
- [Fast CPU bundle adjustment with Full and Schur](sfm/SfmLevenbergMarquardtOptimizerExample.ipynb)
- [CardinalSplineBasisExample](basis/CardinalSplineBasisExample.ipynb)
- [CumulativeSplineTrajectoryExample](basis/CumulativeSplineTrajectoryExample.ipynb)
- [FitBasisExample](basis/FitBasisExample.ipynb)
- [PseudoSpectralChebyshevExample](basis/PseudoSpectralChebyshevExample.ipynb)
- [DogLegOptimizerExample](nonlinear/DogLegOptimizerExample.ipynb)
- [GPSFactorExample](navigation/GPSFactorExample.ipynb)
- PlanarManipulatorExample
- PreintegrationExample
- SFMData

Additional Notebook Examples

- [BearingRange3DExample](slam/BearingRange3DExample.ipynb)
- [DiscreteBayesTree](discrete/DiscreteBayesTree.ipynb)
- [DiscreteMotionModel](discrete/DiscreteMotionModel.ipynb)
- [DiscreteSwitching](discrete/DiscreteSwitching.ipynb)
- [EKF_SLAM](slam/EKF_SLAM.ipynb)
- [EqF](navigation/EqF.ipynb)
- [Gal3ImuASVExample](navigation/Gal3ImuASVExample.ipynb)
- [Gal3ImuExample](navigation/Gal3ImuExample.ipynb)
- [Gal3ImuNEESReset](navigation/Gal3ImuNEESReset.ipynb)
- [LQRExample](linear/LQRExample.ipynb)
- [NavStateImuASVExample](navigation/NavStateImuASVExample.ipynb)
- [NavStateImuExample](navigation/NavStateImuExample.ipynb)
- [NonlinearEqualityExample](nonlinear/NonlinearEqualityExample.ipynb)
- [RangeSLAMExample_plaza2](slam/RangeSLAMExample_plaza2.ipynb)
- [SL4SLAMExample](slam/SL4SLAMExample.ipynb)
- [SinglePointPositioningExample](navigation/SinglePointPositioningExample.ipynb)
- [iLQRExample](nonlinear/iLQRExample.ipynb)
