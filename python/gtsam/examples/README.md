# Porting Progress

| C++ Example Name                                      | Ported |
|-------------------------------------------------------|--------|
| CameraResectioning                                    |        |
| CombinedImuFactorsExample                             |        |
| CreateSFMExampleData                                  |        |
| DiscreteBayesNetExample                               |        |
| DiscreteBayesNet_FG                                   | none of the required discrete functionality is exposed through Python |
| easyPoint2KalmanFilter                                | ExtendedKalmanFilter not yet exposed through Python |
| elaboratePoint2KalmanFilter                           | GaussianSequentialSolver not yet exposed through Python |
| FisheyeExample                                        |        |
| HMMExample                                            |        |
| ImuFactorsExample2                                    | :heavy_check_mark:      |
| ImuFactorsExample                                     |        |
| IMUKittiExampleGPS                                    |        |
| InverseKinematicsExampleExpressions.cpp               |        |
| ISAM2Example_SmartFactor                              |        |
| ISAM2_SmartFactorStereo_IMU                           |        |
| LocalizationExample                                   | :heavy_check_mark:      |
| METISOrderingExample                                  |        |
| OdometryExample                                       | :heavy_check_mark:      |
| PlanarSLAMExample                                     | :heavy_check_mark:      |
| Pose2SLAMExample                                      | :heavy_check_mark:      |
| Pose2SLAMExampleExpressions                           |        |
| Pose2SLAMExample_g2o                                  | :heavy_check_mark:      |
| Pose2SLAMExample_graph                                |        |
| Pose2SLAMExample_graphviz                             |        |
| Pose2SLAMExample_lago                                 | lago not yet exposed through Python |
| Pose2SLAMStressTest                                   |        |
| Pose2SLAMwSPCG                                        |        |
| Pose3Localization                                 |        |
| Pose3SLAMExample_changeKeys                           |        |
| Pose3SLAMExampleExpressions_BearingRangeWithTransform |        |
| Pose3SLAMExample_g2o                                  | :heavy_check_mark:      |
| Pose3SLAMExample_initializePose3Chordal               | :heavy_check_mark:        |
| Pose3SLAMExample_initializePose3Gradient              |        |
| RangeISAMExample_plaza2                               |        |
| SelfCalibrationExample                                |        |
| SFMdata                                               |        |     
| SFMExample_bal_COLAMD_METIS                           |        |
| SFMExample_bal                                        | :heavy_check_mark:      |
| SFMExample                                            | :heavy_check_mark:      |
| SFMExampleExpressions_bal                             |        |
| SFMExampleExpressions                                 |        |
| SFMExample_SmartFactor                                |        |
| SFMExample_SmartFactorPCG                             |        |
| ShonanAveragingCLI                                    | :heavy_check_mark:       |
| SimpleRotation                                        | :heavy_check_mark:      |
| SolverComparer                                        |        |
| StereoVOExample                                       |        |
| StereoVOExample_large                                 |        |
| TimeTBB                                               |        |
| UGM_chain                                             | discrete functionality not yet exposed |
| UGM_small                                             | discrete functionality not yet exposed |
| VisualISAM2Example                                    | :heavy_check_mark:      |
| VisualISAMExample                                     | :heavy_check_mark:      |

Extra Examples (with no C++ equivalent)
- DogLegOptimizerExample
- GPSFactorExample
- PlanarManipulatorExample
- PreintegrationExample
- SFMData
