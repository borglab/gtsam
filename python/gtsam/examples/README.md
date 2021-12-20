# Porting Progress

| C++ Example Name                                      | Ported |
|-------------------------------------------------------|--------|
| CameraResectioning                                    |        |
| CreateSFMExampleData                                  |        |
| DiscreteBayesNet_FG                                   | none of the required discrete functionality is exposed through Python |
| easyPoint2KalmanFilter                                | ExtendedKalmanFilter not yet exposed through Python |
| elaboratePoint2KalmanFilter                           | GaussianSequentialSolver not yet exposed through Python |
| ImuFactorExample2                                     | X      |
| ImuFactorsExample                                     |        |
| ISAM2Example_SmartFactor                              |        |
| ISAM2_SmartFactorStereo_IMU                           |        |
| LocalizationExample                                   | impossible? |
| METISOrderingExample                                  |        |
| OdometryExample                                       | X      |
| PlanarSLAMExample                                     | X      |
| Pose2SLAMExample                                      | X      |
| Pose2SLAMExampleExpressions                           |        |
| Pose2SLAMExample_g2o                                  | X      |
| Pose2SLAMExample_graph                                |        |
| Pose2SLAMExample_graphviz                             |        |
| Pose2SLAMExample_lago                                 | lago not yet exposed through Python |
| Pose2SLAMStressTest                                   |        |
| Pose2SLAMwSPCG                                        |        |
| Pose3SLAMExample_changeKeys                           |        |
| Pose3SLAMExampleExpressions_BearingRangeWithTransform |        |
| Pose3SLAMExample_g2o                                  | X      |
| Pose3SLAMExample_initializePose3Chordal               |        |
| Pose3SLAMExample_initializePose3Gradient              |        |
| RangeISAMExample_plaza2                               |        |
| SelfCalibrationExample                                |        |
| SFMExample_bal_COLAMD_METIS                           |        |
| SFMExample_bal                                        |        |
| SFMExample                                            | X      |
| SFMExampleExpressions_bal                             |        |
| SFMExampleExpressions                                 |        |
| SFMExample_SmartFactor                                |        |
| SFMExample_SmartFactorPCG                             |        |
| SimpleRotation                                        | X      |
| SolverComparer                                        |        |
| StereoVOExample                                       |        |
| StereoVOExample_large                                 |        |
| TimeTBB                                               |        |
| UGM_chain                                             | discrete functionality not yet exposed |
| UGM_small                                             | discrete functionality not yet exposed |
| VisualISAM2Example                                    | X      |
| VisualISAMExample                                     | X      |

Extra Examples (with no C++ equivalent)
- PlanarManipulatorExample
- SFMData