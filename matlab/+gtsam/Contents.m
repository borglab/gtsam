% GTSAM MATLAB toolbox interface to the Georgia Tech Smoothing and Mapping C++ library
% Version 2.1 08-Sep-2012
%
%% Basic Utilities
%   linear_independent                - check whether the rows of two matrices are linear independent
%   Permutation                       - class Permutation, see Doxygen page for details
% 
%% General Inference Classes
%   IndexFactor                       - class IndexFactor, see Doxygen page for details
%   SymbolicFactorGraph               - class SymbolicFactorGraph, see Doxygen page for details
%   IndexConditional                  - class IndexConditional, see Doxygen page for details
%   SymbolicBayesNet                  - class SymbolicBayesNet, see Doxygen page for details
%   SymbolicBayesTreeClique           - class SymbolicBayesTreeClique, see Doxygen page for details
%   SymbolicBayesTree                 - class SymbolicBayesTree, see Doxygen page for details
%   SymbolicMultifrontalSolver        - class SymbolicMultifrontalSolver, see Doxygen page for details
%   SymbolicSequentialSolver          - class SymbolicSequentialSolver, see Doxygen page for details
%   VariableIndex                     - class VariableIndex, see Doxygen page for details
% 
%% Linear-Gaussian Graphical Models
%   Errors                            - class Errors, see Doxygen page for details
%   VectorValues                      - class VectorValues, see Doxygen page for details
%   GaussianFactor                    - class GaussianFactor, see Doxygen page for details
%   HessianFactor                     - class HessianFactor, see Doxygen page for details
%   JacobianFactor                    - class JacobianFactor, see Doxygen page for details
%   GaussianFactorGraph               - class GaussianFactorGraph, see Doxygen page for details
%   GaussianDensity                   - class GaussianDensity, see Doxygen page for details
%   GaussianConditional               - class GaussianConditional, see Doxygen page for details
%   GaussianBayesNet                  - class GaussianBayesNet, see Doxygen page for details
%   GaussianBayesTreeClique           - class GaussianBayesTreeClique, see Doxygen page for details
%   GaussianBayesTree                 - class GaussianBayesTree, see Doxygen page for details
%   GaussianISAM                      - class GaussianISAM, see Doxygen page for details
%   noiseModel.Gaussian               - class noiseModel.Gaussian, see Doxygen page for details
%   noiseModel.Diagonal               - class noiseModel.Diagonal, see Doxygen page for details
%   noiseModel.Constrained            - class noiseModel.Constrained, see Doxygen page for details
%   noiseModel.Isotropic              - class noiseModel.Isotropic, see Doxygen page for details
%   noiseModel.Unit                   - class noiseModel.Unit, see Doxygen page for details
% 
%% Linear Inference
%   GaussianSequentialSolver          - class GaussianSequentialSolver, see Doxygen page for details
%   GaussianMultifrontalSolver        - class GaussianMultifrontalSolver, see Doxygen page for details
%   IterativeOptimizationParameters   - class IterativeOptimizationParameters, see Doxygen page for details
%   KalmanFilter                      - class KalmanFilter, see Doxygen page for details
%   SubgraphSolver                    - class SubgraphSolver, see Doxygen page for details
%   SubgraphSolverParameters          - class SubgraphSolverParameters, see Doxygen page for details
%   SuccessiveLinearizationParams     - class SuccessiveLinearizationParams, see Doxygen page for details
%   Sampler                           - class Sampler, see Doxygen page for details
%
%% Nonlinear Factor Graphs
%   Ordering                          - class Ordering, see Doxygen page for details
%   Value                             - class Value, see Doxygen page for details
%   Values                            - class Values, see Doxygen page for details
%   NonlinearFactor                   - class NonlinearFactor, see Doxygen page for details
%   NonlinearFactorGraph              - class NonlinearFactorGraph, see Doxygen page for details
%
%% Nonlinear Optimization
%   ConjugateGradientParameters       - class ConjugateGradientParameters, see Doxygen page for details
%   DoglegOptimizer                   - class DoglegOptimizer, see Doxygen page for details
%   DoglegParams                      - class DoglegParams, see Doxygen page for details
%   GaussNewtonOptimizer              - class GaussNewtonOptimizer, see Doxygen page for details
%   GaussNewtonParams                 - class GaussNewtonParams, see Doxygen page for details
%   ISAM2Clique                       - class ISAM2Clique, see Doxygen page for details
%   ISAM2BayesTree                    - class ISAM2BayesTree, see Doxygen page for details
%   ISAM2                             - class ISAM2, see Doxygen page for details
%   ISAM2DoglegParams                 - class ISAM2DoglegParams, see Doxygen page for details
%   ISAM2GaussNewtonParams            - class ISAM2GaussNewtonParams, see Doxygen page for details
%   ISAM2Params                       - class ISAM2Params, see Doxygen page for details
%   ISAM2Result                       - class ISAM2Result, see Doxygen page for details
%   ISAM2ThresholdMap                 - class ISAM2ThresholdMap, see Doxygen page for details
%   ISAM2ThresholdMapValue            - class ISAM2ThresholdMapValue, see Doxygen page for details
%   InvertedOrdering                  - class InvertedOrdering, see Doxygen page for details
%   JointMarginal                     - class JointMarginal, see Doxygen page for details
%   KeyList                           - class KeyList, see Doxygen page for details
%   KeySet                            - class KeySet, see Doxygen page for details
%   KeyVector                         - class KeyVector, see Doxygen page for details
%   LevenbergMarquardtOptimizer       - class LevenbergMarquardtOptimizer, see Doxygen page for details
%   LevenbergMarquardtParams          - class LevenbergMarquardtParams, see Doxygen page for details
%   Marginals                         - class Marginals, see Doxygen page for details
%   NonlinearISAM                     - class NonlinearISAM, see Doxygen page for details
%   NonlinearOptimizer                - class NonlinearOptimizer, see Doxygen page for details
%   NonlinearOptimizerParams          - class NonlinearOptimizerParams, see Doxygen page for details
%
%% Geometry
%   Cal3_S2                           - class Cal3_S2, see Doxygen page for details
%   Cal3_S2Stereo                     - class Cal3_S2Stereo, see Doxygen page for details
%   Cal3DS2                           - class Cal3DS2, see Doxygen page for details
%   CalibratedCamera                  - class CalibratedCamera, see Doxygen page for details
%   Point2                            - class Point2, see Doxygen page for details
%   Point3                            - class Point3, see Doxygen page for details
%   Pose2                             - class Pose2, see Doxygen page for details
%   Pose3                             - class Pose3, see Doxygen page for details
%   Rot2                              - class Rot2, see Doxygen page for details
%   Rot3                              - class Rot3, see Doxygen page for details
%   SimpleCamera                      - class SimpleCamera, see Doxygen page for details
%   PinholeCameraCal3_S2              - class PinholeCameraCal3_S2, see Doxygen page for details
%   StereoPoint2                      - class StereoPoint2, see Doxygen page for details
% 
%% SLAM and SFM
%   BearingFactor2D                   - class BearingFactor2D, see Doxygen page for details
%   BearingFactor3D                   - class BearingFactor3D, see Doxygen page for details
%   BearingRangeFactor2D              - class BearingRangeFactor2D, see Doxygen page for details
%   BetweenFactorPoint2               - class BetweenFactorPoint2, see Doxygen page for details
%   BetweenFactorPoint3               - class BetweenFactorPoint3, see Doxygen page for details
%   BetweenFactorPose2                - class BetweenFactorPose2, see Doxygen page for details
%   BetweenFactorPose3                - class BetweenFactorPose3, see Doxygen page for details
%   BetweenFactorRot2                 - class BetweenFactorRot2, see Doxygen page for details
%   BetweenFactorRot3                 - class BetweenFactorRot3, see Doxygen page for details
%   GeneralSFMFactor2Cal3_S2          - class GeneralSFMFactor2Cal3_S2, see Doxygen page for details
%   GeneralSFMFactorCal3_S2           - class GeneralSFMFactorCal3_S2, see Doxygen page for details
%   GenericProjectionFactorCal3_S2    - class GenericProjectionFactorCal3_S2, see Doxygen page for details
%   GenericStereoFactor3D             - class GenericStereoFactor3D, see Doxygen page for details
%   NonlinearEqualityCal3_S2          - class NonlinearEqualityCal3_S2, see Doxygen page for details
%   NonlinearEqualityCalibratedCamera - class NonlinearEqualityCalibratedCamera, see Doxygen page for details
%   NonlinearEqualityPoint2           - class NonlinearEqualityPoint2, see Doxygen page for details
%   NonlinearEqualityPoint3           - class NonlinearEqualityPoint3, see Doxygen page for details
%   NonlinearEqualityPose2            - class NonlinearEqualityPose2, see Doxygen page for details
%   NonlinearEqualityPose3            - class NonlinearEqualityPose3, see Doxygen page for details
%   NonlinearEqualityRot2             - class NonlinearEqualityRot2, see Doxygen page for details
%   NonlinearEqualityRot3             - class NonlinearEqualityRot3, see Doxygen page for details
%   NonlinearEqualitySimpleCamera     - class NonlinearEqualitySimpleCamera, see Doxygen page for details
%   NonlinearEqualityStereoPoint2     - class NonlinearEqualityStereoPoint2, see Doxygen page for details
%   PriorFactorCal3_S2                - class PriorFactorCal3_S2, see Doxygen page for details
%   PriorFactorCalibratedCamera       - class PriorFactorCalibratedCamera, see Doxygen page for details
%   PriorFactorPoint2                 - class PriorFactorPoint2, see Doxygen page for details
%   PriorFactorPoint3                 - class PriorFactorPoint3, see Doxygen page for details
%   PriorFactorPose2                  - class PriorFactorPose2, see Doxygen page for details
%   PriorFactorPose3                  - class PriorFactorPose3, see Doxygen page for details
%   PriorFactorRot2                   - class PriorFactorRot2, see Doxygen page for details
%   PriorFactorRot3                   - class PriorFactorRot3, see Doxygen page for details
%   PriorFactorSimpleCamera           - class PriorFactorSimpleCamera, see Doxygen page for details
%   PriorFactorStereoPoint2           - class PriorFactorStereoPoint2, see Doxygen page for details
%   RangeFactorCalibratedCamera       - class RangeFactorCalibratedCamera, see Doxygen page for details
%   RangeFactorCalibratedCameraPoint  - class RangeFactorCalibratedCameraPoint, see Doxygen page for details
%   RangeFactorPose2                  - class RangeFactorPose2, see Doxygen page for details
%   RangeFactorPose3                  - class RangeFactorPose3, see Doxygen page for details
%   RangeFactor2D                     - class RangeFactor2D, see Doxygen page for details
%   RangeFactor3D                     - class RangeFactor3D, see Doxygen page for details
%   RangeFactorSimpleCamera           - class RangeFactorSimpleCamera, see Doxygen page for details
%   RangeFactorSimpleCameraPoint      - class RangeFactorSimpleCameraPoint, see Doxygen page for details
%   VisualISAMGenerateData            - VisualISAMGenerateData creates data for viusalSLAM::iSAM examples
%   VisualISAMInitialize              - VisualISAMInitialize initializes visualSLAM::iSAM object and noise parameters
%   VisualISAMPlot                    - VisualISAMPlot plots current state of ISAM2 object
%   VisualISAMStep                    - VisualISAMStep executes one update step of visualSLAM::iSAM object
%
%% MATLAB-only Utilities
%   CHECK                             - throw an error if an assertion fails
%   circlePose2                       - circlePose2 generates a set of poses in a circle. This function
%   circlePose3                       - circlePose3 generates a set of poses in a circle. This function
%   covarianceEllipse                 - covarianceEllipse plots a Gaussian as an uncertainty ellipse
%   covarianceEllipse3D               - covarianceEllipse3D plots a Gaussian as an uncertainty ellipse
%   EQUALITY                          - test equality of two vectors/matrices up to tolerance
%   findExampleDataFile               - Find a dataset in the examples folder
%   load2D                            - load2D reads a TORO-style 2D pose graph
%   load3D                            - load3D reads a TORO-style 3D pose graph
%   plot2DPoints                      - Plots the Point2's in a values, with optional covariances
%   plot2DTrajectory                  - Plots the Pose2's in a values, with optional covariances
%   plot3DPoints                      - Plots the Point3's in a values, with optional covariances
%   plot3DTrajectory                  - plot3DTrajectory plots a 3D trajectory
%   plotCamera                        - 
%   plotPoint2                        - plotPoint2 shows a Point2, possibly with covariance matrix
%   plotPoint3                        - Plot a Point3 with an optional covariance matrix
%   plotPose2                         - plotPose2 shows a Pose2, possibly with covariance matrix
%   plotPose3                         - plotPose3 shows a Pose, possibly with covariance matrix
%   symbol                            - create a Symbol key
%   symbolChr                         - get character from a symbol key
%   symbolIndex                       - get index from a symbol key
%
%% Wrapped C++ Convenience Functions for use within MATLAB
%   utilities.createKeyList           - Create KeyList from indices
%   utilities.createKeyVector         - Create KeyVector from indices
%   utilities.createKeySet            - Create KeySet from indices
%   utilities.extractPoint2           - Extract all Point2 values into a single matrix [x y]
%   utilities.extractPoint3           - Extract all Point3 values into a single matrix [x y z]
%   utilities.extractPose2            - Extract all Pose2 values into a single matrix [x y theta]
%   utilities.allPose3s               - Extract all Pose3 values
%   utilities.extractPose3            - Extract all Pose3 values into a single matrix [r11 r12 r13 r21 r22 r23 r31 r32 r33 x y z]
%   utilities.perturbPoint2           - Perturb all Point2 values using normally distributed noise
%   utilities.perturbPose2            - Perturb all Pose2 values using normally distributed noise
%   utilities.perturbPoint3           - Perturb all Point3 values using normally distributed noise
%   utilities.insertBackprojections   - Insert a number of initial point values by backprojecting
%   utilities.insertProjectionFactors - Insert multiple projection factors for a single pose key
%   utilities.reprojectionErrors      - Calculate the errors of all projection factors in a graph
%   utilities.localToWorld            - Convert from local to world coordinates
