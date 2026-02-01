# Nonlinear

The `nonlinear` module in GTSAM includes a comprehensive set of tools for nonlinear optimization using factor graphs. Here's an overview of key components organized by category:

## Core Classes

- [NonlinearFactorGraph](doc/NonlinearFactorGraph.ipynb): Represents the optimization problem as a graph of factors.
- [NonlinearFactor](doc/NonlinearFactor.ipynb): Base class for all nonlinear factors.
- [NoiseModelFactor](doc/NonlinearFactor.ipynb): Base class for factors with noise models.
- [Values](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/Values.h): Container for variable assignments used in optimization.

## Batch Optimizers

- [NonlinearOptimizer](doc/NonlinearOptimizer.ipynb): Base class for all batch optimizers.
    - [NonlinearOptimizerParams](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/NonlinearOptimizerParams.h): Base parameters class for all optimizers.

- [GaussNewtonOptimizer](doc/GaussNewtonOptimizer.ipynb): Implements Gauss-Newton optimization.
    - [GaussNewtonParams](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/GaussNewtonParams.h): Parameters for Gauss-Newton optimization.

- [LevenbergMarquardtOptimizer](doc/LevenbergMarquardtOptimizer.ipynb): Implements Levenberg-Marquardt optimization.
    - [LevenbergMarquardtParams](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/LevenbergMarquardtParams.h): Parameters for Levenberg-Marquardt optimization.

- [DoglegOptimizer](doc/DoglegOptimizer.ipynb): Implements Powell's Dogleg optimization.
    - [DoglegParams](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/DoglegParams.h): Parameters for Dogleg optimization.

- [GncOptimizer](doc/GncOptimizer.ipynb): Implements robust optimization using Graduated Non-Convexity.
    - [GncParams](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/GncParams.h): Parameters for Graduated Non-Convexity optimization.

## Incremental Optimizers

- [ISAM2](doc/ISAM2.ipynb): Incremental Smoothing and Mapping 2, with fluid relinearization.
    - [ISAM2Params](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/ISAM2Params.h): Parameters controlling the ISAM2 algorithm.
    - [ISAM2Result](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/ISAM2Result.h): Results from ISAM2 update operations.
- [NonlinearISAM](doc/NonlinearISAM.ipynb): Original iSAM implementation (mostly superseded by ISAM2).

## Specialized Factors

- [PriorFactor](doc/PriorFactor.ipynb): Imposes a prior constraint on a variable.
- [NonlinearEquality](doc/NonlinearEquality.ipynb): Enforces equality constraints between variables.
- [LinearContainerFactor](doc/LinearContainerFactor.ipynb): Wraps linear factors for inclusion in nonlinear factor graphs.
- [WhiteNoiseFactor](doc/WhiteNoiseFactor.ipynb): Binary factor to estimate parameters of zero-mean Gaussian white noise.

## Filtering and Smoothing

- [ExtendedKalmanFilter](doc/ExtendedKalmanFilter.ipynb): Nonlinear Kalman filter implementation.
- [FixedLagSmoother](doc/FixedLagSmoother.ipynb): Base class for fixed-lag smoothers.
    - [BatchFixedLagSmoother](doc/BatchFixedLagSmoother.ipynb): Implementation of a fixed-lag smoother using batch optimization.
    - [IncrementalFixedLagSmoother](doc/IncrementalFixedLagSmoother.ipynb): Implementation of a fixed-lag smoother using iSAM2.

## Analysis and Visualization

- [Marginals](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/Marginals.h): Computes marginal covariances from optimization results.
- [GraphvizFormatting](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/GraphvizFormatting.h): Provides customization for factor graph visualization.