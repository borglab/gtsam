# Navigation Factors

This directory contains factors related to navigation, including various IMU factors.

## IMU Factor:

![IMU Factor Diagram](https://www.mathworks.com/help/examples/shared_positioning/win64/FactorGraphPedestrianIMUGPSLocalizationExample_02.png)

The `ImuFactor` is a 5-ways factor involving previous state (pose and velocity of
the vehicle at previous time step), current state (pose and velocity at
current time step), and the bias estimate.
Following the preintegration
scheme proposed in [2], the `ImuFactor` includes many IMU measurements, which
are "summarized" using the PreintegratedIMUMeasurements class.
The figure above, courtesy of [Mathworks' navigation toolbox](https://www.mathworks.com/help/nav/index.html), which are also using our work, shows the factor graph fragment for two time slices.

Note that this factor does not model "temporal consistency" of the biases
(which are usually slowly varying quantities), which is up to the caller.
See also `CombinedImuFactor` for a class that does this for you.

If you are using the factor, please cite:
> Christian Forster, Luca Carlone, Frank Dellaert, and Davide Scaramuzza, "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", IEEE Transactions on Robotics, 2017.

## REFERENCES:
1. G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups",
    Volume 2, 2008.
2. T. Lupton and S.Sukkarieh, "Visual-Inertial-Aided Navigation for
    High-Dynamic Motion in Built Environments Without Initial Conditions",
    TRO, 28(1):61-76, 2012.
3. L. Carlone, S. Williams, R. Roberts, "Preintegrated IMU factor:
    Computation of the Jacobian Matrices", Tech. Report, 2013.
    Available in this repo as "PreintegratedIMUJacobians.pdf".
4. C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, "IMU Preintegration on
    Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation",
    Robotics: Science and Systems (RSS), 2015.

## The Attitude Factor

The `AttitudeFactor` in GTSAM is a factor that constrains the orientation (attitude) of a robot or sensor platform based on directional measurements. Both `Rot3` and `Pose3` versions are available. 

Written up in detail with the help of ChatGPT [here](AttitudeFactor.md).

