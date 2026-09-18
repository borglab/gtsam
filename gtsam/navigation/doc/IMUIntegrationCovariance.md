# IMU integration covariance

`PreintegrationParams::integrationCovariance` represents **additional uncertainty introduced while integrating position from velocity** during IMU preintegration. It is separate from accelerometer and gyroscope measurement noise.

This note documents the behavior implemented by GTSAM. It is not a re-derivation of either the manifold preintegration formulation or the tangent-space formulation.

## Where it enters the preintegrated covariance

During each call to `PreintegratedImuMeasurements::integrateMeasurement`, GTSAM first propagates the existing preintegrated covariance and then adds the accelerometer and gyroscope measurement-noise contributions. After those terms, the implementation adds

```text
integrationCovariance * dt
```

directly to the position-position block of the 9x9 preintegrated measurement covariance:

```cpp
preintMeasCov_.block<3, 3>(3, 3).noalias() += iCov * dt;
```

The state ordering of this covariance is rotation, position, velocity. Although the integration term is injected into the position-position block at the current step, later propagation through the state-transition Jacobian can couple that uncertainty into other blocks.

## How it differs from IMU measurement noise

The three covariance parameters play different roles:

- `gyroscopeCovariance` models continuous-time gyroscope measurement white noise.
- `accelerometerCovariance` models continuous-time accelerometer measurement white noise.
- `integrationCovariance` models extra position-integration or discretization uncertainty that is not already represented by those sensor-noise terms.

In `ImuFactor.cpp`, accelerometer and gyroscope noise enter through their measurement Jacobians:

```cpp
preintMeasCov_.noalias() += B * (aCov / dt) * B.transpose();
preintMeasCov_.noalias() += C * (wCov / dt) * C.transpose();
```

`integrationCovariance` instead contributes directly to the position covariance. It therefore changes the uncertainty associated with the preintegrated measurement, and consequently the noise model used by `ImuFactor`, without perturbing the mean preintegrated motion.

## Units

Because GTSAM accumulates the term as

```text
integrationCovariance * dt
```

and the destination block is a position covariance with units of `m^2`, `integrationCovariance` has units of `m^2/s`.

Equivalently, for an isotropic setting

```cpp
params->setIntegrationCovariance(Matrix3::Identity() * q);
```

`q` is a continuous-time position-covariance density. Multiplying it by the integration interval `dt` gives the position covariance added during that step.

## Choosing a value

This parameter should not be confused with accelerometer noise density obtained from an IMU datasheet or an Allan-variance analysis. Those sensor-noise quantities belong in `accelerometerCovariance` and `gyroscopeCovariance`.

`integrationCovariance` is an additional modeling term. Its value should reflect integration/discretization uncertainty that the application intentionally wants to represent beyond the IMU measurement-noise model. GTSAM does not infer this value from the other IMU noise parameters, so applications should document the value and the modeling assumption behind it rather than tune it as though it were another sensor-noise parameter.

For example, `examples/ImuFactorsExample.cpp` uses a small isotropic value for this extra integration uncertainty while specifying accelerometer and gyroscope noise separately.

## Effect on `ImuFactor`

`PreintegratedImuMeasurements` accumulates all of these uncertainty sources in `preintMeasCov_`. When an `ImuFactor` is constructed from the preintegrated measurements, that accumulated covariance determines the factor noise model. Increasing `integrationCovariance` therefore reduces confidence in the preintegrated position constraint while leaving the nominal preintegrated rotation, position, and velocity increments unchanged.

## Relevant implementation files

- `gtsam/navigation/PreintegrationParams.h` defines `integrationCovariance`.
- `gtsam/navigation/ImuFactor.cpp` applies it during covariance propagation.
- `examples/ImuFactorsExample.cpp` shows it configured separately from accelerometer and gyroscope noise.
