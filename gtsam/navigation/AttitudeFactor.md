# AttitudeFactor in GTSAM

[Cautionary note: this was generated from the source using ChatGPT but edited by Frank]

## Introduction

The `AttitudeFactor` in GTSAM is a factor that constrains the orientation (attitude) of a robot or sensor platform based on directional measurements. This is particularly useful in GPS-denied navigation contexts where orientation must be estimated from inertial sensors like accelerometers or magnetometers.

This document explains the mathematical foundation of the `AttitudeFactor` and guides users on how to use this factor effectively in GTSAM.

## Mathematical Foundation

### Concept

The `AttitudeFactor` constrains the rotation $\mathbf{R}_{nb}$ (from body frame $b$ to navigation frame $n$) such that a known reference direction in the navigation frame aligns with a measured direction in the body frame, when rotated. The factor enforces that:

$$
\text{nRef} \approx \mathbf{R}_{nb} \cdot \text{bMeasured}
$$

where:

- $\mathbf{R}_{nb}$ is the rotation matrix representing the orientation from body to navigation frame.
- $\text{bMeasured}$ is the measured direction in the body frame (e.g., the accelerometer reading).
- $\text{nRef}$ is the known reference direction in the navigation frame (e.g., the gravity vector in nav).

### Error Function

The error function computes the angular difference between the rotated reference direction and the measured direction:

$$
\mathbf{e} = \text{error}(\text{nRef}, \mathbf{R}_{nb} \cdot \text{bMeasured})
$$

This error is minimal (zero) when the rotated body reference direction aligns perfectly with the measured navigation direction.

The error is computed internally using the `attitudeError` function:

```cpp
Vector AttitudeFactor::attitudeError(const Rot3& nRb) const {
  Unit3 nRotated = nRb * bMeasured_;
  return nRef_.error(nRotated);
}
```

#### Explanation:
- The function computes the rotated measurement `nRotated` and then the error between `nRef` and `nRotated`.
- `nRef_.error(nRotated)` is a 2D vector-valued error between two directions, defined in [Unit3.h](../geometry/Unit3.h).

### Jacobians

For optimization, the $2 \times 3$ Jacobian of the error function with respect to the rotation parameters is required. The Jacobian is computed using chain rule differentiation, involving the derivative of the rotated vector with respect to the rotation parameters and the derivative of the error with respect to the rotated vector.

Note the Jacobian for this specific error function vanishes 180 degrees away from the true direction, and the factor is only expected to behave well when the `nRb` value is initialized in the correct hemisphere.

## Usage in GTSAM

### Including the Header

Include the `AttitudeFactor.h` header in your code:

```cpp
#include <gtsam/navigation/AttitudeFactor.h>
```

### Creating an Attitude Factor

You can create an attitude factor for either a `Rot3` (rotation only) or a `Pose3` (position and rotation) variable.

#### For `Rot3` Variables

```cpp
// Define keys
gtsam::Key rotKey = ...;

// Known reference direction in navigation frame (e.g., gravity)
gtsam::Unit3 nRef(0, 0, -1); // Assuming gravity points down in navigation frame

// Measured direction in body frame (e.g., accelerometer direction)
gtsam::Unit3 bMeasured(0, 0, 9.8); // will be normalized automatically

// Noise model
auto noiseModel = gtsam::noiseModel::Isotropic::Sigma(2, 0.1); // 2D error, sigma = 0.1

// Add to factor graph
gtsam::NonlinearFactorGraph graph;
graph.emplace_shared<Rot3AttitudeFactor>(rotKey, nRef, noiseModel, bMeasured);
```

#### For `Pose3` Variables

There is also a `Pose3AttitudeFactor` that automatically extracts the rotation from the pose, taking into account the chain rule for this operation so the Jacobians with respect to pose are correct.
The same caveat about vanishing Jacobian holds.

```cpp
// Define keys
gtsam::Key poseKey = ...;

// Known reference direction in navigation frame (e.g., gravity)
gtsam::Unit3 nRef(0, 0, -1); // Assuming gravity points down in navigation frame

// Measured direction in body frame (e.g., accelerometer direction)
gtsam::Unit3 bMeasured(0, 0, 9.8); // will be normalized automatically

// Noise model
auto noiseModel = gtsam::noiseModel::Isotropic::Sigma(2, 0.1);

// Add to factor graph
gtsam::NonlinearFactorGraph graph;
graph.emplace_shared<Pose3AttitudeFactor>(poseKey, nRef, noiseModel, bMeasured);
```

### Explanation of Parameters

- **Key**: The variable key in the factor graph corresponding to the `Rot3` or `Pose3` variable you are constraining.
- **nRef**: The known direction in the navigation frame. This is typically obtained from sensors like accelerometers or magnetometers.
- **bMeasured**: The measured direction in the body frame. By default, this is set to the Z-axis `[0; 0; 1]`, but it should match the direction your sensor measures. When constructing a `Unit3`, will be automatically normalized.
- **noiseModel**: The noise model representing the uncertainty in the measurement. Adjust the standard deviation based on the confidence in your measurements.

## Example in GPS-Denied Navigation

In GPS-denied environments, orientation estimation relies heavily on inertial measurements. By incorporating the `AttitudeFactor`, you can:

- Constrain the roll and pitch angles using gravity measurements from an accelerometer.
- Constrain the yaw angle using magnetic field measurements from a magnetometer (with caution due to magnetic disturbances).

This factor helps maintain an accurate orientation estimate over time, which is crucial for applications like drone flight, underwater vehicles, or indoor robotics.

## Conclusion

The `AttitudeFactor` is a useful tool in GTSAM for incorporating orientation measurements into your factor graph. By aligning a measured direction in the body frame with a known reference direction in the navigation frame, it provides a constraint that improves the estimation of the rotation variable. Correct understanding and usage of this factor enhance the performance of navigation algorithms, especially in challenging environments where GPS is unavailable.

# References

- [GTSAM Documentation](https://gtsam.org/)
- [Unit3 Class Reference](https://gtsam.org/doxygen/)