# AttitudeFactor in GTSAM

[Cautionary note: this was generated from the source using ChatGPT but edited by Frank]

## Introduction

The `AttitudeFactor` in GTSAM is a factor that constrains the orientation (attitude) of a robot or sensor platform based on directional measurements. This is particularly useful in GPS-denied navigation contexts where orientation must be estimated from inertial sensors like accelerometers or magnetometers.

This document explains the mathematical foundation of the `AttitudeFactor` and guides users on how to use this factor effectively in GTSAM.

## Mathematical Foundation

### Concept

The `AttitudeFactor` constrains the rotation $\mathbf{R}_{nb}$ (from body frame $b$ to navigation frame $n$) such that a known reference direction in the body frame aligns with a measured direction in the navigation frame when rotated. The factor enforces that:

$$
\mathbf{R}_{nb} \cdot \mathbf{b}_{\text{Ref}} \approx \mathbf{nZ}
$$

where:

- $\mathbf{R}_{nb}$ is the rotation matrix representing the orientation from body to navigation frame.
- $\mathbf{b}_{\text{Ref}}$ is a known reference direction in the body frame (e.g., the accelerometer's sensitive axis).
- $\mathbf{nZ}$ is the measured direction in the navigation frame (e.g., the gravity vector measured by an IMU).

### Error Function

The error function computes the angular difference between the rotated reference direction and the measured direction:

$$
\mathbf{e} = \text{error}(\mathbf{nZ}, \mathbf{R}_{nb} \cdot \mathbf{b}_{\text{Ref}})
$$

This error is minimal (zero) when the rotated body reference direction aligns perfectly with the measured navigation direction.

The error is computed internally using the `attitudeError` function:

```cpp
Vector AttitudeFactor::attitudeError(const Rot3& nRb) const {
  Unit3 nRef = nRb * bRef_;
  return nZ_.error(nRef);
}
```

#### Explanation:
- The function computes the rotated reference direction `nRef` and then the error between `nRef` and the measured direction `nZ`.
- `nZ_.error(nRef)` is a 2D vector-valued error between two directions, defined in [Unit3.h](../geometry/Unit3.h).

### Jacobians

For optimization, the $2 \times 3$ Jacobian of the error function with respect to the rotation parameters is required. The Jacobian is computed using chain rule differentiation, involving the derivative of the rotated vector with respect to the rotation parameters and the derivative of the error with respect to the rotated vector.

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

// Measured direction in navigation frame (e.g., gravity)
gtsam::Unit3 nZ(0, 0, -1); // Assuming gravity points down in navigation frame

// Reference direction in body frame (e.g., accelerometer axis)
gtsam::Unit3 bRef(0, 0, 1); // Default is the Z-axis

// Noise model
auto noiseModel = gtsam::noiseModel::Isotropic::Sigma(2, 0.1); // 2D error, sigma = 0.1

// Add to factor graph
gtsam::NonlinearFactorGraph graph;
graph.emplace_shared<Rot3AttitudeFactor>(rotKey, nZ, noiseModel, bRef);
```

#### For `Pose3` Variables

There is also a `Pose3AttitudeFactor` that automatically extracts the rotation from the pose, taking into account the chain rule for this operation so the Jacobians with respect to pose are correct.

```cpp
// Define keys
gtsam::Key poseKey = ...;

// Measured direction in navigation frame
gtsam::Unit3 nZ(0, 0, -1);

// Reference direction in body frame
gtsam::Unit3 bRef(0, 0, 1);

// Noise model
auto noiseModel = gtsam::noiseModel::Isotropic::Sigma(2, 0.1);

// Add to factor graph
gtsam::NonlinearFactorGraph graph;
graph.emplace_shared<Pose3AttitudeFactor>(poseKey, nZ, noiseModel, bRef);
```

### Explanation of Parameters

- **Key**: The variable key in the factor graph corresponding to the `Rot3` or `Pose3` variable you are constraining.
- **nZ**: The measured direction in the navigation frame. This is typically obtained from sensors like accelerometers or magnetometers.
- **bRef**: The known reference direction in the body frame. By default, this is set to the Z-axis `[0; 0; 1]`, but it should match the direction your sensor measures.
- **noiseModel**: The noise model representing the uncertainty in the measurement.

## Practical Tips

- **Gravity Measurement**: When using an IMU, the accelerometer readings (after removing the dynamic acceleration) can provide the gravity direction in the body frame.
- **Magnetic North Measurement**: A magnetometer can provide the direction of the Earth's magnetic field, which can be used similarly.
- **Reference Direction**: Ensure that `bRef` correctly represents the sensor's measurement axis in the body frame.
- **Noise Model**: The choice of noise model affects the weight of this factor in the optimization. Adjust the standard deviation based on the confidence in your measurements.

## Example in GPS-Denied Navigation

In GPS-denied environments, orientation estimation relies heavily on inertial measurements. By incorporating the `AttitudeFactor`, you can:

- Constrain the roll and pitch angles using gravity measurements from an accelerometer.
- Constrain the yaw angle using magnetic field measurements from a magnetometer (with caution due to magnetic disturbances).

This factor helps maintain an accurate orientation estimate over time, which is crucial for applications like drone flight, underwater vehicles, or indoor robotics.

## Conclusion

The `AttitudeFactor` is a powerful tool in GTSAM for incorporating orientation measurements into your factor graph. By aligning a known reference direction in the body frame with a measured direction in the navigation frame, it provides a constraint that improves the estimation of the rotation variable. Correct understanding and usage of this factor enhance the performance of navigation algorithms, especially in challenging environments where GPS is unavailable.

Remember to verify the alignment of your reference and measured directions and to adjust the noise model to reflect the reliability of your sensors.

# References

- [GTSAM Documentation](https://gtsam.org/)
- [Unit3 Class Reference](https://gtsam.org/doxygen/)