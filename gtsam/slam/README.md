# SLAM Factors

## GenericProjectionFactor (defined in ProjectionFactor.h)

Non-linear factor that minimizes the re-projection error with respect to a 2D measurement. 
The calibration is assumed known and passed in the constructor.
The main building block for visual SLAM.

Templated on
- `POSE`, default `Pose3`
- `LANDMARK`, default `Point3`
- `CALIBRATION`, default `Cal3_S2`

## SmartFactors

These are "structure-less" factors, i.e., rather than introducing a new variable for an observed 3D point or landmark, a single factor is created that provides a multi-view constraint on several poses and/or cameras.
While one typically adds multiple GenericProjectionFactors (one for each observation of a landmark), a SmartFactor collects all measurements for a landmark, i.e., the factor graph contains 1 smart factor per landmark.

### SmartFactorBase

This is the base class for smart factors, templated on a `CAMERA` type.
It has no internal point, but it saves the measurements, keeps a noise model, and an optional sensor pose.

### SmartProjectionFactor

Also templated on `CAMERA`. Triangulates a 3D point and keeps an estimate of it around.
This factor operates with monocular cameras, and is used to optimize the camera pose
*and* calibration for each camera, and requires variables of type `CAMERA` in values.

If the calibration is fixed use `SmartProjectionPoseFactor` instead!


### SmartProjectionPoseFactor

Derives from `SmartProjectionFactor` but is templated on a `CALIBRATION` type, setting `CAMERA = PinholePose<CALIBRATION>`.
This factor assumes that the camera calibration is fixed and the same for all cameras involved in this factor.
The factor only constrains poses.

If the calibration should be optimized, as well, use `SmartProjectionFactor` instead!

### SmartProjectionRigFactor

Same as `SmartProjectionPoseFactor`, except:
- it is templated on `CAMERA`, i.e., it allows cameras beyond pinhole;
- it allows measurements from multiple cameras, each camera with fixed but potentially different intrinsics and extrinsics;
- it allows multiple observations from the same pose/key, again, to model a multi-camera system.

## Linearized Smart Factors

The factors below are less likely to be relevant to the user, but result from using the non-linear smart factors above.


### RegularImplicitSchurFactor

A specialization of a GaussianFactor to structure-less SFM, which is very fast in a conjugate gradient (CG) solver. 
It is produced by calling `createRegularImplicitSchurFactor` in `SmartFactorBase` or `SmartProjectionFactor`. 

### JacobianFactorQ

A RegularJacobianFactor that uses some badly documented reduction on the Jacobians.

### JacobianFactorQR

A RegularJacobianFactor that eliminates a point using sequential elimination.

### JacobianFactorSVD

A RegularJacobianFactor that uses the "Nullspace Trick" by Mourikis et al. See the documentation in the file, which *is* well documented.
