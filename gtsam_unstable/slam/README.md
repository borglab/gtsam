# SLAM Factors

## SmartFactors

These are "structure-less" factors, i.e., rather than introducing a new variable for an observed 3D point or landmark, a single factor is created that provides a multi-view constraint on several poses and/or cameras.

### SmartRangeFactor

An experiment in creating a structure-less 2D range-SLAM factor with range-only measurements.
It uses a sophisticated `triangulate` logic based on circle intersections.

### SmartStereoProjectionFactor

Version of `SmartProjectionFactor` for stereo observations, specializes SmartFactorBase for `CAMERA == StereoCamera`.

TODO: a lot of commented out code and could move a lot to .cpp file.

### SmartStereoProjectionPoseFactor

Derives from `SmartStereoProjectionFactor` but adds an array of `Cal3_S2Stereo` calibration objects .

TODO: Again, as no template arguments, we could move a lot to .cpp file.

### SmartStereoProjectionFactorPP

Similar `SmartStereoProjectionPoseFactor` but *additionally* adds an array of body_P_cam poses. The dimensions seem to be hardcoded and the types defined in the SmartFactorBase have been re-defined.  
The body_P_cam poses are optimized here!

TODO: See above, same issues as `SmartStereoProjectionPoseFactor`.

### SmartProjectionPoseFactorRollingShutter

Is templated on a `CAMERA` type and derives from `SmartProjectionFactor`.

This factor optimizes two consecutive poses of a body assuming a rolling
shutter model of the camera with given readout time. The factor requires that
values contain (for each 2D observation) two consecutive camera poses from
which the 2D observation pose can be interpolated.

TODO: the dimensions seem to be hardcoded and the types defined in the SmartFactorBase have been re-defined. Also, possibly a lot of copy/paste computation of things that (should) happen in base class.