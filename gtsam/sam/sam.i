//*************************************************************************
// sam
//*************************************************************************

namespace gtsam {

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

// #####

#include <gtsam/sam/RangeFactor.h>
template <POSE, POINT>
virtual class RangeFactor : gtsam::NoiseModelFactor {
  RangeFactor(size_t key1, size_t key2, double measured,
              const gtsam::noiseModel::Base* noiseModel);

  // enabling serialization functionality
  void serialize() const;
};

typedef gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2> RangeFactor2D;
typedef gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3> RangeFactor3D;
typedef gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2> RangeFactorPose2;
typedef gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3> RangeFactorPose3;
typedef gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3>
    RangeFactorCalibratedCameraPoint;
typedef gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3>
    RangeFactorSimpleCameraPoint;
typedef gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera>
    RangeFactorCalibratedCamera;
typedef gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>,
                           gtsam::PinholeCamera<gtsam::Cal3_S2>>
    RangeFactorSimpleCamera;

#include <gtsam/sam/RangeFactor.h>
template <POSE, POINT>
virtual class RangeFactorWithTransform : gtsam::NoiseModelFactor {
  RangeFactorWithTransform(size_t key1, size_t key2, double measured,
                           const gtsam::noiseModel::Base* noiseModel,
                           const POSE& body_T_sensor);

  // enabling serialization functionality
  void serialize() const;
};

typedef gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2>
    RangeFactorWithTransform2D;
typedef gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3>
    RangeFactorWithTransform3D;
typedef gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2>
    RangeFactorWithTransformPose2;
typedef gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3>
    RangeFactorWithTransformPose3;

#include <gtsam/sam/BearingFactor.h>
template <POSE, POINT, BEARING>
virtual class BearingFactor : gtsam::NoiseModelFactor {
  BearingFactor(size_t key1, size_t key2, const BEARING& measured,
                const gtsam::noiseModel::Base* noiseModel);

  // enabling serialization functionality
  void serialize() const;
};

typedef gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2>
    BearingFactor2D;
typedef gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3>
    BearingFactor3D;
typedef gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2>
    BearingFactorPose2;

#include <gtsam/sam/BearingRangeFactor.h>
template <POSE, POINT, BEARING, RANGE>
virtual class BearingRangeFactor : gtsam::NoiseModelFactor {
  BearingRangeFactor(size_t poseKey, size_t pointKey,
                     const BEARING& measuredBearing, const RANGE& measuredRange,
                     const gtsam::noiseModel::Base* noiseModel);

  gtsam::BearingRange<POSE, POINT, BEARING, RANGE> measured() const;

  // enabling serialization functionality
  void serialize() const;
};

typedef gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2,
                                  double>
    BearingRangeFactor2D;
typedef gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2,
                                  double>
    BearingRangeFactorPose2;

}  // namespace gtsam
