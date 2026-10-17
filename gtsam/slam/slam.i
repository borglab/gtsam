//*************************************************************************
// slam
//*************************************************************************

namespace gtsam {

#include <gtsam/geometry/SO4.h>
#include <gtsam/geometry/SL4.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/ExtendedPose3.h>
#include <gtsam/geometry/Similarity2.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/geometry/Gal3.h>
#include <gtsam/geometry/SphericalCamera.h>
// Following header defines PinholeCamera{Cal3_S2|Cal3DS2|Cal3Bundler|Cal3Fisheye|Cal3Unified}
#include <gtsam/geometry/SimpleCamera.h>

// ######

#include <gtsam/slam/BetweenFactor.h>
template <T = {double, gtsam::Vector, gtsam::Point2, gtsam::Point3, gtsam::Rot2, gtsam::SO3,
               gtsam::SO4, gtsam::SL4, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3,
               gtsam::Similarity2, gtsam::Similarity3, gtsam::Gal3, gtsam::NavState,
               gtsam::Se23, gtsam::ExtendedPose3d, gtsam::imuBias::ConstantBias,
               gtsam::SOn}>
virtual class BetweenFactor : gtsam::NoiseModelFactor {
  BetweenFactor(gtsam::Key key1, gtsam::Key key2, const T& relativePose,
                const gtsam::noiseModel::Base* noiseModel = nullptr);
  const T& measured() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/slam/PlanarProjectionFactor.h>
virtual class PlanarProjectionFactor1 : gtsam::NoiseModelFactor {
  PlanarProjectionFactor1(
    gtsam::Key poseKey,
    const gtsam::Point3& landmark,
    const gtsam::Point2& measured,
    const gtsam::Pose3& bTc,
    const gtsam::Cal3DS2& calib,
    const gtsam::noiseModel::Base* model);
  void serialize() const;
};
virtual class PlanarProjectionFactor2 : gtsam::NoiseModelFactor {
  PlanarProjectionFactor2(
    gtsam::Key poseKey,
    gtsam::Key landmarkKey,
    const gtsam::Point2& measured,
    const gtsam::Pose3& bTc,
    const gtsam::Cal3DS2& calib,
    const gtsam::noiseModel::Base* model);
  void serialize() const;
};
virtual class PlanarProjectionFactor3 : gtsam::NoiseModelFactor {
  PlanarProjectionFactor3(
    gtsam::Key poseKey,
    gtsam::Key offsetKey,
    gtsam::Key calibKey,
    const gtsam::Point3& landmark,
    const gtsam::Point2& measured,
    const gtsam::noiseModel::Base* model);
  void serialize() const;
};

#include <gtsam/slam/ProjectionFactor.h>
template <POSE, LANDMARK, CALIBRATION>
virtual class GenericProjectionFactor : gtsam::NoiseModelFactor {
  GenericProjectionFactor(const gtsam::Point2& measured,
                          const gtsam::noiseModel::Base* noiseModel,
                          gtsam::Key poseKey, gtsam::Key pointKey,
                          const CALIBRATION* k);
  GenericProjectionFactor(const gtsam::Point2& measured,
                          const gtsam::noiseModel::Base* noiseModel,
                          gtsam::Key poseKey, gtsam::Key pointKey, const CALIBRATION* k,
                          const POSE& body_P_sensor);

  GenericProjectionFactor(const gtsam::Point2& measured,
                          const gtsam::noiseModel::Base* noiseModel,
                          gtsam::Key poseKey, gtsam::Key pointKey, const CALIBRATION* k,
                          bool throwCheirality, bool verboseCheirality);
  GenericProjectionFactor(const gtsam::Point2& measured,
                          const gtsam::noiseModel::Base* noiseModel,
                          gtsam::Key poseKey, gtsam::Key pointKey, const CALIBRATION* k,
                          bool throwCheirality, bool verboseCheirality,
                          const POSE& body_P_sensor);

  const gtsam::Point2& measured() const;
  const std::shared_ptr<CALIBRATION> calibration() const;
  bool verboseCheirality() const;
  bool throwCheirality() const;

  // enabling serialization functionality
  void serialize() const;
};
typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3,
                                       gtsam::Cal3_S2>
    GenericProjectionFactorCal3_S2;
typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3,
                                       gtsam::Cal3DS2>
    GenericProjectionFactorCal3DS2;
typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3,
                                       gtsam::Cal3Fisheye>
    GenericProjectionFactorCal3Fisheye;
typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3,
                                       gtsam::Cal3Unified>
    GenericProjectionFactorCal3Unified;

#include <gtsam/slam/GeneralSFMFactor.h>
template <CAMERA, LANDMARK>
virtual class GeneralSFMFactor : gtsam::NoiseModelFactor {
  GeneralSFMFactor(const CAMERA::Measurement& measured,
                   const gtsam::noiseModel::Base* model, gtsam::Key cameraKey,
                   gtsam::Key landmarkKey);
  const CAMERA::Measurement measured() const;
};
typedef gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>,
                                gtsam::Point3>
    GeneralSFMFactorCal3_S2;
typedef gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3DS2>,
                                gtsam::Point3>
    GeneralSFMFactorCal3DS2;
typedef gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>,
                                gtsam::Point3>
    GeneralSFMFactorCal3Bundler;
typedef gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>,
                                gtsam::Point3>
    GeneralSFMFactorCal3Fisheye;
typedef gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>,
                                gtsam::Point3>
    GeneralSFMFactorCal3Unified;

typedef gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3_S2>,
                                gtsam::Point3>
    GeneralSFMFactorPoseCal3_S2;
typedef gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3DS2>,
                                gtsam::Point3>
    GeneralSFMFactorPoseCal3DS2;
typedef gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3Bundler>,
                                gtsam::Point3>
    GeneralSFMFactorPoseCal3Bundler;
typedef gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3Fisheye>,
                                gtsam::Point3>
    GeneralSFMFactorPoseCal3Fisheye;
typedef gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3Unified>,
                                gtsam::Point3>
    GeneralSFMFactorPoseCal3Unified;
typedef gtsam::GeneralSFMFactor<gtsam::SphericalCamera, gtsam::Point3>
    GeneralSFMFactorSphericalCamera;

template <CALIBRATION = {gtsam::Cal3_S2, gtsam::Cal3DS2, gtsam::Cal3f, gtsam::Cal3Bundler,
                         gtsam::Cal3Fisheye, gtsam::Cal3Unified}>
virtual class GeneralSFMFactor2 : gtsam::NoiseModelFactor {
  GeneralSFMFactor2(const gtsam::Point2& measured,
                    const gtsam::noiseModel::Base* model, gtsam::Key poseKey,
                    gtsam::Key landmarkKey, gtsam::Key calibKey);
  const gtsam::Point2 measured() const;

  // enabling serialization functionality
  void serialize() const;
};

}  // namespace gtsam
