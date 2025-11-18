//*************************************************************************
// slam
//*************************************************************************

namespace gtsam {

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/geometry/SL4.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Similarity2.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/geometry/Gal3.h>

// ######

#include <gtsam/slam/BetweenFactor.h>
template <T = {double, gtsam::Vector, gtsam::Point2, gtsam::Point3, gtsam::Rot2, gtsam::SO3,
               gtsam::SO4, gtsam::SL4, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3,
               gtsam::Similarity2, gtsam::Similarity3, gtsam::imuBias::ConstantBias}>
virtual class BetweenFactor : gtsam::NoiseModelFactor {
  BetweenFactor(gtsam::Key key1, gtsam::Key key2, const T& relativePose,
                const gtsam::noiseModel::Base* noiseModel);
  T measured() const;

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

  gtsam::Point2 measured() const;
  CALIBRATION* calibration() const;
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
  GeneralSFMFactor(const gtsam::Point2& measured,
                   const gtsam::noiseModel::Base* model, gtsam::Key cameraKey,
                   gtsam::Key landmarkKey);
  gtsam::Point2 measured() const;
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

template <CALIBRATION = {gtsam::Cal3_S2, gtsam::Cal3DS2, gtsam::Cal3f, gtsam::Cal3Bundler,
                         gtsam::Cal3Fisheye, gtsam::Cal3Unified}>
virtual class GeneralSFMFactor2 : gtsam::NoiseModelFactor {
  GeneralSFMFactor2(const gtsam::Point2& measured,
                    const gtsam::noiseModel::Base* model, gtsam::Key poseKey,
                    gtsam::Key landmarkKey, gtsam::Key calibKey);
  gtsam::Point2 measured() const;

  // enabling serialization functionality
  void serialize() const;
};

// Following header defines PinholeCamera{Cal3_S2|Cal3DS2|Cal3Bundler|Cal3Fisheye|Cal3Unified}
#include <gtsam/geometry/SimpleCamera.h>

#include <gtsam/slam/SmartFactorBase.h>

// Currently not wrapping SphericalCamera, since measurement type is not Point2 but Unit3
template <
    CAMERA = {gtsam::PinholeCameraCal3_S2, gtsam::PinholeCameraCal3DS2,
              gtsam::PinholeCameraCal3Bundler, gtsam::PinholeCameraCal3Fisheye,
              gtsam::PinholeCameraCal3Unified, gtsam::PinholePoseCal3_S2,
              gtsam::PinholePoseCal3DS2, gtsam::PinholePoseCal3Bundler,
              gtsam::PinholePoseCal3Fisheye, gtsam::PinholePoseCal3Unified}>
virtual class SmartFactorBase : gtsam::NonlinearFactor {
  void add(const gtsam::Point2& measured, gtsam::Key key);
  void add(const gtsam::Point2Vector& measurements, const gtsam::KeyVector& cameraKeys);
  size_t dim() const;
  const std::vector<gtsam::Point2>& measured() const;
  std::vector<CAMERA> cameras(const gtsam::Values& values) const;

  void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter =
    gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::NonlinearFactor& p, double tol = 1e-9) const;
};

#include <gtsam/slam/SmartProjectionFactor.h>

/// Linearization mode: what factor to linearize to
enum LinearizationMode { HESSIAN, IMPLICIT_SCHUR, JACOBIAN_Q, JACOBIAN_SVD };

/// How to manage degeneracy
enum DegeneracyMode { IGNORE_DEGENERACY, ZERO_ON_DEGENERACY, HANDLE_INFINITY };

class SmartProjectionParams {
  SmartProjectionParams();
  SmartProjectionParams(gtsam::LinearizationMode linMode = gtsam::LinearizationMode::HESSIAN,
    gtsam::DegeneracyMode degMode = gtsam::DegeneracyMode::IGNORE_DEGENERACY, bool throwCheirality = false,
    bool verboseCheirality = false, double retriangulationTh = 1e-5);

  void setLinearizationMode(gtsam::LinearizationMode linMode);
  void setDegeneracyMode(gtsam::DegeneracyMode degMode);
  void setRankTolerance(double rankTol);
  void setEnableEPI(bool enableEPI);
  void setLandmarkDistanceThreshold(bool landmarkDistanceThreshold);
  void setDynamicOutlierRejectionThreshold(bool dynOutRejectionThreshold);

  void print(const std::string& str = "") const;
};

template <
    CAMERA = {gtsam::PinholeCameraCal3_S2, gtsam::PinholeCameraCal3DS2,
              gtsam::PinholeCameraCal3Bundler, gtsam::PinholeCameraCal3Fisheye,
              gtsam::PinholeCameraCal3Unified, gtsam::PinholePoseCal3_S2,
              gtsam::PinholePoseCal3DS2, gtsam::PinholePoseCal3Bundler,
              gtsam::PinholePoseCal3Fisheye, gtsam::PinholePoseCal3Unified}>
virtual class SmartProjectionFactor : gtsam::SmartFactorBase<CAMERA> {
  SmartProjectionFactor();

  SmartProjectionFactor(
      const gtsam::noiseModel::Base* sharedNoiseModel,
      const gtsam::SmartProjectionParams& params = gtsam::SmartProjectionParams());

  bool decideIfTriangulate(const gtsam::CameraSet<CAMERA>& cameras) const;
  gtsam::TriangulationResult triangulateSafe(const gtsam::CameraSet<CAMERA>& cameras) const;
  bool triangulateForLinearize(const gtsam::CameraSet<CAMERA>& cameras) const;

  gtsam::HessianFactor createHessianFactor(
      const gtsam::CameraSet<CAMERA>& cameras, const double lambda = 0.0,
      bool diagonalDamping = false) const;
  gtsam::JacobianFactor createJacobianQFactor(
      const gtsam::CameraSet<CAMERA>& cameras, double lambda) const;
  gtsam::JacobianFactor createJacobianQFactor(
      const gtsam::Values& values, double lambda) const;
  gtsam::JacobianFactor createJacobianSVDFactor(
      const gtsam::CameraSet<CAMERA>& cameras, double lambda) const;
  gtsam::HessianFactor linearizeToHessian(
      const gtsam::Values& values, double lambda = 0.0) const;
  gtsam::JacobianFactor linearizeToJacobian(
      const gtsam::Values& values, double lambda = 0.0) const;

  gtsam::GaussianFactor linearizeDamped(const gtsam::CameraSet<CAMERA>& cameras,
      const double lambda = 0.0) const;

  gtsam::GaussianFactor linearizeDamped(const gtsam::Values& values,
      const double lambda = 0.0) const;

  gtsam::GaussianFactor linearize(
      const gtsam::Values& values) const;

  bool triangulateAndComputeE(gtsam::Matrix& E, const gtsam::CameraSet<CAMERA>& cameras) const;

  bool triangulateAndComputeE(gtsam::Matrix& E, const gtsam::Values& values) const;

  gtsam::Vector reprojectionErrorAfterTriangulation(const gtsam::Values& values) const;

  double totalReprojectionError(const gtsam::CameraSet<CAMERA>& cameras,
    gtsam::Point3 externalPoint) const;

  double error(const gtsam::Values& values) const;

  gtsam::TriangulationResult point() const;

  gtsam::TriangulationResult point(const gtsam::Values& values) const;

  bool isValid() const;
  bool isDegenerate() const;
  bool isPointBehindCamera() const;
  bool isOutlier() const;
  bool isFarPoint() const;
};

#include <gtsam/slam/SmartProjectionPoseFactor.h>
// We are not deriving from SmartProjectionFactor yet - too complicated in wrapper
template <CALIBRATION = {gtsam::Cal3_S2, gtsam::Cal3DS2, gtsam::Cal3Bundler,
                         gtsam::Cal3Fisheye, gtsam::Cal3Unified}>
virtual class SmartProjectionPoseFactor : gtsam::NonlinearFactor {
  SmartProjectionPoseFactor(const gtsam::noiseModel::Base* noise,
                            const CALIBRATION* K);
  SmartProjectionPoseFactor(const gtsam::noiseModel::Base* noise,
                            const CALIBRATION* K,
                            const gtsam::Pose3& body_P_sensor);
  SmartProjectionPoseFactor(const gtsam::noiseModel::Base* noise,
                            const CALIBRATION* K,
                            const gtsam::SmartProjectionParams& params);
  SmartProjectionPoseFactor(const gtsam::noiseModel::Base* noise,
                            const CALIBRATION* K,
                            const gtsam::Pose3& body_P_sensor,
                            const gtsam::SmartProjectionParams& params);

  void add(const gtsam::Point2& measured_i, gtsam::Key poseKey_i);

  // enabling serialization functionality
  void serialize() const;

  gtsam::TriangulationResult point() const;
  gtsam::TriangulationResult point(const gtsam::Values& values) const;
};

#include <gtsam/slam/SmartProjectionRigFactor.h>
// Only for PinholePose cameras -> PinholeCamera is not supported
template <CAMERA = {gtsam::PinholePoseCal3_S2, gtsam::PinholePoseCal3DS2,
  gtsam::PinholePoseCal3Bundler,
  gtsam::PinholePoseCal3Fisheye,
  gtsam::PinholePoseCal3Unified}>
virtual class SmartProjectionRigFactor : gtsam::SmartProjectionFactor<CAMERA> {
  SmartProjectionRigFactor();

  SmartProjectionRigFactor(
      const gtsam::noiseModel::Base* sharedNoiseModel,
      const gtsam::CameraSet<CAMERA>* cameraRig,
      const gtsam::SmartProjectionParams& params = gtsam::SmartProjectionParams());

  void add(const gtsam::Point2& measured, const gtsam::Key& poseKey,
           const size_t& cameraId = 0);

  void add(const gtsam::Point2Vector& measurements, const gtsam::KeyVector& poseKeys,
           const gtsam::FastVector<size_t>& cameraIds = gtsam::FastVector<size_t>());

  const gtsam::KeyVector& nonUniqueKeys() const;
  const gtsam::CameraSet<CAMERA>& cameraRig() const;
  const gtsam::FastVector<size_t>& cameraIds() const;
};

#include <gtsam/slam/StereoFactor.h>
template <POSE, LANDMARK>
virtual class GenericStereoFactor : gtsam::NoiseModelFactor {
  GenericStereoFactor(const gtsam::StereoPoint2& measured,
                      const gtsam::noiseModel::Base* noiseModel, gtsam::Key poseKey,
                      gtsam::Key landmarkKey, const gtsam::Cal3_S2Stereo* K);
  GenericStereoFactor(const gtsam::StereoPoint2& measured,
                      const gtsam::noiseModel::Base* noiseModel, gtsam::Key poseKey,
                      gtsam::Key landmarkKey, const gtsam::Cal3_S2Stereo* K,
                      POSE body_P_sensor);

  GenericStereoFactor(const gtsam::StereoPoint2& measured,
                      const gtsam::noiseModel::Base* noiseModel, gtsam::Key poseKey,
                      gtsam::Key landmarkKey, const gtsam::Cal3_S2Stereo* K,
                      bool throwCheirality, bool verboseCheirality);
  GenericStereoFactor(const gtsam::StereoPoint2& measured,
                      const gtsam::noiseModel::Base* noiseModel, gtsam::Key poseKey,
                      gtsam::Key landmarkKey, const gtsam::Cal3_S2Stereo* K,
                      bool throwCheirality, bool verboseCheirality,
                      POSE body_P_sensor);
  gtsam::StereoPoint2 measured() const;
  gtsam::Cal3_S2Stereo* calibration() const;

  // enabling serialization functionality
  void serialize() const;
};
typedef gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>
    GenericStereoFactor3D;

#include <gtsam/slam/ReferenceFrameFactor.h>
template<LANDMARK = {gtsam::Point3}, POSE = {gtsam::Pose3}>
class ReferenceFrameFactor : gtsam::NoiseModelFactor {
  ReferenceFrameFactor(gtsam::Key globalKey, gtsam::Key transKey, 
                       gtsam::Key localKey, const gtsam::noiseModel::Base* model);

  gtsam::Vector evaluateError(const LANDMARK& global, const POSE& trans, const LANDMARK& local);

  void print(const std::string& s="",
    const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter);
};

#include <gtsam/slam/RotateFactor.h>
class RotateFactor : gtsam::NoiseModelFactor {
  RotateFactor(gtsam::Key key, const gtsam::Rot3& P, const gtsam::Rot3& Z,
    const gtsam::noiseModel::Base* model);

  void print(const std::string& s = "",
    const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;

  gtsam::Vector evaluateError(const gtsam::Rot3& R) const;
};
class RotateDirectionsFactor : gtsam::NoiseModelFactor {
  RotateDirectionsFactor(gtsam::Key key, const gtsam::Unit3& i_p, const gtsam::Unit3& c_z,
    const gtsam::noiseModel::Base* model);

  static gtsam::Rot3 Initialize(const gtsam::Unit3& i_p, const gtsam::Unit3& c_z);

  void print(const std::string& s = "",
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;

  gtsam::Vector evaluateError(const gtsam::Rot3& iRc) const;
};

#include <gtsam/slam/OrientedPlane3Factor.h>
class OrientedPlane3Factor : gtsam::NoiseModelFactor {
  OrientedPlane3Factor();
  OrientedPlane3Factor(const gtsam::Vector& z, const gtsam::noiseModel::Gaussian* noiseModel,
      gtsam::Key poseKey, gtsam::Key landmarkKey);

  void print(const std::string& s = "OrientedPlane3Factor",
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose, const gtsam::OrientedPlane3& plane) const;
};
class OrientedPlane3DirectionPrior : gtsam::NoiseModelFactor {
  OrientedPlane3DirectionPrior();
  OrientedPlane3DirectionPrior(gtsam::Key key, const gtsam::Vector& z,
                               const gtsam::noiseModel::Gaussian* noiseModel);

  void print(const std::string& s = "OrientedPlane3DirectionPrior",
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;

  bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const;

  gtsam::Vector evaluateError(const gtsam::OrientedPlane3& plane) const;
};

#include <gtsam/slam/PoseTranslationPrior.h>
template <POSE>
virtual class PoseTranslationPrior : gtsam::NoiseModelFactor {
  PoseTranslationPrior(gtsam::Key key, const POSE::Translation& measured,
                       const gtsam::noiseModel::Base* model);
  PoseTranslationPrior(gtsam::Key key, const POSE& pose_z,
                       const gtsam::noiseModel::Base* model);
  POSE::Translation measured() const;

  // enabling serialization functionality
  void serialize() const;
};

typedef gtsam::PoseTranslationPrior<gtsam::Pose2> PoseTranslationPrior2D;
typedef gtsam::PoseTranslationPrior<gtsam::Pose3> PoseTranslationPrior3D;

#include <gtsam/slam/PoseRotationPrior.h>
template <POSE>
virtual class PoseRotationPrior : gtsam::NoiseModelFactor {
  PoseRotationPrior(gtsam::Key key, const POSE::Rotation& rot_z,
                    const gtsam::noiseModel::Base* model);
  PoseRotationPrior(gtsam::Key key, const POSE& pose_z,
                    const gtsam::noiseModel::Base* model);
  POSE::Rotation measured() const;
};

typedef gtsam::PoseRotationPrior<gtsam::Pose2> PoseRotationPrior2D;
typedef gtsam::PoseRotationPrior<gtsam::Pose3> PoseRotationPrior3D;

#include <gtsam/slam/EssentialMatrixFactor.h>
virtual class EssentialMatrixFactor : gtsam::NoiseModelFactor {
  EssentialMatrixFactor(gtsam::Key key, 
                        const gtsam::Point2& pA, const gtsam::Point2& pB,
                        const gtsam::noiseModel::Base* model);
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  gtsam::Vector evaluateError(const gtsam::EssentialMatrix& E) const;
};

virtual class EssentialMatrixFactor2 : gtsam::NoiseModelFactor {
  EssentialMatrixFactor2(gtsam::Key key1, gtsam::Key key2, 
                         const gtsam::Point2& pA, const gtsam::Point2& pB,
                         const gtsam::noiseModel::Base* model);
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  gtsam::Vector evaluateError(const gtsam::EssentialMatrix& E, double d) const;
};

virtual class EssentialMatrixFactor3 : gtsam::EssentialMatrixFactor2 {
  EssentialMatrixFactor3(gtsam::Key key1, gtsam::Key key2, 
                         const gtsam::Point2& pA, const gtsam::Point2& pB,
                         const gtsam::Rot3& cRb, const gtsam::noiseModel::Base* model);
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  gtsam::Vector evaluateError(const gtsam::EssentialMatrix& E, double d) const;
};

template <CALIBRATION = {gtsam::Cal3_S2, gtsam::Cal3DS2, gtsam::Cal3f, gtsam::Cal3Bundler,
                         gtsam::Cal3Fisheye, gtsam::Cal3Unified}>
virtual class EssentialMatrixFactor4 : gtsam::NoiseModelFactor {
  EssentialMatrixFactor4(gtsam::Key keyE, gtsam::Key keyK,
                         const gtsam::Point2& pA, const gtsam::Point2& pB,
                         const gtsam::noiseModel::Base* model = nullptr);
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  gtsam::Vector evaluateError(const gtsam::EssentialMatrix& E, const CALIBRATION& K) const;
};

template <CALIBRATION = {gtsam::Cal3_S2, gtsam::Cal3DS2, gtsam::Cal3f, gtsam::Cal3Bundler,
                         gtsam::Cal3Fisheye, gtsam::Cal3Unified}>
virtual class EssentialMatrixFactor5 : gtsam::NoiseModelFactor {
  EssentialMatrixFactor5(gtsam::Key keyE, gtsam::Key keyKa, gtsam::Key keyKb,
                         const gtsam::Point2& pA, const gtsam::Point2& pB,
                         const gtsam::noiseModel::Base* model = nullptr);
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  gtsam::Vector evaluateError(const gtsam::EssentialMatrix& E, 
                              const CALIBRATION& Ka, const CALIBRATION& Kb) const;
};

#include <gtsam/slam/EssentialMatrixConstraint.h>
virtual class EssentialMatrixConstraint : gtsam::NoiseModelFactor {
  EssentialMatrixConstraint(gtsam::Key key1, gtsam::Key key2, const gtsam::EssentialMatrix &measuredE,
                            const gtsam::noiseModel::Base *model);
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::EssentialMatrixConstraint& expected, double tol) const;
  gtsam::Vector evaluateError(const gtsam::Pose3& p1, const gtsam::Pose3& p2) const;
  const gtsam::EssentialMatrix& measured() const;
};

#include <gtsam/slam/dataset.h>

enum NoiseFormat {
  NoiseFormatG2O,
  NoiseFormatTORO,
  NoiseFormatGRAPH,
  NoiseFormatCOV,
  NoiseFormatAUTO
};

enum KernelFunctionType {
  KernelFunctionTypeNONE,
  KernelFunctionTypeHUBER,
  KernelFunctionTypeTUKEY
};

pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load2D(
    string filename, gtsam::noiseModel::Diagonal* model = nullptr,
    size_t maxIndex = 0, bool addNoise = false, bool smart = true,
    gtsam::NoiseFormat noiseFormat = gtsam::NoiseFormat::NoiseFormatAUTO,
    gtsam::KernelFunctionType kernelFunctionType =
        gtsam::KernelFunctionType::KernelFunctionTypeNONE);

void save2D(const gtsam::NonlinearFactorGraph& graph,
            const gtsam::Values& config, gtsam::noiseModel::Diagonal* model,
            string filename);

// std::vector<gtsam::BetweenFactor<Pose2>::shared_ptr>
// Used in Matlab wrapper
class BetweenFactorPose2s {
  BetweenFactorPose2s();
  size_t size() const;
  gtsam::BetweenFactor<gtsam::Pose2>* at(size_t i) const;
  void push_back(const gtsam::BetweenFactor<gtsam::Pose2>* factor);
};
gtsam::BetweenFactorPose2s parse2DFactors(string filename);

// std::vector<gtsam::BetweenFactor<Pose3>::shared_ptr>
// Used in Matlab wrapper
class BetweenFactorPose3s {
  BetweenFactorPose3s();
  size_t size() const;
  gtsam::BetweenFactor<gtsam::Pose3>* at(size_t i) const;
  void push_back(const gtsam::BetweenFactor<gtsam::Pose3>* factor);
};

// std::vector<gtsam::BetweenFactor<SL4>::shared_ptr>
// Used in Matlab wrapper
class BetweenFactorSL4s {
  BetweenFactorSL4s();
  size_t size() const;
  gtsam::BetweenFactor<gtsam::SL4>* at(size_t i) const;
  void push_back(const gtsam::BetweenFactor<gtsam::SL4>* factor);
};

gtsam::BetweenFactorPose3s parse3DFactors(string filename);

pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load3D(string filename);

pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> readG2o(
    string filename, const bool is3D = false,
    gtsam::KernelFunctionType kernelFunctionType =
        gtsam::KernelFunctionType::KernelFunctionTypeNONE);
void writeG2o(const gtsam::NonlinearFactorGraph& graph,
              const gtsam::Values& estimate, string filename);

#include <gtsam/slam/InitializePose3.h>
class InitializePose3 {
  static gtsam::Values computeOrientationsChordal(
      const gtsam::NonlinearFactorGraph& pose3Graph);
  static gtsam::Values computeOrientationsGradient(
      const gtsam::NonlinearFactorGraph& pose3Graph,
      const gtsam::Values& givenGuess, size_t maxIter, const bool setRefFrame);
  static gtsam::Values computeOrientationsGradient(
      const gtsam::NonlinearFactorGraph& pose3Graph,
      const gtsam::Values& givenGuess);
  static gtsam::NonlinearFactorGraph buildPose3graph(
      const gtsam::NonlinearFactorGraph& graph);
  static gtsam::Values initializeOrientations(
      const gtsam::NonlinearFactorGraph& graph);
  static gtsam::Values initialize(const gtsam::NonlinearFactorGraph& graph,
                                  const gtsam::Values& givenGuess,
                                  bool useGradient);
  static gtsam::Values initialize(const gtsam::NonlinearFactorGraph& graph);
};

#include <gtsam/slam/KarcherMeanFactor-inl.h>
template <T = {gtsam::Point2, gtsam::Rot2, gtsam::Pose2, gtsam::Point3,
               gtsam::SO3, gtsam::SO4, gtsam::Rot3, gtsam::Pose3, gtsam::Similarity2, gtsam::Similarity3, gtsam::Gal3, gtsam::SL4}>
virtual class KarcherMeanFactor : gtsam::NonlinearFactor {
  KarcherMeanFactor(const gtsam::KeyVector& keys);
  KarcherMeanFactor(const gtsam::KeyVector& keys, int d, double beta);
};

template <T = {gtsam::Point2, gtsam::Rot2, gtsam::Pose2, gtsam::Point3,
  gtsam::SO3, gtsam::SO4, gtsam::Rot3, gtsam::Pose3}>
T FindKarcherMean(const std::vector<T>& elements);

#include <gtsam/slam/FrobeniusFactor.h>
gtsam::noiseModel::Isotropic* ConvertNoiseModel(gtsam::noiseModel::Base* model,
                                                size_t d);

template <T = {gtsam::Rot2, gtsam::Rot3, gtsam::SO3, gtsam::SO4, gtsam::Pose2, gtsam::Pose3, gtsam::Similarity2, gtsam::Similarity3, gtsam::Gal3, gtsam::SL4}>
class FrobeniusPrior : gtsam::NoiseModelFactor {
  FrobeniusPrior(gtsam::Key j, const gtsam::Matrix& M,
    const gtsam::noiseModel::Base* model);

    gtsam::Vector evaluateError(const T& g) const;
};

template <T = {gtsam::Rot2, gtsam::Rot3, gtsam::SO3, gtsam::SO4, gtsam::Pose2, gtsam::Pose3, gtsam::Similarity2, gtsam::Similarity3, gtsam::Gal3, gtsam::SL4}>
virtual class FrobeniusFactor : gtsam::NoiseModelFactor {
  FrobeniusFactor(gtsam::Key key1, gtsam::Key key2);
  FrobeniusFactor(gtsam::Key j1, gtsam::Key j2, gtsam::noiseModel::Base* model);

  gtsam::Vector evaluateError(const T& T1, const T& T2);
};

// Available for all Matrix Lie groups
template <T = {gtsam::Rot2, gtsam::Rot3, gtsam::SO3, gtsam::SO4, gtsam::Pose2, gtsam::Pose3, gtsam::Similarity2, gtsam::Similarity3, gtsam::Gal3, gtsam::SL4}>
virtual class FrobeniusBetweenFactorNL : gtsam::NoiseModelFactor {
  FrobeniusBetweenFactorNL(gtsam::Key j1, gtsam::Key j2, const T& T12);
  FrobeniusBetweenFactorNL(gtsam::Key key1, gtsam::Key key2, const T& T12,
                         gtsam::noiseModel::Base* model);

  gtsam::Vector evaluateError(const T& T1, const T& T2);
};

// FrobeniusBetweenFactor is only available for a subset of matrix Lie Groups
template <T = {gtsam::Rot2, gtsam::Rot3, gtsam::SO3, gtsam::SO4, gtsam::Pose2, gtsam::Pose3, gtsam::Gal3}>
virtual class FrobeniusBetweenFactor : gtsam::NoiseModelFactor {
  FrobeniusBetweenFactor(gtsam::Key j1, gtsam::Key j2, const T& T12);
  FrobeniusBetweenFactor(gtsam::Key key1, gtsam::Key key2, const T& T12,
                         gtsam::noiseModel::Base* model);

  gtsam::Vector evaluateError(const T& T1, const T& T2);
};

#include <gtsam/slam/TriangulationFactor.h>
template <CAMERA>
virtual class TriangulationFactor : gtsam::NoiseModelFactor {
  TriangulationFactor();
  TriangulationFactor(const CAMERA& camera, const gtsam::This::Measurement& measured,
                      const gtsam::noiseModel::Base* model, gtsam::Key pointKey,
                      bool throwCheirality = false,
                      bool verboseCheirality = false);

  void print(const string& s = "", const gtsam::KeyFormatter& keyFormatter =
                                       gtsam::DefaultKeyFormatter) const;
  bool equals(const This& p, double tol = 1e-9) const;

  gtsam::Vector evaluateError(const gtsam::Point3& point) const;

  const gtsam::This::Measurement& measured() const;
};
typedef gtsam::TriangulationFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>
    TriangulationFactorCal3_S2;
typedef gtsam::TriangulationFactor<gtsam::PinholeCamera<gtsam::Cal3DS2>>
    TriangulationFactorCal3DS2;
typedef gtsam::TriangulationFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>>
    TriangulationFactorCal3Bundler;
typedef gtsam::TriangulationFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>
    TriangulationFactorCal3Fisheye;
typedef gtsam::TriangulationFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>>
    TriangulationFactorCal3Unified;

typedef gtsam::TriangulationFactor<gtsam::PinholePose<gtsam::Cal3_S2>>
    TriangulationFactorPoseCal3_S2;
typedef gtsam::TriangulationFactor<gtsam::PinholePose<gtsam::Cal3DS2>>
    TriangulationFactorPoseCal3DS2;
typedef gtsam::TriangulationFactor<gtsam::PinholePose<gtsam::Cal3Bundler>>
    TriangulationFactorPoseCal3Bundler;
typedef gtsam::TriangulationFactor<gtsam::PinholePose<gtsam::Cal3Fisheye>>
    TriangulationFactorPoseCal3Fisheye;
typedef gtsam::TriangulationFactor<gtsam::PinholePose<gtsam::Cal3Unified>>
    TriangulationFactorPoseCal3Unified;

#include <gtsam/slam/lago.h>
namespace lago {
  gtsam::Values initialize(const gtsam::NonlinearFactorGraph& graph, bool useOdometricPath = true);
  gtsam::Values initialize(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initialGuess);
  gtsam::VectorValues initializeOrientations(const gtsam::NonlinearFactorGraph& graph, bool useOdometricPath = true);
}

}  // namespace gtsam
