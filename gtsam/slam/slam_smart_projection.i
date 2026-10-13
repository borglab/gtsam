//*************************************************************************
// slam_smart_projection: Smart projection factors, kept separate from slam.i
// to reduce wrapper build memory.
//*************************************************************************

namespace gtsam {

#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/geometry/SphericalCamera.h>

#include <gtsam/slam/SmartFactorBase.h>

// Concrete linear-factor specializations returned by wrapped smart factors.
template <D = {6, 9, 11, 15, 16}>
class RegularHessianFactor : gtsam::HessianFactor {
};

template <D = {6, 9, 11, 15, 16}, ZDIM = {2}>
class JacobianFactorQ : gtsam::JacobianFactor {
};

#include <gtsam/slam/SmartFactorBase.h>

template <CAMERA = {gtsam::PinholeCameraCal3_S2, gtsam::PinholeCameraCal3DS2,
                    gtsam::PinholeCameraCal3Bundler,
                    gtsam::PinholeCameraCal3Fisheye,
                    gtsam::PinholeCameraCal3Unified, gtsam::PinholePoseCal3_S2,
                    gtsam::PinholePoseCal3DS2, gtsam::PinholePoseCal3Bundler,
                    gtsam::PinholePoseCal3Fisheye,
                    gtsam::PinholePoseCal3Unified, gtsam::SphericalCamera}>
virtual class SmartFactorBase : gtsam::NonlinearFactor {
  void add(const CAMERA::Measurement& measured, const gtsam::Key& key);
  void add(const CAMERA::MeasurementVector& measurements,
           const gtsam::KeyVector& cameraKeys);
  const CAMERA::MeasurementVector& measured() const;
  gtsam::CameraSet<CAMERA> cameras(const gtsam::Values& values) const;
};

#include <gtsam/slam/SmartProjectionFactor.h>

/// Linearization mode: what factor to linearize to
enum LinearizationMode { HESSIAN, IMPLICIT_SCHUR, JACOBIAN_Q, JACOBIAN_SVD };

/// How to manage degeneracy
enum DegeneracyMode { IGNORE_DEGENERACY, ZERO_ON_DEGENERACY, HANDLE_INFINITY };

class SmartProjectionParams {
  SmartProjectionParams();
  SmartProjectionParams(
      gtsam::LinearizationMode linMode = gtsam::LinearizationMode::HESSIAN,
      gtsam::DegeneracyMode degMode = gtsam::DegeneracyMode::IGNORE_DEGENERACY,
      bool throwCheirality = false, bool verboseCheirality = false,
      double retriangulationTh = 1e-5);

  void setLinearizationMode(gtsam::LinearizationMode linMode);
  void setDegeneracyMode(gtsam::DegeneracyMode degMode);
  void setRankTolerance(double rankTol);
  void setEnableEPI(bool enableEPI);
  void setLandmarkDistanceThreshold(double landmarkDistanceThreshold);
  void setDynamicOutlierRejectionThreshold(double dynOutRejectionThreshold);

  void print(const std::string& str = "") const;
};

template <CAMERA = {gtsam::PinholeCameraCal3_S2, gtsam::PinholeCameraCal3DS2,
                    gtsam::PinholeCameraCal3Bundler,
                    gtsam::PinholeCameraCal3Fisheye,
                    gtsam::PinholeCameraCal3Unified, gtsam::PinholePoseCal3_S2,
                    gtsam::PinholePoseCal3DS2, gtsam::PinholePoseCal3Bundler,
                    gtsam::PinholePoseCal3Fisheye,
                    gtsam::PinholePoseCal3Unified, gtsam::SphericalCamera}>
virtual class SmartProjectionFactor : gtsam::SmartFactorBase<CAMERA> {
  SmartProjectionFactor();

  SmartProjectionFactor(
      const gtsam::noiseModel::Base* sharedNoiseModel,
      const gtsam::SmartProjectionParams& params = gtsam::SmartProjectionParams());

  bool decideIfTriangulate(const gtsam::CameraSet<CAMERA>& cameras) const;
  gtsam::TriangulationResult triangulateSafe(const gtsam::CameraSet<CAMERA>& cameras) const;
  bool triangulateForLinearize(const gtsam::CameraSet<CAMERA>& cameras) const;

  gtsam::This::SharedHessianFactor createHessianFactor(
      const gtsam::CameraSet<CAMERA>& cameras, const double _lambda = 0.0,
      bool diagonalDamping = false) const;
  gtsam::This::SharedJacobianFactor createJacobianQFactor(
      const gtsam::CameraSet<CAMERA>& cameras, double _lambda) const;
  gtsam::This::SharedJacobianFactor createJacobianQFactor(
      const gtsam::Values& values, double _lambda) const;
  gtsam::JacobianFactor* createJacobianSVDFactor(
      const gtsam::CameraSet<CAMERA>& cameras, double _lambda) const;
  gtsam::This::SharedHessianFactor linearizeToHessian(
      const gtsam::Values& values, double _lambda = 0.0) const;
  gtsam::This::SharedJacobianFactor linearizeToJacobian(
      const gtsam::Values& values, double _lambda = 0.0) const;

  gtsam::GaussianFactor* linearizeDamped(const gtsam::CameraSet<CAMERA>& cameras,
      const double _lambda = 0.0) const;

  gtsam::GaussianFactor* linearizeDamped(const gtsam::Values& values,
      const double _lambda = 0.0) const;

  bool triangulateAndComputeE(gtsam::Matrix& E, const gtsam::CameraSet<CAMERA>& cameras) const;

  bool triangulateAndComputeE(gtsam::Matrix& E, const gtsam::Values& values) const;

  gtsam::Vector reprojectionErrorAfterTriangulation(const gtsam::Values& values) const;

  double totalReprojectionError(
      const gtsam::CameraSet<CAMERA>& cameras,
      std::optional<gtsam::Point3> externalPoint = std::nullopt) const;

  gtsam::TriangulationResult point() const;

  gtsam::TriangulationResult point(const gtsam::Values& values) const;

  bool isValid() const;
  bool isDegenerate() const;
  bool isPointBehindCamera() const;
  bool isOutlier() const;
  bool isFarPoint() const;
};

#include <gtsam/slam/SmartProjectionPoseFactor.h>
// We are not deriving from SmartProjectionFactor yet - too complicated in
// wrapper
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

  void add(const gtsam::Point2& measured, const gtsam::Key& key);

  // enabling serialization functionality
  void serialize() const;

  gtsam::TriangulationResult point() const;
  gtsam::TriangulationResult point(const gtsam::Values& values) const;
};

#include <gtsam/slam/SmartProjectionRigFactor.h>
// Only for pose-only cameras (e.g., PinholePose or SphericalCamera)
template <CAMERA = {gtsam::PinholePoseCal3_S2, gtsam::PinholePoseCal3DS2,
                    gtsam::PinholePoseCal3Bundler,
                    gtsam::PinholePoseCal3Fisheye,
                    gtsam::PinholePoseCal3Unified, gtsam::SphericalCamera}>
virtual class SmartProjectionRigFactor : gtsam::SmartProjectionFactor<CAMERA> {
  SmartProjectionRigFactor();

  SmartProjectionRigFactor(const gtsam::noiseModel::Base* sharedNoiseModel,
                           const gtsam::CameraSet<CAMERA>* cameraRig,
                           const gtsam::SmartProjectionParams& params =
                               gtsam::SmartProjectionParams());

  void add(const CAMERA::Measurement& measured, const gtsam::Key& poseKey,
           const size_t& cameraId = 0);

  void add(
      const CAMERA::MeasurementVector& measurements,
      const gtsam::KeyVector& poseKeys,
      const gtsam::FastVector<size_t>& cameraIds = gtsam::FastVector<size_t>());

  const gtsam::KeyVector& nonUniqueKeys() const;
  const std::shared_ptr<gtsam::This::Cameras>& cameraRig() const;
  const gtsam::FastVector<size_t>& cameraIds() const;
};

}  // namespace gtsam
