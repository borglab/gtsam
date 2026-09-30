//*************************************************************************
// slam_synchronization: Lie-group synchronization and averaging, plus smaller
// SLAM factors and dataset I/O, kept separate from slam.i to reduce wrapper
// build memory.
//*************************************************************************

namespace gtsam {

// Headers needed by classes separated from the original module.
#include <gtsam/geometry/ExtendedPose3.h>
#include <gtsam/geometry/Gal3.h>
#include <gtsam/geometry/Similarity2.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/geometry/SL4.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/geometry/SphericalCamera.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/PlanarProjectionFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

#include <gtsam/slam/StereoFactor.h>
template <POSE, LANDMARK>
virtual class GenericStereoFactor : gtsam::NoiseModelFactor {
  GenericStereoFactor(const gtsam::StereoPoint2& measured,
                      const gtsam::noiseModel::Base* noiseModel,
                      gtsam::Key poseKey, gtsam::Key landmarkKey,
                      const gtsam::Cal3_S2Stereo* K);
  GenericStereoFactor(const gtsam::StereoPoint2& measured,
                      const gtsam::noiseModel::Base* noiseModel,
                      gtsam::Key poseKey, gtsam::Key landmarkKey,
                      const gtsam::Cal3_S2Stereo* K, POSE body_P_sensor);

  GenericStereoFactor(const gtsam::StereoPoint2& measured,
                      const gtsam::noiseModel::Base* noiseModel,
                      gtsam::Key poseKey, gtsam::Key landmarkKey,
                      const gtsam::Cal3_S2Stereo* K, bool throwCheirality,
                      bool verboseCheirality);
  GenericStereoFactor(const gtsam::StereoPoint2& measured,
                      const gtsam::noiseModel::Base* noiseModel,
                      gtsam::Key poseKey, gtsam::Key landmarkKey,
                      const gtsam::Cal3_S2Stereo* K, bool throwCheirality,
                      bool verboseCheirality, POSE body_P_sensor);
  const gtsam::StereoPoint2& measured() const;
  const gtsam::Cal3_S2Stereo::shared_ptr calibration() const;

  // enabling serialization functionality
  void serialize() const;
};
typedef gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>
    GenericStereoFactor3D;

#include <gtsam/slam/ReferenceFrameFactor.h>
template <LANDMARK = {gtsam::Point3}, POSE = {gtsam::Pose3}>
class ReferenceFrameFactor : gtsam::NoiseModelFactor {
  ReferenceFrameFactor(gtsam::Key globalKey, gtsam::Key transKey,
                       gtsam::Key localKey,
                       const gtsam::noiseModel::Base* model);

  gtsam::Vector evaluateError(const LANDMARK& _global, const POSE& trans,
                              const LANDMARK& local) const;

  void print(
      const std::string& s = "",
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter);
};

#include <gtsam/slam/RotateFactor.h>
class RotateFactor : gtsam::NoiseModelFactor {
  RotateFactor(gtsam::Key key, const gtsam::Rot3& P, const gtsam::Rot3& Z,
               const gtsam::noiseModel::Base* model);

  gtsam::Vector evaluateError(const gtsam::Rot3& R) const;
};

class RotateDirectionsFactor : gtsam::NoiseModelFactor {
  RotateDirectionsFactor(gtsam::Key key, const gtsam::Unit3& i_p,
                         const gtsam::Unit3& c_z,
                         const gtsam::noiseModel::Base* model);

  static gtsam::Rot3 Initialize(const gtsam::Unit3& i_p,
                                const gtsam::Unit3& c_z);

  gtsam::Vector evaluateError(const gtsam::Rot3& iRc) const;
};

#include <gtsam/slam/OrientedPlane3Factor.h>
class OrientedPlane3Factor : gtsam::NoiseModelFactor {
  OrientedPlane3Factor();
  OrientedPlane3Factor(const gtsam::Vector4& z,
                       const gtsam::noiseModel::Gaussian* noiseModel,
                       gtsam::Key poseKey, gtsam::Key landmarkKey);

  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                              const gtsam::OrientedPlane3& plane) const;
};

class OrientedPlane3DirectionPrior : gtsam::NoiseModelFactor {
  OrientedPlane3DirectionPrior();
  OrientedPlane3DirectionPrior(gtsam::Key key, const gtsam::Vector4& z,
                               const gtsam::noiseModel::Gaussian* noiseModel);

  gtsam::Vector evaluateError(const gtsam::OrientedPlane3& plane) const;
};

#include <gtsam/slam/PoseTranslationPrior.h>
template <POSE>
virtual class PoseTranslationPrior : gtsam::NoiseModelFactor {
  PoseTranslationPrior(gtsam::Key key, const POSE::Translation& measured,
                       const gtsam::noiseModel::Base* model);
  PoseTranslationPrior(gtsam::Key key, const POSE& pose_z,
                       const gtsam::noiseModel::Base* model);
  const POSE::Translation& measured() const;

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
  const POSE::Rotation& measured() const;
};

typedef gtsam::PoseRotationPrior<gtsam::Pose2> PoseRotationPrior2D;
typedef gtsam::PoseRotationPrior<gtsam::Pose3> PoseRotationPrior3D;

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
    const string& filename,
    std::shared_ptr<gtsam::noiseModel::Base> model = nullptr,
    size_t maxIndex = 0, bool addNoise = false, bool smart = true,
    gtsam::NoiseFormat noiseFormat = gtsam::NoiseFormat::NoiseFormatAUTO,
    gtsam::KernelFunctionType kernelFunctionType =
        gtsam::KernelFunctionType::KernelFunctionTypeNONE);

void save2D(const gtsam::NonlinearFactorGraph& graph,
            const gtsam::Values& config,
            const std::shared_ptr<gtsam::noiseModel::Diagonal> model,
            const string& filename);

// std::vector<gtsam::BetweenFactor<Pose2>::shared_ptr>
// Used in Matlab wrapper
class BetweenFactorPose2s {
  BetweenFactorPose2s();
  size_t size() const;
  gtsam::BetweenFactor<gtsam::Pose2>* at(size_t i) const;
  void push_back(const gtsam::BetweenFactor<gtsam::Pose2>* factor);
};
gtsam::BetweenFactorPose2s parse2DFactors(
    const string& filename,
    const std::shared_ptr<gtsam::noiseModel::Diagonal>& model = nullptr,
    size_t maxIndex = 0);

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

gtsam::BetweenFactorPose3s parse3DFactors(
    const string& filename,
    const std::shared_ptr<gtsam::noiseModel::Diagonal>& model = nullptr,
    size_t maxIndex = 0);

pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load3D(
    const string& filename);

// In 3D, EDGE_SE3_TRACKXYZ records are exposed as BearingRangeFactor3D.
pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> readG2o(
    const string& g2oFile, const bool is3D = false,
    gtsam::KernelFunctionType kernelFunctionType =
        gtsam::KernelFunctionType::KernelFunctionTypeNONE);
void writeG2o(const gtsam::NonlinearFactorGraph& graph,
              const gtsam::Values& estimate, const string& filename);

#include <gtsam/slam/InitializePose3.h>
class InitializePose3 {
  static gtsam::Values computeOrientationsChordal(
      const gtsam::NonlinearFactorGraph& pose3Graph);
  static gtsam::Values computeOrientationsGradient(
      const gtsam::NonlinearFactorGraph& pose3Graph,
      const gtsam::Values& givenGuess, size_t maxIter, const bool setRefFrame);
  static gtsam::Values computeOrientationsGradient(
      const gtsam::NonlinearFactorGraph& pose3Graph,
      const gtsam::Values& givenGuess, size_t maxIter = 10000,
      const bool setRefFrame = true);
  static gtsam::NonlinearFactorGraph buildPose3graph(
      const gtsam::NonlinearFactorGraph& graph);
  static gtsam::Values initializeOrientations(
      const gtsam::NonlinearFactorGraph& graph);
  static gtsam::Values initialize(const gtsam::NonlinearFactorGraph& graph,
                                  const gtsam::Values& givenGuess,
                                  bool useGradient);
  static gtsam::Values initialize(const gtsam::NonlinearFactorGraph& graph);
};

#include <gtsam/slam/FastSync.h>
template <T = {gtsam::Rot2, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3,
               gtsam::Similarity2, gtsam::Similarity3, gtsam::SL4}>
gtsam::Values fastSync(
    const gtsam::NonlinearFactorGraph& graph,
    gtsam::Ordering::OrderingType orderingType = gtsam::Ordering::METIS);
template <T = {gtsam::Rot2, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3,
               gtsam::Similarity2, gtsam::Similarity3, gtsam::SL4}>
gtsam::Values fastSync(const gtsam::NonlinearFactorGraph& graph,
                       const gtsam::Ordering& ordering);

#include <gtsam/slam/KarcherMeanFactor-inl.h>
template <T = {gtsam::Rot2, gtsam::Pose2, gtsam::SO3, gtsam::SO4, gtsam::Rot3,
               gtsam::Pose3, gtsam::Similarity2, gtsam::Similarity3,
               gtsam::Gal3, gtsam::SL4}>
virtual class KarcherMeanFactor : gtsam::NonlinearFactor {
  KarcherMeanFactor(const gtsam::KeyVector& keys);
  KarcherMeanFactor(const gtsam::KeyVector& keys, int d, double beta);
};

template <T = {gtsam::Rot2, gtsam::Pose2, gtsam::SO3, gtsam::SO4, gtsam::Rot3,
               gtsam::Pose3, gtsam::Similarity2, gtsam::Similarity3,
               gtsam::Gal3, gtsam::SL4}>
T FindKarcherMean(const std::vector<T>& elements);

#include <gtsam/slam/FrobeniusFactor.h>
std::shared_ptr<gtsam::noiseModel::Base> ConvertNoiseModel(
    const std::shared_ptr<gtsam::noiseModel::Base>& model, size_t n,
    bool defaultToUnit = true);

template <T = {gtsam::Rot2, gtsam::Rot3, gtsam::SO3, gtsam::SO4, gtsam::Pose2,
               gtsam::Pose3, gtsam::Similarity2, gtsam::Similarity3,
               gtsam::Gal3, gtsam::SL4}>
class FrobeniusPrior : gtsam::NoiseModelFactor {
  FrobeniusPrior(gtsam::Key j, const gtsam::Matrix& M,
                 const gtsam::noiseModel::Base* model = nullptr);

  gtsam::Vector evaluateError(const T& g) const;
};

template <T = {gtsam::Rot2, gtsam::Rot3, gtsam::SO3, gtsam::SO4, gtsam::Pose2,
               gtsam::Pose3, gtsam::Similarity2, gtsam::Similarity3,
               gtsam::Gal3, gtsam::SL4}>
virtual class FrobeniusFactor : gtsam::NoiseModelFactor {
  FrobeniusFactor(gtsam::Key key1, gtsam::Key key2);
  FrobeniusFactor(gtsam::Key j1, gtsam::Key j2, gtsam::noiseModel::Base* model);

  gtsam::Vector evaluateError(const T& T1, const T& T2) const;
};

// Available for all Matrix Lie groups
template <T = {gtsam::Rot2, gtsam::Rot3, gtsam::SO3, gtsam::SO4, gtsam::Pose2,
               gtsam::Pose3, gtsam::Similarity2, gtsam::Similarity3,
               gtsam::Gal3, gtsam::SL4}>
virtual class FrobeniusBetweenFactorNL : gtsam::NoiseModelFactor {
  FrobeniusBetweenFactorNL(gtsam::Key j1, gtsam::Key j2, const T& T12);
  FrobeniusBetweenFactorNL(gtsam::Key key1, gtsam::Key key2, const T& T12,
                           gtsam::noiseModel::Base* model);

  gtsam::Vector evaluateError(const T& T1, const T& T2) const;
};

// FrobeniusBetweenFactor is only available for a subset of matrix Lie Groups
template <T = {gtsam::Rot2, gtsam::Rot3, gtsam::SO3, gtsam::SO4, gtsam::Pose2,
               gtsam::Pose3, gtsam::Gal3}>
virtual class FrobeniusBetweenFactor : gtsam::NoiseModelFactor {
  FrobeniusBetweenFactor(gtsam::Key j1, gtsam::Key j2, const T& T12);
  FrobeniusBetweenFactor(gtsam::Key key1, gtsam::Key key2, const T& T12,
                         gtsam::noiseModel::Base* model);

  gtsam::Vector evaluateError(const T& T1, const T& T2) const;
};

template <T = {gtsam::Rot2, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3}>
virtual class FrobeniusLeftBetweenFactor : gtsam::NoiseModelFactor {
  FrobeniusLeftBetweenFactor(gtsam::Key j1, gtsam::Key j2, const T& iTj);
  FrobeniusLeftBetweenFactor(gtsam::Key j1, gtsam::Key j2, const T& iTj,
                             gtsam::noiseModel::Base* model);

  gtsam::Vector evaluateError(const T& iTw, const T& jTw) const;
};

#include <gtsam/slam/KnownLandmarkFactor.h>
template <POSE = {gtsam::Pose2, gtsam::Pose3}>
virtual class KnownLandmarkFactor : gtsam::NoiseModelFactor {
  KnownLandmarkFactor(gtsam::Key key, const POSE::Translation& wL,
                      const POSE::Translation& measured_kP,
                      const gtsam::noiseModel::Base* model);

  gtsam::Vector evaluateError(const POSE& wTk) const;
};

template <POSE = {gtsam::Pose2, gtsam::Pose3}>
virtual class KnownLandmarkFactor2 : gtsam::NoiseModelFactor {
  KnownLandmarkFactor2(gtsam::Key key, const POSE::Translation& wL,
                       const POSE::Translation& measured_kP,
                       const gtsam::noiseModel::Base* model);

  gtsam::Vector evaluateError(const POSE& kTw) const;
};

#include <gtsam/slam/WahbaFactor.h>
class WahbaFactor : gtsam::NoiseModelFactor {
  WahbaFactor(gtsam::Key key, const gtsam::Unit3& bDirection,
              const gtsam::Unit3& measured_aDirection,
              const gtsam::noiseModel::Base* model);

  gtsam::Vector evaluateError(const gtsam::Rot3& aRb) const;
};

#include <gtsam/slam/RelativeTranslationFactor.h>
// Relative translation measurements for certifiable SE(d) synchronization.
// Concrete declarations let the wrappers use their supported fixed-size Eigen
// aliases while preserving the templated C++ implementation.
virtual class RelativeTranslationFactor2 : gtsam::NoiseModelFactor {
  RelativeTranslationFactor2(gtsam::Key rotationKey, gtsam::Key translationKey1,
                             gtsam::Key translationKey2,
                             const gtsam::Vector2& measured, double weight);

  const gtsam::Vector2& measured() const;
  double weight() const;
};

virtual class RelativeTranslationFactor3 : gtsam::NoiseModelFactor {
  RelativeTranslationFactor3(gtsam::Key rotationKey, gtsam::Key translationKey1,
                             gtsam::Key translationKey2,
                             const gtsam::Vector3& measured, double weight);

  const gtsam::Vector3& measured() const;
  double weight() const;
};

#include <gtsam/slam/TriangulationFactor.h>
template <CAMERA>
virtual class TriangulationFactor : gtsam::NoiseModelFactor {
  TriangulationFactor();
  TriangulationFactor(const CAMERA& camera,
                      const gtsam::This::Measurement& measured,
                      const gtsam::noiseModel::Base* model, gtsam::Key pointKey,
                      bool throwCheirality = false,
                      bool verboseCheirality = false);

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
gtsam::Values initialize(const gtsam::NonlinearFactorGraph& graph,
                         bool useOdometricPath = true);
gtsam::Values initialize(const gtsam::NonlinearFactorGraph& graph,
                         const gtsam::Values& initialGuess);
gtsam::VectorValues initializeOrientations(
    const gtsam::NonlinearFactorGraph& graph, bool useOdometricPath = true);
}  // namespace lago

}  // namespace gtsam