//*************************************************************************
// slam
//*************************************************************************

namespace gtsam {

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Similarity3.h>

// ######

#include <gtsam/slam/BetweenFactor.h>
template <T = {double, gtsam::Vector, gtsam::Point2, gtsam::Point3, gtsam::Rot2, gtsam::SO3,
               gtsam::SO4, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3, gtsam::Similarity3,
               gtsam::imuBias::ConstantBias}>
virtual class BetweenFactor : gtsam::NoiseModelFactor {
  BetweenFactor(size_t key1, size_t key2, const T& relativePose,
                const gtsam::noiseModel::Base* noiseModel);
  T measured() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/slam/ProjectionFactor.h>
template <POSE, LANDMARK, CALIBRATION>
virtual class GenericProjectionFactor : gtsam::NoiseModelFactor {
  GenericProjectionFactor(const gtsam::Point2& measured,
                          const gtsam::noiseModel::Base* noiseModel,
                          size_t poseKey, size_t pointKey,
                          const CALIBRATION* k);
  GenericProjectionFactor(const gtsam::Point2& measured,
                          const gtsam::noiseModel::Base* noiseModel,
                          size_t poseKey, size_t pointKey, const CALIBRATION* k,
                          const POSE& body_P_sensor);

  GenericProjectionFactor(const gtsam::Point2& measured,
                          const gtsam::noiseModel::Base* noiseModel,
                          size_t poseKey, size_t pointKey, const CALIBRATION* k,
                          bool throwCheirality, bool verboseCheirality);
  GenericProjectionFactor(const gtsam::Point2& measured,
                          const gtsam::noiseModel::Base* noiseModel,
                          size_t poseKey, size_t pointKey, const CALIBRATION* k,
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
                   const gtsam::noiseModel::Base* model, size_t cameraKey,
                   size_t landmarkKey);
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

template <CALIBRATION = {gtsam::Cal3_S2, gtsam::Cal3DS2, gtsam::Cal3Bundler,
                         gtsam::Cal3Fisheye, gtsam::Cal3Unified}>
virtual class GeneralSFMFactor2 : gtsam::NoiseModelFactor {
  GeneralSFMFactor2(const gtsam::Point2& measured,
                    const gtsam::noiseModel::Base* model, size_t poseKey,
                    size_t landmarkKey, size_t calibKey);
  gtsam::Point2 measured() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/slam/SmartProjectionFactor.h>

/// Linearization mode: what factor to linearize to
enum LinearizationMode { HESSIAN, IMPLICIT_SCHUR, JACOBIAN_Q, JACOBIAN_SVD };

/// How to manage degeneracy
enum DegeneracyMode { IGNORE_DEGENERACY, ZERO_ON_DEGENERACY, HANDLE_INFINITY };

class SmartProjectionParams {
  SmartProjectionParams();
  void setLinearizationMode(gtsam::LinearizationMode linMode);
  void setDegeneracyMode(gtsam::DegeneracyMode degMode);
  void setRankTolerance(double rankTol);
  void setEnableEPI(bool enableEPI);
  void setLandmarkDistanceThreshold(bool landmarkDistanceThreshold);
  void setDynamicOutlierRejectionThreshold(bool dynOutRejectionThreshold);
};

#include <gtsam/slam/SmartProjectionPoseFactor.h>
template <CALIBRATION>
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

  void add(const gtsam::Point2& measured_i, size_t poseKey_i);

  // enabling serialization functionality
  void serialize() const;

  gtsam::TriangulationResult point() const;
  gtsam::TriangulationResult point(const gtsam::Values& values) const;
};

typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>
    SmartProjectionPose3Factor;

#include <gtsam/slam/StereoFactor.h>
template <POSE, LANDMARK>
virtual class GenericStereoFactor : gtsam::NoiseModelFactor {
  GenericStereoFactor(const gtsam::StereoPoint2& measured,
                      const gtsam::noiseModel::Base* noiseModel, size_t poseKey,
                      size_t landmarkKey, const gtsam::Cal3_S2Stereo* K);
  GenericStereoFactor(const gtsam::StereoPoint2& measured,
                      const gtsam::noiseModel::Base* noiseModel, size_t poseKey,
                      size_t landmarkKey, const gtsam::Cal3_S2Stereo* K,
                      POSE body_P_sensor);

  GenericStereoFactor(const gtsam::StereoPoint2& measured,
                      const gtsam::noiseModel::Base* noiseModel, size_t poseKey,
                      size_t landmarkKey, const gtsam::Cal3_S2Stereo* K,
                      bool throwCheirality, bool verboseCheirality);
  GenericStereoFactor(const gtsam::StereoPoint2& measured,
                      const gtsam::noiseModel::Base* noiseModel, size_t poseKey,
                      size_t landmarkKey, const gtsam::Cal3_S2Stereo* K,
                      bool throwCheirality, bool verboseCheirality,
                      POSE body_P_sensor);
  gtsam::StereoPoint2 measured() const;
  gtsam::Cal3_S2Stereo* calibration() const;

  // enabling serialization functionality
  void serialize() const;
};
typedef gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>
    GenericStereoFactor3D;

#include <gtsam/slam/PoseTranslationPrior.h>
template <POSE>
virtual class PoseTranslationPrior : gtsam::NoiseModelFactor {
  PoseTranslationPrior(size_t key, const POSE& pose_z,
                       const gtsam::noiseModel::Base* noiseModel);
  POSE::Translation measured() const;

  // enabling serialization functionality
  void serialize() const;
};

typedef gtsam::PoseTranslationPrior<gtsam::Pose2> PoseTranslationPrior2D;
typedef gtsam::PoseTranslationPrior<gtsam::Pose3> PoseTranslationPrior3D;

#include <gtsam/slam/PoseRotationPrior.h>
template <POSE>
virtual class PoseRotationPrior : gtsam::NoiseModelFactor {
  PoseRotationPrior(size_t key, const POSE& pose_z,
                    const gtsam::noiseModel::Base* noiseModel);
  POSE::Rotation measured() const;
};

typedef gtsam::PoseRotationPrior<gtsam::Pose2> PoseRotationPrior2D;
typedef gtsam::PoseRotationPrior<gtsam::Pose3> PoseRotationPrior3D;

#include <gtsam/slam/EssentialMatrixFactor.h>
virtual class EssentialMatrixFactor : gtsam::NoiseModelFactor {
  EssentialMatrixFactor(size_t key, const gtsam::Point2& pA,
                        const gtsam::Point2& pB,
                        const gtsam::noiseModel::Base* noiseModel);
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::EssentialMatrixFactor& other, double tol) const;
  gtsam::Vector evaluateError(const gtsam::EssentialMatrix& E) const;
};

#include <gtsam/slam/EssentialMatrixConstraint.h>
virtual class EssentialMatrixConstraint : gtsam::NoiseModelFactor {
  EssentialMatrixConstraint(size_t key1, size_t key2, const gtsam::EssentialMatrix &measuredE,
                            const gtsam::noiseModel::Base *model);
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::EssentialMatrixConstraint& other, double tol) const;
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
               gtsam::SO3, gtsam::SO4, gtsam::Rot3, gtsam::Pose3}>
virtual class KarcherMeanFactor : gtsam::NonlinearFactor {
  KarcherMeanFactor(const gtsam::KeyVector& keys);
};

gtsam::Rot3 FindKarcherMean(const gtsam::Rot3Vector& rotations);

#include <gtsam/slam/FrobeniusFactor.h>
gtsam::noiseModel::Isotropic* ConvertNoiseModel(gtsam::noiseModel::Base* model,
                                                size_t d);

template <T = {gtsam::SO3, gtsam::SO4}>
virtual class FrobeniusFactor : gtsam::NoiseModelFactor {
  FrobeniusFactor(size_t key1, size_t key2);
  FrobeniusFactor(size_t key1, size_t key2, gtsam::noiseModel::Base* model);

  gtsam::Vector evaluateError(const T& R1, const T& R2);
};

template <T = {gtsam::SO3, gtsam::SO4}>
virtual class FrobeniusBetweenFactor : gtsam::NoiseModelFactor {
  FrobeniusBetweenFactor(size_t key1, size_t key2, const T& R12);
  FrobeniusBetweenFactor(size_t key1, size_t key2, const T& R12,
                         gtsam::noiseModel::Base* model);

  gtsam::Vector evaluateError(const T& R1, const T& R2);
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
}

}  // namespace gtsam
