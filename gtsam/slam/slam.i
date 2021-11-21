//*************************************************************************
// slam
//*************************************************************************

namespace gtsam {

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/navigation/ImuBias.h>

// ######

#include <gtsam/slam/BetweenFactor.h>
template <T = {Vector, gtsam::Point2, gtsam::Point3, gtsam::Rot2, gtsam::SO3,
               gtsam::SO4, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3,
               gtsam::imuBias::ConstantBias}>
virtual class BetweenFactor : gtsam::NoiseModelFactor {
  BetweenFactor(size_t key1, size_t key2, const T& relativePose,
                const gtsam::noiseModel::Base* noiseModel);
  T measured() const;

  // enabling serialization functionality
  void serialize() const;

  // enable pickling in python
  void pickle() const;
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
};

typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>
    SmartProjectionPose3Factor;

#include <gtsam/slam/StereoFactor.h>
template <POSE, LANDMARK>
virtual class GenericStereoFactor : gtsam::NoiseModelFactor {
  GenericStereoFactor(const gtsam::StereoPoint2& measured,
                      const gtsam::noiseModel::Base* noiseModel, size_t poseKey,
                      size_t landmarkKey, const gtsam::Cal3_S2Stereo* K);
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
};

typedef gtsam::PoseTranslationPrior<gtsam::Pose2> PoseTranslationPrior2D;
typedef gtsam::PoseTranslationPrior<gtsam::Pose3> PoseTranslationPrior3D;

#include <gtsam/slam/PoseRotationPrior.h>
template <POSE>
virtual class PoseRotationPrior : gtsam::NoiseModelFactor {
  PoseRotationPrior(size_t key, const POSE& pose_z,
                    const gtsam::noiseModel::Base* noiseModel);
};

typedef gtsam::PoseRotationPrior<gtsam::Pose2> PoseRotationPrior2D;
typedef gtsam::PoseRotationPrior<gtsam::Pose3> PoseRotationPrior3D;

#include <gtsam/slam/EssentialMatrixFactor.h>
virtual class EssentialMatrixFactor : gtsam::NoiseModelFactor {
  EssentialMatrixFactor(size_t key, const gtsam::Point2& pA,
                        const gtsam::Point2& pB,
                        const gtsam::noiseModel::Base* noiseModel);
};

#include <gtsam/slam/dataset.h>

class SfmTrack {
  SfmTrack();
  SfmTrack(const gtsam::Point3& pt);
  const Point3& point3() const;

  double r;
  double g;
  double b;

  std::vector<pair<size_t, gtsam::Point2>> measurements;

  size_t number_measurements() const;
  pair<size_t, gtsam::Point2> measurement(size_t idx) const;
  pair<size_t, size_t> siftIndex(size_t idx) const;
  void add_measurement(size_t idx, const gtsam::Point2& m);

  // enabling serialization functionality
  void serialize() const;

  // enable pickling in python
  void pickle() const;

  // enabling function to compare objects
  bool equals(const gtsam::SfmTrack& expected, double tol) const;
};

class SfmData {
  SfmData();
  size_t number_cameras() const;
  size_t number_tracks() const;
  gtsam::PinholeCamera<gtsam::Cal3Bundler> camera(size_t idx) const;
  gtsam::SfmTrack track(size_t idx) const;
  void add_track(const gtsam::SfmTrack& t);
  void add_camera(const gtsam::SfmCamera& cam);

  // enabling serialization functionality
  void serialize() const;

  // enable pickling in python
  void pickle() const;

  // enabling function to compare objects
  bool equals(const gtsam::SfmData& expected, double tol) const;
};

gtsam::SfmData readBal(string filename);
bool writeBAL(string filename, gtsam::SfmData& data);
gtsam::Values initialCamerasEstimate(const gtsam::SfmData& db);
gtsam::Values initialCamerasAndPointsEstimate(const gtsam::SfmData& db);

pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load2D(
    string filename, gtsam::noiseModel::Diagonal* model, int maxIndex,
    bool addNoise, bool smart);
pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load2D(
    string filename, gtsam::noiseModel::Diagonal* model, int maxIndex,
    bool addNoise);
pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load2D(
    string filename, gtsam::noiseModel::Diagonal* model, int maxIndex);
pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load2D(
    string filename, gtsam::noiseModel::Diagonal* model);
pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load2D(string filename);
pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load2D_robust(
    string filename, gtsam::noiseModel::Base* model, int maxIndex);
void save2D(const gtsam::NonlinearFactorGraph& graph,
            const gtsam::Values& config, gtsam::noiseModel::Diagonal* model,
            string filename);

// std::vector<gtsam::BetweenFactor<Pose2>::shared_ptr>
// Ignored by pybind -> will be List[BetweenFactorPose2]
class BetweenFactorPose2s {
  BetweenFactorPose2s();
  size_t size() const;
  gtsam::BetweenFactor<gtsam::Pose2>* at(size_t i) const;
  void push_back(const gtsam::BetweenFactor<gtsam::Pose2>* factor);
};
gtsam::BetweenFactorPose2s parse2DFactors(string filename);

// std::vector<gtsam::BetweenFactor<Pose3>::shared_ptr>
// Ignored by pybind -> will be List[BetweenFactorPose3]
class BetweenFactorPose3s {
  BetweenFactorPose3s();
  size_t size() const;
  gtsam::BetweenFactor<gtsam::Pose3>* at(size_t i) const;
  void push_back(const gtsam::BetweenFactor<gtsam::Pose3>* factor);
};
gtsam::BetweenFactorPose3s parse3DFactors(string filename);

pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load3D(string filename);

pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> readG2o(string filename);
pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> readG2o(string filename,
                                                           bool is3D);
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

#include <gtsam/slam/FrobeniusFactor.h>
gtsam::noiseModel::Isotropic* ConvertNoiseModel(gtsam::noiseModel::Base* model,
                                                size_t d);

template <T = {gtsam::SO3, gtsam::SO4}>
virtual class FrobeniusFactor : gtsam::NoiseModelFactor {
  FrobeniusFactor(size_t key1, size_t key2);
  FrobeniusFactor(size_t key1, size_t key2, gtsam::noiseModel::Base* model);

  Vector evaluateError(const T& R1, const T& R2);
};

template <T = {gtsam::SO3, gtsam::SO4}>
virtual class FrobeniusBetweenFactor : gtsam::NoiseModelFactor {
  FrobeniusBetweenFactor(size_t key1, size_t key2, const T& R12);
  FrobeniusBetweenFactor(size_t key1, size_t key2, const T& R12,
                         gtsam::noiseModel::Base* model);

  Vector evaluateError(const T& R1, const T& R2);
};
  
#include <gtsam/slam/lago.h>
namespace lago {
  gtsam::Values initialize(const gtsam::NonlinearFactorGraph& graph, bool useOdometricPath = true);
  gtsam::Values initialize(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initialGuess);
}
  
}  // namespace gtsam
