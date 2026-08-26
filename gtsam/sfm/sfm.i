//*************************************************************************
// sfm
//*************************************************************************

namespace gtsam {

#include <gtsam/sfm/SfmEliminationMode.h>
enum class SfmEliminationMode { Full, Schur };

#include <gtsam/sfm/SfmLevenbergMarquardt.h>
virtual class SfmLevenbergMarquardtParams
    : gtsam::LevenbergMarquardtParams {
  SfmLevenbergMarquardtParams();

  static gtsam::SfmLevenbergMarquardtParams legacyDefaults();
  static gtsam::SfmLevenbergMarquardtParams ceresDefaults();

  gtsam::SfmEliminationMode getEliminationMode() const;
  void setEliminationMode(gtsam::SfmEliminationMode mode);
  void print(const string& str = "") const;
};

virtual class SfmLevenbergMarquardtOptimizer
    : gtsam::LevenbergMarquardtOptimizer {
  static gtsam::Ordering CreateReducedOrdering(
      const gtsam::NonlinearFactorGraph& graph,
      const gtsam::Values& initialValues);
  static gtsam::Ordering CreateSchurOrdering(
      const gtsam::NonlinearFactorGraph& graph,
      const gtsam::Ordering& reducedOrdering);

  SfmLevenbergMarquardtOptimizer(
      const gtsam::NonlinearFactorGraph& graph,
      const gtsam::Values& initialValues,
      const gtsam::SfmLevenbergMarquardtParams& params =
          gtsam::SfmLevenbergMarquardtParams());
};

#include <gtsam/sfm/SfmTrack.h>
class SfmTrack2d {
  std::vector<gtsam::SfmMeasurement> measurements;

  SfmTrack2d();
  SfmTrack2d(const std::vector<gtsam::SfmMeasurement>& measurements);
  size_t numberMeasurements() const;
  const gtsam::SfmMeasurement& measurement(size_t idx) const;
  const gtsam::SiftIndex& siftIndex(size_t idx) const;
  void addMeasurement(size_t idx, const gtsam::Point2& m);
  bool hasUniqueCameras() const;
  Eigen::MatrixX2d measurementMatrix() const;
  Eigen::VectorXi indexVector() const;
};

virtual class SfmTrack : gtsam::SfmTrack2d {
  SfmTrack();
  SfmTrack(const gtsam::Point3& pt);
  SfmTrack(const gtsam::Point3& pt, float r, float g, float b);
  const gtsam::Point3& point3() const;

  Point3 p;

  double r;
  double g;
  double b;

  // enabling serialization functionality
  void serialize() const;

  // enabling function to compare objects
  bool equals(const gtsam::SfmTrack& sfmTrack, double tol) const;
};

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/SfmData.h>
class SfmData {
  SfmData();
  static gtsam::SfmData FromBundlerFile(const string& filename);
  static gtsam::SfmData FromBalFile(const string& filename);

  const std::vector<gtsam::SfmTrack>& trackList() const;
  const std::vector<gtsam::PinholeCamera<gtsam::Cal3Bundler>>& cameraList()
      const;

  void addTrack(const gtsam::SfmTrack& t);
  void addCamera(const gtsam::SfmCamera& cam);
  size_t numberTracks() const;
  size_t numberCameras() const;
  const gtsam::SfmTrack& track(size_t idx) const;
  const gtsam::PinholeCamera<gtsam::Cal3Bundler>& camera(size_t idx) const;

  gtsam::NonlinearFactorGraph generalSfmFactors(
      const gtsam::SharedNoiseModel& model =
          gtsam::noiseModel::Isotropic::Sigma(2, 1.0)) const;
  gtsam::NonlinearFactorGraph sfmFactorGraph(
      const gtsam::SharedNoiseModel& model =
          gtsam::noiseModel::Isotropic::Sigma(2, 1.0),
      std::optional<size_t> fixedCamera = 0,
      std::optional<size_t> fixedPoint = 0) const;

  // enabling serialization functionality
  void serialize() const;

  // enabling function to compare objects
  bool equals(const gtsam::SfmData& sfmData, double tol) const;
};

gtsam::SfmData readBal(const string& filename);
bool writeBAL(const string& filename, const gtsam::SfmData& data);
gtsam::Values initialCamerasEstimate(const gtsam::SfmData& db);
gtsam::Values initialCamerasAndPointsEstimate(const gtsam::SfmData& db);

#include <gtsam/sfm/TransferFactor.h>
#include <gtsam/geometry/FundamentalMatrix.h>
template <F = {gtsam::SimpleFundamentalMatrix, gtsam::FundamentalMatrix}>
virtual class TransferFactor : gtsam::NoiseModelFactor {
  TransferFactor(gtsam::EdgeKey edge1, gtsam::EdgeKey edge2,
                 const std::vector<std::tuple<gtsam::Point2, gtsam::Point2, gtsam::Point2>>& triplets,
                 const gtsam::noiseModel::Base* model = nullptr);
};

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3f.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Cal3Unified.h>
template <K = {gtsam::Cal3_S2, gtsam::Cal3f, gtsam::Cal3Bundler}>
virtual class EssentialTransferFactor : gtsam::NoiseModelFactor {
  EssentialTransferFactor(gtsam::EdgeKey edge1, gtsam::EdgeKey edge2,
                          const std::vector<std::tuple<gtsam::Point2, gtsam::Point2, gtsam::Point2>>& triplets,
                          const K* calibration,
                          const gtsam::noiseModel::Base* model = nullptr);
};

template <K = {gtsam::Cal3_S2, gtsam::Cal3f, gtsam::Cal3Bundler}>
virtual class EssentialTransferFactorK : gtsam::NoiseModelFactor {
  EssentialTransferFactorK(gtsam::EdgeKey edge1, gtsam::EdgeKey edge2,
                           const std::vector<std::tuple<gtsam::Point2, gtsam::Point2, gtsam::Point2>>& triplets,
                           const gtsam::noiseModel::Base* model = nullptr);
  EssentialTransferFactorK(gtsam::EdgeKey edge1, gtsam::EdgeKey edge2, gtsam::Key keyK,
                           const std::vector<std::tuple<gtsam::Point2, gtsam::Point2, gtsam::Point2>>& triplets,
                           const gtsam::noiseModel::Base* model = nullptr);
};

#include <gtsam/sfm/SelfCalibrationFactor.h>
virtual class SelfCalibrationFactor : gtsam::NoiseModelFactor {
  SelfCalibrationFactor(gtsam::Key fi_key, gtsam::Key fj_key, const gtsam::Matrix3& F,
               const gtsam::Vector2& pp_i, const gtsam::Vector2& pp_j,
               const gtsam::noiseModel::Base* model = nullptr);
};

#include <gtsam/sfm/EssentialMatrixFactor.h>
virtual class EssentialMatrixFactor : gtsam::NoiseModelFactor {
  EssentialMatrixFactor(gtsam::Key key,
                        const gtsam::Point2& pA, const gtsam::Point2& pB,
                        const gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::EssentialMatrix& E) const;
};

virtual class EssentialMatrixFactor2 : gtsam::NoiseModelFactor {
  EssentialMatrixFactor2(gtsam::Key key1, gtsam::Key key2,
                         const gtsam::Point2& pA, const gtsam::Point2& pB,
                         const gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::EssentialMatrix& E,
                              const double& d) const;
};

virtual class EssentialMatrixFactor3 : gtsam::EssentialMatrixFactor2 {
  EssentialMatrixFactor3(gtsam::Key key1, gtsam::Key key2,
                         const gtsam::Point2& pA, const gtsam::Point2& pB,
                         const gtsam::Rot3& cRb,
                         const gtsam::noiseModel::Base* model);
};

template <CALIBRATION = {gtsam::Cal3_S2, gtsam::Cal3DS2, gtsam::Cal3f, gtsam::Cal3Bundler,
                         gtsam::Cal3Fisheye, gtsam::Cal3Unified}>
virtual class EssentialMatrixFactor4 : gtsam::NoiseModelFactor {
  EssentialMatrixFactor4(gtsam::Key keyE, gtsam::Key keyK,
                         const gtsam::Point2& pA, const gtsam::Point2& pB,
                         const gtsam::noiseModel::Base* model = nullptr);
  gtsam::Vector evaluateError(const gtsam::EssentialMatrix& E,
                              const CALIBRATION& K) const;
};

template <CALIBRATION = {gtsam::Cal3_S2, gtsam::Cal3DS2, gtsam::Cal3f, gtsam::Cal3Bundler,
                         gtsam::Cal3Fisheye, gtsam::Cal3Unified}>
virtual class EssentialMatrixFactor5 : gtsam::NoiseModelFactor {
  EssentialMatrixFactor5(gtsam::Key keyE, gtsam::Key keyKa, gtsam::Key keyKb,
                         const gtsam::Point2& pA, const gtsam::Point2& pB,
                         const gtsam::noiseModel::Base* model = nullptr);
  gtsam::Vector evaluateError(const gtsam::EssentialMatrix& E,
                              const CALIBRATION& Ka,
                              const CALIBRATION& Kb) const;
};

#include <gtsam/sfm/EssentialMatrixConstraint.h>
virtual class EssentialMatrixConstraint : gtsam::NoiseModelFactor {
  EssentialMatrixConstraint(
      gtsam::Key key1, gtsam::Key key2,
      const gtsam::EssentialMatrix& measuredE,
      const gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::Pose3& p1,
                              const gtsam::Pose3& p2) const;
  const gtsam::EssentialMatrix& measured() const;
};

#include <gtsam/sfm/ShonanFactor.h>

virtual class ShonanFactor3 : gtsam::NoiseModelFactor {
  ShonanFactor3(gtsam::Key key1, gtsam::Key key2, const gtsam::Rot3& R12, size_t p);
  ShonanFactor3(gtsam::Key key1, gtsam::Key key2, const gtsam::Rot3& R12, size_t p,
                gtsam::noiseModel::Base* model);
  gtsam::Vector evaluateError(const gtsam::SOn& Q1, const gtsam::SOn& Q2,
                              gtsam::OptionalMatrixType H1 = nullptr,
                              gtsam::OptionalMatrixType H2 = nullptr) const;
};

#include <gtsam/sfm/UnaryMeasurement.h>
template <T>
class UnaryMeasurement {
  UnaryMeasurement(gtsam::Key key, const T& measured,
                   const gtsam::noiseModel::Base* model);
  gtsam::Key key() const;
  const T& measured() const;
  const std::shared_ptr<gtsam::noiseModel::Base>& noiseModel() const;
};

typedef gtsam::UnaryMeasurement<gtsam::Pose3> UnaryMeasurementPose3;
typedef gtsam::UnaryMeasurement<gtsam::Rot3> UnaryMeasurementRot3;
typedef gtsam::UnaryMeasurement<gtsam::Point3> UnaryMeasurementPoint3;

#include <gtsam/sfm/BinaryMeasurement.h>
template <T>
class BinaryMeasurement {
  BinaryMeasurement(gtsam::Key key1, gtsam::Key key2, const T& measured,
                    const gtsam::noiseModel::Base* model);
  gtsam::Key key1() const;
  gtsam::Key key2() const;
  const T& measured() const;
  const std::shared_ptr<gtsam::noiseModel::Base>& noiseModel() const;
};

typedef gtsam::BinaryMeasurement<gtsam::Unit3> BinaryMeasurementUnit3;
typedef gtsam::BinaryMeasurement<gtsam::Rot3> BinaryMeasurementRot3;
typedef gtsam::BinaryMeasurement<gtsam::Point3> BinaryMeasurementPoint3;

// Used in Matlab wrapper
class BinaryMeasurementsUnit3 {
  BinaryMeasurementsUnit3();
  size_t size() const;
  gtsam::BinaryMeasurement<gtsam::Unit3> at(size_t idx) const;
  void push_back(const gtsam::BinaryMeasurement<gtsam::Unit3>& measurement);
};

// Used in Matlab wrapper
class BinaryMeasurementsPoint3 {
  BinaryMeasurementsPoint3();
  size_t size() const;
  gtsam::BinaryMeasurement<gtsam::Point3> at(size_t idx) const;
  void push_back(const gtsam::BinaryMeasurement<gtsam::Point3>& measurement);
};

// Used in Matlab wrapper
class BinaryMeasurementsRot3 {
  BinaryMeasurementsRot3();
  size_t size() const;
  gtsam::BinaryMeasurement<gtsam::Rot3> at(size_t idx) const;
  void push_back(const gtsam::BinaryMeasurement<gtsam::Rot3>& measurement);
};

#include <gtsam/sfm/TrajectoryAlignerSim3.h>
class TrajectoryAlignerSim3 {
  TrajectoryAlignerSim3(
      const std::vector<gtsam::UnaryMeasurement<gtsam::Pose3>>& aTi,
      const std::vector<std::vector<gtsam::UnaryMeasurement<gtsam::Pose3>>>& bTi_all);

  TrajectoryAlignerSim3(
      const std::vector<gtsam::UnaryMeasurement<gtsam::Pose3>>& aTi,
      const std::vector<std::vector<gtsam::UnaryMeasurement<gtsam::Pose3>>>& bTi_all,
      const std::vector<gtsam::Similarity3>& bSa_all);

  TrajectoryAlignerSim3(
    const std::vector<gtsam::UnaryMeasurement<gtsam::Pose3>>& aTi,
    const std::vector<std::vector<gtsam::UnaryMeasurement<gtsam::Pose3>>>& bTi_all,
    const std::vector<gtsam::Similarity3>& bSa_all, const bool use_gnc_optimizer);

  TrajectoryAlignerSim3(    
    const std::vector<gtsam::UnaryMeasurement<gtsam::Pose3>>& aTi,
    const std::vector<std::vector<gtsam::UnaryMeasurement<gtsam::Pose3>>>& bTi_all,
    const std::vector<gtsam::Similarity3>& bSa_all, const bool use_gnc_optimizer,
    const std::vector<std::vector<std::pair<gtsam::Point3, gtsam::Point3>>> &overlapping_points);

  TrajectoryAlignerSim3(
    const std::vector<gtsam::UnaryMeasurement<gtsam::Pose3>>& aTi,
    const std::vector<std::vector<gtsam::UnaryMeasurement<gtsam::Pose3>>>& bTi_all,
    const std::vector<gtsam::Similarity3>& bSa_all, const bool use_gnc_optimizer,
    const std::vector<std::vector<std::pair<gtsam::Point3, gtsam::Point3>>> &overlapping_points,
    const double point3_factor_sigma);

  gtsam::Values solve() const;
  gtsam::Marginals marginalize(
    const gtsam::Values& solution, 
    const gtsam::Ordering::OrderingType ordering_type = gtsam::Ordering::COLAMD) const;
};

#include <gtsam/slam/dataset.h>
#include <gtsam/sfm/ShonanAveraging.h>

template <d={2, 3}>
class ShonanAveragingParameters {
  ShonanAveragingParameters(const gtsam::LevenbergMarquardtParams& lm);
  ShonanAveragingParameters(const gtsam::LevenbergMarquardtParams& lm,
                             string method);
  gtsam::LevenbergMarquardtParams getLMParams() const;
  void setOptimalityThreshold(double value);
  double getOptimalityThreshold() const;
  void setAnchor(size_t index, const gtsam::This::Rot& value);
  pair<size_t, gtsam::This::Rot> getAnchor() const;
  void setAnchorWeight(double value);
  double getAnchorWeight() const;
  void setKarcherWeight(double value);
  double getKarcherWeight() const;
  void setGaugesWeight(double value);
  double getGaugesWeight() const;
  void setUseHuber(bool value);
  bool getUseHuber() const;
  void setCertifyOptimality(bool value);
  bool getCertifyOptimality() const;
};

// NOTE(Varun): Not templated because each class has specializations defined.
class ShonanAveraging2 {
  ShonanAveraging2(string g2oFile);
  ShonanAveraging2(string g2oFile,
                   const gtsam::ShonanAveragingParameters2& parameters);
  ShonanAveraging2(const gtsam::BetweenFactorPose2s& factors,
                   const gtsam::ShonanAveragingParameters2& parameters);

  // Query properties
  size_t nrUnknowns() const;
  size_t numberMeasurements() const;
  const gtsam::Rot2& measured(size_t k) const;
  const gtsam::KeyVector& keys(size_t k) const;

  // gtsam::Matrix API (advanced use, debugging)
  gtsam::Matrix denseD() const;
  gtsam::Matrix denseQ() const;
  gtsam::Matrix denseL() const;
  // gtsam::Matrix computeLambda_(gtsam::Matrix S) const;
  gtsam::Matrix computeLambda_(const gtsam::Values& values) const;
  gtsam::Matrix computeA_(const gtsam::Values& values) const;
  double computeMinEigenValue(const gtsam::Values& values) const;
  double computeMinEigenValue(const gtsam::Values& values,
                              gtsam::Vector& minEigenVector) const;
  gtsam::Values initializeWithDescent(size_t p, const gtsam::Values& values,
                                      const gtsam::Vector& minEigenVector,
                                      double minEigenValue,
                                      double gradienTolerance = 1e-2,
                                      double preconditionedGradNormTolerance = 1e-4) const;

  // Advanced API
  gtsam::NonlinearFactorGraph buildGraphAt(size_t p) const;
  gtsam::Values initializeRandomlyAt(size_t p) const;
  double costAt(size_t p, const gtsam::Values& values) const;
  pair<double, gtsam::Vector> computeMinEigenVector(const gtsam::Values& values) const;
  bool checkOptimality(const gtsam::Values& values) const;
  gtsam::LevenbergMarquardtOptimizer* createOptimizerAt(
      size_t p, const gtsam::Values& initial) const;
  // gtsam::Values tryOptimizingAt(size_t p) const;
  gtsam::Values tryOptimizingAt(size_t p, const gtsam::Values& initial) const;
  gtsam::Values projectFrom(size_t p, const gtsam::Values& values) const;
  gtsam::Values roundSolution(const gtsam::Values& values) const;

  // Basic API
  double cost(const gtsam::Values& values) const;
  gtsam::Values initializeRandomly() const;
  pair<gtsam::Values, double> run(const gtsam::Values& initial, size_t min_p,
                                  size_t max_p) const;
  pair<gtsam::Values, double> run(size_t min_p, size_t max_p) const;
};

class ShonanAveraging3 {
  ShonanAveraging3(const gtsam::This::Measurements& measurements,
                   const gtsam::ShonanAveragingParameters3& parameters =
                       gtsam::ShonanAveragingParameters3());
  ShonanAveraging3(string g2oFile);
  ShonanAveraging3(string g2oFile,
                   const gtsam::ShonanAveragingParameters3& parameters);

  ShonanAveraging3(const gtsam::BetweenFactorPose3s& factors,
                   const gtsam::ShonanAveragingParameters3& parameters =
                       gtsam::ShonanAveragingParameters3());

  // Query properties
  size_t nrUnknowns() const;
  size_t numberMeasurements() const;
  const gtsam::Rot3& measured(size_t k) const;
  const gtsam::KeyVector& keys(size_t k) const;

  // gtsam::Matrix API (advanced use, debugging)
  gtsam::Matrix denseD() const;
  gtsam::Matrix denseQ() const;
  gtsam::Matrix denseL() const;
  // gtsam::Matrix computeLambda_(gtsam::Matrix S) const;
  gtsam::Matrix computeLambda_(const gtsam::Values& values) const;
  gtsam::Matrix computeA_(const gtsam::Values& values) const;
  double computeMinEigenValue(const gtsam::Values& values) const;
  double computeMinEigenValue(const gtsam::Values& values,
                              gtsam::Vector& minEigenVector) const;
  gtsam::Values initializeWithDescent(size_t p, const gtsam::Values& values,
                                      const gtsam::Vector& minEigenVector,
                                      double minEigenValue,
                                      double gradienTolerance = 1e-2,
                                      double preconditionedGradNormTolerance = 1e-4) const;

  // Advanced API
  gtsam::NonlinearFactorGraph buildGraphAt(size_t p) const;
  gtsam::Values initializeRandomlyAt(size_t p) const;
  double costAt(size_t p, const gtsam::Values& values) const;
  pair<double, gtsam::Vector> computeMinEigenVector(const gtsam::Values& values) const;
  bool checkOptimality(const gtsam::Values& values) const;
  gtsam::LevenbergMarquardtOptimizer* createOptimizerAt(
      size_t p, const gtsam::Values& initial) const;
  // gtsam::Values tryOptimizingAt(size_t p) const;
  gtsam::Values tryOptimizingAt(size_t p, const gtsam::Values& initial) const;
  gtsam::Values projectFrom(size_t p, const gtsam::Values& values) const;
  gtsam::Values roundSolution(const gtsam::Values& values) const;

  // Basic API
  double cost(const gtsam::Values& values) const;
  gtsam::Values initializeRandomly() const;
  pair<gtsam::Values, double> run(const gtsam::Values& initial, size_t min_p,
                                  size_t max_p) const;
  pair<gtsam::Values, double> run(size_t min_p, size_t max_p) const;
};

#include <gtsam/sfm/MFAS.h>

// Used in Matlab wrapper
class KeyPairDoubleMap {
  KeyPairDoubleMap();
  KeyPairDoubleMap(const gtsam::KeyPairDoubleMap& other);

  size_t size() const;
  bool empty() const;
  void clear();
  size_t at(const pair<gtsam::Key, gtsam::Key>& keypair) const;
};

class MFAS {
  MFAS(const gtsam::BinaryMeasurementsUnit3& relativeTranslations,
       const gtsam::Unit3& projectionDirection);

  gtsam::KeyPairDoubleMap computeOutlierWeights() const;
  gtsam::KeyVector computeOrdering() const;
};

#include <gtsam/sfm/LocationRecovery.h>

class LocationRecovery {
  LocationRecovery(const gtsam::LevenbergMarquardtParams& lmParams);
  LocationRecovery();
  gtsam::NonlinearFactorGraph buildGraph(
      const gtsam::LocationRecovery::DirectionEdges& edges,
      bool bilinear = true) const;
  void addAnchorPrior(gtsam::Key anchorKey,
                      gtsam::NonlinearFactorGraph @graph,
                      const gtsam::SharedNoiseModel& priorNoiseModel =
                          gtsam::noiseModel::Isotropic::Sigma(3, 0.01)) const;
  gtsam::Values initializeRandomly(
      const std::set<gtsam::Key>& keys, size_t numEdges, bool bilinear,
      const gtsam::Values& initialValues = gtsam::Values()) const;
};

#include <gtsam/sfm/TranslationRecovery.h>

class TranslationRecovery : gtsam::LocationRecovery {
  TranslationRecovery(const gtsam::LevenbergMarquardtParams& lmParams,
                      const bool use_bilinear_translation_factor);
  TranslationRecovery(const gtsam::LevenbergMarquardtParams& lmParams);
  TranslationRecovery();  // default params.
  void addPrior(
                const std::vector<gtsam::BinaryMeasurement<gtsam::Unit3>>&
                    relativeTranslations,
                const double scale,
                const std::vector<gtsam::BinaryMeasurement<gtsam::Point3>>&
                    betweenTranslations,
                gtsam::NonlinearFactorGraph @graph,
                const gtsam::SharedNoiseModel& priorNoiseModel =
                    gtsam::noiseModel::Isotropic::Sigma(3, 0.01)) const;
  gtsam::NonlinearFactorGraph buildGraph(
      const std::vector<gtsam::BinaryMeasurement<gtsam::Unit3>>&
          relativeTranslations) const;
  gtsam::Values run(
      const gtsam::TranslationRecovery::TranslationEdges&
          relativeTranslations,
      const double scale = 1.0,
      const std::vector<gtsam::BinaryMeasurement<gtsam::Point3>>&
          betweenTranslations =
              std::vector<gtsam::BinaryMeasurement<gtsam::Point3>>(),
      const gtsam::Values& initialValues = gtsam::Values()) const;
};

#include <gtsam/sfm/GlobalPositioner.h>

class GlobalPositioner : gtsam::LocationRecovery {
  GlobalPositioner(const gtsam::LevenbergMarquardtParams& lmParams);
  GlobalPositioner();
  gtsam::Values initializeRandomly(
      const std::set<gtsam::Key>& cameraKeys,
      const std::set<gtsam::Key>& landmarkKeys,
      const gtsam::GlobalPositioner::CameraPointDirections&
          cameraPointDirections,
      const gtsam::Values& initialValues = gtsam::Values()) const;
  gtsam::Values run(
      const gtsam::GlobalPositioner::CameraPointDirections&
          cameraPointDirections,
      const std::set<gtsam::Key>& cameraKeys,
      const std::set<gtsam::Key>& landmarkKeys, gtsam::Key anchorCameraKey,
      const gtsam::Values& initialValues = gtsam::Values()) const;
};

namespace gtsfm {

#include <gtsam/sfm/DsfTrackGenerator.h>

class MatchIndicesMap {
  MatchIndicesMap();
  MatchIndicesMap(const gtsam::gtsfm::MatchIndicesMap& other);

  size_t size() const;
  bool empty() const;
  void clear();
  gtsam::gtsfm::CorrespondenceIndices at(const gtsam::IndexPair& keypair) const;
};

class Keypoints {
  Keypoints(const Eigen::MatrixX2d& coordinates);
  Eigen::MatrixX2d coordinates;
};

class KeypointsVector {
  KeypointsVector();
  KeypointsVector(const gtsam::gtsfm::KeypointsVector& other);
  void push_back(const gtsam::gtsfm::Keypoints& keypoints);
  size_t size() const;
  bool empty() const;
  void clear();
  gtsam::gtsfm::Keypoints at(const size_t& index) const;
};

gtsam::SfmTrack2dVector tracksFromPairwiseMatches(
    const gtsam::gtsfm::MatchIndicesMap& matches_dict,
    const gtsam::gtsfm::KeypointsVector& keypoints_list, bool verbose = false);

}  // namespace gtsfm

}  // namespace gtsam
