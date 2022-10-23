//*************************************************************************
// sfm
//*************************************************************************

namespace gtsam {

#include <gtsam/sfm/SfmTrack.h>
class SfmTrack2d
{
  std::vector<pair<size_t, gtsam::Point2>> measurements;

  SfmTrack2d();
  SfmTrack2d(std::vector<gtsam::SfmMeasurement> &measurements);
  size_t numberMeasurements() const;
  pair<size_t, gtsam::Point2> measurement(size_t idx) const;
  pair<size_t, size_t> siftIndex(size_t idx) const;
  void addMeasurement(size_t idx, const gtsam::Point2& m);
  gtsam::SfmMeasurement measurement(size_t idx) const;
  bool hasUniqueCameras();
};

virtual class SfmTrack : gtsam::SfmTrack2d {
  SfmTrack();
  SfmTrack(const gtsam::Point3& pt);
  const Point3& point3() const;
  
  Point3 p;

  double r;
  double g;
  double b;

  // enabling serialization functionality
  void serialize() const;

  // enabling function to compare objects
  bool equals(const gtsam::SfmTrack& expected, double tol) const;
};

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/sfm/SfmData.h>
class SfmData {
  SfmData();
  static gtsam::SfmData FromBundlerFile(string filename);
  static gtsam::SfmData FromBalFile(string filename);

  std::vector<gtsam::SfmTrack>& trackList() const;
  std::vector<gtsam::PinholeCamera<gtsam::Cal3Bundler>>& cameraList() const;

  void addTrack(const gtsam::SfmTrack& t);
  void addCamera(const gtsam::SfmCamera& cam);
  size_t numberTracks() const;
  size_t numberCameras() const;
  gtsam::SfmTrack& track(size_t idx) const;
  gtsam::PinholeCamera<gtsam::Cal3Bundler>& camera(size_t idx) const;

  gtsam::NonlinearFactorGraph generalSfmFactors(
      const gtsam::SharedNoiseModel& model =
          gtsam::noiseModel::Isotropic::Sigma(2, 1.0)) const;
  gtsam::NonlinearFactorGraph sfmFactorGraph(
      const gtsam::SharedNoiseModel& model =
          gtsam::noiseModel::Isotropic::Sigma(2, 1.0),
      size_t fixedCamera = 0, size_t fixedPoint = 0) const;

  // enabling serialization functionality
  void serialize() const;

  // enabling function to compare objects
  bool equals(const gtsam::SfmData& expected, double tol) const;
};

gtsam::SfmData readBal(string filename);
bool writeBAL(string filename, gtsam::SfmData& data);
gtsam::Values initialCamerasEstimate(const gtsam::SfmData& db);
gtsam::Values initialCamerasAndPointsEstimate(const gtsam::SfmData& db);

#include <gtsam/sfm/ShonanFactor.h>

virtual class ShonanFactor3 : gtsam::NoiseModelFactor {
  ShonanFactor3(size_t key1, size_t key2, const gtsam::Rot3& R12, size_t p);
  ShonanFactor3(size_t key1, size_t key2, const gtsam::Rot3& R12, size_t p,
                gtsam::noiseModel::Base* model);
  Vector evaluateError(const gtsam::SOn& Q1, const gtsam::SOn& Q2);
};

#include <gtsam/sfm/BinaryMeasurement.h>
template <T>
class BinaryMeasurement {
  BinaryMeasurement(size_t key1, size_t key2, const T& measured,
                    const gtsam::noiseModel::Base* model);
  size_t key1() const;
  size_t key2() const;
  T measured() const;
  gtsam::noiseModel::Base* noiseModel() const;
};

typedef gtsam::BinaryMeasurement<gtsam::Unit3> BinaryMeasurementUnit3;
typedef gtsam::BinaryMeasurement<gtsam::Rot3> BinaryMeasurementRot3;
typedef gtsam::BinaryMeasurement<gtsam::Point3> BinaryMeasurementPoint3;

class BinaryMeasurementsUnit3 {
  BinaryMeasurementsUnit3();
  size_t size() const;
  gtsam::BinaryMeasurement<gtsam::Unit3> at(size_t idx) const;
  void push_back(const gtsam::BinaryMeasurement<gtsam::Unit3>& measurement);
};

class BinaryMeasurementsPoint3 {
  BinaryMeasurementsPoint3();
  size_t size() const;
  gtsam::BinaryMeasurement<gtsam::Point3> at(size_t idx) const;
  void push_back(const gtsam::BinaryMeasurement<gtsam::Point3>& measurement);
};

class BinaryMeasurementsRot3 {
  BinaryMeasurementsRot3();
  size_t size() const;
  gtsam::BinaryMeasurement<gtsam::Rot3> at(size_t idx) const;
  void push_back(const gtsam::BinaryMeasurement<gtsam::Rot3>& measurement);
};

#include <gtsam/sfm/ShonanAveraging.h>

// TODO(frank): copy/pasta below until we have integer template parameters in
// wrap!

class ShonanAveragingParameters2 {
  ShonanAveragingParameters2(const gtsam::LevenbergMarquardtParams& lm);
  ShonanAveragingParameters2(const gtsam::LevenbergMarquardtParams& lm,
                             string method);
  gtsam::LevenbergMarquardtParams getLMParams() const;
  void setOptimalityThreshold(double value);
  double getOptimalityThreshold() const;
  void setAnchor(size_t index, const gtsam::Rot2& value);
  pair<size_t, gtsam::Rot2> getAnchor();
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

class ShonanAveragingParameters3 {
  ShonanAveragingParameters3(const gtsam::LevenbergMarquardtParams& lm);
  ShonanAveragingParameters3(const gtsam::LevenbergMarquardtParams& lm,
                             string method);
  gtsam::LevenbergMarquardtParams getLMParams() const;
  void setOptimalityThreshold(double value);
  double getOptimalityThreshold() const;
  void setAnchor(size_t index, const gtsam::Rot3& value);
  pair<size_t, gtsam::Rot3> getAnchor();
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

class ShonanAveraging2 {
  ShonanAveraging2(string g2oFile);
  ShonanAveraging2(string g2oFile,
                   const gtsam::ShonanAveragingParameters2& parameters);
  ShonanAveraging2(const gtsam::BetweenFactorPose2s& factors,
                   const gtsam::ShonanAveragingParameters2& parameters);

  // Query properties
  size_t nrUnknowns() const;
  size_t numberMeasurements() const;
  gtsam::Rot2 measured(size_t i);
  gtsam::KeyVector keys(size_t i);

  // Matrix API (advanced use, debugging)
  Matrix denseD() const;
  Matrix denseQ() const;
  Matrix denseL() const;
  // Matrix computeLambda_(Matrix S) const;
  Matrix computeLambda_(const gtsam::Values& values) const;
  Matrix computeA_(const gtsam::Values& values) const;
  double computeMinEigenValue(const gtsam::Values& values) const;
  gtsam::Values initializeWithDescent(size_t p, const gtsam::Values& values,
                                      const Vector& minEigenVector,
                                      double minEigenValue) const;

  // Advanced API
  gtsam::NonlinearFactorGraph buildGraphAt(size_t p) const;
  gtsam::Values initializeRandomlyAt(size_t p) const;
  double costAt(size_t p, const gtsam::Values& values) const;
  pair<double, Vector> computeMinEigenVector(const gtsam::Values& values) const;
  bool checkOptimality(const gtsam::Values& values) const;
  gtsam::LevenbergMarquardtOptimizer* createOptimizerAt(
      size_t p, const gtsam::Values& initial);
  // gtsam::Values tryOptimizingAt(size_t p) const;
  gtsam::Values tryOptimizingAt(size_t p, const gtsam::Values& initial) const;
  gtsam::Values projectFrom(size_t p, const gtsam::Values& values) const;
  gtsam::Values roundSolution(const gtsam::Values& values) const;

  // Basic API
  double cost(const gtsam::Values& values) const;
  gtsam::Values initializeRandomly() const;
  pair<gtsam::Values, double> run(const gtsam::Values& initial, size_t min_p,
                                  size_t max_p) const;
};

class ShonanAveraging3 {
  ShonanAveraging3(
      const std::vector<gtsam::BinaryMeasurement<gtsam::Rot3>>& measurements,
      const gtsam::ShonanAveragingParameters3& parameters =
          gtsam::ShonanAveragingParameters3());
  ShonanAveraging3(string g2oFile);
  ShonanAveraging3(string g2oFile,
                   const gtsam::ShonanAveragingParameters3& parameters);

  // TODO(frank): deprecate once we land pybind wrapper
  ShonanAveraging3(const gtsam::BetweenFactorPose3s& factors);
  ShonanAveraging3(const gtsam::BetweenFactorPose3s& factors,
                   const gtsam::ShonanAveragingParameters3& parameters);

  // Query properties
  size_t nrUnknowns() const;
  size_t numberMeasurements() const;
  gtsam::Rot3 measured(size_t i);
  gtsam::KeyVector keys(size_t i);

  // Matrix API (advanced use, debugging)
  Matrix denseD() const;
  Matrix denseQ() const;
  Matrix denseL() const;
  // Matrix computeLambda_(Matrix S) const;
  Matrix computeLambda_(const gtsam::Values& values) const;
  Matrix computeA_(const gtsam::Values& values) const;
  double computeMinEigenValue(const gtsam::Values& values) const;
  gtsam::Values initializeWithDescent(size_t p, const gtsam::Values& values,
                                      const Vector& minEigenVector,
                                      double minEigenValue) const;

  // Advanced API
  gtsam::NonlinearFactorGraph buildGraphAt(size_t p) const;
  gtsam::Values initializeRandomlyAt(size_t p) const;
  double costAt(size_t p, const gtsam::Values& values) const;
  pair<double, Vector> computeMinEigenVector(const gtsam::Values& values) const;
  bool checkOptimality(const gtsam::Values& values) const;
  gtsam::LevenbergMarquardtOptimizer* createOptimizerAt(
      size_t p, const gtsam::Values& initial);
  // gtsam::Values tryOptimizingAt(size_t p) const;
  gtsam::Values tryOptimizingAt(size_t p, const gtsam::Values& initial) const;
  gtsam::Values projectFrom(size_t p, const gtsam::Values& values) const;
  gtsam::Values roundSolution(const gtsam::Values& values) const;

  // Basic API
  double cost(const gtsam::Values& values) const;
  gtsam::Values initializeRandomly() const;
  pair<gtsam::Values, double> run(const gtsam::Values& initial, size_t min_p,
                                  size_t max_p) const;
};

#include <gtsam/sfm/MFAS.h>

class KeyPairDoubleMap {
  KeyPairDoubleMap();
  KeyPairDoubleMap(const gtsam::KeyPairDoubleMap& other);

  size_t size() const;
  bool empty() const;
  void clear();
  size_t at(const pair<size_t, size_t>& keypair) const;
};

class MFAS {
  MFAS(const gtsam::BinaryMeasurementsUnit3& relativeTranslations,
       const gtsam::Unit3& projectionDirection);

  gtsam::KeyPairDoubleMap computeOutlierWeights() const;
  gtsam::KeyVector computeOrdering() const;
};

#include <gtsam/sfm/TranslationRecovery.h>

class TranslationRecovery {
  TranslationRecovery(const gtsam::LevenbergMarquardtParams& lmParams);
  TranslationRecovery();  // default params.
  void addPrior(const gtsam::BinaryMeasurementsUnit3& relativeTranslations,
                const double scale,
                const gtsam::BinaryMeasurementsPoint3& betweenTranslations,
                gtsam::NonlinearFactorGraph @graph,
                const gtsam::SharedNoiseModel& priorNoiseModel) const;
  void addPrior(const gtsam::BinaryMeasurementsUnit3& relativeTranslations,
                const double scale,
                const gtsam::BinaryMeasurementsPoint3& betweenTranslations,
                gtsam::NonlinearFactorGraph @graph) const;
  gtsam::NonlinearFactorGraph buildGraph(
      const gtsam::BinaryMeasurementsUnit3& relativeTranslations) const;
  gtsam::Values run(const gtsam::BinaryMeasurementsUnit3& relativeTranslations,
                    const double scale,
                    const gtsam::BinaryMeasurementsPoint3& betweenTranslations,
                    const gtsam::Values& initialValues) const;
  // default random initial values
  gtsam::Values run(
      const gtsam::BinaryMeasurementsUnit3& relativeTranslations,
      const double scale,
      const gtsam::BinaryMeasurementsPoint3& betweenTranslations) const;
  // default empty betweenTranslations
  gtsam::Values run(const gtsam::BinaryMeasurementsUnit3& relativeTranslations,
                    const double scale) const;
  // default scale = 1.0, empty betweenTranslations
  gtsam::Values run(
      const gtsam::BinaryMeasurementsUnit3& relativeTranslations) const;
};

namespace gtsfm {

#include <gtsam/sfm/DsfTrackGenerator.h>

class MatchIndicesMap {
  MatchIndicesMap();
  MatchIndicesMap(const gtsam::gtsfm::MatchIndicesMap& other);

  size_t size() const;
  bool empty() const;
  void clear();
  gtsam::gtsfm::CorrespondenceIndices at(const pair<size_t, size_t>& keypair) const;
};


class Keypoints
{
  Keypoints(const gtsam::gtsfm::KeypointCoordinates& coordinates);
  gtsam::gtsfm::KeypointCoordinates coordinates;
}; // check if this should be a method


class KeypointsVector {
  KeypointsVector();
  KeypointsVector(const gtsam::gtsfm::KeypointsVector& other);
  void push_back(const gtsam::gtsfm::Keypoints& keypoints);
  size_t size() const;
  bool empty() const;
  void clear();
  gtsam::gtsfm::Keypoints at(const size_t& index) const;
};

class DsfTrackGenerator {
  DsfTrackGenerator();
  const gtsam::SfmTrack2dVector tracksFromPairwiseMatches(
    const gtsam::gtsfm::MatchIndicesMap& matches_dict,
    const gtsam::gtsfm::KeypointsVector& keypoints_list,
    bool verbose = false);
};
}  // namespace gtsfm

}  // namespace gtsam
