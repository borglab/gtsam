//*************************************************************************
// sfm
//*************************************************************************

namespace gtsam {

// #####

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

class BinaryMeasurementsUnit3 {
  BinaryMeasurementsUnit3();
  size_t size() const;
  gtsam::BinaryMeasurement<gtsam::Unit3> at(size_t idx) const;
  void push_back(const gtsam::BinaryMeasurement<gtsam::Unit3>& measurement);
};

#include <gtsam/sfm/ShonanAveraging.h>

// TODO(frank): copy/pasta below until we have integer template paremeters in
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
  ShonanAveraging2(const gtsam::BetweenFactorPose2s &factors,
                   const gtsam::ShonanAveragingParameters2 &parameters);

  // Query properties
  size_t nrUnknowns() const;
  size_t nrMeasurements() const;
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
  ShonanAveraging3(string g2oFile);
  ShonanAveraging3(string g2oFile,
                   const gtsam::ShonanAveragingParameters3& parameters);

  // TODO(frank): deprecate once we land pybind wrapper
  ShonanAveraging3(const gtsam::BetweenFactorPose3s& factors);
  ShonanAveraging3(const gtsam::BetweenFactorPose3s& factors,
                   const gtsam::ShonanAveragingParameters3& parameters);

  // Query properties
  size_t nrUnknowns() const;
  size_t nrMeasurements() const;
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
  TranslationRecovery(
      const gtsam::BinaryMeasurementsUnit3& relativeTranslations,
      const gtsam::LevenbergMarquardtParams& lmParams);
  TranslationRecovery(
      const gtsam::BinaryMeasurementsUnit3&
          relativeTranslations);  // default LevenbergMarquardtParams
  gtsam::Values run(const double scale) const;
  gtsam::Values run() const;  // default scale = 1.0
};

}  // namespace gtsam
